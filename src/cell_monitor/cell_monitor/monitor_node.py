"""
This is the fault injector the reliability benchmark runs the cell against. Between two calls,
/monitor/start_run and /monitor/end_run, it watches the skills' action feedback and, at the
moment each skill is most exposed, may move the object it is working on in the simulator:

  a Pick that starts its descent may have its object nudged aside (nudge)
  a Pick that starts its lift may lose the object from the closed fingers (slip)
  a Place or Insert that starts its approach may drop the object on the way (drop)

and at the start of a run it may shift the fixture (fixture_shift) and turn on perception's
colour noise (camera_noise). Which faults are on, how often each opportunity fires and the seed
come with start_run (cell_interfaces/StartRun); faults.py makes the decisions. The executor and
the skills are never told: they have to notice from what they see and feel, as on a real cell.

Every fault is published as JSON on /monitor/faults with the executor's step and item at the
time, so the benchmark can tell which ones the task recovered from, and end_run returns them all.
With a bag_dir, the run is also recorded as an MCAP rosbag (executor state, faults, joint states,
TF, the true object poses and the skills' feedback and status; not the camera images, which
would make every bag hundreds of megabytes). The node is single-threaded and never blocks:
every call it makes to the simulator or to perception is asynchronous.
"""
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from cell_benchmarks.scene import SceneConfig
from cell_interfaces.action import Insert, Pick, Place
from cell_interfaces.srv import StartRun
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from cell_monitor.faults import (CAMERA_NOISE, FaultPlan, drop_target, fixture_offset,
                                 nudge_target)

FIXTURE = "peg_fixture"
NEAR_TCP = 0.06        # m; an object this close to the TCP is the one in or under the gripper
DROP_DELAY = 0.3       # s after a transport starts; the move itself takes 2 to 4 s
BAG_TOPICS = [
    "/executor/state", "/monitor/faults", "/joint_states", "/tf", "/tf_static",
    "/free_joint_state_publisher/free_joint_states",
    "/skills/pick/_action/feedback", "/skills/pick/_action/status",
    "/skills/place/_action/feedback", "/skills/place/_action/status",
    "/skills/insert/_action/feedback", "/skills/insert/_action/status",
    "/skills/detect_objects/_action/status",
]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Monitor(Node):
    def __init__(self):
        super().__init__("monitor")
        desc = get_package_share_directory("cell_description")
        self.scene = SceneConfig(f"{desc}/config/objects.yaml", f"{desc}/config/workspace.yaml")
        self.world_frame = self.declare_parameter("world_frame", "world").value
        self.tcp_frame = self.declare_parameter("tcp_frame", "tcp").value

        self.poses = {}        # free joint name -> Pose, from the simulator
        self.state = {}        # the executor's last /executor/state
        self.phases = {}       # skill -> its last feedback phase
        self.plan = None       # the run's FaultPlan, between start_run and end_run
        self.faults = []
        self.run_id = ""
        self.t0 = 0.0
        self.bag = None
        self.pending_drop = None
        self.faulted = set()   # objects that already had their fault this run

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(FreeJointStateArray,
                                 "/free_joint_state_publisher/free_joint_states", self.on_poses, 10)
        self.create_subscription(String, "/executor/state", self.on_state, 10)
        for skill, action in (("Pick", Pick), ("Place", Place), ("Insert", Insert)):
            self.create_subscription(
                action.Impl.FeedbackMessage, f"/skills/{skill.lower()}/_action/feedback",
                lambda msg, s=skill: self.on_feedback(s, msg.feedback.phase), 10)
        self.fault_pub = self.create_publisher(String, "/monitor/faults", 50)
        self.set_state = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")
        self.set_params = self.create_client(SetParameters, "/perception/set_parameters")
        self.create_service(StartRun, "/monitor/start_run", self.start_run)
        self.create_service(Trigger, "/monitor/end_run", self.end_run)
        self.create_timer(0.05, self.tick)
        self.get_logger().info("Monitor ready (faults off until /monitor/start_run)")

    # - inputs

    def on_poses(self, msg):
        for fj in msg.free_joints:
            self.poses[fj.name] = fj.pose.pose

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def tcp(self):
        """(x, y, z, x axis (ax, ay)) of the TCP in the world frame, or None without TF."""
        try:
            t = self.tf.lookup_transform(self.world_frame, self.tcp_frame, Time())
        except Exception:  # noqa: BLE001 - no transform yet
            return None
        p, q = t.transform.translation, t.transform.rotation
        # the first column of the rotation matrix, projected on the table
        ax = 1 - 2 * (q.y * q.y + q.z * q.z)
        ay = 2 * (q.x * q.y + q.w * q.z)
        n = math.hypot(ax, ay) or 1.0
        return p.x, p.y, p.z, (ax / n, ay / n)

    def objects(self):
        """{name: Pose} of the objects the benchmark lays out (not the fixture)."""
        return {n: p for n, p in self.poses.items() if n in self.scene.objects}

    def nearest(self, x, y, z=None, radius=NEAR_TCP):
        """The object nearest (x, y), or (x, y, z) when z is given, within radius, or None."""
        best, best_d = None, radius
        for name, p in self.objects().items():
            d = math.hypot(p.position.x - x, p.position.y - y)
            if z is not None:
                d = math.hypot(d, p.position.z - z)
            if d <= best_d:
                best, best_d = name, d
        return best

    def eligible(self, name):
        """An object gets at most one fault a run, so the benchmark measures recovery from a
        fault rather than the odds of three in a row on one cube (an early run lost a cube to a
        slip, a nudge of the pick that recovered it, and a second slip)."""
        return name is not None and name not in self.faulted

    def others(self, name):
        return [(p.position.x, p.position.y, 0.0) for n, p in self.objects().items() if n != name]

    # - faults

    def on_feedback(self, skill, phase):
        if self.plan is None or self.phases.get(skill) == phase:
            return
        self.phases[skill] = phase
        if skill == "Pick" and phase == "descend" and self.plan.fires("nudge"):
            self.nudge()
        elif skill == "Pick" and phase == "lift" and self.plan.fires("slip"):
            self.slip()
        elif skill in ("Place", "Insert") and phase == "approach" and self.plan.fires("drop"):
            self.pending_drop = (time.monotonic() + DROP_DELAY, skill)

    def tick(self):
        if self.pending_drop and time.monotonic() >= self.pending_drop[0]:
            skill = self.pending_drop[1]
            self.pending_drop = None
            self.drop(skill)

    def nudge(self):
        tcp = self.tcp()
        if tcp is None:
            return
        x, y, _, axis = tcp
        name = self.nearest(x, y, radius=0.03)
        if not self.eligible(name):
            return
        p = self.poses[name].position
        to = nudge_target(self.scene, (p.x, p.y), axis, self.others(name))
        if to is not None:
            self.move(name, to, "nudge", "Pick")

    def slip(self):
        tcp = self.tcp()
        if tcp is None:
            return
        name = self.nearest(*tcp[:3])
        if not self.eligible(name):
            return
        to = drop_target(self.scene, tcp[:2], self.others(name), self.plan.rng)
        if to is not None:
            self.move(name, to, "slip", "Pick")

    def drop(self, skill):
        tcp = self.tcp()
        if tcp is None:
            return
        name = self.nearest(*tcp[:3])
        if not self.eligible(name):
            return
        to = drop_target(self.scene, tcp[:2], self.others(name), self.plan.rng)
        if to is not None:
            self.move(name, to, "drop", skill)

    def move(self, name, xy, kind, skill):
        """Teleport `name` upright to xy on the table, at rest, and record the fault."""
        p = self.poses[name]
        yaw = yaw_of(p.orientation)
        z = self.scene.objects[name].size[2] / 2 + 0.001
        self.teleport(name, xy[0], xy[1], z, yaw)
        self.faulted.add(name)
        self.record(kind, skill, name, (p.position.x, p.position.y, p.position.z), (*xy, z))

    def teleport(self, name, x, y, z, yaw):
        s = FreeJointState(name=name)
        s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z = x, y, z
        s.pose.pose.orientation.z, s.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.set_state.call_async(SetFreeJointState.Request(free_joints=[s]))

    def record(self, kind, skill, name, frm=None, to=None, **extra):
        fault = {"t": round(time.monotonic() - self.t0, 2), "kind": kind, "skill": skill,
                 "object": name, "step": self.state.get("step"), "item": self.state.get("item"),
                 **extra}
        if frm is not None:
            fault["from"] = [round(v, 4) for v in frm]
            fault["to"] = [round(v, 4) for v in to]
        self.faults.append(fault)
        self.fault_pub.publish(String(data=json.dumps(fault)))
        self.get_logger().info(f"fault {kind}: {json.dumps(fault)}")

    def set_noise(self, value):
        if not self.set_params.service_is_ready():
            self.get_logger().warn("perception parameters not available; no camera noise")
            return False
        param = Parameter(name="color_noise", value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
        self.set_params.call_async(SetParameters.Request(parameters=[param]))
        return True

    # - runs

    def start_run(self, req, res):
        if self.plan is not None:
            self.finish()
        try:
            self.plan = FaultPlan(list(req.faults), req.rate, req.seed)
        except ValueError as e:
            res.success, res.message = False, str(e)
            return res
        self.faults, self.phases, self.pending_drop, self.faulted = [], {}, None, set()
        self.run_id, self.t0 = req.run_id, time.monotonic()
        if req.bag_dir:
            self.start_bag(req.bag_dir)
        if self.plan.has("fixture_shift"):
            p = self.poses.get(FIXTURE)
            if p is None:
                self.get_logger().warn("no pose for the fixture yet; not shifting it")
            else:
                dx, dy = fixture_offset(self.plan.rng)
                frm = (p.position.x, p.position.y, p.position.z)
                self.teleport(FIXTURE, frm[0] + dx, frm[1] + dy, frm[2], yaw_of(p.orientation))
                self.record("fixture_shift", "", FIXTURE, frm, (frm[0] + dx, frm[1] + dy, frm[2]),
                            shift_mm=round(1000 * math.hypot(dx, dy), 2))
        if self.plan.has("camera_noise") and self.set_noise(CAMERA_NOISE):
            self.record("camera_noise", "DetectObjects", "", color_noise=CAMERA_NOISE)
        res.success = True
        res.message = (f"run {self.run_id}: faults {sorted(self.plan.faults) or 'none'} at rate "
                       f"{self.plan.rate}" + (f", recording {req.bag_dir}" if req.bag_dir else ""))
        self.get_logger().info(res.message)
        return res

    def end_run(self, req, res):
        del req
        faults = self.finish()
        res.success = True
        res.message = json.dumps({"run_id": self.run_id, "faults": faults})
        return res

    def finish(self):
        if self.plan is not None and self.plan.has("camera_noise"):
            self.set_noise(0.0)
        self.plan = None
        self.pending_drop = None
        self.stop_bag()
        return self.faults

    def start_bag(self, bag_dir):
        self.stop_bag()
        os.makedirs(os.path.dirname(os.path.abspath(bag_dir)), exist_ok=True)
        self.bag = subprocess.Popen(
            ["ros2", "bag", "record", "-s", "mcap", "--include-hidden-topics", "-o", bag_dir,
             *BAG_TOPICS],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def stop_bag(self):
        if self.bag is None:
            return
        # SIGINT lets the recorder write the MCAP footer; without one the bag reads as truncated
        os.killpg(self.bag.pid, signal.SIGINT)
        try:
            self.bag.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(self.bag.pid, signal.SIGKILL)
        self.bag = None


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_bag()
        node.destroy_node()
        rclpy.try_shutdown()
