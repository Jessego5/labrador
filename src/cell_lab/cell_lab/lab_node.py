"""
This is the lab node, which plays the part of the real world in the lab scene: it runs the
samples' chemistry (chemistry.py) on simulation time and makes it visible by moving MuJoCo
bodies, the only way a colour can change while the simulation runs. Every sample has a body
for each look it can have, parked under the table; showing a new look means teleporting that
twin into the place of the one shown now and parking the old one, in a single
set_free_joint_state call so no frame shows both or neither. It reads the twins' poses from the
free-joint publisher to know where each sample is, whether it is in the gripper (lifted above
held_height, or moving), and which vial stands in the dispenser. The robot does not use this
node's state: it sees the changes through the camera like a real cell would, and only the
benchmarks read /lab/state, to score what the robot did against what truly happened.

Interfaces: /lab/reset (cell_interfaces/srv/LabReset) sets out one protocol's samples with a
seed; /lab/dispense (std_srvs/srv/Trigger) doses the vial standing in the dispenser; /lab/state
(std_msgs/String, JSON) is the true state, published twice a second and after every change.
cell.launch.py starts it with scene:=lab; set the protocol parameter to set out a protocol at
start-up.
"""
import json

import rclpy
import yaml
from cell_interfaces.srv import LabReset
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cell_lab.chemistry import Lab, parking_spots, twin_name
from cell_lab.layout import slot_floor


class LabNode(Node):
    def __init__(self):
        super().__init__("lab")
        lab_file = self.declare_parameter("lab_file", "").value
        workspace_file = self.declare_parameter("workspace_file", "").value
        self.held_height = self.declare_parameter("held_height", 0.065).value  # m, sample centre
        self.held_speed = self.declare_parameter("held_speed", 0.03).value     # m/s
        self.dispenser = self.declare_parameter("dispenser_target", "dispenser").value
        self.dispenser_radius = self.declare_parameter("dispenser_radius", 0.01).value  # m
        start_protocol = self.declare_parameter("protocol", "").value
        start_seed = self.declare_parameter("seed", 0).value
        period = self.declare_parameter("period", 0.05).value

        with open(lab_file) as f:
            self.cfg = yaml.safe_load(f)
        with open(workspace_file) as f:
            self.workspace = yaml.safe_load(f)
        self.lab = Lab(self.cfg)
        self.parking = parking_spots(self.cfg)
        self.heights = {twin_name(s.name, look): self.cfg["types"][look]["size"][2]
                        for s in self.lab.samples.values() for look in s.looks}

        self.poses = {}       # twin name -> (x, y, z, qx, qy, qz, qw)
        self.speeds = {}      # twin name -> m/s, from consecutive poses
        self.last_seen = {}   # twin name -> (t, x, y, z)

        # One thread and no blocking calls: the first version ran on rclpy's multithreaded
        # executor so its services could wait for the simulator, and it took 95% of a core with
        # nothing to do, enough to starve the software-rendered camera until detections timed
        # out. Writes to the simulator are fire-and-forget now, and the poses they set are
        # recorded straight away.
        self.create_subscription(FreeJointStateArray,
                                 "/free_joint_state_publisher/free_joint_states",
                                 self.on_poses, 10)
        self.set_state = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")
        self.state_pub = self.create_publisher(String, "/lab/state", 10)
        self.create_service(LabReset, "/lab/reset", self.on_reset)
        self.create_service(Trigger, "/lab/dispense", self.on_dispense)
        self.create_timer(period, self.tick)
        self.create_timer(0.5, self.publish_state)

        self.pending_start = (start_protocol, start_seed) if start_protocol else None
        self.get_logger().info(
            f"Lab ready: {len(self.lab.samples)} samples in {len(self.lab.protocols)} protocols, "
            f"{len(self.parking)} twins")

    # - simulator

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_poses(self, msg):
        t = self.now()
        for fj in msg.free_joints:
            if fj.name not in self.parking:
                continue
            p, q = fj.pose.pose.position, fj.pose.pose.orientation
            self.poses[fj.name] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            prev = self.last_seen.get(fj.name)
            # A step to or from the parking floor under the table is a twin swap, not a grasp. The
            # swap's call is answered asynchronously, so a message already on its way can report
            # the new twin still parked after teleport() cleared it, and the 0.8 m jump to the
            # next message read as 8 m/s and recorded a lift 0.2 s after the change (a growth
            # dish scored as harvested two stages early).
            if prev and t > prev[0] and min(p.z, prev[3]) > 0.0:
                d = ((p.x - prev[1]) ** 2 + (p.y - prev[2]) ** 2 + (p.z - prev[3]) ** 2) ** 0.5
                self.speeds[fj.name] = d / (t - prev[0])
            self.last_seen[fj.name] = (t, p.x, p.y, p.z)

    def teleport(self, placements):
        """Write {twin: (x, y, z, qx, qy, qz, qw)} in one call, at rest, without waiting for the
        answer (a refusal is logged when it comes). False when the simulator is not there."""
        if not self.set_state.service_is_ready():
            self.get_logger().error("set_free_joint_state is not available")
            return False
        states = []
        for name, (x, y, z, qx, qy, qz, qw) in placements.items():
            s = FreeJointState(name=name)
            s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z = x, y, z
            o = s.pose.pose.orientation
            o.x, o.y, o.z, o.w = qx, qy, qz, qw
            states.append(s)
        future = self.set_state.call_async(SetFreeJointState.Request(free_joints=states))
        future.add_done_callback(self.on_teleported)
        # the poses just written, until the publisher reports them
        for name, pose in placements.items():
            self.poses[name] = pose
            self.speeds[name] = 0.0
            self.last_seen.pop(name, None)
        return True

    def on_teleported(self, future):
        result = future.result()
        if result is None or not result.success:
            msg = result.message if result is not None else "no answer"
            self.get_logger().error(f"set_free_joint_state failed: {msg}")

    def shown(self, name):
        """The pose of the twin that shows sample `name` now, or None before any arrived."""
        return self.poses.get(twin_name(name, self.lab.samples[name].look))

    def held(self, name):
        """True while a sample is lifted or moving, i.e. in the gripper."""
        twin = twin_name(name, self.lab.samples[name].look)
        pose = self.poses.get(twin)
        speed = self.speeds.get(twin, 0.0)
        return pose is not None and (pose[2] > self.held_height or speed > self.held_speed)

    # - chemistry

    def set_out(self, protocol, seed):
        """Park every twin, then put each of the protocol's samples in its slot, start look."""
        active = self.lab.reset(protocol, seed, self.now())
        placements = {n: (x, y, z, 0.0, 0.0, 0.0, 1.0) for n, (x, y, z) in self.parking.items()}
        for name in active:
            s = self.lab.samples[name]
            # a rack slot by name, or [x, y] on the table for samples too wide for a rack
            if isinstance(s.slot, str):
                x, y, floor = slot_floor(self.workspace, s.slot)
            else:
                (x, y), floor = s.slot, 0.0
            twin = twin_name(name, s.look)
            placements[twin] = (x, y, floor + self.heights[twin] / 2, 0.0, 0.0, 0.0, 1.0)
        if not self.teleport(placements):
            raise RuntimeError("could not set the samples out")
        return active

    def tick(self):
        if self.pending_start and self.set_state.service_is_ready():
            protocol, seed = self.pending_start
            self.pending_start = None
            try:
                self.set_out(protocol, seed)
                self.get_logger().info(f"set out protocol '{protocol}' with seed {seed}")
            except (KeyError, RuntimeError) as e:
                self.get_logger().error(str(e))
        if not self.lab.active:
            return
        t = self.now()
        for name in self.lab.active:
            if self.held(name):
                self.lab.lifted(name, t)
        swaps = self.lab.step(t, self.held)
        if not swaps:
            return
        placements = {}
        for name, old, new in swaps:
            pose = self.shown_twin_pose(name, old)
            if pose is None:
                continue
            placements[twin_name(name, new)] = pose
            x, y, z = self.parking[twin_name(name, old)]
            placements[twin_name(name, old)] = (x, y, z, 0.0, 0.0, 0.0, 1.0)
            self.get_logger().info(f"{name}: {old} -> {new}")
        if placements:
            self.teleport(placements)
        self.publish_state()

    def shown_twin_pose(self, name, look):
        return self.poses.get(twin_name(name, look))

    # - services

    def on_reset(self, request, response):
        try:
            response.samples = self.set_out(request.protocol, int(request.seed))
            response.success = True
            response.message = (f"set out {len(response.samples)} '{request.protocol}' samples "
                                f"with seed {request.seed}")
        except (KeyError, RuntimeError) as e:
            response.success, response.message = False, str(e)
        self.get_logger().info(f"reset: {response.message}")
        self.publish_state()
        return response

    def on_dispense(self, request, response):
        del request
        try:
            x, y, _ = slot_floor(self.workspace, self.dispenser)
        except KeyError as e:
            response.success, response.message = False, str(e)
            return response
        for name in self.lab.active:
            pose = self.shown(name)
            if pose is None or self.held(name):
                continue
            if ((pose[0] - x) ** 2 + (pose[1] - y) ** 2) ** 0.5 <= self.dispenser_radius:
                try:
                    doses = self.lab.dispense(name, self.now())
                except ValueError as e:
                    response.success, response.message = False, str(e)
                    return response
                response.success, response.message = True, f"{name}: dose {doses}"
                self.get_logger().info(f"dispense: {response.message}")
                self.publish_state()
                return response
        response.success, response.message = False, "no sample stands in the dispenser"
        return response

    def publish_state(self):
        self.state_pub.publish(String(data=json.dumps(self.lab.state(self.now()))))


def main():
    rclpy.init()
    node = LabNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
