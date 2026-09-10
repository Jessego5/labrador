"""
This is the ROS side of the benchmarks, the one node every trial runner talks to the cell
through. It teleports objects by setting MuJoCo free joints, reads their true poses from the
simulator's free joint publisher, and sends goals to the skill actions and waits for the
results. Ground truth is only ever used here to set up a trial and to score it; the skills under
test never see it.
"""
import math
import time

import rclpy
from cell_interfaces.action import DetectObjects, Insert, MoveToNamedPose, Pick, Place
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_of(q):
    """The rotation about z, in radians, of the quaternion q."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class CellClient(Node):
    """
    A node with clients for every skill action and for set_free_joint_state, plus the latest
    ground-truth pose of each free joint, keyed by name.
    """

    def __init__(self, name="cell_benchmark"):
        super().__init__(name)
        self.ground_truth = {}
        self.create_subscription(
            FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self._on_gt, 10)
        self.actions = {
            "move": ActionClient(self, MoveToNamedPose, "/skills/move_to_named_pose"),
            "detect": ActionClient(self, DetectObjects, "/skills/detect_objects"),
            "pick": ActionClient(self, Pick, "/skills/pick"),
            "place": ActionClient(self, Place, "/skills/place"),
            "insert": ActionClient(self, Insert, "/skills/insert"),
        }
        self.set_free_joint = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")

    def _on_gt(self, msg):
        for fj in msg.free_joints:
            self.ground_truth[fj.name] = fj.pose.pose

    def wait(self, future, timeout):
        """The future's result, spinning this node until it is done. Raises TimeoutError."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("timed out")
        return future.result()

    def spin_for(self, seconds):
        """Spin for a fixed time, so subscriptions and the simulation keep moving."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_ready(self, timeout=120.0):
        """
        Block until every skill action and the set_free_joint_state service are up. Raises
        RuntimeError naming the first one that does not appear within the timeout.
        """
        for name, client in self.actions.items():
            if not client.wait_for_server(timeout_sec=timeout):
                raise RuntimeError(f"skill action '{name}' not available")
        if not self.set_free_joint.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("set_free_joint_state service not available")

    def fresh_ground_truth(self, timeout=5.0):
        """
        {free joint name: pose} from a message received after this call, so the poses reflect
        anything just teleported or moved rather than an older message.
        """
        self.ground_truth = {}
        end = time.monotonic() + timeout
        while not self.ground_truth:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > end:
                raise RuntimeError("no ground truth from the simulator")
        self.spin_for(0.1)
        return dict(self.ground_truth)

    def run(self, action, goal, timeout=120.0):
        """
        The result of one skill action ("move", "detect", "pick", "place" or "insert"). Raises
        RuntimeError when the goal is rejected and TimeoutError when it does not finish.
        """
        handle = self.wait(self.actions[action].send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError(f"{action} goal rejected")
        return self.wait(handle.get_result_async(), timeout).result

    def set_object_poses(self, poses, sizes, settle=0.5):
        """
        Teleport objects to {name: (x, y, yaw)}, each resting on the table (its centre half its
        height up), then let the simulation settle.
        """
        self.set_free_joints(
            {name: (x, y, sizes[name][2] / 2, yaw) for name, (x, y, yaw) in poses.items()}, settle)

    def set_free_joints(self, poses, settle=0.5):
        """
        Teleport free joints to {free joint name: (x, y, z, yaw)} in the world frame, then let
        the simulation settle. Raises RuntimeError when the simulator refuses.
        """
        states = []
        for name, (x, y, z, yaw) in poses.items():
            s = FreeJointState(name=name)
            s.pose.header.frame_id = "world"
            s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z = x, y, z
            s.pose.pose.orientation.w = math.cos(yaw / 2)
            s.pose.pose.orientation.z = math.sin(yaw / 2)
            states.append(s)
        result = self.wait(
            self.set_free_joint.call_async(SetFreeJointState.Request(free_joints=states)), 5.0)
        if not result.success:
            raise RuntimeError(f"set_free_joint_state failed: {result.message}")
        self.spin_for(settle)
