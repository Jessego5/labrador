#!/usr/bin/env python3
"""This is the part 3 acceptance test, pick an object from a start pose and place it in a target.
It is the test the part 3 milestone was accepted with, and it repeats the same pick and place.
Each trial puts the object back at its start pose, sends the arm home, picks the object and
places it, then scores the placement against the simulator's ground truth, the xy error from
the target centre and whether the object's bottom came to rest on the pocket floor. The object
pose handed to Pick comes from ground truth too, which was a stand-in until perception arrived
in part 4, and the scoring keeps using ground truth after part 4 as well. The part 3 run of ten
trials gave 10/10 at about 1.3 mm placement error. Run it with ros2 run cell_skills
run_pick_place_trials.py --object cyl_blue_0 --target tray_1/A1 -n 10, and add --compliance to
place with the compliance controller.
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from cell_interfaces.action import MoveToNamedPose, Pick, Place
from cell_interfaces.srv import ResolveTarget
from geometry_msgs.msg import Vector3
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState
from rclpy.action import ActionClient
from rclpy.node import Node

# bounding boxes (x, y, z) of the scene's objects by name prefix, as in config/objects.yaml
OBJECT_SIZES = {"cube": (0.04, 0.04, 0.04), "cyl": (0.03, 0.03, 0.05), "peg": (0.02, 0.02, 0.07)}
POCKET_FLOOR_TOLERANCE = 0.003  # m, the object counts as seated when its bottom is this close


class Trials(Node):
    """
    The node a trial run works through: clients for the skills, the workspace resolver and the
    simulator's set_free_joint_state service, and a cache of the ground-truth object poses.
    """

    def __init__(self):
        super().__init__("pick_place_trials")
        self.ground_truth = {}
        self.create_subscription(
            FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self.on_gt, 10)
        self.move = ActionClient(self, MoveToNamedPose, "/skills/move_to_named_pose")
        self.pick = ActionClient(self, Pick, "/skills/pick")
        self.place = ActionClient(self, Place, "/skills/place")
        self.resolve = self.create_client(ResolveTarget, "/workspace/resolve_target")
        self.set_free_joint = self.create_client(
            SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")

    def on_gt(self, msg):
        self.ground_truth = {fj.name: fj.pose for fj in msg.free_joints}

    def wait(self, future, timeout):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("timed out")
        return future.result()

    def settle(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def fresh_pose(self, name):
        """The next ground-truth pose of `name` to arrive, not the cached one."""
        self.ground_truth.pop(name, None)
        end = time.monotonic() + 5.0
        while name not in self.ground_truth:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() > end:
                raise RuntimeError(f"no ground truth for '{name}'")
        return self.ground_truth[name]

    def action(self, client, goal, timeout=120.0):
        handle = self.wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError("goal rejected")
        return self.wait(handle.get_result_async(), timeout).result

    def wait_for_servers(self):
        for client in (self.move, self.pick, self.place):
            if not client.wait_for_server(timeout_sec=60.0):
                raise RuntimeError(f"action server {client._action_name} not available")
        for client in (self.resolve, self.set_free_joint):
            if not client.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f"service {client.srv_name} not available")

    def reset_object(self, name, pose):
        """Put the object back at `pose` in the simulator and let it settle for 0.5 s."""
        state = FreeJointState(name=name)
        state.pose.header.frame_id = "world"
        state.pose.pose = pose.pose
        result = self.wait(
            self.set_free_joint.call_async(SetFreeJointState.Request(free_joints=[state])), 5.0)
        if not result.success:
            raise RuntimeError(f"set_free_joint_state failed: {result.message}")
        self.settle(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--object", default="cyl_blue_0")
    parser.add_argument("--target", default="tray_1/A1")
    parser.add_argument("-n", "--trials", type=int, default=10)
    parser.add_argument("--compliance", action="store_true", help="place with the compliance controller")
    args = parser.parse_args()

    size = next((s for prefix, s in OBJECT_SIZES.items() if args.object.startswith(prefix)), None)
    if size is None:
        sys.exit(f"unknown object size for '{args.object}'")

    rclpy.init()
    node = Trials()
    node.wait_for_servers()
    target = node.wait(node.resolve.call_async(ResolveTarget.Request(name=args.target)), 5.0)
    if not target.success:
        sys.exit(f"cannot resolve '{args.target}': {target.message}")
    slot = target.pose.pose.position
    start_pose = node.fresh_pose(args.object)
    print(f"{args.object} -> {args.target} (slot centre {slot.x:.3f}, {slot.y:.3f}), "
          f"{args.trials} trials\n")
    print(f"{'trial':>5} {'pick':>6} {'place':>6} {'seated':>6} {'err_mm':>7} {'time_s':>6}  note")

    rows = []
    for i in range(1, args.trials + 1):
        t0 = time.monotonic()
        note, picked, placed, seated, err_mm = "", False, False, False, float("nan")
        try:
            node.reset_object(args.object, start_pose)
            home = node.action(node.move, MoveToNamedPose.Goal(name="home"))
            if not home.success:
                raise RuntimeError(f"home: {home.message}")

            pose = node.fresh_pose(args.object)
            pick = node.action(node.pick, Pick.Goal(
                object_id=args.object, object_pose=pose, object_size=Vector3(x=size[0], y=size[1], z=size[2])))
            picked = pick.success
            if not picked:
                raise RuntimeError(f"pick: {pick.message}")

            place = node.action(node.place, Place.Goal(target_name=args.target, use_compliance=args.compliance))
            placed = place.success
            if not placed:
                raise RuntimeError(f"place: {place.message}")

            node.settle(1.0)
            final = node.fresh_pose(args.object).pose.position
            err_mm = 1000 * math.hypot(final.x - slot.x, final.y - slot.y)
            # for a slot target the pocket floor is taken as 15 mm below the opening
            pocket_floor = slot.z - 0.015 if "/" in args.target else slot.z
            bottom = final.z - size[2] / 2
            seated = abs(bottom - pocket_floor) < POCKET_FLOOR_TOLERANCE
            if not seated:
                note = f"bottom at z={bottom:.4f}, expected {pocket_floor:.4f}"
        except Exception as e:  # noqa: BLE001 - report and continue with the next trial
            note = str(e)
        dt = time.monotonic() - t0
        ok = picked and placed and seated
        rows.append((ok, err_mm, dt))
        print(f"{i:>5} {('ok' if picked else 'FAIL'):>6} {('ok' if placed else 'FAIL'):>6} "
              f"{('ok' if seated else 'no'):>6} {err_mm:>7.2f} {dt:>6.1f}  {note}", flush=True)

    successes = [r for r in rows if r[0]]
    errors = [r[1] for r in successes]
    print(f"\nsuccess: {len(successes)}/{len(rows)}")
    if errors:
        print(f"placement error (mm): mean {statistics.mean(errors):.2f}, max {max(errors):.2f}")
        print(f"cycle time (s): mean {statistics.mean(r[2] for r in successes):.1f}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if len(successes) == len(rows) else 1)


if __name__ == "__main__":
    main()
