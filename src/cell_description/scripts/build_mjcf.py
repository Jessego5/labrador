#!/usr/bin/env python3
"""
This builds mjcf/ur5e_2f85.xml, the UR5e with a Robotiq 2F-85 on its flange, out of the two
MuJoCo Menagerie models, so that the robot MuJoCo simulates is the same one the URDF describes
to mujoco_ros2_control and MoveIt. Menagerie's models are close but do not line up with
ur_description and robotiq_description as they come, so it changes four things. It removes the
180 degree yaw Menagerie puts on the UR5e base, to match ur_description's base_link. It gives
every robot body gravity compensation. It renames the arm actuators after their joints
(shoulder_pan_joint and so on), because mujoco_ros2_control finds actuators by joint name. And
it attaches the gripper with the prefix `robotiq_`, renames the 2F-85's `left_driver_joint` to
`85_left_knuckle_joint` so it becomes `robotiq_85_left_knuckle_joint`, the actuated joint in
robotiq_description, and replaces its 0-255 actuator with a position actuator of the same name
in radians, 0 open and 0.8 closed. It also adds the wrist force-torque sensor. The output is
committed, so this only needs re-running when the robot model changes, and it needs the
`mujoco` Python package at the same version as ros-jazzy-mujoco-vendor (mujoco==3.12.0). Run it
with python3 build_mjcf.py [--menagerie PATH].
"""
import argparse
import os
import re
import shutil
from pathlib import Path

import mujoco
import numpy as np

PKG = Path(__file__).resolve().parents[1]
OUT_DIR = PKG / "mjcf"
# The gripper servo. Menagerie limits the tendon force to 5 N, which leaves a few newtons on the
# pads: enough to lift parts but not to stop a long peg pivoting between the pads when its tip
# drags on a surface, so it was too weak for insertion. A real 2F-85 grips with 20-235 N, and
# this gives roughly the low end of that. Going further, to kp 200 and force 20, knocked
# cylinders away as the fingers closed and broke the part 3 and part 4 picks (2/10 and 27/50), which
# the part 5 insertion benchmark alone did not show; rerun those benchmarks after changing these.
GRIPPER_KP = 100
GRIPPER_KV = 40
GRIPPER_FORCE = 15
GRIPPER_PREFIX = "robotiq_"  # namespaces every gripper element, so no names clash with the arm's
GRIPPER_JOINT_LOCAL = "85_left_knuckle_joint"
GRIPPER_JOINT = GRIPPER_PREFIX + GRIPPER_JOINT_LOCAL  # actuated joint name in robotiq_description


def load_with_abs_meshes(xml_path: Path, subdir: str, edit=None):
    text = xml_path.read_text()
    if edit:
        text = edit(text)
    asset_dir = xml_path.parent / "assets"
    text = re.sub(r'meshdir="[^"]*"', f'meshdir="{asset_dir}"', text)
    spec = mujoco.MjSpec.from_string(text)
    for k in list(spec.keys):
        spec.delete(k)
    for m in spec.meshes:
        m.file = str(asset_dir / m.file)
    return spec, asset_dir, subdir


def edit_ur5e(text):
    # name each actuator after its joint
    return re.sub(r'(<general class="\w+") name="\w+" joint="(\w+)"', r'\1 name="\2" joint="\2"', text)


def edit_2f85(text):
    # attached with prefix GRIPPER_PREFIX, so this becomes GRIPPER_JOINT in the final model
    text = text.replace('"left_driver_joint"', f'"{GRIPPER_JOINT_LOCAL}"')
    return re.sub(
        r"<actuator>.*?</actuator>",
        f'<actuator><position name="{GRIPPER_JOINT_LOCAL}" tendon="split" kp="{GRIPPER_KP}" kv="{GRIPPER_KV}" '
        f'ctrlrange="0 0.8" forcerange="-{GRIPPER_FORCE} {GRIPPER_FORCE}"/></actuator>',
        text, flags=re.S)


FT_SITE = "ft_frame"
FT_SENSOR = "wrist_ft"  # mujoco_ros2_control reads <name>_force + <name>_torque as <name>_fts


def add_wrist_ft_sensor(arm):
    """
    Adds a force-torque sensor between wrist_3 and the gripper, in the tool flange frame.
    MuJoCo force and torque sensors report the interaction between the site's body and its
    parent, so the site goes on the gripper's root body, placed exactly at attachment_site,
    which is the URDF's tool0.
    """
    m = arm.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mount = m.body(GRIPPER_PREFIX + "base_mount").id
    flange = m.site("attachment_site").id
    R_m, p_m = d.xmat[mount].reshape(3, 3), d.xpos[mount]
    R_f, p_f = d.site_xmat[flange].reshape(3, 3), d.site_xpos[flange]
    rel_p = R_m.T @ (p_f - p_m)
    rel_q = np.zeros(4)
    mujoco.mju_mat2Quat(rel_q, (R_m.T @ R_f).flatten())
    body = arm.body(GRIPPER_PREFIX + "base_mount")
    body.add_site(name=FT_SITE, pos=rel_p, quat=rel_q, size=[0.006, 0.006, 0.006],
                  rgba=[1, 0.5, 0, 1], group=4)
    for kind, suffix in ((mujoco.mjtSensor.mjSENS_FORCE, "_force"),
                         (mujoco.mjtSensor.mjSENS_TORQUE, "_torque")):
        arm.add_sensor(name=FT_SENSOR + suffix, type=kind, objtype=mujoco.mjtObj.mjOBJ_SITE,
                       objname=FT_SITE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--menagerie", default=os.environ.get("MENAGERIE", os.path.expanduser("~/models/mujoco_menagerie")))
    args = ap.parse_args()
    men = Path(args.menagerie)

    arm, arm_assets, _ = load_with_abs_meshes(men / "universal_robots_ur5e/ur5e.xml", "ur5e", edit_ur5e)
    grip, grip_assets, _ = load_with_abs_meshes(men / "robotiq_2f85/2f85.xml", "robotiq_2f85", edit_2f85)

    arm.modelname = "ur5e_2f85"
    # Menagerie yaws the UR5e base by 180 deg relative to ur_description's base_link. Drop
    # that so MuJoCo and the URDF agree, with the arm facing +x at shoulder_pan = 0.
    arm.body("base").quat = [1, 0, 0, 0]
    arm.option.impratio = 10
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    site = arm.site("attachment_site")
    site.attach_body(grip.worldbody.first_body(), GRIPPER_PREFIX, "")
    add_wrist_ft_sensor(arm)

    # Gravity compensation for every robot body, as the UR controller does internally. Without
    # it the position servos sag about 0.016 rad at the shoulder, more than 1 cm at the TCP.
    # Payloads (grasped objects) are not compensated.
    for b in arm.bodies:
        if b.name != "world":
            b.gravcomp = 1.0

    # copy the mesh files and their licenses into the package
    assets_out = OUT_DIR / "assets"
    if assets_out.exists():
        shutil.rmtree(assets_out)
    for src, sub in ((arm_assets, "ur5e"), (grip_assets, "robotiq_2f85")):
        shutil.copytree(src, assets_out / sub)
        shutil.copy(src.parent / "LICENSE", assets_out / sub / "LICENSE")
    # mesh paths are absolute here (to_xml() compiles), so rewrite them to package-relative ones
    out = OUT_DIR / "ur5e_2f85.xml"
    xml = arm.to_xml()
    xml = xml.replace(f"{arm_assets}/", "ur5e/").replace(f"{grip_assets}/", "robotiq_2f85/")
    xml = re.sub(r'\s+meshdir="[^"]*"', "", xml)
    xml = re.sub(r"<compiler", '<compiler meshdir="assets"', xml, count=1)
    if "<compiler" not in xml:
        xml = re.sub(r"(<mujoco[^>]*>)", r'\1\n  <compiler meshdir="assets"/>', xml, count=1)
    header = ("<!-- GENERATED by cell_description/scripts/build_mjcf.py from MuJoCo Menagerie. "
              "Do not edit by hand. -->\n")
    out.write_text(header + xml)

    # load the generated file back, which fails if it is not valid
    m = mujoco.MjModel.from_xml_path(str(out))
    print(f"wrote {out}: nq={m.nq} nu={m.nu}")
    print("actuators:", [m.actuator(i).name for i in range(m.nu)])

    # the geometry the URDF has to match: pinch point and finger direction in the flange frame
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    s = m.site("attachment_site").id
    R, p = d.site_xmat[s].reshape(3, 3), d.site_xpos[s]
    pinch = R.T @ (d.site_xpos[m.site(GRIPPER_PREFIX + "pinch").id] - p)
    jbody = m.jnt_bodyid[m.joint(GRIPPER_JOINT).id]
    left = R.T @ (d.xpos[jbody] - p)
    print(f"pinch point in tool frame: {np.round(pinch, 4)}")
    ft = m.site(FT_SITE).id
    ft_err = (np.abs(d.site_xpos[ft] - p).max(), np.abs(d.site_xmat[ft].reshape(3, 3) - R).max())
    assert max(ft_err) < 1e-9, f"ft_frame does not coincide with the flange: {ft_err}"
    print(f"ft sensor: site {FT_SITE} at the flange; sensors {FT_SENSOR}_force/_torque")
    print(f"left driver in tool frame: {np.round(left, 4)}  -> gripper yaw = "
          f"{np.degrees(np.arctan2(left[1], left[0])):.1f} deg")


if __name__ == "__main__":
    main()
