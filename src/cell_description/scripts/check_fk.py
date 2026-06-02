#!/usr/bin/env python3
"""
This checks that the URDF and the MuJoCo model put the robot in the same place. MoveIt plans on
the URDF while MuJoCo moves the MJCF, so any difference between them turns straight into
placement error; the URDF takes Menagerie's link lengths from config/ur5e_mujoco_kinematics.yaml
so the two agree exactly, and this is how that is confirmed. It compares world->tcp in the URDF
with the MuJoCo site robotiq_pinch at the home pose and at 19 random joint configurations and
prints the worst position and orientation difference, then compares tool0 and the gripper
knuckle at home. Run it with xacro urdf/cell.urdf.xacro > /tmp/cell.urdf and then
python3 scripts/check_fk.py /tmp/cell.urdf mjcf/scene.xml.
"""
import sys, xml.etree.ElementTree as ET
import numpy as np, mujoco

urdf, mjcf = sys.argv[1], sys.argv[2]
J = {}
for j in ET.parse(urdf).getroot().findall("joint"):
    o = j.find("origin")
    xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
    rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()])
    ax = j.find("axis")
    axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.zeros(3)
    J[j.find("child").get("link")] = dict(name=j.get("name"), parent=j.find("parent").get("link"),
                                         xyz=xyz, rpy=rpy, axis=axis, type=j.get("type"))

def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])

def rot_axis(a, t):
    a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * K + (1 - np.cos(t)) * K @ K

def urdf_fk(link, q):
    chain = []
    while link in J:
        chain.append(J[link]); link = J[link]["parent"]
    T = np.eye(4)
    for j in reversed(chain):
        A = np.eye(4); A[:3, :3] = rot_rpy(*j["rpy"]); A[:3, 3] = j["xyz"]
        if j["type"] in ("revolute", "continuous"):
            R = np.eye(4); R[:3, :3] = rot_axis(j["axis"], q.get(j["name"], 0.0)); A = A @ R
        T = T @ A
    return T

m = mujoco.MjModel.from_xml_path(mjcf); d = mujoco.MjData(m)
names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
sid = m.site("robotiq_pinch").id
rng = np.random.default_rng(0); worst_p = worst_r = 0
for i in range(20):
    qv = rng.uniform(-2.5, 2.5, 6) if i else np.array([0, -1.5708, 1.5708, -1.5708, -1.5708, 0])
    q = dict(zip(names, qv))
    for n, v in q.items():
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = v
    mujoco.mj_kinematics(m, d)
    T = urdf_fk("tcp", q)
    dp = np.linalg.norm(T[:3, 3] - d.site_xpos[sid])
    Rm = d.site_xmat[sid].reshape(3, 3); dR = T[:3, :3].T @ Rm
    ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    worst_p, worst_r = max(worst_p, dp), max(worst_r, ang)
    if i == 0:
        print("home tcp  URDF:", np.round(T[:3, 3], 4), " MuJoCo:", np.round(d.site_xpos[sid], 4))
        print("home rotation diff (URDF^T * MJ):\n", np.round(dR, 3))
print(f"worst over 20 configs: position {1000*worst_p:.3f} mm, orientation {worst_r:.3f} deg")

# a frame-by-frame comparison at home, for the tool flange and the gripper knuckle
q = dict(zip(names, [0, -1.5708, 1.5708, -1.5708, -1.5708, 0]))
for n, v in q.items():
    d.qpos[m.jnt_qposadr[m.joint(n).id]] = v
mujoco.mj_kinematics(m, d)
a = m.site("attachment_site").id
T0 = urdf_fk("tool0", q)
print("tool0 pos URDF", np.round(T0[:3, 3], 4), "MJ attachment", np.round(d.site_xpos[a], 4))
print("tool0 rot diff (URDF^T*MJ):\n", np.round(T0[:3, :3].T @ d.site_xmat[a].reshape(3, 3), 3))
kb = m.jnt_bodyid[m.joint("robotiq_85_left_knuckle_joint").id]
Ra = d.site_xmat[a].reshape(3, 3)
print("MJ left knuckle in attachment frame:", np.round(Ra.T @ (d.xpos[kb] - d.site_xpos[a]), 4))
print("MJ left knuckle in URDF tool0 frame:", np.round(T0[:3, :3].T @ (d.xpos[kb] - T0[:3, 3]), 4))
