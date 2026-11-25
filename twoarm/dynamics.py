"""The dynamic parameters of the cell: what FANUC does not publish, estimated, in ONE file you can edit.

    robot/dynamics_estimated.json      read at start-up; replace any number with a measured one
    python -m twoarm.dynamics          re-estimate and rewrite it (prints the table)

How each number is estimated (so you know how far to trust it)
  link masses     the arm's published 250 kg, split between the links in proportion to the volume of their collision
                  shapes. Each link's centre of mass and inertia then follow from its shape at uniform density.
  torque limits   per joint: the largest gravity torque the joint sees anywhere in its range holding the rated 20 kg
                  + 2 kg gripper, plus the torque to accelerate the largest inertia it ever sees at the joint's
                  acceleration (full published speed reached in 0.25 s). A motor sized for the data sheet, no more.
  joint 6         sized from joint 5 by the ratio of the published wrist moments (22 / 44 N.m).
  armature        rotor + gearbox inertia seen at the joint: half the largest load inertia the joint is rated to carry
                  (motors are matched to their load; for the wrist the rated load inertia is published).
  frictionloss    dry friction in the gearbox: 2 % of the torque limit.
  damping         viscous friction: 2 % of the torque limit at full speed.
  gripper         500 N grip force per jaw (pneumatic class), 0.3 kg fingers, 1.4 kg body (2 kg in all).
  friction        rubber-padded jaws 0.9, metal on metal 0.4.
  servo           each joint: feedforward from the controller's own model + PD at 10 Hz bandwidth, critically damped,
                  plus an integrator. Collision detection trips when the feedback torque passes 50 % of the limit
                  (for the wrist that is just above its published 44 N.m rated moment); following error faults
                  at 3 degrees.
"""
from __future__ import annotations

import json
import math

import numpy as np

from . import robot as R
from . import scene as S

PATH = S.ROBOT_DIR / "dynamics_estimated.json"
ARM_MASS_KG = 250.0
#: the load inertia FANUC allows at the wrist joints (J4 1.04, J5 1.04, J6 0.28 kg.m^2 -- from memory of the M-20iA data
#: sheet, confirm). The motors are sized at least for these.
RATED_LOAD_INERTIA = np.array([0.0, 0.0, 0.0, 1.04, 1.04, 0.28])


def inertia_diag(m, d, dofs) -> np.ndarray:
    """The inertia each joint in `dofs` sees right now (the diagonal of the joint-space inertia matrix)."""
    import mujoco
    v, r, out = np.zeros(m.nv), np.zeros(m.nv), np.zeros(len(dofs))
    for k, i in enumerate(dofs):
        v[:] = 0
        v[i] = 1
        mujoco.mj_mulM(m, d, r, v)
        out[k] = r[i]
    return out


def _base():
    return {
        "note": "ESTIMATED, not from FANUC. See twoarm/dynamics.py for how. Replace with measured values when you have them.",
        "link_mass": {n: 1.0 for n in R.LINKS},
        "joints": {jn: {"torque_limit": 1e6, "armature": 0.0, "damping": 0.0, "frictionloss": 0.0} for jn, _, _ in R.CHAIN},
        "gripper": {"grip_force": 500.0, "body_mass": 1.4, "finger_mass": 0.3, "finger_kp": 40000.0, "finger_kv": 400.0, "finger_damping": 20.0},
        "friction": {"finger_pad": 0.9, "metal": 0.4, "arm": 0.5, "floor": 0.6},
        "servo": {"bandwidth_hz": 10.0, "damping_ratio": 1.0, "integral_hz": 1.5,
                  "collision_detect_fraction": 0.50, "following_error_rad": round(math.radians(3.0), 5)},
    }


def estimate(samples: int = 3000, seed: int = 0) -> dict:
    import mujoco
    dyn = _base()
    # 1. masses from volume
    m = mujoco.MjModel.from_xml_string(S.mjcf(dyn, nominal=True))
    # volume of each link's collision shape (signed tetrahedra over its triangles)
    vol = {}
    for n in R.LINKS:
        mid = m.mesh(n).id
        v, f = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]], m.mesh_face[m.mesh_faceadr[mid]:m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        vol[n] = abs(float(np.sum(np.einsum("ij,ij->i", v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]]))) / 6.0))
    total = sum(vol.values())
    dyn["link_mass"] = {n: round(ARM_MASS_KG * vol[n] / total, 2) for n in R.LINKS}
    # 2. what each joint must hold and accelerate, over its whole range, with the rated payload
    m = mujoco.MjModel.from_xml_string(S.mjcf(dyn, nominal=True))
    d = mujoco.MjData(m)
    adr = np.array([m.jnt_dofadr[m.joint(f"a:{jn}").id] for jn, _, _ in R.CHAIN])
    pay = m.body("a:payload").id
    rng = np.random.default_rng(seed)
    g_max, i_max, i_arm = np.zeros(6), np.zeros(6), np.zeros(6)
    for load, store in ((1e-6, i_arm), (R.PAYLOAD_KG - S.GRIPPER["mass"], i_max)):
        m.body_mass[pay] = load
        for _ in range(samples):
            d.qpos[:] = 0
            d.qpos[adr] = rng.uniform(R.Q_MIN, R.Q_MAX)
            d.qvel[:] = 0
            mujoco.mj_forward(m, d)
            store[:] = np.maximum(store, inertia_diag(m, d, adr))
            if load > 1:
                g_max = np.maximum(g_max, np.abs(d.qfrc_bias[adr]))
    i_max = np.maximum(i_max, RATED_LOAD_INERTIA)
    for i, (jn, _, _) in enumerate(R.CHAIN):
        arm = 0.5 * i_max[i]
        limit = g_max[i] + (i_max[i] + arm) * R.A_MAX[i]
        dyn["joints"][jn] = {"torque_limit": round(float(limit), 1), "armature": round(float(arm), 3),
                             "damping": round(float(0.02 * limit / R.V_MAX[i]), 3), "frictionloss": round(float(0.02 * limit), 2),
                             "_max_gravity_torque": round(float(g_max[i]), 1), "_max_inertia": round(float(i_max[i]), 2)}
    # joint 6 turns about the tool axis, where a point payload asks nothing of it: size it from joint 5 by the ratio of
    # the published allowable wrist moments (J5 44 N.m, J6 22 N.m -- from memory of the data sheet, confirm)
    j5, j6 = dyn["joints"]["joint_5"], dyn["joints"]["joint_6"]
    lim = round(j5["torque_limit"] * 22.0 / 44.0, 1)
    lim = max(lim, j6["torque_limit"])
    j6.update(torque_limit=lim, damping=round(0.02 * lim / R.V_MAX[5], 3), frictionloss=round(0.02 * lim, 2))
    return dyn


def load() -> dict:
    if not PATH.exists():
        PATH.write_text(json.dumps(estimate(), indent=2))
    return json.loads(PATH.read_text())


if __name__ == "__main__":
    dyn = estimate()
    PATH.write_text(json.dumps(dyn, indent=2))
    print(f"wrote {PATH}\n")
    print("link masses (kg):", dyn["link_mass"])
    print(f'{"joint":9}{"limit N.m":>11}{"gravity":>10}{"inertia":>10}{"armature":>10}{"friction":>10}{"damping":>9}')
    for jn, j in dyn["joints"].items():
        print(f'{jn:9}{j["torque_limit"]:11.1f}{j["_max_gravity_torque"]:10.1f}{j["_max_inertia"]:10.2f}{j["armature"]:10.3f}{j["frictionloss"]:10.2f}{j["damping"]:9.3f}')
