"""The cell: where the two arms stand, what is around them, the gripper, the parts. Every size is a number here.

World frame: metres, z up, the floor at z = 0. Arm A stands at -x facing +x, arm B at +x facing -x; they share the
space between them and are not joined in any way: each has its own base, its own controller and its own gripper.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import robot as R

ROBOT_DIR = Path(__file__).resolve().parents[1] / "robot"

#: arm -> (x, y, yaw) of its base_link on the floor
ARMS = {"a": (-1.25, 0.0, 0.0), "b": (1.25, 0.0, math.pi)}

#: a generic two-finger parallel gripper, in the tool0 frame (z out of the wrist, jaws close along y)
GRIPPER = dict(
    tcp_z=0.20,                # flange face -> working point between the fingers
    stroke=0.12,               # widest opening between the fingers
    jaw_speed=0.15,            # m/s, opening changes at this rate
    mass=2.0,                  # kg, comes off the arm's 20 kg payload
    body=dict(size=(0.08, 0.22, 0.10), z=0.07),          # box size (x, y, z), centre z
    finger=dict(size=(0.04, 0.02, 0.10), z=0.17),        # each finger; its inner face is at +-opening/2
)

#: fixed things, each a box standing on the floor: name -> (centre x, centre y, size x, size y, height, colour)
FIXTURES = {
    "table_a": (-1.05, 1.05, 0.90, 0.60, 0.75, "#8a9099"),
    "table_b": (1.05, -1.05, 0.90, 0.60, 0.75, "#8a9099"),
    "stand_1_l": (-0.25, 0.55, 0.12, 0.25, 0.70, "#3f4650"),
    "stand_1_r": (0.25, 0.55, 0.12, 0.25, 0.70, "#3f4650"),
    "stand_2_l": (-0.25, -0.55, 0.12, 0.25, 0.70, "#3f4650"),
    "stand_2_r": (0.25, -0.55, 0.12, 0.25, 0.70, "#3f4650"),
}

#: parts: name -> size (x, y, z), mass kg, start centre (x, y, z), start yaw, colour
PARTS = {
    # aluminium bar 500 x 50 x 50 mm, 3.4 kg: one arm can carry it. Used for the hand-over.
    "bar": dict(size=(0.50, 0.05, 0.05), mass=3.4, pos=(-0.95, 1.00, 0.775), yaw=0.0, color="#c9ced6"),
    # aluminium billet 1200 x 80 x 80 mm, 20.7 kg: over one arm's payload, so it takes both. Used for the carry.
    "beam": dict(size=(1.20, 0.08, 0.08), mass=20.7, pos=(0.0, 0.55, 0.74), yaw=0.0, color="#d98a3a"),
}


def base_T(arm: str) -> np.ndarray:
    x, y, yaw = ARMS[arm]
    return R.T_of(R.rz(yaw), (x, y, 0.0))


def describe() -> dict:
    """Everything a viewer or a client needs to draw and reason about the cell (static)."""
    return {
        "robot": {"name": R.NAME, "links": R.LINKS,
                  "chain": [{"joint": n, "origin": list(o), "axis": list(a)} for n, o, a in R.CHAIN],
                  "q_min": R.Q_MIN.tolist(), "q_max": R.Q_MAX.tolist(), "v_max": R.V_MAX.tolist(),
                  "a_max": R.A_MAX.tolist(), "home": R.HOME.tolist(), "payload_kg": R.PAYLOAD_KG, "reach_m": R.REACH_M,
                  "repeatability_m": R.REPEATABILITY_M, "opw": R.OPW, "flange_to_tool0": R.R_FLANGE_TOOL0.tolist(),
                  "mesh_url": "/robot/meshes/visual/{link}.stl",
                  "colors": {"base_link": "#474747", "link_6": "#262626", "default": "#f5c221"}},
        "arms": {k: {"base": {"pos": [x, y, 0.0], "yaw": yaw}} for k, (x, y, yaw) in ARMS.items()},
        "gripper": GRIPPER,
        "fixtures": {k: {"pos": [v[0], v[1], v[4] / 2], "size": [v[2], v[3], v[4]], "color": v[5]} for k, v in FIXTURES.items()},
        "parts": {k: {"size": list(v["size"]), "mass": v["mass"], "color": v["color"]} for k, v in PARTS.items()},
        "frames": {"world": "metres, z up, floor z=0", "base": "each arm's base_link on the floor, x forward",
                   "tcp": "tool0 moved tcp_z along its z; jaws close along tcp y"},
    }


def _quat(Rm) -> str:
    return " ".join(f"{v:.9f}" for v in R.R_to_quat(np.asarray(Rm)))


def _box_mesh(name, size) -> str:
    """A box as a convex mesh asset. MuJoCo's box-against-box routine returns wrong distances for tilted boxes
    with a margin (two grippers 250 mm apart came back as 396 mm deep); its convex-mesh routine does not.
    Used for the gripper body, which takes part in the arm-to-arm distance."""
    hx, hy, hz = (v / 2 for v in size)
    pts = " ".join(f"{sx * hx} {sy * hy} {sz * hz}" for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1))
    return f'<mesh name="{name}" vertex="{pts}"/>'


TIMESTEP = 0.001                 # s, the physics step (the motors' torque loop runs at this rate too)
ARM_BODIES = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6", "gripper", "finger_l", "finger_r"]


def mjcf(dyn: dict, nominal: bool = False, geometry: bool = False, world: dict | None = None) -> str:
    """The cell as a MuJoCo model. THIS is what moves: rigid bodies with mass, joints driven by torque motors,
    fingers driven by force, parts held by friction, gravity, contacts.

    dyn: the dynamic parameters (dynamics.py; robot/dynamics_estimated.json).
    nominal=True: the model the CONTROLLER has of its two arms -- no parts, no fixtures, no contacts, no joint
    friction, and a point mass at each tcp for the declared payload. Used for the motors' feedforward torque.
    geometry=True: the same shapes with a 150 mm sensing margin, never stepped: the planner measures overlaps and
    the distance between the arms on it. (The margin must not be in the physics model, where it would push.)
    world: the TRUE layout where it differs from the nominal one (reality.py): each arm's real base pose, the
    parts' real starting poses, true link masses and gearbox friction, and `flex` -- gearbox springs: each joint then
    has a motor side (`joint_i`, driven, where the encoder is) and a spring to the link (`flex_i`)."""
    world = world or {}
    bases = world.get("bases") or {p: (x, y, 0.0, yaw) for p, (x, y, yaw) in ARMS.items()}
    starts = world.get("parts") or {k: (v["pos"], v["yaw"]) for k, v in PARTS.items()}
    flex = None if nominal else world.get("flex")
    lscale = world.get("link_scale") or {}
    fscale = world.get("friction_scale") or {}
    meshes = "\n".join(f'<mesh name="{n}" file="{ROBOT_DIR / "meshes" / "collision" / (n + ".stl")}"/>' for n in R.LINKS)
    g = GRIPPER
    bs, fs = g["body"]["size"], g["finger"]["size"]
    meshes += _box_mesh("box:gripper", bs)
    J, gd = dyn["joints"], dyn["gripper"]
    half = g["stroke"] / 2

    def arm(p):
        x, y, z, yaw = bases[p]
        ms = lscale.get(p, {})
        s = (f'<body name="{p}:base_link" pos="{x} {y} {z}" quat="{_quat(R.rz(yaw))}">'
             f'<geom name="{p}:base_link" class="arm" mesh="base_link" mass="{dyn["link_mass"]["base_link"] * ms.get("base_link", 1.0)}"/>')
        closing = 0
        for i, (jn, o, ax) in enumerate(R.CHAIN, start=1):
            j = J[jn]
            fl = 0.0 if nominal else j["frictionloss"] * (fscale[p][i - 1] if p in fscale else 1.0)
            motor = (f'<joint name="{p}:{jn}" axis="{ax[0]} {ax[1]} {ax[2]}" range="{R.Q_MIN[i - 1]} {R.Q_MAX[i - 1]}" '
                     f'armature="{j["armature"]}" damping="{j["damping"]}" frictionloss="{fl}"/>')
            link = f'<geom name="{p}:link_{i}" class="arm" mesh="link_{i}" mass="{dyn["link_mass"][f"link_{i}"] * ms.get(f"link_{i}", 1.0)}"/>'
            if flex:                                               # motor side, then the gearbox spring, then the link
                k, c = flex[jn]
                s += (f'<body name="{p}:rotor_{i}" pos="{o[0]} {o[1]} {o[2]}">{motor}'
                      f'<inertial pos="0 0 0" mass="0.001" diaginertia="1e-6 1e-6 1e-6"/>'
                      f'<body name="{p}:link_{i}"><joint name="{p}:flex_{i}" axis="{ax[0]} {ax[1]} {ax[2]}" stiffness="{k}" damping="{c}"/>{link}')
                closing += 2
            else:
                s += f'<body name="{p}:link_{i}" pos="{o[0]} {o[1]} {o[2]}">{motor}{link}'
                closing += 1
        s += (f'<body name="{p}:gripper" quat="{_quat(R.R_FLANGE_TOOL0)}">'
              f'<geom name="{p}:gripper" class="arm" mesh="box:gripper" pos="0 0 {g["body"]["z"]}" mass="{gd["body_mass"]}"/>'
              f'<site name="{p}:ft" pos="0 0 0"/><site name="{p}:tcp" pos="0 0 {g["tcp_z"]}"/>')
        for side, sign in (("l", 1), ("r", -1)):
            s += (f'<body name="{p}:finger_{side}" pos="0 {sign * fs[1] / 2} {g["finger"]["z"]}">'
                  f'<joint name="{p}:finger_{side}" type="slide" axis="0 {sign} 0" range="0 {half}" damping="{gd["finger_damping"]}"/>'
                  f'<geom name="{p}:finger_{side}" class="finger" size="{fs[0] / 2} {fs[1] / 2} {fs[2] / 2}" mass="{gd["finger_mass"]}"/></body>')
        if nominal:
            s += f'<body name="{p}:payload" pos="0 0 {g["tcp_z"]}"><inertial pos="0 0 0" mass="1e-6" diaginertia="1e-9 1e-9 1e-9"/></body>'
        return s + "</body>" + "</body>" * (closing + 1)

    def extras(p):
        ex = ""
        for i in range(len(ARM_BODIES)):
            for j in range(i + 1, len(ARM_BODIES)):
                a, b = ARM_BODIES[i], ARM_BODIES[j]
                li, lj = min(i, 7), min(j, 7)                       # the gripper and its fingers count as one link after link_6
                # links up to two joints apart cannot meet inside the joint limits (their convex collision shapes
                # overlap, e.g. the forearm's fork around the wrist); nor can the gripper and the wrist it is on
                # (neighbours too: MuJoCo's own parent-child filter skips a base that is fixed to the world)
                if lj - li <= 2 or (lj == 7 and li >= 4):
                    ex += f'<exclude body1="{p}:{a}" body2="{p}:{b}"/>'
        act = "".join(f'<motor name="{p}:{jn}" joint="{p}:{jn}" ctrlrange="{-J[jn]["torque_limit"]} {J[jn]["torque_limit"]}"/>'
                      for jn, _, _ in R.CHAIN)
        act += "".join(f'<position name="{p}:finger_{sd}" joint="{p}:finger_{sd}" kp="{gd["finger_kp"]}" kv="{gd["finger_kv"]}" '
                       f'ctrlrange="0 {half}" forcerange="{-gd["grip_force"]} {gd["grip_force"]}"/>' for sd in "lr")
        eq = f'<joint joint1="{p}:finger_l" joint2="{p}:finger_r"/>'  # the two jaws are geared together
        sens = f'<force name="{p}:force" site="{p}:ft"/><torque name="{p}:torque" site="{p}:ft"/>'
        return ex, act, eq, sens

    ex, act, eq, sens = (" ".join(v) for v in zip(*(extras(p) for p in ARMS)))
    fr = dyn["friction"]
    fixtures = "" if nominal else "\n".join(
        f'<geom name="fix:{k}" class="solid" pos="{v[0]} {v[1]} {v[4] / 2}" size="{v[2] / 2} {v[3] / 2} {v[4] / 2}"/>' for k, v in FIXTURES.items())
    parts = "" if nominal else "\n".join(
        f'<body name="part:{k}" pos="{starts[k][0][0]} {starts[k][0][1]} {starts[k][0][2]}" quat="{_quat(R.rz(starts[k][1]))}"><freejoint name="part:{k}"/>'
        f'<geom name="part:{k}" class="solid" size="{v["size"][0] / 2} {v["size"][1] / 2} {v["size"][2] / 2}" mass="{v["mass"]}"/></body>'
        for k, v in PARTS.items())
    flags = '<flag contact="disable" equality="disable" frictionloss="disable" limit="disable"/>' if nominal else ""
    return f"""<mujoco model="two_arm_cell{'_nominal' if nominal else ''}">
<compiler angle="radian" autolimits="true"/>
<option timestep="{TIMESTEP}" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="5">{flags}</option>
<default>
  <default class="arm"><geom type="mesh" margin="{0.15 if geometry else 0}" friction="{fr["arm"]}"/></default>
  <default class="finger"><geom type="box" friction="{fr["finger_pad"]} 0.02 0.0005" condim="4" solref="0.004 1"/></default>
  <default class="solid"><geom type="box" friction="{fr["metal"]}" solref="0.004 1"/></default>
</default>
<asset>{meshes}</asset>
<worldbody>
<geom name="floor" type="plane" size="20 20 0.1" friction="{fr["floor"]}"/>
{fixtures}
{arm("a")}
{arm("b")}
{parts}
</worldbody>
<contact>{ex}</contact>
<equality>{eq}</equality>
<actuator>{act}</actuator>
<sensor>{sens}</sensor>
</mujoco>"""
