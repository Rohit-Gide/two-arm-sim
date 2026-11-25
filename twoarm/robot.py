"""FANUC M-20iA kinematics. Pure numpy: no simulator needed to use this file.

Source of every number: robot/m20ia.urdf and robot/opw_parameters_m20ia.yaml (ROS-Industrial fanuc_m20ia_support, BSD).

Frames (all right-handed, metres, radians)
  base_link   on the floor under joint 1, x forward, z up. All poses in this file are in THIS frame.
  flange      on the wrist face, x pointing out of the wrist (URDF link_6).
  tool0       same origin as flange, z pointing out of the wrist (the ROS-Industrial / FANUC tool frame convention).
  tcp         tool0 moved `tcp_z` along its z (the gripper's working point). Jaws close along the tcp's y axis.

Joint angles are the URDF's (ROS-Industrial). FANUC's teach pendant shows J3 coupled to J2:
  J3_pendant = joint_3 - joint_2   (see `to_fanuc_deg` / `from_fanuc_deg`; confirm on the real controller before use).
"""
from __future__ import annotations

import math

import numpy as np

NAME = "FANUC M-20iA"
N = 6
#: (joint name, origin in the parent link, axis)  -- from the URDF
CHAIN = [("joint_1", (0.0, 0.0, 0.525), (0, 0, 1)), ("joint_2", (0.150, 0.0, 0.0), (0, 1, 0)),
         ("joint_3", (0.0, 0.0, 0.790), (0, -1, 0)), ("joint_4", (0.0, 0.0, 0.250), (-1, 0, 0)),
         ("joint_5", (0.835, 0.0, 0.0), (0, -1, 0)), ("joint_6", (0.100, 0.0, 0.0), (-1, 0, 0))]
LINKS = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"]
Q_MIN = np.array([-2.967060, -1.745329, -3.228859, -3.490658, -2.443461, -4.712389])
Q_MAX = np.array([2.967060, 2.792527, 4.766843, 3.490658, 2.443461, 4.712389])
V_MAX = np.array([3.403392, 3.054326, 3.141593, 6.283185, 6.283185, 9.599311])       # rad/s, FANUC's data sheet
#: FANUC does not publish joint accelerations. ASSUMPTION: full speed is reached in 0.25 s.
A_MAX = V_MAX / 0.25
PAYLOAD_KG = 20.0
REACH_M = 1.811
REPEATABILITY_M = 0.00008                                                              # +-0.08 mm, data sheet
HOME = np.radians([0.0, 0.0, 0.0, 0.0, -90.0, 0.0])                                    # forearm level, tool down
OPW = dict(a1=0.15, a2=-0.25, b=0.0, c1=0.525, c2=0.79, c3=0.835, c4=0.10)
OPW_OFFSETS = np.array([0.0, 0.0, -math.pi / 2, 0.0, 0.0, math.pi])
OPW_SIGNS = np.array([1, 1, -1, -1, -1, -1])


# ------------------------------------------------------------------ rotations
def rot(axis, angle):
    """Rotation matrix about a unit axis (Rodrigues)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def rx(a): return rot((1, 0, 0), a)
def ry(a): return rot((0, 1, 0), a)
def rz(a): return rot((0, 0, 1), a)


def T_of(R=None, p=(0, 0, 0)):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = p
    return T


def inv(T):
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


def wpr_to_R(w, p, r):
    """FANUC W, P, R (radians): rotations about the fixed X, then Y, then Z axes."""
    return rz(r) @ ry(p) @ rx(w)


def R_to_wpr(R):
    p = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if abs(math.cos(p)) < 1e-9:                                   # gimbal lock: put all of the turn in R
        return 0.0, p, math.atan2(-R[0, 1], R[1, 1])
    return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])


def R_to_quat(R):
    """(w, x, y, z)"""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


def quat_to_R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def slerp_R(R0, R1, s):
    """Rotation a fraction s of the way from R0 to R1, turning about one fixed axis."""
    D = R0.T @ R1
    ang = math.acos(max(-1.0, min(1.0, (np.trace(D) - 1) / 2)))
    if ang < 1e-9:
        return R0.copy()
    if abs(ang - math.pi) < 1e-6:                                 # half a turn: the axis from the symmetric part
        A = (D + np.eye(3)) / 2
        k = int(np.argmax(np.diag(A)))
        axis = A[:, k] / np.linalg.norm(A[:, k])
    else:
        axis = np.array([D[2, 1] - D[1, 2], D[0, 2] - D[2, 0], D[1, 0] - D[0, 1]]) / (2 * math.sin(ang))
    return R0 @ rot(axis, ang * s)


def tool_down(yaw=0.0):
    """Rotation of a tool pointing straight down, its jaw (y) axis turned `yaw` about the vertical."""
    return rz(yaw) @ np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])


R_FLANGE_TOOL0 = rz(0.0) @ ry(-math.pi / 2) @ rx(math.pi)         # URDF: rpy = (pi, -pi/2, 0)


# ------------------------------------------------------------------ forward kinematics
def link_frames(q):
    """4x4 pose of base_link, link_1 .. link_6 (7 frames) in base_link, for joint angles q."""
    T = np.eye(4)
    out = [T.copy()]
    for (_, origin, axis), a in zip(CHAIN, q):
        T = T @ T_of(rot(axis, a), origin)
        out.append(T.copy())
    return out


def fk(q, tcp_z=0.0):
    """4x4 pose of the tcp (tool0 when tcp_z = 0) in base_link."""
    T = link_frames(q)[-1] @ T_of(R_FLANGE_TOOL0)
    T[:3, 3] += T[:3, 2] * tcp_z
    return T


def jacobian(q, tcp_z=0.0):
    """6x6 geometric Jacobian of the tcp in base_link: rows 0-2 linear velocity, rows 3-5 angular velocity."""
    frames = link_frames(q)
    p = fk(q, tcp_z)[:3, 3]
    J = np.zeros((6, N))
    for i, (_, _, axis) in enumerate(CHAIN):
        z = frames[i + 1][:3, :3] @ np.asarray(axis, float)
        J[:3, i] = np.cross(z, p - frames[i + 1][:3, 3])
        J[3:, i] = z
    return J


def manipulability(q, tcp_z=0.0):
    """Yoshikawa's measure; near 0 = near a singularity."""
    J = jacobian(q, tcp_z)
    return float(math.sqrt(max(0.0, np.linalg.det(J @ J.T))))


# ------------------------------------------------------------------ inverse kinematics (analytic, all solutions)
def _acos(x):
    return math.acos(x) if -1.0 <= x <= 1.0 else (math.acos(max(-1.0, min(1.0, x))) if abs(x) < 1.0 + 1e-9 else float("nan"))


def ik_all(T, tcp_z=0.0):
    """Every joint solution (up to 8) putting the tcp at pose T (4x4 in base_link), before joint limits.

    Closed form for an ortho-parallel arm with a spherical wrist (Brandstoetter, Angerer, Hofbaur 2014).
    Returns a list of 6-vectors, each wrapped to (-pi, pi]. Unreachable poses return []."""
    T = np.asarray(T, float)
    M = T[:3, :3]
    a1, a2, b, c1, c2, c3, c4 = (OPW[k] for k in ("a1", "a2", "b", "c1", "c2", "c3", "c4"))
    c = T[:3, 3] - (c4 + tcp_z) * M[:, 2]                         # the wrist centre
    r2 = c[0] ** 2 + c[1] ** 2 - b * b
    if r2 < 0:
        return []
    nx1 = math.sqrt(r2) - a1
    t1 = math.atan2(c[1], c[0])
    t2 = math.atan2(b, nx1 + a1)
    th1 = [t1 - t2, t1 + t2 - math.pi]
    z = c[2] - c1
    s1_2 = nx1 * nx1 + z * z
    s2_2 = (nx1 + 2 * a1) ** 2 + z * z
    k2 = a2 * a2 + c3 * c3
    k = math.sqrt(k2)
    s1, s2 = math.sqrt(s1_2), math.sqrt(s2_2)
    A13 = _acos((s1_2 + c2 * c2 - k2) / (2 * s1 * c2)) if s1 > 1e-12 else float("nan")
    A14 = math.atan2(nx1, z)
    A15 = _acos((s2_2 + c2 * c2 - k2) / (2 * s2 * c2)) if s2 > 1e-12 else float("nan")
    A16 = math.atan2(nx1 + 2 * a1, z)
    th2 = [-A13 + A14, A13 + A14, -A15 - A16, A15 - A16]
    A10 = math.atan2(a2, c3)
    A11 = _acos((s1_2 - c2 * c2 - k2) / (2 * c2 * k))
    A12 = _acos((s2_2 - c2 * c2 - k2) / (2 * c2 * k))
    th3 = [A11 - A10, -A11 - A10, A12 - A10, -A12 - A10]
    out = []
    for i in range(4):
        j1, j2, j3 = th1[i // 2], th2[i], th3[i]
        if math.isnan(j2) or math.isnan(j3):
            continue
        s1_, c1_ = math.sin(j1), math.cos(j1)
        s23, c23 = math.sin(j2 + j3), math.cos(j2 + j3)
        m = M[0, 2] * s23 * c1_ + M[1, 2] * s23 * s1_ + M[2, 2] * c23
        j4 = math.atan2(M[1, 2] * c1_ - M[0, 2] * s1_, M[0, 2] * c23 * c1_ + M[1, 2] * c23 * s1_ - M[2, 2] * s23)
        j5 = math.atan2(math.sqrt(max(0.0, 1 - m * m)), m)
        j6 = math.atan2(M[0, 1] * s23 * c1_ + M[1, 1] * s23 * s1_ + M[2, 1] * c23,
                        -M[0, 0] * s23 * c1_ - M[1, 0] * s23 * s1_ - M[2, 0] * c23)
        for th in ((j1, j2, j3, j4, j5, j6), (j1, j2, j3, j4 + math.pi, -j5, j6 - math.pi)):
            q = (np.array(th) + OPW_OFFSETS) * OPW_SIGNS
            out.append((q + math.pi) % (2 * math.pi) - math.pi)
    return out


def _unwrap(q, ref):
    """The copies of q (each joint +-2pi) inside the joint limits, nearest `ref` first joint by joint."""
    q = np.array(q, float)
    for i in range(N):
        best = None
        for k in (-1, 0, 1):
            v = q[i] + 2 * math.pi * k
            if Q_MIN[i] - 1e-9 <= v <= Q_MAX[i] + 1e-9 and (best is None or abs(v - ref[i]) < abs(best - ref[i])):
                best = v
        if best is None:
            return None
        q[i] = best
    return q


def ik(T, ref=None, tcp_z=0.0):
    """Joint solutions inside the joint limits for tcp pose T, the one nearest `ref` first (default: HOME).
    Joints 4 and 6 turn more than a full turn, so each is given at the turn nearest `ref`."""
    ref = HOME if ref is None else np.asarray(ref, float)
    sols = [s for s in (_unwrap(q, ref) for q in ik_all(T, tcp_z)) if s is not None]
    return sorted(sols, key=lambda s: float(np.max(np.abs(s - ref))))


def config_of(q):
    """FANUC-style configuration string of a joint pose: wrist Flip/No-flip, elbow Up/Down, arm front (T) / back (B)."""
    th = np.asarray(q) * OPW_SIGNS - OPW_OFFSETS
    frames = link_frames(q)
    wc = frames[5][:3, 3]                                          # the wrist centre (joint 5)
    front = (math.cos(q[0]) * wc[0] + math.sin(q[0]) * wc[1]) >= 0
    j5 = (th[4] + math.pi) % (2 * math.pi) - math.pi
    j3 = (th[2] + math.atan2(OPW["a2"], OPW["c3"]) + math.pi) % (2 * math.pi) - math.pi
    return ("N" if j5 >= 0 else "F") + ("U" if (j3 >= 0) == front else "D") + ("T" if front else "B")


def to_fanuc_deg(q):
    d = np.degrees(np.asarray(q, float))
    d[2] = d[2] - d[1]
    return d


def from_fanuc_deg(d):
    q = np.radians(np.asarray(d, float))
    q[2] = q[2] + q[1]
    return q
