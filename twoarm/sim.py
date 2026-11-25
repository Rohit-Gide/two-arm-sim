"""The simulation: two independent arm controllers in one cell, and the layer that lets them work together.

Everything that moves here is moved by PHYSICS (MuJoCo, 1 kHz): the links have mass, each joint has a torque motor with
a limit, rotor inertia and gearbox friction, the fingers squeeze with a force, parts are free rigid bodies held by
friction. Nothing is attached, snapped or decided by a rule.

  planner     turns a command into a joint path inside the speed and acceleration limits and checks it for collisions
              against where the other arm will be. It only PROPOSES the motion (a real cell has this too).
  servo       each joint, every millisecond: torque = what the controller's own model says the motion needs
              (feedforward, for the payload it has been TOLD about) + PD + integrator on the error, cut off at the
              motor's limit. So the arm sags under a load it was not told about, lags when a motor saturates, and
              overshoots if it is asked for more than it has.
  gripper     jaws geared together, closed by a force (500 N unless told otherwise). A part stays in them by friction
              and can slip, turn or drop.
  two arms    holding one part they form a closed chain through the four contact patches. If they disagree, the
              forces between them rise, the part slips in the jaws or a motor reaches its limit.
  protection  like a real controller: a joint whose feedback torque passes 50 % of its motor's limit trips collision
              detection; a joint more than 3 degrees off its command trips a following-error fault. Both stop the
              arm where it is. These thresholds are the only "rules", and they act on measured torque and position.

The dynamic parameters are estimates (FANUC publishes none): robot/dynamics_estimated.json, explained in dynamics.py.
"""
from __future__ import annotations

import collections
import math
import threading
import time

import numpy as np

from . import dynamics as D
from . import robot as R
from .reality import Reality
from . import scene as S

DT = 0.004                       # s, the controller's command cycle (250 Hz); the torque loop runs at 1 kHz inside it
SUB = round(DT / S.TIMESTEP)     # physics steps per command cycle
SMOOTH_S = 0.040                 # s, acceleration is rounded over this window (finite jerk)
IN_POSITION = 3e-4               # rad: a move is done when every joint is this close to its goal ...
IN_POSITION_VEL = 1e-3           # rad/s ... and this still
SCAN_EVERY = 5                   # contacts are read every 5 cycles (20 ms)
CHECK_EVERY = 5                  # the planner checks a path every 5 cycles of it
PENETRATION = 0.002              # m, the planner refuses a path that overlaps deeper than this
LINE_SPEED = 0.5                 # m/s default for straight-line moves
LINE_SPEED_MAX = 2.0             # m/s, the M-20iA's programmed maximum
LINE_ACC = 5.0                   # m/s^2 ASSUMPTION
TURN_SPEED = math.radians(180)   # rad/s cap for the tool's turning on a straight-line move
TURN_ACC = math.radians(720)
GRIP_ON_N = 5.0                  # a part counts as held when both jaws press on it with at least this
TOUCH_N = 1.0                    # a contact is reported from this force
G = 9.81


class SimError(Exception):
    def __init__(self, status: int, code: str, message: str, **detail):
        super().__init__(message)
        self.status, self.code, self.message, self.detail = status, code, message, detail

    def as_dict(self):
        return {"ok": False, "error": self.code, "message": self.message, **self.detail}


class _Lock:
    """A re-entrant lock that is FAIR: it is handed to whoever has waited longest. With a plain lock, a thread that
    lets go and takes it again at once (the simulation loop, or a client polling the state) starves everyone else."""

    def __init__(self):
        self._m = threading.Lock()
        self._queue = collections.deque()
        self._owner = None
        self._depth = 0

    def __enter__(self):
        me = threading.get_ident()
        if self._owner == me:
            self._depth += 1
            return self
        with self._m:
            if self._owner is None:
                self._owner, self._depth = me, 1
                return self
            turn = threading.Event()
            self._queue.append((me, turn))
        turn.wait()                                                # the one letting go made us the owner
        return self

    def __exit__(self, *exc):
        self._depth -= 1
        if self._depth:
            return
        with self._m:
            if self._queue:
                self._owner, turn = self._queue.popleft()
                self._depth = 1
                turn.set()
            else:
                self._owner = None


# ------------------------------------------------------------------ what counts as a collision, for the planner
def _name(o):
    return {"arm": f"{o[1]}:{R.LINKS[min(o[2], 6)]}", "tool": f"{o[1]}:gripper"}.get(o[0], o[1])

def _judge(o1, o2, dist, held):
    """(is this contact a collision, is it between the two arms)."""
    k1, k2 = o1[0], o2[0]
    body1, body2 = k1 in ("arm", "tool"), k2 in ("arm", "tool")
    if body1 and body2:
        if o1[1] != o2[1]:
            return dist < -PENETRATION, True
        return dist < -PENETRATION, False                     # (pairs that cannot meet are excluded in the model)
    if body1 or body2:
        a, o = (o1, o2) if body1 else (o2, o1)
        if o[0] == "part":
            if a[0] == "tool":                                # fingers are meant to touch parts
                return False, False
            if a[1] in held.get(o[1], ()) and a[2] >= 5:      # the wrist carrying it
                return False, False
        return dist < -PENETRATION, False
    parts_in = [o for o in (o1, o2) if o[0] == "part"]        # part against table / floor / part
    if any(held.get(o[1]) for o in parts_in):
        return dist < -0.005, False
    return False, False


# ------------------------------------------------------------------ one arm's controller
class Arm:
    def __init__(self, name: str, index: int):
        self.name, self.index = name, index
        self.sl = slice(6 * index, 6 * index + 6)   # this arm's joints in the 12-long arrays
        self.T_wb = S.base_T(name)                # base_link in the world
        self.T_bw = R.inv(self.T_wb)
        self.reset()

    def reset(self):
        self.q = R.HOME.copy()                    # measured joint angles (the encoders)
        self.qd = np.zeros(6)
        self.q_cmd = R.HOME.copy()                # what the controller is asking for this cycle
        self.v_cmd = np.zeros(6)
        self.buf = np.zeros((0, 6))               # commands not yet sent, one row per cycle
        self.moves = collections.deque()          # (move id, rows left when it ends)
        self.grip = 0.10                          # measured opening between the fingers, m
        self.grip_cmd = self.grip_ctrl = 0.10     # where it is told to go / where the jaw drive is now
        self.grip_force = None                    # N per jaw; None = the gripper's full force
        self.grip_vel = 0.0
        self.grip_n = 0.0                         # measured squeeze on the part, N (mean of the two jaws)
        self.holding = None
        self.payload = 0.0                        # kg the controller has been told it carries
        self.soft = None                          # soft float: fraction of the normal servo stiffness, or None (stiff)
        self.fault = None
        self.torque = np.zeros(6)                 # motor torque now
        self.wrist_f = np.zeros(3)                # wrist force / torque sensor, world frame, filtered
        self.wrist_t = np.zeros(3)
        self.load_kg = 0.0
        self.next_id = 1
        self.done_id = 0

    def tcp_world(self, q=None):
        return self.T_wb @ R.fk(self.q if q is None else q, S.GRIPPER["tcp_z"])

    def to_base(self, T_world):
        return self.T_bw @ T_world

    def q_end(self):
        """Where the arm will be when everything queued has run."""
        return self.buf[-1] if len(self.buf) else self.q_cmd

    def planned(self, k: int):
        """The command k cycles from now."""
        return self.buf[k] if k < len(self.buf) else self.q_end()

    @property
    def in_position(self):
        still = np.max(np.abs(self.qd)) <= IN_POSITION_VEL
        if self.soft:                             # a compliant arm rests where the forces balance, not on its command
            return bool(still)
        return bool(still and np.max(np.abs(self.q - self.q_cmd)) <= IN_POSITION)

    @property
    def gripper_busy(self):
        """Jaws still travelling (jaws stopped on a part are not)."""
        return abs(self.grip_ctrl - self.grip_cmd) > 1e-9 or abs(self.grip_vel) > 0.002

    @property
    def status(self):
        if self.fault:
            return "fault"
        return "idle" if not len(self.buf) and self.in_position else "moving"


# ------------------------------------------------------------------ time parametrisation
def _trapezoid(v: float, a: float) -> np.ndarray:
    """s(t) from 0 to 1 at cycle times, top speed v (1/s), acceleration a (1/s^2)."""
    if v * v / a > 1.0:
        v = math.sqrt(a)
    ta = v / a
    T = 1.0 / v + ta
    t = np.arange(0, math.ceil(T / DT) + 1) * DT
    s = np.where(t < ta, 0.5 * a * t * t, np.where(t < T - ta, v * (t - ta / 2), 1.0 - 0.5 * a * np.clip(T - t, 0, None) ** 2))
    s[-1] = 1.0
    return np.clip(s, 0.0, 1.0)


def time_param(paths: list, speed: float = 1.0, caps: list = ()) -> list:
    """Turn geometric joint paths (each [M, 6], evenly spaced in a shared path fraction s) into per-cycle commands
    that start and end together and keep every joint of every arm inside its speed and acceleration limits.
    caps: extra (v_s, a_s) limits on ds/dt, e.g. from a Cartesian speed."""
    v_s, a_s = math.inf, math.inf
    for Q in paths:
        dq = np.max(np.abs(np.gradient(Q, np.linspace(0, 1, len(Q)), axis=0)), axis=0)
        with np.errstate(divide="ignore"):
            v_s = min(v_s, float(np.min(R.V_MAX * speed / dq)))
            a_s = min(a_s, float(np.min(R.A_MAX * speed / dq)))
    for v, a in caps:
        v_s, a_s = min(v_s, v), min(a_s, a)
    if not math.isfinite(v_s):                                     # nothing moves
        return [Q[-1:].copy() for Q in paths]
    out = None
    for _ in range(6):
        s = _trapezoid(v_s, a_s)
        out = [np.stack([np.interp(s, np.linspace(0, 1, len(Q)), Q[:, j]) for j in range(6)], axis=1) for Q in paths]
        worst = 1.0                                                # path curvature adds acceleration: slow down for it
        for Q in out:
            if len(Q) > 2:
                acc = np.abs(np.diff(Q, 2, axis=0)) / DT ** 2
                worst = max(worst, float(np.max(acc / (R.A_MAX * speed))))
        if worst <= 1.05:
            break
        v_s, a_s = v_s / math.sqrt(worst), a_s / worst
    n = max(1, round(SMOOTH_S / DT))                               # round the corners of the acceleration
    res = []
    for Q in out:
        P = np.concatenate([np.repeat(Q[:1], n, 0), Q, np.repeat(Q[-1:], n, 0)])
        k = np.ones(n) / n
        Qs = np.stack([np.convolve(P[:, j], k, mode="valid") for j in range(6)], axis=1)
        Qs[-1] = Q[-1]
        res.append(Qs[1:])
    return res


# ------------------------------------------------------------------ the cell
class Sim:
    def __init__(self, realtime: bool = True, reality: Reality | None = None):
        import mujoco
        self.mj = mujoco
        self.realtime = realtime
        self.lock = _Lock()
        self.plan_lock = threading.Lock()
        self._thread = None
        self._run = False
        self.dyn = D.load()                                                          # what the controllers believe
        self.arms = {p: Arm(p, i) for i, p in enumerate(S.ARMS)}
        self._build(reality or Reality.ideal())
        self.reset()

    def _build(self, reality: Reality):
        """Build the three models: the world as it IS (with this reality's errors), and the two the controllers and
        the planner have of it (nominal: they know nothing of the errors)."""
        mujoco = self.mj
        self.reality = reality
        true_dyn, world = reality.world(self.dyn)
        self.m = mujoco.MjModel.from_xml_string(S.mjcf(true_dyn, world=world))        # the world as it is
        self.d = mujoco.MjData(self.m)
        self.mc = mujoco.MjModel.from_xml_string(S.mjcf(self.dyn, nominal=True))     # the controllers' model of their arms
        self.dc = mujoco.MjData(self.mc)
        self.mg = mujoco.MjModel.from_xml_string(S.mjcf(self.dyn, geometry=True, world=dict(flex=world["flex"])))   # the planner's shapes, nominal layout
        self.dp = mujoco.MjData(self.mg)
        self.zero = np.concatenate([reality.zero[p] for p in S.ARMS])                # true joint angle = encoder + zero
        m = self.m
        jn = [f"{p}:{n}" for p in S.ARMS for n, _, _ in R.CHAIN]
        self.Q = np.array([m.jnt_qposadr[m.joint(n).id] for n in jn])                # the 12 arm joints, a then b
        self.V = np.array([m.jnt_dofadr[m.joint(n).id] for n in jn])
        self.A = np.array([m.actuator(n).id for n in jn])
        self.Vc = np.array([self.mc.jnt_dofadr[self.mc.joint(n).id] for n in jn])
        self.Qc = np.array([self.mc.jnt_qposadr[self.mc.joint(n).id] for n in jn])
        self.FQ = {p: [m.jnt_qposadr[m.joint(f"{p}:finger_{s}").id] for s in "lr"] for p in S.ARMS}
        self.FV = {p: [m.jnt_dofadr[m.joint(f"{p}:finger_{s}").id] for s in "lr"] for p in S.ARMS}
        self.FA = {p: [m.actuator(f"{p}:finger_{s}").id for s in "lr"] for p in S.ARMS}
        self.FQc = {p: [self.mc.jnt_qposadr[self.mc.joint(f"{p}:finger_{s}").id] for s in "lr"] for p in S.ARMS}
        self.pay = {p: self.mc.body(f"{p}:payload").id for p in S.ARMS}
        self.ft_site = {p: m.site(f"{p}:ft").id for p in S.ARMS}
        self.ft_adr = {p: (m.sensor_adr[m.sensor(f"{p}:force").id], m.sensor_adr[m.sensor(f"{p}:torque").id]) for p in S.ARMS}
        self.part_body = {k: m.body(f"part:{k}").id for k in S.PARTS}
        self.part_q = {k: m.jnt_qposadr[m.joint(f"part:{k}").id] for k in S.PARTS}
        self.part_v = {k: m.jnt_dofadr[m.joint(f"part:{k}").id] for k in S.PARTS}
        self.limit = m.actuator_ctrlrange[self.A, 1].copy()
        sv = self.dyn["servo"]
        wn = 2 * math.pi * sv["bandwidth_hz"]
        self.kp, self.kd, self.ki = wn * wn, 2 * sv["damping_ratio"] * wn, wn * wn * 2 * math.pi * sv["integral_hz"]
        self.trip = sv["collision_detect_fraction"] * self.limit
        self.follow = sv["following_error_rad"]
        self.owner = []
        for g in range(m.ngeom):
            n = m.geom(g).name
            if n == "floor":
                self.owner.append(("floor", "floor", 0))
            elif n.startswith("fix:"):
                self.owner.append(("fix", n[4:], 0))
            elif n.startswith("part:"):
                self.owner.append(("part", n[5:], 0))
            else:
                p, link = n.split(":")
                self.owner.append(("arm", p, R.LINKS.index(link)) if link in R.LINKS else
                                  ("tool", p, 8 if link == "finger_l" else 9 if link == "finger_r" else 7))

    # ---------------------------------------------------------- lifecycle
    def reset(self, reality: Reality | None = None):
        """Everything back to the start. reality: rebuild the world with another draw of the errors."""
        with self.lock:
            if reality is not None:
                self._build(reality)
            mj, m, d = self.mj, self.m, self.d
            mj.mj_resetData(m, d)
            self.t = 0.0
            self.tick = 0
            for p, a in self.arms.items():
                a.reset()
                d.qpos[self.Q[a.sl]] = a.q + self.zero[a.sl]       # the encoders read HOME
                d.qpos[self.FQ[p]] = a.grip / 2
                d.ctrl[self.FA[p]] = a.grip / 2
                m.actuator_forcerange[self.FA[p]] = [-self.dyn["gripper"]["grip_force"], self.dyn["gripper"]["grip_force"]]
                self.mc.body_mass[self.pay[p]] = 1e-6
            mj.mj_forward(m, d)
            self.q_des = np.tile(R.HOME, 2)
            self.v_des = np.zeros(12)
            self.a_des = np.zeros(12)
            self.integ = np.zeros(12)
            self.gp, self.gd, self.gi = np.ones(12), np.ones(12), np.ones(12)   # servo gain scaling (soft float lowers it)
            self.fb = np.zeros(12)                 # feedback torque, filtered: the controller's disturbance estimate
            self.Md = None
            # T: where the part IS. Tb: where the cell BELIEVES it is -- where the layout says, then wherever the
            # grippers that held it took it (by their own kinematics). No camera corrects it.
            self.parts = {k: dict(name=k, size=np.array(v["size"], float), mass=v["mass"], T=np.eye(4), speed=0.0,
                                  Tb=R.T_of(R.rz(v["yaw"]), v["pos"]), held=[], rel={}, resting_on=None)
                          for k, v in S.PARTS.items()}
            self.events = collections.deque(maxlen=200)
            self.event_n = 0
            self.arm_distance = None
            self.touching = []
            self._read()
            for _ in range(25):                                    # 0.1 s: the motors take the weight, the readings are real
                self.step()
            self.t, self.tick = 0.0, 0
            self._scan()
            self.events.clear()
            self._log(None, "reset", {})
        return {"ok": True}

    def start(self):
        if self._thread:
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop_thread(self):
        self._run = False
        if self._thread:
            self._thread.join()
            self._thread = None

    def _loop(self):
        nxt = time.perf_counter()
        while self._run:
            if not self.realtime and self._at_rest():
                time.sleep(0.0002)                                 # fast mode: no time passes while nothing moves
                continue
            with self.lock:
                self.step()
            if self.realtime:
                nxt += DT
                delay = nxt - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.2:
                    nxt = time.perf_counter()

    def _at_rest(self):
        return (all(a.status != "moving" and not a.gripper_busy for a in self.arms.values())
                and all(p["speed"] < 0.002 for p in self.parts.values()))

    def _log(self, arm, event, detail):
        self.event_n += 1
        self.events.append({"n": self.event_n, "t": round(self.t, 3), "arm": arm, "event": event, **detail})

    # ---------------------------------------------------------- one controller cycle (4 ms = 4 physics steps)
    def step(self):
        mj, m, d, mc, dc = self.mj, self.m, self.d, self.mc, self.dc
        q0 = self.q_des.copy()
        q1 = q0.copy()
        for a in self.arms.values():                               # the next command row of each arm
            if len(a.buf) and not a.fault:
                q1[a.sl] = a.buf[0]
                a.buf = a.buf[1:]
            v = (q1[a.sl] - q0[a.sl]) / DT
            self.a_des[a.sl] = (v - a.v_cmd) / DT
            self.v_des[a.sl] = a.v_cmd = v
            a.q_cmd = q1[a.sl].copy()
            self._jaws(a)
        for j in range(1, SUB + 1):                                # the torque loop, 1 kHz
            self.q_des = q0 + (q1 - q0) * (j / SUB)
            for p in self.arms:
                dc.qpos[self.FQc[p]] = d.qpos[self.FQ[p]]
            dc.qpos[self.Qc] = self.q_des
            dc.qvel[:] = 0
            dc.qvel[self.Vc] = self.v_des
            dc.qacc[:] = 0
            dc.qacc[self.Vc] = self.a_des
            mj.mj_inverse(mc, dc)                                  # the torque this motion needs, by the controller's model
            if self.Md is None or j == 1:
                self.Md = D.inertia_diag(mc, dc, self.Vc)
            e = self.q_des - (d.qpos[self.Q] - self.zero)          # against the encoders
            ed = self.v_des - d.qvel[self.V]
            self.integ = np.clip(self.integ + self.Md * self.ki * self.gi * e * S.TIMESTEP, -0.6 * self.limit, 0.6 * self.limit)
            fb = self.Md * (self.kp * self.gp * e + self.kd * self.gd * ed) + self.integ
            d.ctrl[self.A] = np.clip(dc.qfrc_inverse[self.Vc] + fb, -self.limit, self.limit)
            self.fb += (fb - self.fb) * (S.TIMESTEP / 0.02)
            mj.mj_step(m, d)
        self.t += DT
        self.tick += 1
        self._read()
        for a in self.arms.values():
            if a.fault:
                continue
            err = np.abs(a.q_cmd - a.q)
            over = np.abs(self.fb[a.sl]) > self.trip[a.sl]
            if np.any(err > self.follow):
                i = int(np.argmax(err))
                self._fault(a, "following_error", {"joint": i + 1, "error_deg": round(math.degrees(err[i]), 2)})
            elif np.any(over):
                i = int(np.argmax(np.abs(self.fb[a.sl]) / self.trip[a.sl]))
                self._fault(a, "collision_detect", {"joint": i + 1, "disturbance_nm": round(float(self.fb[a.sl][i]), 1),
                                                    "threshold_nm": round(float(self.trip[a.sl][i]), 1)})
            # a move is done when its last command has been sent and (for the last one queued) the arm is in position
            elif a.moves and len(a.buf) <= a.moves[0][1] and (a.moves[0][1] > 0 or a.in_position):
                a.done_id = a.moves.popleft()[0]
                self._log(a.name, "move_done", {"id": a.done_id})
        if self.tick % SCAN_EVERY == 0:
            self._scan()

    def _jaws(self, a: Arm):
        """The jaw drive travels at its own speed toward the commanded opening; the force it may use is the grip force."""
        step = S.GRIPPER["jaw_speed"] * DT
        a.grip_ctrl += float(np.clip(a.grip_cmd - a.grip_ctrl, -step, step))
        self.d.ctrl[self.FA[a.name]] = a.grip_ctrl / 2

    def _read(self):
        """The sensors: encoders, motor torques, wrist force/torque, where the parts are."""
        d = self.d
        for p, a in self.arms.items():
            a.q = d.qpos[self.Q[a.sl]] - self.zero[a.sl]           # the encoders (motor side, with their zero error)
            a.qd = d.qvel[self.V[a.sl]].copy()
            a.torque = d.ctrl[self.A[a.sl]].copy()
            a.grip = float(d.qpos[self.FQ[p]].sum())
            a.grip_vel = float(d.qvel[self.FV[p]].sum())
            Rs = d.site_xmat[self.ft_site[p]].reshape(3, 3)
            fa, ta = self.ft_adr[p]
            k = DT / 0.05
            a.wrist_f += (-Rs @ d.sensordata[fa:fa + 3] - a.wrist_f) * k         # what the tool pulls on the wrist with
            a.wrist_t += (-Rs @ d.sensordata[ta:ta + 3] - a.wrist_t) * k
            a.load_kg = max(0.0, -a.wrist_f[2] / G - S.GRIPPER["mass"])
        for k, p in self.parts.items():
            b = self.part_body[k]
            p["T"] = R.T_of(d.xmat[b].reshape(3, 3), d.xpos[b])
            v = d.qvel[self.part_v[k]:self.part_v[k] + 6]
            p["speed"] = float(np.linalg.norm(v[:3]) + 0.1 * np.linalg.norm(v[3:]))

    def _scan(self):
        """Read the contacts: who presses on whom and how hard. Fills holding, resting_on, touching, arm_distance."""
        mj, m, d = self.mj, self.m, self.d
        f6 = np.zeros(6)
        squeeze = {p: {} for p in self.arms}                       # arm -> part -> [force on jaw l, force on jaw r]
        rest = {k: (0.0, None) for k in self.parts}
        touching = []
        for i in range(d.ncon):
            c = d.contact[i]
            o1, o2 = self.owner[c.geom1], self.owner[c.geom2]
            if c.dist > 0:
                continue                                           # near, not touching
            mj.mj_contactForce(m, d, i, f6)
            n = float(f6[0])
            if n <= 0:
                continue
            for a, b in ((o1, o2), (o2, o1)):
                if a[0] == "tool" and a[2] >= 8 and b[0] == "part":
                    squeeze[a[1]].setdefault(b[1], [0.0, 0.0])[a[2] - 8] += n
                elif a[0] == "part" and b[0] in ("fix", "floor", "part") and (b[0] != "part" or self.parts[b[1]]["T"][2, 3] < self.parts[a[1]]["T"][2, 3]):
                    if n > rest[a[1]][0]:
                        rest[a[1]] = (n, b[1])
            jaw_on_part = any(o[0] == "tool" and o[2] >= 8 for o in (o1, o2)) and any(o[0] == "part" for o in (o1, o2))
            if n >= TOUCH_N and not jaw_on_part and (o1[0] in ("arm", "tool") or o2[0] in ("arm", "tool")):
                touching.append((_name(o1), _name(o2), round(n, 1)))
        self.touching = touching
        self.arm_distance = self._plan_check({})["arm_distance"]
        held = {k: [] for k in self.parts}
        for p, a in self.arms.items():
            now = None
            for k, (fl, fr) in squeeze[p].items():
                if fl >= GRIP_ON_N and fr >= GRIP_ON_N:
                    now, a.grip_n = k, (fl + fr) / 2
            if now != a.holding:
                if a.holding:
                    self._log(p, "release", {"part": a.holding})
                if now:
                    self._log(p, "grasp", {"part": now, "width": round(a.grip, 4), "squeeze_n": round(a.grip_n, 1)})
                a.holding = now
            if not now:
                a.grip_n = 0.0
            else:
                held[now].append(p)
        for k, p in self.parts.items():
            if p["held"] and p["held"][0] in p["rel"]:             # it rode with its first holder until now
                p["Tb"] = self.arms[p["held"][0]].tcp_world() @ p["rel"][p["held"][0]]
            p["held"] = [h for h in p["held"] if h in held[k]] + [h for h in held[k] if h not in p["held"]]   # first holder leads
            p["rel"] = {h: p["rel"].get(h, R.inv(self.arms[h].tcp_world()) @ p["Tb"]) for h in p["held"]}     # as each tcp believes it holds it
            on = rest[k][1] if rest[k][0] > 0.2 else None
            if on != p["resting_on"]:
                if on and not held[k]:
                    self._log(None, "landed", {"part": k, "on": on})
                p["resting_on"] = on

    def _fault(self, a: Arm, code, detail):
        """Stop where it is: drop the queue and hold the measured position."""
        if a.fault:
            return
        a.fault = {"code": code, **detail}
        a.buf = np.zeros((0, 6))
        a.moves.clear()
        self.q_des[a.sl] = a.q_cmd = a.q.copy()
        a.v_cmd = np.zeros(6)
        self.v_des[a.sl] = self.a_des[a.sl] = 0
        self.integ[a.sl] = self.fb[a.sl] = 0
        self._log(a.name, "fault", a.fault)

    # ---------------------------------------------------------- the planner's view of the cell (geometry only)
    def _plan_check(self, qs: dict) -> dict:
        """With the arms at joint angles qs (and what they hold riding along), what overlaps? {"hits", "arm_distance"}."""
        mj, m, dp = self.mj, self.mg, self.dp
        with self.plan_lock:
            dp.qpos[:] = m.qpos0                                    # the nominal cell: no calibration error, no gearbox twist
            held = {}
            for p, a in self.arms.items():
                dp.qpos[self.Q[a.sl]] = qs.get(p, a.q)
                dp.qpos[self.FQ[p]] = a.grip / 2
            for k, part in self.parts.items():
                T = part["Tb"]
                if part["held"]:
                    lead = self.arms[part["held"][0]]
                    T = lead.tcp_world(qs.get(lead.name, lead.q)) @ part["rel"][lead.name]
                    held[k] = tuple(part["held"])
                i = self.part_q[k]
                dp.qpos[i:i + 3] = T[:3, 3]
                dp.qpos[i + 3:i + 7] = R.R_to_quat(T[:3, :3])
            mj.mj_kinematics(m, dp)
            mj.mj_collision(m, dp)
            hits, sep = [], None
            for c in dp.contact[:dp.ncon]:
                o1, o2 = self.owner[c.geom1], self.owner[c.geom2]
                bad, between = _judge(o1, o2, c.dist, held)
                if between and (sep is None or c.dist < sep):
                    sep = float(c.dist)
                if bad:
                    hits.append((_name(o1), _name(o2), round(-float(c.dist), 4)))
            return {"hits": hits, "arm_distance": sep}

    # ---------------------------------------------------------- planning
    def _arm(self, name) -> Arm:
        if name not in self.arms:
            raise SimError(404, "no_such_arm", f"arm '{name}' does not exist; arms are {list(self.arms)}")
        return self.arms[name]

    def _ready(self, a: Arm):
        if a.fault:
            raise SimError(409, "arm_in_fault", f"arm {a.name} is in fault ({a.fault['code']}); POST /api/arm/{a.name}/reset_fault",
                           fault=a.fault)

    @staticmethod
    def _limits(q, what="goal"):
        q = np.asarray(q, float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise SimError(400, "bad_joints", "joints must be 6 numbers")
        bad = [i + 1 for i in range(6) if not R.Q_MIN[i] - 1e-9 <= q[i] <= R.Q_MAX[i] + 1e-9]
        if bad:
            raise SimError(422, "joint_limit", f"{what} is outside the limits of joint(s) {bad}", joints=bad)
        return q

    def _path_joint(self, a: Arm, q1):
        q0 = a.q_end()
        m = max(2, int(math.ceil(float(np.max(np.abs(q1 - q0))) / 0.01)) + 1)
        return q0 + np.linspace(0, 1, m)[:, None] * (q1 - q0)

    def _solve(self, a: Arm, T_world, ref, config=None):
        sols = R.ik(a.to_base(T_world), ref, S.GRIPPER["tcp_z"])
        if config:
            sols = [s for s in sols if R.config_of(s) == config.upper()]
        if not sols:
            raise SimError(422, "unreachable", f"arm {a.name} cannot reach that pose" + (f" in configuration {config}" if config else ""),
                           pos=[round(float(v), 4) for v in T_world[:3, 3]])
        return sols[0]

    def _path_line(self, a: Arm, poses: list):
        """Joint path following tcp world poses (a list sampled along the line), staying in one configuration."""
        q = a.q_end()
        out = []
        for i, T in enumerate(poses):
            sols = R.ik(a.to_base(T), q, S.GRIPPER["tcp_z"])
            if not sols:
                raise SimError(422, "unreachable", f"arm {a.name} cannot follow the line: no solution {100 * i // max(1, len(poses) - 1)} % along it",
                               at=[round(float(v), 4) for v in T[:3, 3]])
            if float(np.max(np.abs(sols[0] - q))) > 0.35:
                raise SimError(422, "singularity", f"arm {a.name}'s joints would have to jump {100 * i // max(1, len(poses) - 1)} % along the line "
                               "(a singularity or a joint limit on the way); use a joint move or another route",
                               at=[round(float(v), 4) for v in T[:3, 3]])
            q = sols[0]
            out.append(q)
        return np.array(out)

    @staticmethod
    def _line_poses(T0, T1):
        """Poses from T0 to T1: position on a straight line, orientation turning evenly. Also (length m, angle rad)."""
        L = float(np.linalg.norm(T1[:3, 3] - T0[:3, 3]))
        D = T0[:3, :3].T @ T1[:3, :3]
        ang = math.acos(max(-1.0, min(1.0, (np.trace(D) - 1) / 2)))
        m = max(2, int(math.ceil(max(L / 0.002, ang / math.radians(0.5)))) + 1)
        poses = []
        for s in np.linspace(0, 1, m):
            poses.append(R.T_of(R.slerp_R(T0[:3, :3], T1[:3, :3], s), T0[:3, 3] + s * (T1[:3, 3] - T0[:3, 3])))
        return poses, L, ang

    @staticmethod
    def _cap(L, ang, speed, acc=LINE_ACC):
        speed = min(float(speed), LINE_SPEED_MAX)
        if speed <= 0:
            raise SimError(400, "bad_speed", "speed must be above 0")
        v = min(speed / L if L > 1e-9 else math.inf, TURN_SPEED / ang if ang > 1e-9 else math.inf)
        acc_s = min(acc / L if L > 1e-9 else math.inf, TURN_ACC / ang if ang > 1e-9 else math.inf)
        return (v, acc_s) if math.isfinite(v) else (1e6, 1e9)

    def _check_plan(self, trajs: dict):
        """Run the new commands (arm -> [K, 6]) forward in time next to everything already queued; raise on a hit."""
        off = {p: len(self.arms[p].buf) for p in trajs}
        # to the end of everything queued on EVERY arm: the other arm may arrive after this move has finished
        horizon = max([off[p] + len(t) for p, t in trajs.items()] + [len(a.buf) for a in self.arms.values()])

        def at(p, k):
            a = self.arms[p]
            if p in trajs and k >= off[p]:
                return trajs[p][min(k - off[p], len(trajs[p]) - 1)]
            return a.planned(k)
        ks = list(range(min(off.values()), horizon, CHECK_EVERY)) + [horizon - 1]
        for k in ks:
            r = self._plan_check({p: at(p, k) for p in self.arms})
            if r["hits"]:
                raise SimError(409, "collision", f"planned path collides {k * DT:.2f} s from now: " +
                               ", ".join(f"{h[0]} x {h[1]} ({h[2] * 1000:.0f} mm deep)" for h in r["hits"][:3]),
                               t=round(k * DT, 3), hits=[list(h) for h in r["hits"][:6]])

    def _commit(self, trajs: dict, kind: str, check=True):
        if check:
            self._check_plan(trajs)
        out = {}
        late, k = self.reality.late, self.reality.clock_cycles
        if len(trajs) > 1 and k and late in trajs:                 # two controllers: one starts the shared move a little late
            trajs = dict(trajs)
            first = self.arms[late].q_end() if not len(trajs[late]) else trajs[late][:1]
            trajs[late] = np.concatenate([np.repeat(np.atleast_2d(first), k, 0), trajs[late]])
        end = max(len(self.arms[p].buf) + len(t) for p, t in trajs.items())
        for p, t in trajs.items():
            a = self.arms[p]
            pad = end - len(a.buf) - len(t)                        # arms moving together finish on the same cycle
            if pad > 0 and len(trajs) > 1:
                t = np.concatenate([t, np.repeat(t[-1:], pad, 0)])
            a.buf = np.concatenate([a.buf, t]) if len(a.buf) else t.copy()
            mid = a.next_id
            a.next_id += 1
            a.moves = collections.deque((i, left + len(t)) for i, left in a.moves)
            a.moves.append((mid, 0))
            out[p] = {"id": mid, "duration": round(len(t) * DT, 3), "starts_in": round((len(a.buf) - len(t)) * DT, 3)}
            self._log(p, "move_start", {"id": mid, "kind": kind, "duration": out[p]["duration"]})
        return out

    # ---------------------------------------------------------- single-arm commands
    def move_joint(self, arm, joints, speed=1.0, check=True):
        """Joint-interpolated move to joint angles (rad). speed: fraction of the arm's joint speeds, 0-1."""
        with self.lock:
            a = self._arm(arm)
            self._ready(a)
            q1 = self._limits(joints)
            speed = float(np.clip(speed, 0.01, 1.0))
            (traj,) = time_param([self._path_joint(a, q1)], speed)
            return {"ok": True, **self._commit({arm: traj}, "joint", check)[arm]}

    def move_pose(self, arm, T_world, speed=1.0, config=None, check=True):
        """Joint-interpolated move putting the tcp at a world pose (the tool does NOT travel in a straight line)."""
        with self.lock:
            a = self._arm(arm)
            self._ready(a)
            q1 = self._solve(a, T_world, a.q_end(), config)
            speed = float(np.clip(speed, 0.01, 1.0))
            (traj,) = time_param([self._path_joint(a, q1)], speed)
            return {"ok": True, "joints": q1.tolist(), "config": R.config_of(q1), **self._commit({arm: traj}, "pose", check)[arm]}

    def move_line(self, arm, T_world, speed=LINE_SPEED, check=True):
        """Straight-line move of the tcp to a world pose, at `speed` m/s (slower where the joints cannot keep up)."""
        with self.lock:
            a = self._arm(arm)
            self._ready(a)
            poses, L, ang = self._line_poses(a.tcp_world(a.q_end()), T_world)
            Q = self._path_line(a, poses)
            (traj,) = time_param([Q], 1.0, [self._cap(L, ang, speed)])
            return {"ok": True, "length": round(L, 4), **self._commit({arm: traj}, "line", check)[arm]}

    def gripper(self, arm, width=None, action=None, force=None):
        """Set the opening the jaws go to (m). action: "open" (0.10) or "close" (0). force: N per jaw the jaws may
        squeeze with (default the gripper's full force). Takes effect now."""
        with self.lock:
            a = self._arm(arm)
            if action == "open":
                width = 0.10
            elif action == "close":
                width = 0.0
            if width is None:
                raise SimError(400, "bad_gripper", 'give "width" in metres, or "action": "open" | "close"')
            full = self.dyn["gripper"]["grip_force"]
            if force is not None:
                a.grip_force = float(np.clip(force, 1.0, full))
            f = a.grip_force or full
            self.m.actuator_forcerange[self.FA[arm]] = [-f, f]
            a.grip_cmd = float(np.clip(width, 0.0, S.GRIPPER["stroke"]))
            self._log(arm, "gripper", {"target": a.grip_cmd, "force_n": f})
            return {"ok": True, "target": a.grip_cmd, "force_n": f,
                    "duration": round(abs(a.grip_ctrl - a.grip_cmd) / S.GRIPPER["jaw_speed"], 3)}

    def set_payload(self, arm, mass):
        """Tell the arm's controller what it carries (kg at the tcp), as a FANUC PAYLOAD instruction does. Its
        feedforward then holds that weight; without it the feedback has to, and collision detection is nearer."""
        with self.lock:
            a = self._arm(arm)
            a.payload = float(np.clip(mass, 0.0, 100.0))
            self.mc.body_mass[self.pay[arm]] = max(a.payload, 1e-6)
            self._log(arm, "payload", {"kg": a.payload})
            return {"ok": True, "payload_kg": a.payload}

    def soft_float(self, arm, enabled=True, stiffness=0.1):
        """Make the arm compliant, as FANUC's Soft Float does: its servo holds position with only `stiffness` (0.01-1)
        of the normal spring and no integrator, while still carrying its own weight and its declared payload. For
        the FOLLOWER when two arms hold one rigid part: it then gives way to the leader instead of fighting it."""
        with self.lock:
            a = self._arm(arm)
            a.soft = float(np.clip(stiffness, 0.01, 1.0)) if enabled else None
            k = a.soft or 1.0
            self.gp[a.sl], self.gd[a.sl], self.gi[a.sl] = k, math.sqrt(k), 0.0 if a.soft else 1.0
            self.integ[a.sl] = 0
            if not a.soft:                                         # stiff again from where it is, not where it was told to be
                self.q_des[a.sl] = a.q_cmd = a.q.copy() if not len(a.buf) else a.q_cmd
            self._log(arm, "soft_float", {"enabled": bool(a.soft), "stiffness": k})
            return {"ok": True, "soft_float": a.soft}

    def stop(self, arm=None):
        """Drop everything queued; the arm brakes onto the command it had. arm=None: both."""
        with self.lock:
            for a in ([self._arm(arm)] if arm else self.arms.values()):
                a.buf = np.zeros((0, 6))
                a.moves.clear()
                self._log(a.name, "stop", {})
            return {"ok": True}

    def reset_fault(self, arm):
        with self.lock:
            a = self._arm(arm)
            a.fault = None
            self.q_des[a.sl] = a.q_cmd = a.q.copy()
            self.integ[a.sl] = self.fb[a.sl] = 0
            self._log(arm, "fault_reset", {})
            return {"ok": True}

    # ---------------------------------------------------------- the two arms together
    def _spec_path(self, a: Arm, spec: dict):
        kind = spec.get("type", "joint")
        if kind == "joint":
            return self._path_joint(a, self._limits(spec["joints"])), None
        if kind == "pose":
            return self._path_joint(a, self._solve(a, spec["T"], a.q_end(), spec.get("config"))), None
        if kind == "line":
            poses, L, ang = self._line_poses(a.tcp_world(a.q_end()), spec["T"])
            return self._path_line(a, poses), self._cap(L, ang, spec.get("speed", LINE_SPEED))
        raise SimError(400, "bad_type", 'move type must be "joint", "pose" or "line"')

    def move_together(self, specs: dict, speed=1.0, check=True):
        """Each arm makes its own move; both start on the same cycle and arrive on the same cycle.
        specs: arm -> {"type": "joint", "joints": [...]} | {"type": "pose"|"line", "T": 4x4 world pose, ...}."""
        with self.lock:
            arms = [self._arm(p) for p in specs]
            for a in arms:
                self._ready(a)
            self._idle(arms)
            paths, caps = [], []
            for a in arms:
                Q, cap = self._spec_path(a, specs[a.name])
                paths.append(Q)
                if cap:
                    caps.append(cap)
            trajs = time_param(paths, float(np.clip(speed, 0.01, 1.0)), caps)
            return {"ok": True, "moves": self._commit({a.name: t for a, t in zip(arms, trajs)}, "together", check)}

    def _idle(self, arms):
        busy = [a.name for a in arms if len(a.buf)]
        if busy:
            raise SimError(409, "busy", f"arm(s) {busy} still have moves queued; a move together starts from rest (wait first)")

    def move_part(self, part, T_world, speed=0.25, check=True):
        """Move a part held by one or both arms so the PART goes in a straight line to a world pose. Each gripper's
        path is the part's path seen from where that gripper holds it now. speed: m/s of the part's centre."""
        with self.lock:
            if part not in self.parts:
                raise SimError(404, "no_such_part", f"part '{part}' does not exist; parts are {list(self.parts)}")
            p = self.parts[part]
            if not p["held"]:
                raise SimError(409, "not_held", f"no arm is holding '{part}'")
            arms = [self.arms[h] for h in p["held"]]
            for a in arms:
                self._ready(a)
            self._idle(arms)
            lead = arms[0]
            T0 = lead.tcp_world(lead.q_end()) @ p["rel"][lead.name]   # where the cell believes the part is (it cannot see it)
            rel = {a.name: R.inv(a.tcp_world(a.q_end())) @ T0 for a in arms}   # from the COMMANDED poses: no drift builds up
            poses, L, ang = self._line_poses(T0, T_world)
            paths = [self._path_line(a, [T @ R.inv(rel[a.name]) for T in poses]) for a in arms]
            trajs = time_param(paths, 1.0, [self._cap(L, ang, speed, acc=2.0)])
            return {"ok": True, "length": round(L, 4), "holders": [a.name for a in arms],
                    "moves": self._commit({a.name: t for a, t in zip(arms, trajs)}, "part", check)}

    # ---------------------------------------------------------- queries
    def wait(self, arms=None, timeout=60.0):
        """Block until the arms (default both), their grippers and the parts are at rest, or an arm faults."""
        names = [arms] if isinstance(arms, str) else list(arms or self.arms)
        end = time.time() + timeout
        while True:
            with self.lock:
                sel = [self._arm(n) for n in names]
                fault = [a.name for a in sel if a.fault]
                busy = [a.name for a in sel if a.status == "moving" or a.gripper_busy]
                loose = [p["name"] for p in self.parts.values() if p["speed"] >= 0.002 and not p["held"]]
                if fault:
                    raise SimError(409, "fault", f"arm(s) {fault} faulted", faults={n: self.arms[n].fault for n in fault})
                if not busy and not loose:
                    return {"ok": True, "t": round(self.t, 3)}
            if time.time() > end:
                raise SimError(408, "timeout", f"still moving after {timeout} s: arms {busy}, parts {loose}")
            time.sleep(0.004 if self.realtime else 0.0005)

    def check_pose(self, joints: dict):
        """Would these joint angles (arm -> 6 rad; an arm left out stays where it is) overlap anything?"""
        with self.lock:
            r = self._plan_check({p: np.asarray(q, float) for p, q in joints.items()})
            return {"ok": True, "collides": bool(r["hits"]), "hits": [list(h) for h in r["hits"]], "arm_distance": r["arm_distance"]}
