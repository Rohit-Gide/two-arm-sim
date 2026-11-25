"""The endpoints, independent of HTTP: `Api.handle(method, path, body) -> (status, json-able dict)`.
server.py puts this behind HTTP; tests and in-process users can call it directly. See API.md for the reference."""
from __future__ import annotations

import math

import numpy as np

from . import robot as R
from . import scene as S
from .reality import Reality
from .sim import LINE_SPEED, Sim, SimError

ENDPOINTS = [
    ("GET", "/api", "this list"),
    ("GET", "/api/scene", "the cell, static: robot chain, limits, base poses, gripper, fixtures, parts"),
    ("GET", "/api/state", "everything measured: joints, motor torques, tcp poses, wrist force, grippers, parts, contacts, faults"),
    ("GET", "/api/events?since=N", "what happened (move_start, move_done, grasp, release, landed, fault ...) after event N"),
    ("POST", "/api/fk", "{arm, joints|joints_deg} -> tcp pose in world and base, link frames, configuration"),
    ("POST", "/api/ik", "{arm, pos, <orientation>, frame?} -> every joint solution inside the limits, nearest first"),
    ("POST", "/api/jacobian", "{arm, joints?} -> 6x6 Jacobian of the tcp (base frame), manipulability"),
    ("POST", "/api/check", "{a?: joints, b?: joints} -> would these poses collide; distance between the arms"),
    ("POST", "/api/arm/{a|b}/move_joint", "{joints|joints_deg, speed 0-1, wait?} joint-interpolated move"),
    ("POST", "/api/arm/{a|b}/move_pose", "{pos, <orientation>, config?, speed 0-1, wait?} joint-interpolated move to a tcp pose"),
    ("POST", "/api/arm/{a|b}/move_line", "{pos, <orientation>, speed m/s, wait?} straight-line tcp move"),
    ("POST", "/api/arm/{a|b}/gripper", '{width m | action "open"|"close", force N per jaw?, wait?} force-closed jaws; parts are held by friction'),
    ("POST", "/api/arm/{a|b}/payload", "{mass kg} tell the controller what the arm carries (feedforward); 0 when it lets go"),
    ("POST", "/api/arm/{a|b}/soft_float", "{enabled, stiffness 0.01-1} compliant servo (FANUC Soft Float): for the follower when both arms hold one part"),
    ("POST", "/api/arm/{a|b}/stop", "drop the arm's queued moves"),
    ("POST", "/api/arm/{a|b}/reset_fault", "clear a fault"),
    ("POST", "/api/together/move", "{a: {type joint|pose|line, ...}, b: {...}, speed, wait?} both start and arrive together"),
    ("POST", "/api/together/move_part", "{part, pos, <orientation>, speed m/s, wait?} move a part every holder keeps hold of"),
    ("POST", "/api/wait", "{arms?: [..], timeout?} block until at rest (409 if an arm faults)"),
    ("POST", "/api/stop", "drop every queued move"),
    ("POST", "/api/reset", '{seed?, only?, ideal?} everything back to the start; "seed": n rebuilds the world with another draw of the real-world errors, "ideal": true with none'),
    ("GET", "/api/reality", "the errors in this world: what is true against what the controllers believe"),
]


def pose_json(T) -> dict:
    T = np.asarray(T)
    return {"pos": [round(float(v), 6) for v in T[:3, 3]], "quat_wxyz": [round(float(v), 7) for v in R.R_to_quat(T[:3, :3])],
            "wpr_deg": [round(math.degrees(v), 4) for v in R.R_to_wpr(T[:3, :3])],
            "matrix": [[round(float(v), 7) for v in row] for row in T]}


def _vec(b, key, n):
    v = b.get(key)
    if v is None:
        return None
    try:
        a = np.array(v, float)
    except (TypeError, ValueError):
        a = None
    if a is None or a.shape != (n,) or not np.all(np.isfinite(a)):
        raise SimError(400, "bad_request", f'"{key}" must be {n} numbers')
    return a


def joints_of(b: dict, required=True):
    q = _vec(b, "joints", 6)
    if q is None:
        d = _vec(b, "joints_deg", 6)
        q = None if d is None else np.radians(d)
    if q is None:
        d = _vec(b, "joints_fanuc_deg", 6)
        q = None if d is None else R.from_fanuc_deg(d)
    if q is None and required:
        raise SimError(400, "bad_request", 'give "joints" (rad), "joints_deg" or "joints_fanuc_deg": 6 numbers')
    return q


class Api:
    def __init__(self, sim: Sim):
        self.sim = sim

    # ------------------------------------------------------ poses in and out
    def _pose(self, b: dict, arm=None, default_R=None):
        """A world pose from a request body. Orientation, one of: "quat_wxyz", "wpr_deg" (FANUC W,P,R), "matrix"
        (3x3), "tool_down" (yaw in degrees of the jaw axis; the tool points straight down). None given: keep the
        orientation the arm will have. "frame": "world" (default) or "base" (that arm's base_link)."""
        pos = _vec(b, "pos", 3)
        if pos is None:
            raise SimError(400, "bad_request", '"pos" [x, y, z] in metres is required')
        if "quat_wxyz" in b:
            Rm = R.quat_to_R(_vec(b, "quat_wxyz", 4))
        elif "wpr_deg" in b:
            Rm = R.wpr_to_R(*np.radians(_vec(b, "wpr_deg", 3)))
        elif "matrix" in b:
            Rm = np.array(b["matrix"], float)
            if Rm.shape != (3, 3) or abs(np.linalg.det(Rm) - 1) > 1e-3:
                raise SimError(400, "bad_request", '"matrix" must be a 3x3 rotation')
        elif "tool_down" in b:
            Rm = R.tool_down(math.radians(float(b["tool_down"])))
        elif "yaw_deg" in b:
            Rm = R.rz(math.radians(float(b["yaw_deg"])))
        elif default_R is not None:
            Rm = default_R
        else:
            raise SimError(400, "bad_request", 'give an orientation: "quat_wxyz", "wpr_deg", "matrix" or "tool_down"')
        T = R.T_of(Rm, pos)
        frame = b.get("frame", "world")
        if frame == "base":
            if arm is None:
                raise SimError(400, "bad_request", '"frame": "base" needs an arm')
            T = self.sim.arms[arm].T_wb @ T
        elif frame != "world":
            raise SimError(400, "bad_request", '"frame" must be "world" or "base"')
        return T

    def _arm_pose(self, arm, b):
        a = self.sim._arm(arm)
        return self._pose(b, arm, a.tcp_world(a.q_end())[:3, :3] if b.get("frame", "world") == "world" else None)

    # ------------------------------------------------------ state
    def state(self) -> dict:
        s = self.sim
        with s.lock:
            arms = {}
            full = s.dyn["gripper"]["grip_force"]
            for p, a in s.arms.items():
                Tw = a.tcp_world()
                J = R.jacobian(a.q, S.GRIPPER["tcp_z"])
                lim = s.limit[a.sl]
                warn = []
                if a.load_kg + S.GRIPPER["mass"] > R.PAYLOAD_KG:
                    warn.append("over_rated_payload")
                if np.max(np.abs(a.torque) / lim) > 0.98:
                    warn.append("motor_saturated")
                arms[p] = {
                    "status": a.status, "fault": a.fault, "warnings": warn,
                    "joints": [round(float(v), 6) for v in a.q], "joints_deg": [round(float(v), 4) for v in np.degrees(a.q)],
                    "joints_fanuc_deg": [round(float(v), 4) for v in R.to_fanuc_deg(a.q)],
                    "joints_cmd": [round(float(v), 6) for v in a.q_cmd],
                    "joint_vel": [round(float(v), 5) for v in a.qd],
                    "following_error_deg": [round(float(v), 4) for v in np.degrees(a.q_cmd - a.q)],
                    "joint_torque": [round(float(v), 2) for v in a.torque],
                    "motor_load": [round(float(v), 3) for v in np.abs(a.torque) / lim],
                    "disturbance_torque": [round(float(v), 2) for v in s.fb[a.sl]],
                    "config": R.config_of(a.q),
                    "tcp": {"world": pose_json(Tw), "base": pose_json(a.T_bw @ Tw)},
                    "tcp_speed": round(float(np.linalg.norm(J[:3] @ a.qd)), 5),
                    "manipulability": round(R.manipulability(a.q, S.GRIPPER["tcp_z"]), 5),
                    "gripper": {"width": round(a.grip, 5), "target": round(a.grip_cmd, 5), "force_limit_n": a.grip_force or full,
                                "squeeze_n": round(a.grip_n, 1), "holding": a.holding},
                    "wrist": {"force": [round(float(v), 2) for v in a.wrist_f], "torque": [round(float(v), 2) for v in a.wrist_t]},
                    "load_kg": round(a.load_kg + S.GRIPPER["mass"], 2), "payload_kg": R.PAYLOAD_KG, "payload_declared_kg": a.payload,
                    "soft_float": a.soft,
                    "queued_moves": len(a.moves), "queued_s": round(len(a.buf) * 0.004, 3), "last_done_id": a.done_id,
                }
            parts = {k: {"pose": pose_json(p["T"]), "believed_pose": pose_json(p["Tb"]),
                         "held_by": list(p["held"]), "resting_on": p["resting_on"],
                         "speed": round(p["speed"], 4), "mass": p["mass"], "size": p["size"].tolist()} for k, p in s.parts.items()}
            return {"t": round(s.t, 3), "arms": arms, "parts": parts,
                    "arm_distance": None if s.arm_distance is None else round(s.arm_distance, 4),
                    "touching": [list(h) for h in s.touching], "last_event": s.event_n}

    # ------------------------------------------------------ routing
    def handle(self, method: str, path: str, b: dict | None = None, query: dict | None = None):
        b, query = b or {}, query or {}
        try:
            return 200, self._route(method, path.rstrip("/") or "/", b, query)
        except SimError as e:
            return e.status, e.as_dict()
        except (KeyError, TypeError, ValueError) as e:
            return 400, {"ok": False, "error": "bad_request", "message": f"{type(e).__name__}: {e}"}

    def _after(self, arms, b, out):
        if b.get("wait"):
            out["wait"] = self.sim.wait(arms, float(b.get("timeout", 120)))
        return out

    def _route(self, method, path, b, query):
        s = self.sim
        seg = path.strip("/").split("/")
        if method == "GET":
            if path == "/api":
                return {"endpoints": [{"method": m, "path": p, "what": w} for m, p, w in ENDPOINTS]}
            if path == "/api/scene":
                return S.describe()
            if path == "/api/state":
                return self.state()
            if path == "/api/events":
                since = int(query.get("since", 0))
                with s.lock:
                    return {"events": [e for e in s.events if e["n"] > since], "last_event": s.event_n}
            if path == "/api/reality":
                return s.reality.summary()
            raise SimError(404, "not_found", f"no GET {path}; see GET /api")
        if method != "POST":
            raise SimError(405, "method", "use GET or POST")
        check = bool(b.get("check", True))
        if path == "/api/fk":
            arm = b.get("arm", "a")
            a = s._arm(arm)
            q = joints_of(b)
            frames = R.link_frames(q)
            Tb = R.fk(q, S.GRIPPER["tcp_z"])
            return {"ok": True, "arm": arm, "tcp": {"world": pose_json(a.T_wb @ Tb), "base": pose_json(Tb)},
                    "flange": {"world": pose_json(a.T_wb @ R.fk(q)), "base": pose_json(R.fk(q))},
                    "links": {n: pose_json(a.T_wb @ F) for n, F in zip(R.LINKS, frames)},
                    "config": R.config_of(q), "within_limits": bool(np.all((q >= R.Q_MIN) & (q <= R.Q_MAX)))}
        if path == "/api/ik":
            arm = b.get("arm", "a")
            a = s._arm(arm)
            T = self._pose(b, arm)
            ref = joints_of({"joints": b["ref"]} if "ref" in b else {}, required=False)
            sols = R.ik(a.to_base(T), a.q if ref is None else ref, S.GRIPPER["tcp_z"])
            out = []
            for q in sols:
                c = s.check_pose({arm: q})
                out.append({"joints": [round(float(v), 6) for v in q], "joints_deg": [round(float(v), 4) for v in np.degrees(q)],
                            "config": R.config_of(q), "collides": c["collides"]})
            return {"ok": True, "arm": arm, "reachable": bool(out), "solutions": out}
        if path == "/api/jacobian":
            a = s._arm(b.get("arm", "a"))
            q = joints_of(b, required=False)
            q = a.q if q is None else q
            return {"ok": True, "jacobian": R.jacobian(q, S.GRIPPER["tcp_z"]).round(7).tolist(),
                    "manipulability": R.manipulability(q, S.GRIPPER["tcp_z"]), "frame": "base", "rows": "vx vy vz wx wy wz"}
        if path == "/api/check":
            return s.check_pose({p: joints_of({"joints": b[p]} if isinstance(b[p], list) else b[p]) for p in s.arms if p in b})
        if len(seg) == 4 and seg[:2] == ["api", "arm"]:
            arm, cmd = seg[2], seg[3]
            s._arm(arm)
            if cmd == "move_joint":
                return self._after(arm, b, s.move_joint(arm, joints_of(b), b.get("speed", 1.0), check))
            if cmd == "move_pose":
                return self._after(arm, b, s.move_pose(arm, self._arm_pose(arm, b), b.get("speed", 1.0), b.get("config"), check))
            if cmd == "move_line":
                return self._after(arm, b, s.move_line(arm, self._arm_pose(arm, b), b.get("speed", LINE_SPEED), check))
            if cmd == "gripper":
                return self._after(arm, b, s.gripper(arm, b.get("width"), b.get("action"), b.get("force")))
            if cmd == "soft_float":
                return s.soft_float(arm, bool(b.get("enabled", True)), float(b.get("stiffness", 0.1)))
            if cmd == "payload":
                return s.set_payload(arm, float(b.get("mass", 0.0)))
            if cmd == "stop":
                return s.stop(arm)
            if cmd == "reset_fault":
                return s.reset_fault(arm)
        if path == "/api/together/move":
            specs = {}
            for p in s.arms:
                if p not in b:
                    continue
                spec = dict(b[p])
                kind = spec.setdefault("type", "joint")
                if kind == "joint":
                    spec["joints"] = joints_of(spec)
                else:
                    spec["T"] = self._arm_pose(p, spec)
                specs[p] = spec
            if not specs:
                raise SimError(400, "bad_request", 'give a move for "a" and/or "b"')
            return self._after(list(specs), b, s.move_together(specs, b.get("speed", 1.0), check))
        if path == "/api/together/move_part":
            name = b.get("part")
            if name not in s.parts:
                raise SimError(404, "no_such_part", f"part '{name}' does not exist; parts are {list(s.parts)}")
            T = self._pose(b, None, s.parts[name]["T"][:3, :3])
            r = s.move_part(name, T, b.get("speed", 0.25), check)
            return self._after(r["holders"], b, r)
        if path == "/api/wait":
            return s.wait(b.get("arms"), float(b.get("timeout", 120)))
        if path == "/api/stop":
            return s.stop()
        if path == "/api/reset":
            if b.get("ideal"):
                return s.reset(Reality.ideal())
            if "seed" in b:
                return {**s.reset(Reality.sample(int(b["seed"]), only=b.get("only"))), "reality": s.reality.summary()}
            return s.reset()
        raise SimError(404, "not_found", f"no POST {path}; see GET /api")
