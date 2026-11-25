"""Regenerate API.md from a RUNNING simulator, so every example in it is a real response.
    ./run.sh --ideal &   then   python3 docs/make_api_md.py [http://127.0.0.1:8700]"""
import json
import sys
import urllib.error
import urllib.request
from pathlib import Path

U = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8700"


def call(method, path, body=None):
    req = urllib.request.Request(U + path, data=None if body is None else json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method=method)
    try:
        return json.loads(urllib.request.urlopen(req, timeout=300).read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def trim(o):
    if isinstance(o, dict):
        return {k: ("[[4x4]]" if k == "matrix" else trim(v)) for k, v in o.items()}
    return [trim(v) for v in o] if isinstance(o, list) else o


out = []


def ex(method, path, body=None, note="", keep=None):
    r = call(method, path, body)
    shown = keep(r) if keep else r
    out.append(f"### `{method} {path}`\n\n{note}\n")
    if body is not None:
        out.append("Request\n\n```json\n" + json.dumps(body) + "\n```\n")
    out.append("Response\n\n```json\n" + json.dumps(trim(shown)) + "\n```\n")
    return r


call("POST", "/api/reset", {})
out.append("""# API reference

Base URL `http://127.0.0.1:8700`. JSON in, JSON out. `GET /api` returns this list in machine-readable form.
Every response below is a real one from the simulator (4x4 matrices shortened to `"[[4x4]]"`); regenerate this file
with `python3 docs/make_api_md.py`.

**Conventions**

- Units: metres, radians, seconds, newtons, newton-metres, kilograms. Fields ending in `_deg` are degrees.
- Arms are `a` and `b`. Joints are 6 numbers: `joints` (rad), `joints_deg`, or `joints_fanuc_deg` (pendant convention, J3 coupled to J2).
- A pose is `pos: [x, y, z]` plus one orientation: `quat_wxyz`, `wpr_deg` (FANUC W, P, R), `matrix` (3x3), or `tool_down: <yaw deg>`.
  In move commands the orientation may be left out: the tool keeps the orientation it has. `frame` is `"world"` (default) or `"base"`.
- Moves return immediately (`id`, `duration`, `starts_in`) and queue behind the arm's current moves. `"wait": true` blocks until the arm is at rest and adds `"wait": {...}`.
- A move's `duration` is the PLAN. What the arm actually does is physics: read it from `/api/state`.
- `"check": false` on a move skips the planner's collision check. The arm will then really hit things.
- Gripper, payload and soft-float commands are **not** queued: they act at once. Wait for the move first.

**Refusals** -- nothing has moved when a command is refused.

| status | `error` | meaning |
|---|---|---|
| 400 | `bad_request`, `bad_joints`, `bad_json`, `bad_speed`, `bad_gripper`, `bad_type` | malformed request |
| 404 | `no_such_arm`, `no_such_part`, `not_found` | |
| 408 | `timeout` | `wait` ran out |
| 409 | `collision` | the planned path overlaps something; `t` and `hits` say when and what |
| 409 | `arm_in_fault` | clear it with `reset_fault` first |
| 409 | `busy` | a two-arm move must start from rest |
| 409 | `not_held` | `move_part` on a part no gripper is squeezing |
| 409 | `fault` | returned by `wait` when an arm faulted; `faults` has the details |
| 422 | `unreachable`, `joint_limit`, `singularity` | the arm cannot do it |

**Faults** -- raised by the arm's own protection from what it MEASURES; the arm stops where it is and ignores moves until `reset_fault`.

| `code` | trips when |
|---|---|
| `collision_detect` | a joint's feedback (disturbance) torque passes 50 % of its motor's limit: it hit something, is pressing on something, is fighting the other arm, or carries much more than it was told |
| `following_error` | a joint is more than 3 degrees from its command: the motor cannot keep up |

**Warnings** (in `state.arms.<arm>.warnings`, no stop): `over_rated_payload` (the wrist measures more than 20 kg), `motor_saturated`.

---

## Reading
""")
ex("GET", "/api/scene", note="The static cell. Enough to rebuild the kinematics and draw the scene.",
   keep=lambda r: {**r, "robot": {**r["robot"], "chain": r["robot"]["chain"][:2] + ["... 6 joints"]},
                   "fixtures": {"table_a": r["fixtures"]["table_a"], "...": "5 more"}})
ex("GET", "/api/state", note="""Everything measured. Poll it (the viewer does, at 30 Hz).

- `joints` are the encoders, `joints_cmd` the command; `following_error_deg` is their difference.
- `joint_torque` (N.m) is what each motor is applying, `motor_load` the same as a fraction of its limit, `disturbance_torque` the feedback part (what the controller's model did not expect).
- `wrist` is a force/torque sensor at the flange, world frame: what the tool and its load pull on the wrist with. `load_kg` is the weight it measures (gripper included).
- `gripper.squeeze_n` is the measured force of the jaws on the part; `holding` names the part when both jaws press on it.
- `parts.<name>.held_by` / `resting_on` come from the contacts, `touching` lists every other contact of an arm with its force in N.
- `arm_distance`: closest distance between the two arms, `null` beyond 0.3 m. `status`: `idle`, `moving` or `fault`.""",
   keep=lambda r: {**r, "arms": {"a": r["arms"]["a"], "b": "{... same fields}"}, "parts": {"bar": r["parts"]["bar"], "beam": "{...}"}})
ex("GET", "/api/events?since=0", note="The event log after event number `since` (keep `last_event` and pass it next time). Events: `reset`, `move_start`, `move_done`, `gripper`, `payload`, `soft_float`, `grasp`, `release`, `landed`, `stop`, `fault`, `fault_reset`.")
out.append("\n## Kinematics\n")
ex("POST", "/api/fk", {"arm": "a", "joints_deg": [20, 10, -15, 30, -70, 45]},
   note="Forward kinematics. Returns the tcp and the flange in world and base frames, and every link's frame in the world.",
   keep=lambda r: {**r, "links": {"base_link": r["links"]["base_link"], "...": "link_1 .. link_6"}})
ex("POST", "/api/ik", {"arm": "a", "pos": [-0.3, 0.2, 1.0], "tool_down": 0},
   note="Inverse kinematics: every solution inside the joint limits, nearest to the arm's current joints first (or to `ref`, 6 rad). `config` is the FANUC-style label (wrist **N**o-flip/**F**lip, elbow **U**p/**D**own, arm fron**T**/**B**ack). `collides`: would that pose overlap something, with the other arm where it is now.")
ex("POST", "/api/jacobian", {"arm": "a"},
   note="6x6 Jacobian of the tcp in the arm's base frame at the given joints (default: where the arm is), and Yoshikawa's manipulability (near 0 = near a singularity).",
   keep=lambda r: {**r, "jacobian": [r["jacobian"][0], "... 6 rows"]})
ex("POST", "/api/check", {"a": {"joints_deg": [0, 60, -20, 0, -90, 0]}, "b": {"joints_deg": [0, 60, -20, 0, -90, 0]}},
   note="Would these joint angles overlap anything (geometry only)? An arm left out stays where it is. `hits` are `[what, what, depth m]`.")
out.append("\n## One arm\n")
ex("POST", "/api/arm/a/move_joint", {"joints_deg": [30, 10, -10, 0, -80, 0], "speed": 0.5, "wait": True},
   note="Joint-interpolated move. `speed` is the fraction of the arm's joint speeds and accelerations, 0-1 (default 1).")
ex("POST", "/api/arm/a/move_pose", {"pos": [-0.3, 0.2, 1.0], "tool_down": 0, "speed": 0.5, "wait": True},
   note="Joint-interpolated move that ends with the tcp at a pose (the tool does **not** travel in a straight line). Uses the solution nearest the arm's joints, or the one named by `config` (e.g. `\"NUT\"`).")
ex("POST", "/api/arm/a/move_line", {"pos": [-0.3, 0.2, 0.8], "speed": 0.25, "wait": True},
   note="Straight-line move of the tcp. `speed` in m/s (default 0.5, at most 2.0). The arm stays in its configuration; a line through a singularity is refused with 422 `singularity`.")
ex("POST", "/api/arm/a/move_pose", {"pos": [3.0, 0, 1.0], "tool_down": 0}, note="A refused move, for example out of reach:")
call("POST", "/api/arm/a/move_pose", {"pos": [0, 0, 1.2], "tool_down": 0, "speed": 0.2})
ex("POST", "/api/arm/b/move_pose", {"pos": [0.0, 0.0, 1.2], "tool_down": 0}, note="...or into the other arm's path (arm A is on its way to the same point):")
ex("POST", "/api/arm/a/stop", {}, note="Drop the arm's queued moves; its motors brake it onto the command it had. `POST /api/stop` does it for both.")
call("POST", "/api/reset", {})
call("POST", "/api/arm/a/gripper", {"action": "open"})
call("POST", "/api/arm/a/move_pose", {"pos": [-1.10, 1.0, 0.95], "tool_down": 0, "wait": True})
call("POST", "/api/arm/a/move_line", {"pos": [-1.10, 1.0, 0.775], "wait": True})
ex("POST", "/api/arm/a/gripper", {"action": "close", "force": 500, "wait": True},
   note="Send the jaws to an opening: `width` (m, 0-0.12), or `\"action\": \"open\"` (0.10) / `\"close\"` (0). `force` is the most each jaw may squeeze with, N (default and maximum 500). The jaws are geared together and stop where the part stops them; the part is then held by friction alone (jaw pads 0.9): too little force, too much weight or too hard a move and it slips. Here arm A closes on the 3.4 kg bar:")
ex("POST", "/api/arm/a/payload", {"mass": 3.4},
   note="Tell the arm's controller what it carries, kg at the tcp (a FANUC `PAYLOAD` instruction). Its feedforward then supplies the torque for that weight. Undeclared, the feedback has to hold it: the arm sags more while moving and is nearer a `collision_detect` trip. Set it back to 0 after letting go.")
call("POST", "/api/arm/a/move_line", {"pos": [-1.10, 1.0, 1.0], "wait": True})
ex("GET", "/api/state", note="Holding the bar 150 mm off its middle, the wrist measures its weight and the moment it hangs with:",
   keep=lambda r: {"arms": {"a": {k: r["arms"]["a"][k] for k in ("gripper", "wrist", "load_kg", "payload_declared_kg", "joint_torque", "motor_load", "warnings")}},
                   "parts": {"bar": {k: v for k, v in r["parts"]["bar"].items() if k != "pose"}}})
ex("POST", "/api/arm/a/soft_float", {"enabled": True, "stiffness": 0.1},
   note="Make the arm compliant (FANUC Soft Float): the servo holds position with `stiffness` (0.01-1) of its normal spring and no integrator, while still carrying its own weight and its declared payload. For the **follower** when both arms hold one rigid part. `enabled: false` makes it stiff again from where it is.")
call("POST", "/api/arm/a/soft_float", {"enabled": False})
ex("POST", "/api/arm/a/reset_fault", {}, note="Clear a fault. The arm holds the position it stopped at.")
call("POST", "/api/reset", {})
out.append("\n## Both arms together\n")
ex("POST", "/api/together/move", {"a": {"type": "pose", "pos": [-0.45, 0.55, 0.95], "tool_down": 0},
                                  "b": {"type": "pose", "pos": [0.45, 0.55, 0.95], "tool_down": 0}, "speed": 0.5, "wait": True},
   note="Each arm makes its own move -- `type` is `joint` (default; give joints), `pose` or `line` (give a pose; `line` takes its own `speed` in m/s) -- planned as one: both start and arrive on the same cycle, the slower arm sets the pace. Both arms must be at rest. One arm alone is allowed.")
call("POST", "/api/together/move", {"a": {"type": "line", "pos": [-0.45, 0.55, 0.74]}, "b": {"type": "line", "pos": [0.45, 0.55, 0.74]}, "wait": True})
call("POST", "/api/arm/a/gripper", {"action": "close"})
call("POST", "/api/arm/b/gripper", {"action": "close"})
call("POST", "/api/wait", {})
for p in "ab":
    call("POST", f"/api/arm/{p}/payload", {"mass": 10.35})
ex("POST", "/api/together/move_part", {"part": "beam", "pos": [0.0, 0.55, 1.1], "speed": 0.2, "wait": True},
   note="Move a held part in a straight line to a world pose (orientation: any of the usual forms, or `yaw_deg` for a turn about the vertical; left out, it keeps its orientation). Each gripper's path is the part's path seen from where that gripper holds it. `speed` is the part centre's, m/s (default 0.25). Here both arms have closed on the 20.7 kg billet, both still stiff:")
ex("GET", "/api/state", note="Both hold it. The wrist sensors show how the weight is shared and how hard the arms push on each other through the part (the horizontal force):",
   keep=lambda r: {"parts": {"beam": {k: v for k, v in r["parts"]["beam"].items() if k != "pose"}},
                   "arms": {p: {k: a[k] for k in ("load_kg", "wrist", "motor_load", "disturbance_torque", "soft_float")} for p, a in r["arms"].items()}})
call("POST", "/api/together/move_part", {"part": "beam", "pos": [0.0, 0.0, 1.2], "yaw_deg": 25, "speed": 0.3})
ex("POST", "/api/wait", {"timeout": 60},
   note="Block until the named `arms` (default both), their grippers and the parts are at rest. If an arm faults, 409 with the faults. Here the two STIFF arms were asked to carry the billet across the cell and turn it: they disagree by tenths of a millimetre, each controller pushes to remove its own error, and the force between them climbs until a wrist motor trips. (Put the follower in soft float first and the same move goes through.)")
call("POST", "/api/reset", {})
out.append("\n## Cell\n")
ex("POST", "/api/reset", {}, note="Everything back to the start: arms home, parts on their fixtures, faults, payloads and events cleared. `{\"seed\": n}` rebuilds the world with another draw of the real-world errors, `{\"ideal\": true}` with none.")
ex("GET", "/api/reality", note="The errors in this world: what is true against what the controllers believe (this reference was generated in the ideal world, so all are zero).")
Path(__file__).resolve().parents[1].joinpath("API.md").write_text("\n".join(out))
print("wrote API.md")
