# API reference

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

### `GET /api/scene`

The static cell. Enough to rebuild the kinematics and draw the scene.

Response

```json
{"robot": {"name": "FANUC M-20iA", "links": ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6"], "chain": [{"joint": "joint_1", "origin": [0.0, 0.0, 0.525], "axis": [0, 0, 1]}, {"joint": "joint_2", "origin": [0.15, 0.0, 0.0], "axis": [0, 1, 0]}, "... 6 joints"], "q_min": [-2.96706, -1.745329, -3.228859, -3.490658, -2.443461, -4.712389], "q_max": [2.96706, 2.792527, 4.766843, 3.490658, 2.443461, 4.712389], "v_max": [3.403392, 3.054326, 3.141593, 6.283185, 6.283185, 9.599311], "a_max": [13.613568, 12.217304, 12.566372, 25.13274, 25.13274, 38.397244], "home": [0.0, 0.0, 0.0, 0.0, -1.5707963267948966, 0.0], "payload_kg": 20.0, "reach_m": 1.811, "repeatability_m": 8e-05, "opw": {"a1": 0.15, "a2": -0.25, "b": 0.0, "c1": 0.525, "c2": 0.79, "c3": 0.835, "c4": 0.1}, "flange_to_tool0": [[6.123233995736766e-17, -1.2246467991473532e-16, 1.0], [0.0, -1.0, -1.2246467991473532e-16], [1.0, 7.498798913309288e-33, -6.123233995736766e-17]], "mesh_url": "/robot/meshes/visual/{link}.stl", "colors": {"base_link": "#474747", "link_6": "#262626", "default": "#f5c221"}}, "arms": {"a": {"base": {"pos": [-1.25, 0.0, 0.0], "yaw": 0.0}}, "b": {"base": {"pos": [1.25, 0.0, 0.0], "yaw": 3.141592653589793}}}, "gripper": {"tcp_z": 0.2, "stroke": 0.12, "jaw_speed": 0.15, "mass": 2.0, "body": {"size": [0.08, 0.22, 0.1], "z": 0.07}, "finger": {"size": [0.04, 0.02, 0.1], "z": 0.17}}, "fixtures": {"table_a": {"pos": [-1.05, 1.05, 0.375], "size": [0.9, 0.6, 0.75], "color": "#8a9099"}, "...": "5 more"}, "parts": {"bar": {"size": [0.5, 0.05, 0.05], "mass": 3.4, "color": "#c9ced6"}, "beam": {"size": [1.2, 0.08, 0.08], "mass": 20.7, "color": "#d98a3a"}}, "frames": {"world": "metres, z up, floor z=0", "base": "each arm's base_link on the floor, x forward", "tcp": "tool0 moved tcp_z along its z; jaws close along tcp y"}}
```

### `GET /api/state`

Everything measured. Poll it (the viewer does, at 30 Hz).

- `joints` are the encoders, `joints_cmd` the command; `following_error_deg` is their difference.
- `joint_torque` (N.m) is what each motor is applying, `motor_load` the same as a fraction of its limit, `disturbance_torque` the feedback part (what the controller's model did not expect).
- `wrist` is a force/torque sensor at the flange, world frame: what the tool and its load pull on the wrist with. `load_kg` is the weight it measures (gripper included).
- `gripper.squeeze_n` is the measured force of the jaws on the part; `holding` names the part when both jaws press on it.
- `parts.<name>.held_by` / `resting_on` come from the contacts, `touching` lists every other contact of an arm with its force in N.
- `arm_distance`: closest distance between the two arms, `null` beyond 0.3 m. `status`: `idle`, `moving` or `fault`.

Response

```json
{"t": 0.012, "arms": {"a": {"status": "idle", "fault": null, "warnings": [], "joints": [-0.0, -0.0, 0.0, -0.0, -1.570796, 0.0], "joints_deg": [-0.0, -0.0, 0.0, -0.0, -90.0, 0.0], "joints_fanuc_deg": [-0.0, -0.0, 0.0, -0.0, -90.0, 0.0], "joints_cmd": [0.0, 0.0, 0.0, 0.0, -1.570796, 0.0], "joint_vel": [-0.0, -0.0, 0.0, -0.0, 0.0, -0.0], "following_error_deg": [0.0, 0.0, -0.0, 0.0, 0.0, -0.0], "joint_torque": [0.0, -216.56, 216.69, 9.43, 0.0, 0.0], "motor_load": [0.0, 0.045, 0.178, 0.074, 0.0, 0.0], "disturbance_torque": [0.0, 0.0, -0.0, 0.0, 0.0, 0.0], "config": "NUT", "tcp": {"world": {"pos": [-0.265, -0.0, 1.265], "quat_wxyz": [0.0, -1.0, 0.0, -0.0], "wpr_deg": [-180.0, -0.0, -0.0], "matrix": "[[4x4]]"}, "base": {"pos": [0.985, -0.0, 1.265], "quat_wxyz": [0.0, -1.0, 0.0, -0.0], "wpr_deg": [-180.0, -0.0, -0.0], "matrix": "[[4x4]]"}}, "tcp_speed": 0.0, "manipulability": 0.64976, "gripper": {"width": 0.1, "target": 0.1, "force_limit_n": 500.0, "squeeze_n": 0.0, "holding": null}, "wrist": {"force": [-0.0, -0.01, -17.8], "torque": [-0.0, 0.0, -0.0]}, "load_kg": 2.0, "payload_kg": 20.0, "payload_declared_kg": 0.0, "soft_float": null, "queued_moves": 0, "queued_s": 0.0, "last_done_id": 0}, "b": "{... same fields}"}, "parts": {"bar": {"pose": {"pos": [-0.95, 1.0, 0.774996], "quat_wxyz": [1.0, 0.0, 0.0, 0.0], "wpr_deg": [0.0, 0.0, 0.0], "matrix": "[[4x4]]"}, "believed_pose": {"pos": [-0.95, 1.0, 0.775], "quat_wxyz": [1.0, 0.0, 0.0, 0.0], "wpr_deg": [0.0, -0.0, 0.0], "matrix": "[[4x4]]"}, "held_by": [], "resting_on": "table_a", "speed": 0.0, "mass": 3.4, "size": [0.5, 0.05, 0.05]}, "beam": "{...}"}, "arm_distance": null, "touching": [], "last_event": 3}
```

### `GET /api/events?since=0`

The event log after event number `since` (keep `last_event` and pass it next time). Events: `reset`, `move_start`, `move_done`, `gripper`, `payload`, `soft_float`, `grasp`, `release`, `landed`, `stop`, `fault`, `fault_reset`.

Response

```json
{"events": [{"n": 3, "t": 0.0, "arm": null, "event": "reset"}], "last_event": 3}
```


## Kinematics

### `POST /api/fk`

Forward kinematics. Returns the tcp and the flange in world and base frames, and every link's frame in the world.

Request

```json
{"arm": "a", "joints_deg": [20, 10, -15, 30, -70, 45]}
```

Response

```json
{"ok": true, "arm": "a", "tcp": {"world": {"pos": [-0.131088, 0.257251, 0.91206], "quat_wxyz": [0.2243303, 0.760661, 0.602032, -0.0928889], "wpr_deg": [165.4203, 24.2942, 73.5659], "matrix": "[[4x4]]"}, "base": {"pos": [1.118912, 0.257251, 0.91206], "quat_wxyz": [0.2243303, 0.760661, 0.602032, -0.0928889], "wpr_deg": [165.4203, 24.2942, 73.5659], "matrix": "[[4x4]]"}}, "flange": {"world": {"pos": [-0.156847, 0.347875, 1.088479], "quat_wxyz": [0.2243303, 0.760661, 0.602032, -0.0928889], "wpr_deg": [165.4203, 24.2942, 73.5659], "matrix": "[[4x4]]"}, "base": {"pos": [1.093153, 0.347875, 1.088479], "quat_wxyz": [0.2243303, 0.760661, 0.602032, -0.0928889], "wpr_deg": [165.4203, 24.2942, 73.5659], "matrix": "[[4x4]]"}}, "links": {"base_link": {"pos": [-1.25, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0], "wpr_deg": [0.0, -0.0, 0.0], "matrix": "[[4x4]]"}, "...": "link_1 .. link_6"}, "config": "NUT", "within_limits": true}
```

### `POST /api/ik`

Inverse kinematics: every solution inside the joint limits, nearest to the arm's current joints first (or to `ref`, 6 rad). `config` is the FANUC-style label (wrist **N**o-flip/**F**lip, elbow **U**p/**D**own, arm fron**T**/**B**ack). `collides`: would that pose overlap something, with the other arm where it is now.

Request

```json
{"arm": "a", "pos": [-0.3, 0.2, 1.0], "tool_down": 0}
```

Response

```json
{"ok": true, "arm": "a", "reachable": true, "solutions": [{"joints": [0.207496, -0.064215, -0.370464, 0.0, -1.264547, -0.207496], "joints_deg": [11.8887, -3.6793, -21.226, 0.0, -72.4532, -11.8887], "config": "NUT", "collides": false}, {"joints": [0.207496, -0.064215, -0.370464, -3.141593, 1.264547, 2.934096], "joints_deg": [11.8887, -3.6793, -21.226, -180.0, 72.4532, 168.1113], "config": "FUT", "collides": false}, {"joints": [-2.934096, -0.321423, 2.50005, -3.141593, -1.890916, -0.207496], "joints_deg": [-168.1113, -18.4162, 143.2423, -180.0, -108.3415, -11.8887], "config": "NUB", "collides": false}, {"joints": [-2.934096, -0.321423, 2.50005, 0.0, 1.890916, 2.934096], "joints_deg": [-168.1113, -18.4162, 143.2423, 0.0, 108.3415, 168.1113], "config": "FUB", "collides": false}]}
```

### `POST /api/jacobian`

6x6 Jacobian of the tcp in the arm's base frame at the given joints (default: where the arm is), and Yoshikawa's manipulability (near 0 = near a singularity).

Request

```json
{"arm": "a"}
```

Response

```json
{"ok": true, "jacobian": [[0.0, 0.74, 0.05, -0.0, 0.3, 0.0], "... 6 rows"], "manipulability": 0.6497552499974136, "frame": "base", "rows": "vx vy vz wx wy wz"}
```

### `POST /api/check`

Would these joint angles overlap anything (geometry only)? An arm left out stays where it is. `hits` are `[what, what, depth m]`.

Request

```json
{"a": {"joints_deg": [0, 60, -20, 0, -90, 0]}, "b": {"joints_deg": [0, 60, -20, 0, -90, 0]}}
```

Response

```json
{"ok": true, "collides": true, "hits": [["a:link_4", "b:link_4", 0.0185], ["a:link_5", "b:link_5", 0.0053], ["a:link_5", "b:link_5", 0.0051], ["a:link_5", "b:link_5", 0.0052], ["a:link_5", "b:link_5", 0.0053]], "arm_distance": -0.018462308021327256}
```


## One arm

### `POST /api/arm/a/move_joint`

Joint-interpolated move. `speed` is the fraction of the arm's joint speeds and accelerations, 0-1 (default 1).

Request

```json
{"joints_deg": [30, 10, -10, 0, -80, 0], "speed": 0.5, "wait": true}
```

Response

```json
{"ok": true, "id": 1, "duration": 0.604, "starts_in": 0.0, "wait": {"ok": true, "t": 0.628}}
```

### `POST /api/arm/a/move_pose`

Joint-interpolated move that ends with the tcp at a pose (the tool does **not** travel in a straight line). Uses the solution nearest the arm's joints, or the one named by `config` (e.g. `"NUT"`).

Request

```json
{"pos": [-0.3, 0.2, 1.0], "tool_down": 0, "speed": 0.5, "wait": true}
```

Response

```json
{"ok": true, "joints": [0.2074962264352025, -0.06421511563291427, -0.37046428083641203, 0.0, -1.264547161591399, -0.2074962264352025], "config": "NUT", "id": 2, "duration": 0.476, "starts_in": 0.0, "wait": {"ok": true, "t": 1.12}}
```

### `POST /api/arm/a/move_line`

Straight-line move of the tcp. `speed` in m/s (default 0.5, at most 2.0). The arm stays in its configuration; a line through a singularity is refused with 422 `singularity`.

Request

```json
{"pos": [-0.3, 0.2, 0.8], "speed": 0.25, "wait": true}
```

Response

```json
{"ok": true, "length": 0.2, "id": 3, "duration": 0.896, "starts_in": 0.0, "wait": {"ok": true, "t": 2.028}}
```

### `POST /api/arm/a/move_pose`

A refused move, for example out of reach:

Request

```json
{"pos": [3.0, 0, 1.0], "tool_down": 0}
```

Response

```json
{"ok": false, "error": "unreachable", "message": "arm a cannot reach that pose", "pos": [3.0, 0.0, 1.0]}
```

### `POST /api/arm/b/move_pose`

...or into the other arm's path (arm A is on its way to the same point):

Request

```json
{"pos": [0.0, 0.0, 1.2], "tool_down": 0}
```

Response

```json
{"ok": false, "error": "collision", "message": "planned path collides 0.80 s from now: a:link_4 x b:gripper (4 mm deep), a:link_4 x b:gripper (4 mm deep), a:link_4 x b:gripper (4 mm deep)", "t": 0.8, "hits": [["a:link_4", "b:gripper", 0.0041], ["a:link_4", "b:gripper", 0.0041], ["a:link_4", "b:gripper", 0.0041]]}
```

### `POST /api/arm/a/stop`

Drop the arm's queued moves; its motors brake it onto the command it had. `POST /api/stop` does it for both.

Request

```json
{}
```

Response

```json
{"ok": true}
```

### `POST /api/arm/a/gripper`

Send the jaws to an opening: `width` (m, 0-0.12), or `"action": "open"` (0.10) / `"close"` (0). `force` is the most each jaw may squeeze with, N (default and maximum 500). The jaws are geared together and stop where the part stops them; the part is then held by friction alone (jaw pads 0.9): too little force, too much weight or too hard a move and it slips. Here arm A closes on the 3.4 kg bar:

Request

```json
{"action": "close", "force": 500, "wait": true}
```

Response

```json
{"ok": true, "target": 0.0, "force_n": 500.0, "duration": 0.667, "wait": {"ok": true, "t": 1.92}}
```

### `POST /api/arm/a/payload`

Tell the arm's controller what it carries, kg at the tcp (a FANUC `PAYLOAD` instruction). Its feedforward then supplies the torque for that weight. Undeclared, the feedback has to hold it: the arm sags more while moving and is nearer a `collision_detect` trip. Set it back to 0 after letting go.

Request

```json
{"mass": 3.4}
```

Response

```json
{"ok": true, "payload_kg": 3.4}
```

### `GET /api/state`

Holding the bar 150 mm off its middle, the wrist measures its weight and the moment it hangs with:

Response

```json
{"arms": {"a": {"gripper": {"width": 0.04942, "target": 0.0, "force_limit_n": 500.0, "squeeze_n": 500.0, "holding": "bar"}, "wrist": {"force": [-0.02, -0.0, -52.85], "torque": [0.0, 5.0, -0.0]}, "load_kg": 5.39, "payload_declared_kg": 3.4, "joint_torque": [-87.07, -358.26, 296.43, 11.18, -1.43, -0.61], "motor_load": [0.018, 0.075, 0.243, 0.088, 0.012, 0.01], "warnings": []}}, "parts": {"bar": {"believed_pose": {"pos": [-0.949982, 0.999983, 1.000009], "quat_wxyz": [1.0, -2.6e-05, -3.1e-05, -5.3e-06], "wpr_deg": [-0.003, -0.0036, -0.0006], "matrix": "[[4x4]]"}, "held_by": ["a"], "resting_on": null, "speed": 0.0004, "mass": 3.4, "size": [0.5, 0.05, 0.05]}}}
```

### `POST /api/arm/a/soft_float`

Make the arm compliant (FANUC Soft Float): the servo holds position with `stiffness` (0.01-1) of its normal spring and no integrator, while still carrying its own weight and its declared payload. For the **follower** when both arms hold one rigid part. `enabled: false` makes it stiff again from where it is.

Request

```json
{"enabled": true, "stiffness": 0.1}
```

Response

```json
{"ok": true, "soft_float": 0.1}
```

### `POST /api/arm/a/reset_fault`

Clear a fault. The arm holds the position it stopped at.

Request

```json
{}
```

Response

```json
{"ok": true}
```


## Both arms together

### `POST /api/together/move`

Each arm makes its own move -- `type` is `joint` (default; give joints), `pose` or `line` (give a pose; `line` takes its own `speed` in m/s) -- planned as one: both start and arrive on the same cycle, the slower arm sets the pace. Both arms must be at rest. One arm alone is allowed.

Request

```json
{"a": {"type": "pose", "pos": [-0.45, 0.55, 0.95], "tool_down": 0}, "b": {"type": "pose", "pos": [0.45, 0.55, 0.95], "tool_down": 0}, "speed": 0.5, "wait": true}
```

Response

```json
{"ok": true, "moves": {"a": {"id": 1, "duration": 0.824, "starts_in": 0.0}, "b": {"id": 1, "duration": 0.824, "starts_in": 0.0}}, "wait": {"ok": true, "t": 0.856}}
```

### `POST /api/together/move_part`

Move a held part in a straight line to a world pose (orientation: any of the usual forms, or `yaw_deg` for a turn about the vertical; left out, it keeps its orientation). Each gripper's path is the part's path seen from where that gripper holds it. `speed` is the part centre's, m/s (default 0.25). Here both arms have closed on the 20.7 kg billet, both still stiff:

Request

```json
{"part": "beam", "pos": [0.0, 0.55, 1.1], "speed": 0.2, "wait": true}
```

Response

```json
{"ok": true, "length": 0.36, "holders": ["a", "b"], "moves": {"a": {"id": 3, "duration": 1.948, "starts_in": 0.0}, "b": {"id": 3, "duration": 1.948, "starts_in": 0.0}}, "wait": {"ok": true, "t": 4.068}}
```

### `GET /api/state`

Both hold it. The wrist sensors show how the weight is shared and how hard the arms push on each other through the part (the horizontal force):

Response

```json
{"parts": {"beam": {"believed_pose": {"pos": [-1.4e-05, 0.550132, 1.099957], "quat_wxyz": [1.0, 5.19e-05, 3.3e-05, 9.06e-05], "wpr_deg": [0.006, 0.0038, 0.0104], "matrix": "[[4x4]]"}, "held_by": ["a", "b"], "resting_on": null, "speed": 0.0008, "mass": 20.7, "size": [1.2, 0.08, 0.08]}}, "arms": {"a": {"load_kg": 11.56, "wrist": {"force": [-33.52, -1.2, -113.41], "torque": [-0.11, 4.81, -0.67]}, "motor_load": [0.012, 0.035, 0.287, 0.117, 0.038, 0.004], "disturbance_torque": [-53.49, 105.62, 25.68, 5.56, 4.79, 0.28], "soft_float": null}, "b": {"load_kg": 11.76, "wrist": {"force": [33.63, 1.43, -115.36], "torque": [0.15, -5.69, -0.43]}, "motor_load": [0.016, 0.037, 0.29, 0.048, 0.063, 0.004], "disturbance_torque": [73.15, 99.62, 29.67, -3.05, 7.62, -0.27], "soft_float": null}}}
```

### `POST /api/wait`

Block until the named `arms` (default both), their grippers and the parts are at rest. If an arm faults, 409 with the faults. Here the two STIFF arms were asked to carry the billet across the cell and turn it: they disagree by tenths of a millimetre, each controller pushes to remove its own error, and the force between them climbs until a wrist motor trips. (Put the follower in soft float first and the same move goes through.)

Request

```json
{"timeout": 60}
```

Response

```json
{"ok": false, "error": "fault", "message": "arm(s) ['b'] faulted", "faults": {"b": {"code": "collision_detect", "joint": 5, "disturbance_nm": 61.4, "threshold_nm": 61.3}}}
```


## Cell

### `POST /api/reset`

Everything back to the start: arms home, parts on their fixtures, faults, payloads and events cleared. `{"seed": n}` rebuilds the world with another draw of the real-world errors, `{"ideal": true}` with none.

Request

```json
{}
```

Response

```json
{"ok": true}
```

### `GET /api/reality`

The errors in this world: what is true against what the controllers believe (this reference was generated in the ideal world, so all are zero).

Response

```json
{"seed": null, "sigma": {"base_mm": 0.0, "base_yaw_deg": 0.0, "zero_deg": 0.0, "part_mm": 0.0, "part_yaw_deg": 0.0, "mass_frac": 0.0, "friction_frac": 0.0, "pad_mu": 0.0, "clock_ms": 0.0, "flex": 0.0}, "base_error_mm": {"a": [0.0, 0.0, -0.0], "b": [0.0, 0.0, 0.0]}, "base_yaw_error_deg": {"a": 0.0, "b": 0.0}, "joint_zero_error_deg": {"a": [-0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "b": [-0.0, 0.0, -0.0, -0.0, 0.0, -0.0]}, "part_error_mm_deg": {"bar": [0.0, 0.0, 0.0], "beam": [0.0, 0.0, 0.0]}, "pad_friction": 0.9, "clock_offset_ms": 0, "late_arm": "b", "gearbox_flex": false}
```
