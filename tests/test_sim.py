"""The simulator through its API (in process, no HTTP). Everything here is physics: the assertions are about
what motors, friction and contacts do, with tolerances wide enough to hold on different MuJoCo versions.
Run: python -m pytest tests -q"""
import numpy as np
import pytest

from twoarm import robot as R
from twoarm import scene as S
from twoarm.api import Api
from twoarm.sim import DT, Sim

HOME_DEG = [0, 0, 0, 0, -90, 0]


@pytest.fixture()
def api():
    sim = Sim(realtime=False)
    sim.start()
    yield Api(sim)
    sim.stop_thread()


def ok(api, path, **body):
    status, r = api.handle("POST", path, body)
    assert status == 200, r
    return r


def watch(api, what):
    """Poll the state while any arm is moving; return the largest value of what(state)."""
    worst = 0.0
    while True:
        st = api.state()
        worst = max(worst, what(st))
        if all(a["status"] != "moving" for a in st["arms"].values()):
            return worst


# ------------------------------------------------------------------ the planner (what is proposed)
def test_planned_joint_move_keeps_the_arms_speed_and_acceleration_limits(api):
    sim = api.sim
    sim.stop_thread()                                            # plan only: read the commands the controller would send
    goal = [1.2, -0.3, 0.4, 1.0, -0.4, 2.0]
    ok(api, "/api/arm/a/move_joint", joints=goal, check=False)
    Q = np.vstack([R.HOME, sim.arms["a"].buf])
    v = np.abs(np.diff(Q, axis=0)) / DT
    acc = np.abs(np.diff(Q, 2, axis=0)) / DT ** 2
    assert np.allclose(Q[-1], goal)
    assert np.all(v.max(0) <= R.V_MAX * 1.001) and np.all(acc.max(0) <= R.A_MAX * 1.06)
    assert np.any(v.max(0) > R.V_MAX * 0.9)                      # and it does use the speed it has


def test_planned_line_is_straight(api):
    sim = api.sim
    sim.stop_thread()
    a = sim.arms["a"]
    p0, p1 = a.tcp_world()[:3, 3], np.array([-0.5, 0.5, 0.9])
    r = ok(api, "/api/arm/a/move_line", pos=p1.tolist(), speed=1.0, check=False)
    Q = np.vstack([R.HOME, a.buf])
    pts = np.array([a.tcp_world(q)[:3, 3] for q in Q[::5]])
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    off = (pts - p0) - np.outer((pts - p0) @ d, d)
    assert np.linalg.norm(off, axis=1).max() < 1e-4 and np.allclose(a.tcp_world(Q[-1])[:3, 3], p1, atol=1e-6)
    assert np.all((np.abs(np.diff(Q, axis=0)) / DT).max(0) <= R.V_MAX * 1.001)
    assert abs(r["length"] - np.linalg.norm(p1 - p0)) < 1e-4


def test_refusals_change_nothing(api):
    h = api.handle
    assert h("POST", "/api/arm/a/move_pose", {"pos": [3.0, 0, 1.0], "tool_down": 0})[1]["error"] == "unreachable"
    assert h("POST", "/api/arm/a/move_joint", {"joints_deg": [0, 0, 0, 0, 170, 0]})[1]["error"] == "joint_limit"
    assert h("POST", "/api/arm/a/move_pose", {"pos": [-1.05, 1.05, 0.5], "tool_down": 0})[1]["error"] == "collision"   # inside table A
    assert h("POST", "/api/arm/c/move_joint", {"joints": [0] * 6})[0] == 404
    assert h("POST", "/api/arm/a/move_joint", {"joints": [0] * 5})[0] == 400
    assert h("POST", "/api/together/move_part", {"part": "beam", "pos": [0, 0, 1]})[1]["error"] == "not_held"
    assert api.state()["arms"]["a"]["status"] == "idle"


def test_a_move_into_the_other_arms_planned_path_is_refused(api):
    ok(api, "/api/arm/a/move_pose", pos=[0.0, 0.0, 1.2], tool_down=0, speed=0.05)       # A is slow: it gets there long after B would
    status, r = api.handle("POST", "/api/arm/b/move_pose", {"pos": [0.0, 0.0, 1.2], "tool_down": 0})
    assert status == 409 and r["error"] == "collision"
    assert any("a:" in h[0] + h[1] and "b:" in h[0] + h[1] for h in r["hits"])


def test_moving_together_starts_and_ends_on_the_same_cycle(api):
    sim = api.sim
    sim.stop_thread()
    ok(api, "/api/together/move", a={"joints_deg": [40, 20, -10, 0, -60, 0]}, b={"type": "line", "pos": [0.4, -0.3, 1.0]})
    assert len(sim.arms["a"].buf) == len(sim.arms["b"].buf) > 0


# ------------------------------------------------------------------ the motors (what happens)
def test_motors_hold_the_arm_against_gravity_and_track_a_full_speed_move(api):
    st = api.state()["arms"]["a"]
    assert abs(st["joint_torque"][2]) > 50                       # joint 3 is working just to stand still
    ok(api, "/api/arm/a/move_joint", joints=[1.2, -0.3, 0.4, 1.0, -0.4, 2.0], check=False)
    lag = watch(api, lambda s: float(np.max(np.abs(s["arms"]["a"]["following_error_deg"]))))
    ok(api, "/api/wait")
    st = api.state()["arms"]["a"]
    assert 0.02 < lag < 1.5                                      # it does lag its command, by a fraction of a degree
    assert st["fault"] is None and np.allclose(st["joints"], [1.2, -0.3, 0.4, 1.0, -0.4, 2.0], atol=5e-4)
    assert max(st["motor_load"]) < 1.0


def test_a_move_ends_at_its_pose(api):
    ok(api, "/api/arm/b/move_pose", pos=[0.5, -0.4, 1.0], tool_down=30, wait=True)
    st = api.state()["arms"]["b"]
    assert st["status"] == "idle" and np.allclose(st["tcp"]["world"]["pos"], [0.5, -0.4, 1.0], atol=5e-4)


def pick_bar(api, force=None):
    ok(api, "/api/arm/a/gripper", action="open")
    ok(api, "/api/arm/a/move_pose", pos=[-1.10, 1.0, 0.95], tool_down=0, wait=True)
    ok(api, "/api/arm/a/move_line", pos=[-1.10, 1.0, 0.775], wait=True)
    ok(api, "/api/arm/a/gripper", action="close", wait=True, **({"force": force} if force else {}))
    ok(api, "/api/arm/a/payload", mass=3.4)


def test_a_part_is_held_by_friction_and_weighed_by_the_wrist(api):
    pick_bar(api)
    g = api.state()["arms"]["a"]["gripper"]
    assert g["holding"] == "bar" and abs(g["width"] - 0.05) < 0.002 and g["squeeze_n"] > 450
    ok(api, "/api/arm/a/move_line", pos=[-1.10, 1.0, 1.1], wait=True)
    for _ in range(3):
        ok(api, "/api/arm/a/move_line", pos=[-1.10, 0.8, 1.1], speed=1.0, wait=True)    # shake it
        ok(api, "/api/arm/a/move_line", pos=[-1.10, 1.0, 1.1], speed=1.0, wait=True)
    st = api.state()
    bar, a = st["parts"]["bar"], st["arms"]["a"]
    assert bar["held_by"] == ["a"] and abs(bar["pose"]["pos"][2] - 1.1) < 0.003
    assert abs(a["load_kg"] - 5.4) < 0.3                         # 3.4 kg bar + 2 kg gripper, measured
    assert abs(abs(a["wrist"]["torque"][1]) - 3.4 * 9.81 * 0.15) < 0.5   # held 150 mm off its middle: 5 N.m at the wrist


def test_a_part_let_go_in_the_air_falls(api):
    pick_bar(api)
    ok(api, "/api/arm/a/move_line", pos=[-1.10, 1.0, 1.1], wait=True)
    ok(api, "/api/arm/a/move_line", pos=[-0.75, 0.35, 1.1], wait=True)                  # out past the table's edge
    ok(api, "/api/arm/a/gripper", action="open", wait=True)
    ok(api, "/api/wait")
    bar = api.state()["parts"]["bar"]
    assert bar["held_by"] == [] and bar["resting_on"] == "floor" and bar["pose"]["pos"][2] < 0.06


def grip_beam_alone(api, force):
    ok(api, "/api/arm/a/gripper", action="open")
    ok(api, "/api/arm/a/move_pose", pos=[0, 0.55, 0.95], tool_down=0, wait=True)
    ok(api, "/api/arm/a/move_line", pos=[0, 0.55, 0.74], wait=True)
    ok(api, "/api/arm/a/gripper", action="close", force=force, wait=True)


def test_a_weak_grip_slips_and_a_strong_one_holds(api):
    grip_beam_alone(api, 40)                                     # 2 x 0.9 x 40 N of friction against 203 N of weight
    ok(api, "/api/arm/a/move_line", pos=[0, 0.55, 0.95], speed=0.1, wait=True)
    assert api.state()["parts"]["beam"]["pose"]["pos"][2] < 0.76
    ok(api, "/api/reset")
    grip_beam_alone(api, 500)
    ok(api, "/api/arm/a/payload", mass=20.7)
    ok(api, "/api/arm/a/move_line", pos=[0, 0.55, 0.95], speed=0.1, wait=True)
    ok(api, "/api/arm/a/move_line", pos=[0, 0.55, 0.95], wait=True)
    st = api.state()
    assert abs(st["parts"]["beam"]["pose"]["pos"][2] - 0.95) < 0.003
    assert "over_rated_payload" in st["arms"]["a"]["warnings"] and st["arms"]["a"]["load_kg"] > 21


def test_a_declared_payload_is_carried_by_feedforward(api):
    held = {}
    for declared in (0.0, 20.7):
        ok(api, "/api/reset")
        grip_beam_alone(api, 500)
        ok(api, "/api/arm/a/payload", mass=declared)
        ok(api, "/api/arm/a/move_line", pos=[0, 0.55, 0.95], speed=0.2, wait=True)
        held[declared] = abs(api.state()["arms"]["a"]["disturbance_torque"][2])
    assert held[0.0] > 100 and held[20.7] < 0.4 * held[0.0]      # undeclared, joint 3's feedback has to hold the weight


def grip_beam(api):
    for p in "ab":
        ok(api, f"/api/arm/{p}/gripper", action="open")
    x = {"a": -0.45, "b": 0.45}
    ok(api, "/api/together/move", wait=True, **{p: {"type": "pose", "pos": [x[p], 0.55, 0.95], "tool_down": 0} for p in "ab"})
    ok(api, "/api/together/move", wait=True, **{p: {"type": "line", "pos": [x[p], 0.55, 0.74]} for p in "ab"})
    for p in "ab":
        ok(api, f"/api/arm/{p}/gripper", action="close")
    ok(api, "/api/wait")
    assert api.state()["parts"]["beam"]["held_by"] == ["a", "b"]
    for p in "ab":
        ok(api, f"/api/arm/{p}/payload", mass=20.7 / 2)


ROUTE = (([0, 0.55, 1.1], 0), ([0, 0, 1.2], 25), ([0, -0.55, 1.1], 0), ([0, -0.55, 0.743], 0))


def test_two_arms_carry_one_part_with_a_compliant_follower(api):
    grip_beam(api)
    ok(api, "/api/arm/b/soft_float", enabled=True, stiffness=0.1)
    push = 0.0
    for pos, yaw in ROUTE:
        ok(api, "/api/together/move_part", part="beam", pos=pos, yaw_deg=yaw, speed=0.3)
        push = max(push, watch(api, lambda s: max(float(np.hypot(*a["wrist"]["force"][:2])) for a in s["arms"].values())))
        ok(api, "/api/wait")
    st = api.state()
    assert all(a["fault"] is None for a in st["arms"].values())
    assert st["parts"]["beam"]["held_by"] == ["a", "b"] and push < 200           # they push on each other, mildly
    for p in "ab":
        ok(api, f"/api/arm/{p}/gripper", action="open")
    ok(api, "/api/wait")
    beam = api.state()["parts"]["beam"]
    assert beam["resting_on"].startswith("stand_2") and np.allclose(beam["pose"]["pos"], [0, -0.55, 0.74], atol=4e-3)


def test_two_stiff_arms_on_one_part_fight_until_a_motor_trips(api):
    grip_beam(api)
    faults = None
    for pos, yaw in ROUTE:
        if api.handle("POST", "/api/together/move_part", {"part": "beam", "pos": pos, "yaw_deg": yaw, "speed": 0.3})[0] != 200:
            break
        status, r = api.handle("POST", "/api/wait", {"timeout": 30})
        if status == 409:
            faults = r["faults"]
            break
    assert faults and {f["code"] for f in faults.values()} == {"collision_detect"}


def test_an_unchecked_collision_is_felt_by_the_motors(api):
    ok(api, "/api/arm/a/move_pose", pos=[0.0, 0.0, 1.2], tool_down=0, wait=True)
    ok(api, "/api/arm/b/move_pose", pos=[0.0, 0.0, 1.2], tool_down=0, check=False)
    status, r = api.handle("POST", "/api/wait", {"timeout": 30})
    assert status == 409 and "b" in r["faults"] and r["faults"]["b"]["code"] in ("collision_detect", "following_error")
    assert any("a:" in t[0] + t[1] and "b:" in t[0] + t[1] for t in api.state()["touching"])
    for p in r["faults"]:
        ok(api, f"/api/arm/{p}/reset_fault")


def test_fk_and_ik_endpoints_agree(api):
    fk = ok(api, "/api/fk", arm="b", joints_deg=[20, 10, -15, 30, -70, 45])
    ik = ok(api, "/api/ik", arm="b", pos=fk["tcp"]["world"]["pos"], quat_wxyz=fk["tcp"]["world"]["quat_wxyz"],
            ref=np.radians([20, 10, -15, 30, -70, 45]).tolist())
    assert np.allclose(ik["solutions"][0]["joints_deg"], [20, 10, -15, 30, -70, 45], atol=1e-3)
    base = ok(api, "/api/ik", arm="b", frame="base", pos=fk["tcp"]["base"]["pos"], wpr_deg=fk["tcp"]["base"]["wpr_deg"])
    assert len(base["solutions"]) == len(ik["solutions"])
