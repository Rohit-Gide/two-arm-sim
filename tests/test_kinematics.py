"""The kinematics library against independent references. Run: python -m pytest tests -q"""
import math

import numpy as np
import pytest

from twoarm import robot as R
from twoarm import scene as S

RNG = np.random.default_rng(0)
QS = [RNG.uniform(R.Q_MIN, R.Q_MAX) for _ in range(500)]


def test_fk_matches_mujoco_reading_the_urdf():
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_path(str(S.ROBOT_DIR / "m20ia.urdf"))
    d = mujoco.MjData(m)
    for q in QS:
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        F = R.link_frames(q)[-1]
        b = m.body("link_6").id
        assert np.allclose(F[:3, 3], d.xpos[b], atol=1e-9)
        assert np.allclose(F[:3, :3], d.xmat[b].reshape(3, 3), atol=1e-9)


def test_home_pose_is_the_data_sheet_geometry():
    T = R.fk(R.HOME)                                   # forearm level, tool down: 150 + 835 mm out, 525 + 790 + 250 - 100 mm up
    assert np.allclose(T[:3, 3], [0.985, 0.0, 1.465], atol=1e-9)
    assert np.allclose(T[:3, 2], [0, 0, -1], atol=1e-9)


def test_every_ik_solution_reproduces_the_pose_and_one_is_the_original():
    for q in QS:
        T = R.fk(q, 0.2)
        sols = R.ik_all(T, 0.2)
        assert sols
        for s in sols:
            assert np.allclose(R.fk(s, 0.2), T, atol=1e-7)
        wrapped = [np.max(np.abs((s - q + math.pi) % (2 * math.pi) - math.pi)) for s in sols]
        assert min(wrapped) < 1e-6


def test_ik_respects_limits_and_orders_by_distance():
    for q in QS[:200]:
        sols = R.ik(R.fk(q, 0.2), q, 0.2)
        assert np.allclose(sols[0], q, atol=1e-6)
        for s in sols:
            assert np.all(s >= R.Q_MIN - 1e-9) and np.all(s <= R.Q_MAX + 1e-9)
        assert len({R.config_of(s) for s in sols}) == len(sols)


def test_out_of_reach_has_no_solution():
    assert R.ik(R.T_of(R.tool_down(), (2.5, 0, 1.0))) == []


def test_jacobian_matches_finite_differences():
    for q in QS[:50]:
        J = R.jacobian(q, 0.2)
        T0 = R.fk(q, 0.2)
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            T1 = R.fk(q + dq, 0.2)
            assert np.allclose((T1[:3, 3] - T0[:3, 3]) / 1e-6, J[:3, i], atol=1e-4)
            W = (T1[:3, :3] @ T0[:3, :3].T - np.eye(3)) / 1e-6
            assert np.allclose([W[2, 1], W[0, 2], W[1, 0]], J[3:, i], atol=1e-4)


def test_rotation_conversions_round_trip():
    for _ in range(200):
        Rm = R.quat_to_R(RNG.normal(size=4))
        assert np.allclose(R.quat_to_R(R.R_to_quat(Rm)), Rm, atol=1e-9)
        assert np.allclose(R.wpr_to_R(*R.R_to_wpr(Rm)), Rm, atol=1e-9)
    q = QS[0]
    assert np.allclose(R.from_fanuc_deg(R.to_fanuc_deg(q)), q)
