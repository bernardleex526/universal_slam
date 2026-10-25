import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('odom_adapter',Path(__file__).parents[1]/'scripts/odometry_adapter.py')
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

def test_rotating_offset_imu_has_stationary_base():
    p,q,v,w=mod.convert_state([1,0,0],[0,0,0,1],[0,2,0],[0,0,2],[1,0,0],[0,0,0,1])
    np.testing.assert_allclose(p,[0,0,0],atol=1e-9)
    np.testing.assert_allclose(v,[0,0,0],atol=1e-9)
    np.testing.assert_allclose(w,[0,0,2],atol=1e-9)

def test_rotated_imu_velocity_is_expressed_in_base():
    s=2**-.5
    p,q,v,w=mod.convert_state([0,0,0],[0,0,s,s],[1,0,0],[0,0,1],[0,0,0],[0,0,s,s])
    np.testing.assert_allclose(v,[0,1,0],atol=1e-9)
    np.testing.assert_allclose(q,[0,0,0,1],atol=1e-9)

def test_divergence_guard_latches_on_jump_until_reset():
    g=mod.DivergenceGuard(1.0,1.0,0.3)
    assert g.check(0.0,[0,0,0],[0.5,0,0],[0,0,0])
    assert g.check(0.1,[0.05,0,0],[0.5,0,0],[0,0,0])
    assert not g.check(0.2,[5.0,0,0],[0.5,0,0],[0,0,0])
    assert 'jumped' in g.reason
    assert not g.check(0.3,[0.1,0,0],[0.5,0,0],[0,0,0]), 'plausible data must not clear the latch'
    g.reset()
    assert g.check(0.4,[0.1,0,0],[0.5,0,0],[0,0,0])

def test_divergence_guard_rejects_speed():
    assert not mod.DivergenceGuard(1.0,1.0,0.3).check(0.0,[0,0,0],[2.0,0,0],[0,0,0])
    assert not mod.DivergenceGuard(1.0,1.0,0.3).check(0.0,[0,0,0],[0,0,0],[0,0,1.5])



def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def test_gravity_alignment_levels_odom_and_keeps_base_heading():
    """camera_init = IMU attitude at start (FAST-LIO upstream). For any start tilt, IMU mounting
    (upside down included) and yaw, odom must be level with the base at the origin heading +x:
    R_odom_B0 = Ry(pitch) Rx(roll), p_odom_B0 = 0."""
    rng = np.random.default_rng(3)
    for k in range(200):
        roll, pitch, yaw = rng.uniform(-.35, .35), rng.uniform(-.35, .35), rng.uniform(-np.pi, np.pi)
        r_bi = _rpy(*rng.uniform(-np.pi, np.pi, 3)) if k % 2 else _rpy(np.pi, 0, rng.uniform(-np.pi, np.pi))
        t_bi = rng.uniform(-.3, .3, 3)
        q_bi = mod.quaternion(r_bi)
        r_wb0 = _rpy(roll, pitch, yaw)
        g_ci = (r_wb0 @ r_bi).T @ np.array([0, 0, -9.81])           # gravity in the start IMU frame
        r_oc, p_oc = mod.odom_from_gravity(g_ci, t_bi, q_bi)
        p_ci, q_ci, _, _ = mod.convert_state([0, 0, 0], [0, 0, 0, 1], [0, 0, 0], [0, 0, 0], t_bi, q_bi)
        p, q = mod.apply_alignment(r_oc, p_oc, p_ci, q_ci)
        np.testing.assert_allclose(p, 0, atol=1e-9)
        np.testing.assert_allclose(mod.rotation(q), _rpy(roll, pitch, 0), atol=1e-9)
        np.testing.assert_allclose(r_oc @ (-g_ci / 9.81), [0, 0, 1], atol=1e-12)


def test_gravity_alignment_rejects_degenerate_input():
    import pytest
    with pytest.raises(ValueError):
        mod.odom_from_gravity([0, 0, 0], [0, 0, 0], [0, 0, 0, 1])
    with pytest.raises(ValueError):  # base x-axis vertical: no heading
        mod.odom_from_gravity([-9.81, 0, 0], [0, 0, 0], [0, 0, 0, 1])
