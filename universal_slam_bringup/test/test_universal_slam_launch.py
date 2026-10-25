#!/usr/bin/env python3
"""Contract tests for the robot-agnostic bringup (no robot/bridge package required)."""
import importlib.util
import math
import os
import re
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).parent
ROOT = HERE.parents[1]
FIXTURE = str(HERE / 'fixture_profile.yaml')


def _load_module(name, relpath):
    spec = importlib.util.spec_from_file_location(name, ROOT / relpath)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


bringup = _load_module('universal_slam_launch_under_test', 'universal_slam_bringup/launch/universal_slam.launch.py')
fastlio = _load_module('fastlio_launch_for_bringup_test', 'slam_nav_core/launch/fastlio.launch.py')


def _ctx(**kwargs):
    from launch.launch_context import LaunchContext
    ctx = LaunchContext()
    ctx.launch_configurations.update(kwargs)
    return ctx


def _close(a, b, eps=1e-9):
    return all(abs(x - y) < eps for x, y in zip(a, b))


def test_generate_launch_description_ok():
    assert list(bringup.generate_launch_description().entities)


def test_algorithm_packages_reference_no_robot_package():
    """Loose coupling: porting to a new robot must only require a new bridge package."""
    banned = re.compile(r'm20|sw01|adapter_|robot_configs', re.IGNORECASE)
    for pkg in ('slam_nav_core', 'universal_slam_bringup', 'uslam_backend'):
        for path in (ROOT / pkg).rglob('*'):
            if path.is_file() and path.suffix in ('.py', '.xml', '.txt', '.yaml') and path.parent.name != 'test':
                hits = [l for l in path.read_text(encoding='utf-8').splitlines() if banned.search(l)]
                assert not hits, f'{path.relative_to(ROOT)} references a robot: {hits[:2]}'


def test_robot_config_is_required():
    with pytest.raises(RuntimeError, match='robot_config'):
        bringup._make_runtime(_ctx())


def test_bridge_is_resolved_from_profile_not_hardcoded():
    acts = bringup._make_bridge(_ctx(robot_config=FIXTURE, use_bridge='false'))
    from launch.actions import LogInfo
    assert len(acts) == 1 and isinstance(acts[0], LogInfo)
    cfg = bringup.load_profile(FIXTURE)
    with pytest.raises(Exception, match='fixture_bridge'):
        bringup.bridge_launch_path(cfg)
    for bad in ({}, {'bridge': {'package': 'p'}}, {'bridge': {'package': 'p', 'launch': '../x.py'}}):
        with pytest.raises(RuntimeError):
            bringup.bridge_launch_path(bad)


def test_fastlio_injected_values_reach_node_as_native_types():
    cfg = bringup.load_profile(FIXTURE)
    lidar_cfg = cfg['lidar']
    actions = bringup._make_fastlio_launch(_ctx(robot_config=FIXTURE, use_sim_time='false'))
    args = dict(actions[0].launch_arguments)
    assert args['scan_line'] == str(int(lidar_cfg['scan_line']))
    assert args['blind'] == str(float(lidar_cfg['blind']))
    params = fastlio.build_fastlio_parameters(args)
    assert type(params['preprocess.scan_line']) is int
    assert type(params['preprocess.lidar_type']) is int
    assert type(params['preprocess.timestamp_unit']) is int
    assert type(params['preprocess.blind']) is float
    assert type(params['use_sim_time']) is bool
    t = [float(v) for v in args['extrinsic_T'].split(',')]
    assert _close(t, (0.37435, 0.0, 0.0205), 1e-9)
    assert all(type(v) is float for v in params['mapping.extrinsic_T'])
    assert all(type(v) is float for v in params['mapping.extrinsic_R'])


def test_back_end_owns_map_to_odom_and_localization_needs_a_map(tmp_path):
    cfg = bringup.load_profile(FIXTURE)
    names = [d['name'] for d in bringup.build_static_tf_descriptions(cfg, static_map_odom=False)]
    assert 'map_to_odom' not in names, 'the back end is the only map->odom publisher'
    params = bringup.backend_parameters(cfg, 'mapping', str(tmp_path / 'map'), False)
    assert params['map_frame'] == 'uslam/map' and params['odom_frame'] == 'uslam/odom'
    with pytest.raises(RuntimeError, match='saved map'):
        bringup.backend_parameters(cfg, 'localization', str(tmp_path / 'missing'), False)
    with pytest.raises(RuntimeError, match='absolute'):
        bringup.backend_parameters(cfg, 'mapping', 'relative/dir', False)
    with pytest.raises(RuntimeError, match='slam_mode'):
        bringup.backend_parameters(cfg, 'slam', str(tmp_path), False)


def test_static_tf_descriptions_numeric_strings():
    descs = bringup.build_static_tf_descriptions(bringup.load_profile(FIXTURE))
    assert len(descs) == 3
    for d in descs:
        assert all(math.isfinite(float(v)) for v in d['args'][:7])
    assert descs[0]['args'][-2:] == ['uslam/map', 'uslam/odom']
    assert descs[1]['args'][-2:] == ['uslam/base_link', 'uslam/imu']
    assert descs[2]['args'][-2:] == ['uslam/base_link', 'uslam/lidar']
    assert all(d['args'][-1] != 'uslam/base_link' for d in descs)


def test_navigation_and_control_reject_unverified_profile():
    for settings in ({'use_nav': 'true'}, {'enable_control': 'true', 'use_bridge': 'true'}):
        with pytest.raises(RuntimeError, match='calibration'):
            bringup._make_runtime(_ctx(robot_config=FIXTURE, **settings))


def test_navigation_requires_base_height(tmp_path):
    import yaml
    cfg = bringup.load_profile(FIXTURE)
    cfg['calibration_verified'] = True
    cfg['navigation'] = {'robot_radius': 0.55}
    path = tmp_path / 'p.yaml'
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(RuntimeError, match='base_height'):
        bringup._make_runtime(_ctx(robot_config=str(path), use_nav='true'))


def test_rpy_to_quat_known_angles():
    q = bringup._rpy_to_quat
    assert _close(q(0, 0, 0), (0, 0, 0, 1))
    s = math.sin(math.radians(45.0))
    assert _close(q(0, 0, math.radians(90.0)), (0, 0, s, s))
    assert _close(q(math.radians(90.0), 0, 0), (s, 0, 0, s))
    assert _close(q(0, math.radians(90.0), 0), (0, s, 0, s))
    qq = q(math.radians(10.0), math.radians(-20.0), math.radians(35.0))
    assert abs(sum(v * v for v in qq) - 1.0) < 1e-12


def test_xyz_quat_args_uses_rpy():
    args = bringup._xyz_quat_args({'x': 1.0, 'y': 2.0, 'z': 3.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 90.0})
    s = math.sin(math.radians(45.0))
    assert [float(v) for v in args[:3]] == [1.0, 2.0, 3.0]
    assert _close([float(v) for v in args[3:7]], (0, 0, s, s))


def test_lidar_to_imu_extrinsic_identity_and_translation():
    ext = {'x': 0.1, 'y': 0.2, 'z': 0.3, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    t, r = bringup.lidar_to_imu_extrinsic(ext, dict(ext))
    assert _close(t, (0, 0, 0), 1e-12)
    assert _close(r, (1, 0, 0, 0, 1, 0, 0, 0, 1), 1e-12)


def test_lidar_to_imu_extrinsic_rotation():
    imu = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 90.0}
    lidar = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    t, r = bringup.lidar_to_imu_extrinsic(imu, lidar)
    assert _close(t, (0, 0, 0), 1e-12)
    assert _close(r, (0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0), 1e-12)


def test_tf_inverse_preserves_arbitrary_rigid_transform():
    xyz = (1.5, -2., .5)
    quat = bringup._rpy_to_quat(.2, -.4, 1.1)
    t, q = bringup._tf_compose(*bringup._tf_inverse(xyz, quat), xyz, quat)
    assert _close(t, (0, 0, 0), 1e-9)
    assert _close(q[:3], (0, 0, 0), 1e-9) and abs(abs(q[3]) - 1) < 1e-9


def _adapter_params(ctx):
    acts = bringup._make_runtime(ctx)
    return {k: v for p in acts[0]._Node__parameters if isinstance(p, dict) for k, v in p.items()}


def _key(k):
    return ''.join(getattr(x, 'text', str(x)) for x in k) if isinstance(k, tuple) else str(k)


def test_odometry_guard_is_not_derived_from_nav_limits(tmp_path):
    """Regression: 3x the Nav2 limits (1.5 m/s on M20) latched correct odometry while the robot
    was driven at 1.5 m/s during manual mapping."""
    import yaml
    params = {_key(k): v for k, v in _adapter_params(_ctx(robot_config=FIXTURE)).items()}
    assert params['max_linear_speed'] == 5.0 and params['max_angular_speed'] == 6.0
    cfg = bringup.load_profile(FIXTURE)
    cfg['velocity'] = {'vx_max': 0.05, 'vy_max': 0.05, 'wz_max': 0.05}
    cfg['odometry_guard'] = {'max_linear_speed': 3.5}
    path = tmp_path / 'p.yaml'
    path.write_text(yaml.safe_dump(cfg))
    params = {_key(k): v for k, v in _adapter_params(_ctx(robot_config=str(path))).items()}
    assert params['max_linear_speed'] == 3.5 and params['max_angular_speed'] == 6.0
    cfg['odometry_guard'] = {'max_linear_speed': -1}
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(RuntimeError, match='odometry_guard'):
        bringup._make_runtime(_ctx(robot_config=str(path)))


def test_static_map_only_in_localization_with_a_saved_grid(tmp_path):
    (tmp_path / 'map2d.yaml').write_text('image: map2d.pgm\n')
    lc = {'use_backend': 'true', 'slam_mode': 'localization', 'map_dir': str(tmp_path)}
    assert bringup.static_map_path(lc) == str(tmp_path / 'map2d.yaml')
    assert bringup.static_map_path(dict(lc, slam_mode='mapping')) == ''
    assert bringup.static_map_path(dict(lc, use_backend='false')) == ''
    assert bringup.static_map_path(dict(lc, map_dir=str(tmp_path / 'none'))) == ''


def test_fastlio_voxel_map_off_when_back_end_saves():
    args = dict(bringup._make_fastlio_launch(_ctx(robot_config=FIXTURE, use_backend='true'))[0].launch_arguments)
    assert args['pcd_save_en'] == 'false'
    assert fastlio.build_fastlio_parameters(args)['pcd_save.pcd_save_en'] is False
    args = dict(bringup._make_fastlio_launch(_ctx(robot_config=FIXTURE, use_backend='false'))[0].launch_arguments)
    assert fastlio.build_fastlio_parameters(args)['pcd_save.pcd_save_en'] is True


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
