import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location('m20pro_bridge_launch', ROOT / 'launch/bridge.launch.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def _profile(name='m20pro.yaml'):
    return yaml.safe_load((ROOT / 'config' / name).read_text(encoding='utf-8'))


def _params(node):
    return {k: v for p in node._Node__parameters if isinstance(p, dict) for k, v in p.items()}


def test_default_profile_is_sensor_only():
    nodes = mod.build_nodes(_profile(), sim=False, control=False)
    assert len(nodes) == 1
    assert nodes[0]._Node__node_executable == 'm20_topic_bridge'


def test_control_rejected_for_unverified_or_simulation():
    with pytest.raises(RuntimeError):
        mod.build_nodes(_profile(), sim=False, control=True)
    verified = dict(_profile(), calibration_verified=True)
    with pytest.raises(RuntimeError):
        mod.build_nodes(verified, sim=True, control=True)


def test_profiles_declare_this_bridge_and_body_frame_imu():
    for name in ('m20pro.yaml', 'm20_official_bag.yaml'):
        cfg = _profile(name)
        assert cfg['bridge'] == {'package': 'm20pro_bridge', 'launch': 'bridge.launch.py'}
        assert cfg['navigation']['base_height'] > 0
    imu = _profile()['extrinsics']['imu']
    assert (imu['x'], imu['y'], imu['z']) == (0.0632, -0.0268, 0.0435), 'official MJCF imu_site'
