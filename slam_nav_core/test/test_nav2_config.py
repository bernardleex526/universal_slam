from pathlib import Path
import yaml

CONFIG = Path(__file__).parents[1] / 'config'

def test_costmaps_are_addressed_to_their_ros_nodes():
    for filename, name in [('nav2_controller.yaml', 'local_costmap'),
                           ('nav2_planner.yaml', 'global_costmap')]:
        data = yaml.safe_load((CONFIG / filename).read_text())
        assert name in data, 'costmap must be a ROS node, not a server parameter'
        p = data[name][name]['ros__parameters']
        assert 'static_layer' not in p['plugins']
        assert p['rolling_window'] is True
        assert isinstance(p['width'], int) and isinstance(p['height'], int)
        assert p['obstacle_layer']['observation_sources'] == 'pointcloud'
        assert p['robot_radius'] >= 0.5


def test_global_planner_can_plan_into_unseen_space():
    p = yaml.safe_load((CONFIG / 'nav2_planner.yaml').read_text())['planner_server']['ros__parameters']
    assert p['GridBased']['allow_unknown'] is True, 'live-only costmap: goals beyond sensor range must be plannable'


def _plugin_strings(node):
    if isinstance(node, dict):
        for key, value in node.items():
            if key == 'plugin' and isinstance(value, str):
                yield value
            else:
                yield from _plugin_strings(value)
    elif isinstance(node, list):
        for value in node:
            yield from _plugin_strings(value)


def test_behavior_names_match_default_bt_action_servers():
    p = yaml.safe_load((CONFIG / 'nav2_behaviors.yaml').read_text())['behavior_server']['ros__parameters']
    assert {'spin', 'backup', 'wait'} <= set(p['behavior_plugins'])
    import re
    source = (CONFIG.parent / 'launch/nav_stack.launch.py').read_text()
    names = re.search(r"names=\[([^\]]*)\]", source).group(1)
    assert names.index('behavior_server') < names.index('bt_navigator'), 'behaviors must activate before the BT'


def test_every_configured_plugin_resolves_in_pluginlib():
    """pluginlib looks a class up by its declared name if one exists, otherwise by its C++ type."""
    import re
    import pytest
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    lookup = set()
    for package in ('nav2_behaviors', 'nav2_controller', 'nav2_costmap_2d', 'nav2_mppi_controller',
                    'nav2_smac_planner', 'nav2_waypoint_follower'):
        try:
            share = Path(get_package_share_directory(package))
        except PackageNotFoundError:
            pytest.skip(package + ' not installed')
        for xml in share.rglob('*.xml'):
            text = xml.read_text(errors='ignore')
            for cls in re.findall(r'<class\b[^>]*>', text):
                name = re.search(r'\bname="([^"]+)"', cls)
                kind = re.search(r'\btype="([^"]+)"', cls)
                lookup.add(name.group(1) if name else kind.group(1) if kind else None)
    for filename in ('nav2_controller.yaml', 'nav2_planner.yaml', 'nav2_bt.yaml', 'nav2_behaviors.yaml'):
        for plugin in _plugin_strings(yaml.safe_load((CONFIG / filename).read_text())):
            assert plugin in lookup, f'{filename}: plugin "{plugin}" is not loadable by pluginlib'


def test_profile_reaches_all_navigation_consumers():
    import importlib.util
    spec=importlib.util.spec_from_file_location('nav_launch',CONFIG.parent/'launch/nav_stack.launch.py')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    profile=yaml.safe_load((CONFIG.parents[1]/'universal_slam_bringup/test/fixture_profile.yaml').read_text())
    result=mod.build_nav_parameters(str(CONFIG),profile,True)
    follow=result['controller_server']['ros__parameters']['FollowPath']
    assert follow['vx_max']==.2 and follow['wz_max']==.3
    assert result['behavior_server']['ros__parameters']['max_rotational_vel']==.3
    for name,world in [('local_costmap','uslam/odom'),('global_costmap','uslam/map')]:
        p=result[name][name]['ros__parameters']
        assert p['use_sim_time'] is True
        assert p['global_frame']==world and p['robot_base_frame']=='uslam/base_link'
        assert p['obstacle_layer']['pointcloud']['topic']=='/uslam/obstacles'
        assert p['obstacle_layer']['pointcloud']['sensor_frame']=='uslam/lidar'

