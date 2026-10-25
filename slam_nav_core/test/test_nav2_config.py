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
        assert p['obstacle_layer']['observation_sources'].split() == ['pointcloud', 'clear_rays']
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



def test_costmaps_mark_low_and_far_obstacles():
    """Regression: the layer-level height gate (default 0..2 m in the odom frame) dropped every
    point below the IMU start height, and the default obstacle_max_range (2.5 m) dropped
    everything farther. Both were measured on a running costmap."""
    import importlib.util
    spec=importlib.util.spec_from_file_location('nav_launch_gates',CONFIG.parent/'launch/nav_stack.launch.py')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    profile=yaml.safe_load((CONFIG.parents[1]/'universal_slam_bringup/test/fixture_profile.yaml').read_text())
    result=mod.build_nav_parameters(str(CONFIG),profile,False)
    for name in ('local_costmap','global_costmap'):
        p=result[name][name]['ros__parameters']
        for layer_name,source_name in (('obstacle_layer','pointcloud'),('obstacle_layer','clear_rays'),
                                       ('terrain_layer','terrain'),('terrain_layer','clear_rays')):
            layer=p[layer_name]; source=layer[source_name]
            for gate in (layer,source):
                assert gate['min_obstacle_height']<=-100 and gate['max_obstacle_height']>=100
            assert source['obstacle_max_range']>=6.0
            assert source['raytrace_max_range']>=source['obstacle_max_range']
            assert source['sensor_frame']=='uslam/lidar'
        terrain=p['terrain_layer']
        assert terrain['terrain']['topic']=='/uslam/terrain_obstacles'
        assert terrain['terrain']['marking'] is True and terrain['terrain']['clearing'] is False, 'rays to marks must not erase marks'
        for layer in (terrain, p['obstacle_layer']):
            assert layer['clear_rays']['topic']=='/uslam/clear_rays'
            assert layer['clear_rays']['marking'] is False and layer['clear_rays']['clearing'] is True
        assert terrain['observation_sources'].split()==['terrain','clear_rays']
        assert 'terrain_layer' in p['plugins'] and 'negative_layer' not in p['plugins'], 'own layer: obstacle raytracing must not erase terrain'
        assert terrain['footprint_clearing_enabled'] is False, 'standing at an edge must not erase it'
        assert p['plugins'].index('terrain_layer')<p['plugins'].index('inflation_layer')
        assert p['inflation_layer']['inflation_radius']>=p['robot_radius']+0.3
    margin=result['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic']['collision_margin_distance']
    local=result['local_costmap']['local_costmap']['ros__parameters']['robot_radius']
    assert local==profile['navigation']['robot_radius']
    assert abs(result['global_costmap']['global_costmap']['ros__parameters']['robot_radius']-(local+margin+mod.PLANNER_SLACK))<1e-9,         'plan only through gaps the controller enters: planner radius = body + MPPI collision margin + slack'
    assert mod.PLANNER_SLACK>0


def test_cyclonedds_keeps_datagrams_below_mtu():
    """Nav2's obstacle layer subscribes best-effort; multi-fragment datagrams were lost on the
    WSL loopback (0/60 clouds delivered), single-fragment datagrams were not (58/60)."""
    import re
    text=(CONFIG/'cyclonedds_localhost.xml').read_text()
    size=re.search(r'<MaxMessageSize>(\d+)B</MaxMessageSize>',text)
    assert size and int(size.group(1))<=1472


def test_static_layer_only_when_a_map_is_given():
    import importlib.util
    spec=importlib.util.spec_from_file_location('nav_launch_static',CONFIG.parent/'launch/nav_stack.launch.py')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    profile=yaml.safe_load((CONFIG.parents[1]/'universal_slam_bringup/test/fixture_profile.yaml').read_text())
    plain=mod.build_nav_parameters(str(CONFIG),profile,False)
    assert 'static_layer' not in plain['global_costmap']['global_costmap']['ros__parameters']['plugins']
    loc=mod.build_nav_parameters(str(CONFIG),profile,False,'/uslam/map')
    g=loc['global_costmap']['global_costmap']['ros__parameters']
    assert g['plugins'][0]=='static_layer' and g['static_layer']['map_topic']=='/uslam/map'
    assert g['static_layer']['map_subscribe_transient_local'] is True
    assert 'static_layer' not in loc['local_costmap']['local_costmap']['ros__parameters']['plugins']


def test_bt_waits_long_enough_for_servers_under_load():
    p = yaml.safe_load((CONFIG / 'nav2_bt.yaml').read_text())['bt_navigator']['ros__parameters']
    assert p['default_server_timeout'] >= 100, '20 ms aborted whole goals when the CPU was busy'
