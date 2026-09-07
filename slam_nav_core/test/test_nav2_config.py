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


def test_profile_reaches_all_navigation_consumers():
    import importlib.util
    spec=importlib.util.spec_from_file_location('nav_launch',CONFIG.parent/'launch/nav_stack.launch.py')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    profile=yaml.safe_load((CONFIG.parents[1]/'robot_configs/config/m20_official_bag.yaml').read_text())
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

