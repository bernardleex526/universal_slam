#!/usr/bin/env python3
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


PLANNER_SLACK=0.05  # m beyond the MPPI collision margin: room for sampled trajectories around the path


def build_nav_parameters(config_dir, profile, sim, static_map_topic=''):
    data={}
    for filename in ('nav2_controller.yaml','nav2_planner.yaml','nav2_bt.yaml','nav2_behaviors.yaml'):
        with open(os.path.join(config_dir,filename),encoding='utf-8') as f:
            data.update(yaml.safe_load(f))
    frames=profile.get('tf',{})
    topics=profile.get('topics',{})
    base=frames.get('base_frame','uslam/base_link')
    odom=frames.get('odom_frame','uslam/odom')
    world=frames.get('map_frame','uslam/map')
    radius=float(profile.get('navigation',{}).get('robot_radius',.55))
    # 全局规划按控制器要求的净空规划：MPPI 重罚离障碍不足 collision_margin_distance 的轨迹点，只按机身
    # 半径规划会选出控制器不肯进入的窄缝（中心线就在临界区），机器人在缝口原地卡死，而不是绕行。
    # 只加碰撞裕量仍会留下中心线贴着临界区的缝（闭环 4 次中 2 次冻结），再给 MPPI 采样留余量
    margin=float(data['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic']['collision_margin_distance'])+PLANNER_SLACK
    for name, value in data.items():
        if name in ('local_costmap','global_costmap'):
            p=value[name]['ros__parameters']
            p['global_frame']=odom if name=='local_costmap' else world
            p['robot_base_frame']=base
            p['robot_radius']=radius if name=='local_costmap' else radius+margin
            p['use_sim_time']=sim
            for layer_name,source_name,topic,default in (
                    ('obstacle_layer','pointcloud','obstacles_topic','/uslam/obstacles'),
                    ('obstacle_layer','clear_rays','clear_rays_topic','/uslam/clear_rays'),
                    ('terrain_layer','terrain','terrain_obstacles_topic','/uslam/terrain_obstacles'),
                    ('terrain_layer','clear_rays','clear_rays_topic','/uslam/clear_rays')):
                layer=p[layer_name]
                source=layer[source_name]
                source['topic']=topics.get(topic,default)
                source['sensor_frame']=frames.get('lidar_frame','uslam/lidar')
                # Input already has terrain/self filtering in the gravity-aligned odom frame.
                # Both gates must be opened: the layer-level one (default 0..2 m in the costmap
                # frame) otherwise drops everything below the odom origin, i.e. the base start height.
                for gate in (source, layer):
                    gate['min_obstacle_height']=-1000.0
                    gate['max_obstacle_height']=1000.0
            inflation=p['inflation_layer']
            inflation['inflation_radius']=max(float(inflation['inflation_radius']),p['robot_radius']+.3)
        else:
            p=value['ros__parameters']
            p['use_sim_time']=sim
            if 'robot_base_frame' in p: p['robot_base_frame']=base
            if 'global_frame' in p: p['global_frame']=world if name=='bt_navigator' else odom
            if 'odom_topic' in p: p['odom_topic']=topics.get('odom_topic','/uslam/odom')
    if static_map_topic:
        # Saved grid of the localized map: the global planner sees walls/doors beyond sensor range.
        # The live layers still mark and clear on top; moved furniture stays in the static layer.
        g=data['global_costmap']['global_costmap']['ros__parameters']
        g['plugins']=['static_layer']+[x for x in g['plugins'] if x!='static_layer']
        g['static_layer']={'plugin':'nav2_costmap_2d::StaticLayer','map_topic':static_map_topic,
                           'map_subscribe_transient_local':True,'subscribe_to_updates':False}
    limits=profile.get('velocity',{'vx_max':.2,'vy_max':.2,'wz_max':.3})
    follow=data['controller_server']['ros__parameters']['FollowPath']
    for key in ('vx_max','vy_max','wz_max'): follow[key]=float(limits[key])
    follow['vx_min']=-float(limits['vx_max'])
    data['behavior_server']['ros__parameters']['max_rotational_vel']=float(limits['wz_max'])
    data['behavior_server']['ros__parameters']['min_rotational_vel']=min(.1,float(limits['wz_max']))
    return data


def _nodes(context):
    lc=context.launch_configurations
    sim=lc.get('use_sim_time','false').lower()=='true'
    profile={}
    if lc.get('robot_config',''):
        with open(lc['robot_config'],encoding='utf-8') as f: profile=yaml.safe_load(f)
    if profile.get('calibration_verified') is not True:
        raise RuntimeError('Nav2 requires a verified robot profile, including direct launch')
    static_map=lc.get('static_map','')
    if static_map and not os.path.isfile(static_map):
        raise RuntimeError('static_map yaml not found: '+static_map)
    map_topic='/uslam/map' if static_map else ''
    data=build_nav_parameters(lc['params_dir'],profile,sim,map_topic)
    with tempfile.NamedTemporaryFile(mode='w',prefix='universal_nav_',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(data,f)
        path=f.name
    # Lifecycle order matters: bt_navigator checks for the behavior action servers on configure.
    names=['controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower']
    packages=['nav2_controller','nav2_planner','nav2_behaviors','nav2_bt_navigator','nav2_waypoint_follower']
    cmd=profile.get('topics',{}).get('cmd_vel_topic','/uslam/cmd_vel')
    actions=[Node(package=p,executable=n,name=n,parameters=[path],
                  remappings=[('/cmd_vel',cmd)]) for p,n in zip(packages,names)]
    if static_map:
        actions.append(Node(package='nav2_map_server',executable='map_server',name='map_server',
            parameters=[{'use_sim_time':sim,'yaml_filename':static_map,'topic_name':map_topic,
                         'frame_id':profile.get('tf',{}).get('map_frame','uslam/map')}]))
        names=['map_server']+names
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_manager_navigation',parameters=[{'use_sim_time':sim,
        'autostart':lc.get('autostart','true').lower()=='true','node_names':names}]))
    def cleanup(_):
        if os.path.exists(path): os.unlink(path)
        return []
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_dir',default_value=os.path.join(get_package_share_directory('slam_nav_core'),'config')),
        DeclareLaunchArgument('robot_config',default_value=''),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('autostart',default_value='true'),
        DeclareLaunchArgument('static_map',default_value='',description='map_server yaml for the global static layer'),
        OpaqueFunction(function=_nodes),
    ])
