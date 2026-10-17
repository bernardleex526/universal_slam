#!/usr/bin/env python3
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def build_nav_parameters(config_dir, profile, sim):
    data={}
    for filename in ('nav2_controller.yaml','nav2_planner.yaml','nav2_bt.yaml','nav2_behaviors.yaml'):
        with open(os.path.join(config_dir,filename),encoding='utf-8') as f:
            data.update(yaml.safe_load(f))
    frames=profile.get('tf',{})
    topics=profile.get('topics',{})
    base=frames.get('base_frame','uslam/base_link')
    odom=frames.get('odom_frame','uslam/odom')
    world=frames.get('map_frame','uslam/map')
    for name, value in data.items():
        if name in ('local_costmap','global_costmap'):
            p=value[name]['ros__parameters']
            p['global_frame']=odom if name=='local_costmap' else world
            p['robot_base_frame']=base
            p['robot_radius']=float(profile.get('navigation',{}).get('robot_radius',.55))
            p['use_sim_time']=sim
            source=p['obstacle_layer']['pointcloud']
            source['topic']=topics.get('obstacles_topic','/uslam/obstacles')
            source['sensor_frame']=frames.get('lidar_frame','uslam/lidar')
            # Input already has ground/self filtering in the LIO world frame.
            source['min_obstacle_height']=-1000.0
            source['max_obstacle_height']=1000.0
        else:
            p=value['ros__parameters']
            p['use_sim_time']=sim
            if 'robot_base_frame' in p: p['robot_base_frame']=base
            if 'global_frame' in p: p['global_frame']=world if name=='bt_navigator' else odom
            if 'odom_topic' in p: p['odom_topic']=topics.get('odom_topic','/uslam/odom')
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
    data=build_nav_parameters(lc['params_dir'],profile,sim)
    with tempfile.NamedTemporaryFile(mode='w',prefix='universal_nav_',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(data,f)
        path=f.name
    # Lifecycle order matters: bt_navigator checks for the behavior action servers on configure.
    names=['controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower']
    packages=['nav2_controller','nav2_planner','nav2_behaviors','nav2_bt_navigator','nav2_waypoint_follower']
    cmd=profile.get('topics',{}).get('cmd_vel_topic','/uslam/cmd_vel')
    actions=[Node(package=p,executable=n,name=n,parameters=[path],
                  remappings=[('/cmd_vel',cmd)]) for p,n in zip(packages,names)]
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
        OpaqueFunction(function=_nodes),
    ])
