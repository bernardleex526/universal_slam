#!/usr/bin/env python3
"""M20 Pro bridge entry point.

Bridge launch contract (what the algorithm bringup passes to any robot bridge):
  robot_config   absolute path of the robot profile
  use_sim_time   'true' | 'false'
  enable_control 'true' | 'false'
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def build_nodes(cfg, sim, control):
    if control and (sim or cfg.get('calibration_verified', False) is not True):
        raise RuntimeError('Control forbidden for simulation/unverified calibration')
    topics, tf = cfg.get('topics', {}), cfg.get('tf', {})
    sensor = dict(cfg.get('sensors', {}))
    sensor.update({'use_sim_time': sim, 'lidar_frame': tf.get('lidar_frame', 'uslam/lidar'),
                   'imu_frame': tf.get('imu_frame', 'uslam/imu'),
                   'output_cloud_topic': topics.get('points_topic', '/uslam/points_raw'),
                   'output_imu_topic': topics.get('imu_topic', '/uslam/imu_raw'),
                   'scan_line': int(cfg['lidar']['scan_line'])})
    nodes = [Node(package='m20pro_bridge', executable='m20_topic_bridge', parameters=[sensor])]
    if control:
        net = cfg.get('network', {})
        params = dict(cfg.get('control', {}))
        params.update({k: float(v) for k, v in cfg['velocity'].items()})
        params.update({'enable_control': True, 'robot_ip': str(net['robot_ip']), 'port': int(net['port']),
                       'cmd_frequency_hz': float(net.get('cmd_frequency_hz', 20)),
                       'heartbeat_hz': float(net.get('heartbeat_hz', 1)),
                       'cmd_vel_topic': topics.get('cmd_vel_topic', '/uslam/cmd_vel'),
                       'odom_topic': topics.get('odom_topic', '/uslam/odom'),
                       'odom_frame': tf.get('odom_frame', 'uslam/odom'),
                       'base_frame': tf.get('base_frame', 'uslam/base_link')})
        nodes.append(Node(package='m20pro_bridge', executable='m20_cmd_vel_bridge', parameters=[params]))
    return nodes


def _nodes(context):
    lc = context.launch_configurations
    path = lc.get('robot_config', '') or os.path.join(
        get_package_share_directory('m20pro_bridge'), 'config', 'm20pro.yaml')
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    return build_nodes(cfg, lc.get('use_sim_time', 'false').lower() == 'true',
                       lc.get('enable_control', 'false').lower() == 'true')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
