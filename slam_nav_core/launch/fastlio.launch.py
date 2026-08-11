#!/usr/bin/env python3
# ============================================================
# slam_nav_core/launch/fastlio.launch.py
# 启动 FAST-LIO2 (fastlio_mapping 节点, 来自 fast_lio 包, Ericsii/FAST_LIO_ROS2)
# 输出 /Odometry → remap 为 /odom；TF camera_init→body 动态发布。
#
# 依赖：适配器已提供 /points_raw (PointCloud2, lidar 帧) 与 /imu_raw (Imu, imu 帧)
#       （约定见 robot_adapters/ADAPTER_INTERFACE.md）。
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_nav_core')
    default_params_file = os.path.join(pkg_share, 'config', 'fastlio_params.yaml')

    params_file = LaunchConfiguration('params_file', default=default_params_file)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    fastlio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # 约定输出话题：FAST-LIO /Odometry → /odom（供 Nav2 使用）
            ('/Odometry', '/odom'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='FAST-LIO2 参数文件（slam_nav_core/config/fastlio_params.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟'),

        fastlio_node,
    ])
