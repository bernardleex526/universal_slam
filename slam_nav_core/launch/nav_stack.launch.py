#!/usr/bin/env python3
# ============================================================
# slam_nav_core/launch/nav_stack.launch.py
# Nav2 导航栈（与机型无关）：planner / controller / bt / waypoint / costmap
# 参数文件均指向本包 slam_nav_core/config/。
# 依赖：FAST-LIO2 已提供 camera_init→body 动态 TF；bringup 提供 odom→camera_init 静态桥接，
# 完整 TF 链路：map→odom→camera_init→body→base_link（见 universal_slam.launch.py）。
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_nav_core')
    config_dir = os.path.join(pkg_share, 'config')

    params_dir = LaunchConfiguration('params_dir', default=config_dir)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')

    # 静态/动态 TF 话题透传（配合 bringup 的静态 TF 发布器）
    remappings = [('/tf', '/tf'), ('/tf_static', '/tf_static')]

    # ---- 参数文件（本包 config）----
    planner_params = PathJoinSubstitution([params_dir, 'nav2_planner.yaml'])
    controller_params = PathJoinSubstitution([params_dir, 'nav2_controller.yaml'])
    bt_params = PathJoinSubstitution([params_dir, 'nav2_bt.yaml'])
    recovery_params = PathJoinSubstitution([params_dir, 'nav2_behaviors.yaml'])

    # ---- 导航核心节点 ----
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[bt_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # 代价地图内嵌于 controller_server（local）和 planner_server（global），
    # 参数分别位于 nav2_controller.yaml 和 nav2_planner.yaml 的 costmap 子树。

    # ---- 恢复行为 ----
    recoveries_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[recovery_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # ---- 生命周期管理（自动 bringup 全部导航节点）----
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'waypoint_follower',
                'behavior_server',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_dir', default_value=config_dir,
            description='slam_nav_core config 目录'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='lifecycle_manager 是否自动启动导航节点'),

        controller_server,
        planner_server,
        bt_navigator,
        waypoint_follower,
        recoveries_server,
        lifecycle_manager,
    ])
