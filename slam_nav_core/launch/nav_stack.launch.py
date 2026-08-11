#!/usr/bin/env python3
# ============================================================
# slam_nav_core/launch/nav_stack.launch.py
# Nav2 导航栈（与机型无关）：planner / controller / bt / waypoint / costmap
# 参数文件均指向本包 slam_nav_core/config/。
# 依赖：FAST-LIO2 已提供 /odom 与 odom→base_link 动态 TF（见 fastlio.launch.py）。
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
    global_costmap_params = PathJoinSubstitution([params_dir, 'nav2_global_costmap.yaml'])
    local_costmap_params = PathJoinSubstitution([params_dir, 'nav2_local_costmap.yaml'])
    recovery_params = PathJoinSubstitution([params_dir, 'nav2_recovery.yaml'])

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

    # ---- 代价地图（全局 + 局部）----
    # 注：Humble 的 planner/controller 亦内嵌 costmap（nav2_bringup 标准做法，
    #     其参数结构为 global_costmap/global_costmap/ros__parameters）。
    #     此处按本仓库约定以独立 costmap 节点方式启动，P1 实测后可与内嵌方式对齐。
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[global_costmap_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[local_costmap_params, {'use_sim_time': use_sim_time}],
        remappings=remappings,
    )

    # ---- 恢复行为 ----
    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
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
                'global_costmap',
                'local_costmap',
                'recoveries_server',
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
        global_costmap,
        local_costmap,
        recoveries_server,
        lifecycle_manager,
    ])
