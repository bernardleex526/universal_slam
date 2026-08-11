#!/usr/bin/env python3
# ============================================================
# universal_slam_bringup/launch/universal_slam.launch.py
# 统一入口：
#   ros2 launch universal_slam_bringup universal_slam.launch.py robot:=m20|sw01
# 一键启动：
#   1. 静态 TF（map→odom 恒等, body→base_link, base_link→lidar/imu，取自 robot_configs）
#   2. FAST-LIO2（fastlio.launch.py，/Odometry → /odom）
#   3. Nav2 导航栈（nav_stack.launch.py）
#   4. 机型适配器（P2 完善，先占位）
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_robot_config(robot_name):
    """P0: 读取 install 的 robot_configs/<robot>.yaml；失败返回 None（回退默认外参）。"""
    try:
        import yaml
    except ImportError:
        print('[universal_slam] 警告: 未安装 python3-yaml，静态 TF 使用默认外参')
        return None
    try:
        path = os.path.join(
            get_package_share_directory('robot_configs'), 'config', f'{robot_name}.yaml')
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f'[universal_slam] 警告: 加载 robot_configs/{robot_name}.yaml 失败({e})，静态 TF 使用默认外参')
        return None


def _make_static_tf_nodes(context):
    """按 robot:= 机型参数构建静态 TF 发布节点（P0 从 yaml 读取外参）。"""
    robot_name = context.launch_configurations.get('robot', 'm20')
    cfg = _load_robot_config(robot_name) or {}
    extrinsics = cfg.get('extrinsics', {})

    def xyz_quat(ext):
        # 外参(米) + 单位四元数（P0 全部外参朝向为 0，P1 标定后补 roll/pitch/yaw→四元数换算）
        x = ext.get('x', 0.0)
        y = ext.get('y', 0.0)
        z = ext.get('z', 0.0)
        return [str(x), str(y), str(z), '0', '0', '0', '1']

    imu_ext = extrinsics.get('imu', {})
    lidar_ext = extrinsics.get('lidar', {})
    if not imu_ext:
        print(f'[universal_slam] 警告: robot_configs/{robot_name}.yaml 缺少 extrinsics.imu，使用默认外参')
    if not lidar_ext:
        print(f'[universal_slam] 警告: robot_configs/{robot_name}.yaml 缺少 extrinsics.lidar，使用默认外参')

    return [
        # map→odom 恒等（D9: 静态层与 FAST-LIO camera_init 帧对齐；偏移参数 P4 预留）
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']),
        # body→base_link 静态补齐（FAST-LIO camera_init→body 与 REP-105 衔接，先恒等）
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'base_link']),
        # base_link→imu（外参初值，robot_configs/<robot>.yaml）
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=xyz_quat(imu_ext) + ['base_link', 'imu']),
        # base_link→lidar（外参初值，robot_configs/<robot>.yaml）
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=xyz_quat(lidar_ext) + ['base_link', 'lidar']),
    ]


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_adapter = LaunchConfiguration('use_adapter')

    # ---- FAST-LIO2（参数文件指向 slam_nav_core config/fastlio_params.yaml）----
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_nav_core'), 'launch', 'fastlio.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- Nav2 导航栈（参数文件指向 slam_nav_core config/）----
    nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_nav_core'), 'launch', 'nav_stack.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- 机型适配器（P2 完善：adapter_m20 / adapter_sw01 提供 launch/adapter.launch.py 后启用）----
    # TODO(P2): 取消注释以下条件包含
    # adapter_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare(PythonExpression(['"adapter_" + "', robot, '"'])),
    #             'launch', 'adapter.launch.py'])),
    #     condition=IfCondition(use_adapter),
    # )

    # ---- 静态 TF（OpaqueFunction: 运行时按 robot:= 读取 robot_configs yaml）----
    static_tf_action = OpaqueFunction(function=_make_static_tf_nodes)

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='m20', choices=['m20', 'sw01'],
            description='目标机型（对应 robot_configs/config/<robot>.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'use_adapter', default_value='false',
            description='是否启动机型适配器（P2 实现，当前占位）'),

        static_tf_action,
        fastlio_launch,
        nav_stack_launch,
        # adapter_launch,  # P2
    ])
