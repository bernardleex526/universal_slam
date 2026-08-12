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


def _make_fastlio_launch(context):
    import yaml
    robot_name = context.launch_configurations.get('robot', 'm20')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
    cfg = _load_robot_config(robot_name) or {}
    lidar_cfg = cfg.get('lidar', {})

    overrides = {}
    if 'scan_line' in lidar_cfg:
        overrides['preprocess.scan_line'] = int(lidar_cfg['scan_line'])
    if 'lidar_type' in lidar_cfg:
        overrides['preprocess.lidar_type'] = int(lidar_cfg['lidar_type'])
    if 'timestamp_unit' in lidar_cfg:
        overrides['preprocess.timestamp_unit'] = int(lidar_cfg['timestamp_unit'])
    if 'blind' in lidar_cfg:
        overrides['preprocess.blind'] = float(lidar_cfg['blind'])

    overrides['use_sim_time'] = use_sim_time == 'true'

    # 构建机型特化参数覆盖：从 robot_configs/<robot>.yaml 读取的 lidar 参数
    # 传递给 fastlio.launch.py 中声明的对应 DeclareLaunchArgument
    extra_args = {'use_sim_time': use_sim_time}
    if overrides.get('preprocess.scan_line') is not None:
        extra_args['scan_line'] = str(overrides['preprocess.scan_line'])
    if overrides.get('preprocess.lidar_type') is not None:
        extra_args['lidar_type'] = str(overrides['preprocess.lidar_type'])
    if overrides.get('preprocess.timestamp_unit') is not None:
        extra_args['timestamp_unit'] = str(overrides['preprocess.timestamp_unit'])
    if overrides.get('preprocess.blind') is not None:
        extra_args['blind'] = str(overrides['preprocess.blind'])

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_nav_core'), 'launch', 'fastlio.launch.py'])),
            launch_arguments=extra_args.items(),
        )
    ]


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
        # odom→camera_init：衔接 FAST-LIO camera_init 坐标系与 REP-105 odom 帧（P1 完善：待实测后决定是否改用 frame_id 重映射）
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init'],
             name='odom_to_camera_init_tf'),
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
    fastlio_action = OpaqueFunction(function=_make_fastlio_launch)

    # ---- Nav2 导航栈（参数文件指向 slam_nav_core config/）----
    nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_nav_core'), 'launch', 'nav_stack.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ---- 机型适配器（P2：adapter_m20 和 adapter_sw01 需分别提供 launch/adapter.launch.py）----
    # 依赖状态：adapter_m20（话题桥，待 M20 rosbag 验证）、adapter_sw01（SDK 待文档）
    # 取消注释前请确保对应 adapter 包已提供 launch/adapter.launch.py 并完成单元测试。
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
        fastlio_action,
        nav_stack_launch,
        # adapter_launch,  # P2
    ])
