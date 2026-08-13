#!/usr/bin/env python3
# ============================================================
# slam_nav_core/launch/fastlio.launch.py
# 启动 FAST-LIO2 (fastlio_mapping 节点, 来自 fast_lio 包, Ericsii/FAST_LIO_ROS2)
# 输出 /Odometry → remap 为 /odom；TF camera_init→body 动态发布。
#
# P0 修复（typed 参数）：
#   所有数字参数在到达 Node 前转换为原生 int/float/bool 类型——
#   FAST-LIO 以 declare_parameter<int>("preprocess.scan_line") 等声明参数，
#   传入字符串会导致 rclcpp 参数类型不匹配而 abort。本文件不再把
#   LaunchConfiguration（字符串）直接放入 parameters dict。
#   契约由 test/test_fastlio_launch.py 锁定（断言原生类型）。
#
# 依赖：适配器已提供 /points_raw (PointCloud2, lidar 帧) 与 /imu_raw (Imu, imu 帧)
#       （约定见 robot_adapters/ADAPTER_INTERFACE.md）。
# ============================================================
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value, name):
    v = str(value).strip().lower()
    if v in ('1', 'true'):
        return True
    if v in ('0', 'false'):
        return False
    raise RuntimeError(
        f'[fastlio] 参数 {name} 无法解析为布尔值: {value!r}')


def _as_int(value, name):
    try:
        return int(value)
    except (TypeError, ValueError) as e:
        raise RuntimeError(
            f'[fastlio] 参数 {name} 无法解析为整数: {value!r}') from e


def _as_float(value, name):
    try:
        return float(value)
    except (TypeError, ValueError) as e:
        raise RuntimeError(
            f'[fastlio] 参数 {name} 无法解析为浮点数: {value!r}') from e


def _as_float_list(value, name):
    """'x,y,z' → [float, ...]；空串 → None（不覆盖参数文件默认值）。"""
    s = str(value).strip()
    if not s:
        return None
    try:
        return [float(x) for x in s.split(',')]
    except ValueError as e:
        raise RuntimeError(
            f'[fastlio] 参数 {name} 无法解析为浮点列表: {value!r}') from e


def build_fastlio_parameters(launch_configurations):
    """可测纯函数：launch 配置（字符串）→ FAST-LIO 原生类型参数 dict。

    返回值中所有数字键均为原生 int/float/bool（绝不出现 str）。
    非法值抛 RuntimeError → launch fail-fast。
    """
    lc = launch_configurations
    params = {
        'use_sim_time': _as_bool(lc.get('use_sim_time', 'false'), 'use_sim_time'),
        'preprocess.scan_line': _as_int(lc.get('scan_line', '96'), 'scan_line'),
        'preprocess.lidar_type': _as_int(lc.get('lidar_type', '2'), 'lidar_type'),
        'preprocess.timestamp_unit': _as_int(
            lc.get('timestamp_unit', '2'), 'timestamp_unit'),
        'preprocess.blind': _as_float(lc.get('blind', '2.0'), 'blind'),
    }
    ext_t = _as_float_list(lc.get('extrinsic_T', ''), 'extrinsic_T')
    if ext_t is not None:
        if len(ext_t) != 3:
            raise RuntimeError(
                f'[fastlio] 参数 extrinsic_T 必须为 3 个浮点数, 实际 {len(ext_t)} 个')
        params['mapping.extrinsic_T'] = ext_t
    ext_r = _as_float_list(lc.get('extrinsic_R', ''), 'extrinsic_R')
    if ext_r is not None:
        if len(ext_r) != 9:
            raise RuntimeError(
                f'[fastlio] 参数 extrinsic_R 必须为 9 个浮点数, 实际 {len(ext_r)} 个')
        params['mapping.extrinsic_R'] = ext_r
    return params


def _make_fastlio_node(context, *args, **kwargs):
    """OpaqueFunction：运行时把 launch 配置转为原生类型参数并构造 Node。"""
    params = build_fastlio_parameters(context.launch_configurations)
    node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            params,  # 全部为原生 int/float/bool/list，无字符串注入
        ],
        remappings=[
            # 约定输出话题：FAST-LIO /Odometry → /odom（供 Nav2 使用）
            ('/Odometry', '/odom'),
        ],
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_nav_core')
    default_params_file = os.path.join(pkg_share, 'config', 'fastlio_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='FAST-LIO2 参数文件（slam_nav_core/config/fastlio_params.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'robot', default_value='m20',
            description='目标机型（用于按机型加载雷达参数覆盖）'),
        DeclareLaunchArgument(
            'scan_line', default_value='96',
            description='LiDAR 扫描线数（由 bringup 按机型注入；到达节点时转为 int）'),
        DeclareLaunchArgument(
            'lidar_type', default_value='2',
            description='LiDAR 类型（1=Livox,2=Velodyne,3=Ouster；到达节点时转为 int）'),
        DeclareLaunchArgument(
            'timestamp_unit', default_value='2',
            description='PointCloud2 time 字段单位（0=s,1=ms,2=us,3=ns；转为 int）'),
        DeclareLaunchArgument(
            'blind', default_value='2.0',
            description='近距盲区(m)（到达节点时转为 float）'),
        DeclareLaunchArgument(
            'extrinsic_T', default_value='',
            description='覆盖 mapping.extrinsic_T（逗号分隔 3 浮点；空=用参数文件）'),
        DeclareLaunchArgument(
            'extrinsic_R', default_value='',
            description='覆盖 mapping.extrinsic_R（逗号分隔 9 浮点；空=用参数文件）'),

        OpaqueFunction(function=_make_fastlio_node),
    ])
