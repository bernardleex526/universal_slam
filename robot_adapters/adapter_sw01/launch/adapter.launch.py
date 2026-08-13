#!/usr/bin/env python3
# ============================================================
# adapter_sw01 / launch/adapter.launch.py
# SW01 适配器启动（由 universal_slam_bringup 在
#   use_adapter:=true 且 robot:=sw01 时包含）。
#
# 参数全部来自 robot_configs/config/sw01.yaml，并以原生类型注入节点
# （int/float/bool；与 fastlio.launch.py 相同的"不注入字符串"原则）。
#
# fail-fast：
#   - 未找到 sw01_adapter 可执行文件（adapter_sw01 未以 ASTRALL_SDK_DIR
#     构建）→ 立即抛 RuntimeError 并给出构建指引，不静默跳过。
# ============================================================
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def _load_sw01_config():
    path = os.path.join(
        get_package_share_directory('robot_configs'), 'config', 'sw01.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_sw01_config() or {}

    # ---- fail-fast：可执行文件必须存在（需以 ASTRALL_SDK_DIR 构建）----
    exe = os.path.join(get_package_prefix('adapter_sw01'), 'lib', 'adapter_sw01',
                       'sw01_adapter')
    if not os.path.isfile(exe):
        raise RuntimeError(
            '[adapter_sw01] 未找到可执行文件 sw01_adapter。'
            'adapter_sw01 需要专有 SDK (libASTRALL_SDK.so) 才能构建：\n'
            '  colcon build --packages-select adapter_sw01 '
            '--cmake-args -DASTRALL_SDK_DIR=/path/to/ASTRALL_SDK\n'
            '构建完成前不要使用 use_adapter:=true + robot:=sw01。')

    topics = cfg.get('topics', {}) or {}
    vel = cfg.get('velocity', {}) or {}
    lidar_cfg = cfg.get('lidar', {}) or {}
    sdk_cfg = cfg.get('sdk', {}) or {}

    adapter = Node(
        package='adapter_sw01',
        executable='sw01_adapter',
        name='sw01_adapter',
        output='screen',
        # 原生类型参数（数字参数均为 int/float，不做字符串注入）
        # 注意：无 robot_ip —— 控制经 SDK，点云为入站 UDP
        parameters=[{
            'lidar_port': int(sdk_cfg.get('pointcloud_udp_port', 6100)),
            'imu_frame': str(cfg.get('tf', {}).get('imu_frame', 'imu')),
            'lidar_frame': str(cfg.get('tf', {}).get('lidar_frame', 'lidar')),
            'point_scale': float(sdk_cfg.get('point_scale', 0.001)),
            'vx_max': float(vel.get('vx_max', 0.5)),
            'vy_max': float(vel.get('vy_max', 0.5)),
            'wz_max': float(vel.get('wz_max', 0.6)),
            'cmd_frequency_hz': float(sdk_cfg.get('control_frequency_hz', 20.0)),
            'cloud_packet_layout': str(
                sdk_cfg.get('cloud_packet_layout', 'dynamic')),
            'cmd_vel_topic': str(topics.get('cmd_vel_topic', '/cmd_vel')),
            'imu_topic': str(topics.get('imu_topic', '/imu_raw')),
            'points_topic': str(topics.get('points_topic', '/points_raw')),
        }],
    )

    # 契约说明（不伪造 ring/time）：SW01 协议无 ring/逐点 time，
    # /points_raw 仅 x,y,z,intensity；FAST-LIO lidar_type=2 走 yaw 角时序回退。
    return LaunchDescription([adapter])
