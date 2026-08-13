#!/usr/bin/env python3
# ============================================================
# adapter_m20 / launch/adapter.launch.py
# M20 适配器启动（由 universal_slam_bringup 在
#   use_adapter:=true 且 robot:=m20 时包含）：
#   1) m20_topic_bridge: 机载 Foxy /IMU、/LIDAR/POINTS → /imu_raw、/points_raw
#   2) m20_cmd_vel_bridge: /cmd_vel → UDP JSON 轴指令（20Hz）
#
# 参数全部来自 robot_configs/config/m20.yaml，并以原生类型注入节点。
#
# 注意（非零配置）：topic_bridge 只做 QoS/frame_id 整形与转发，
# 跨主机 DDS 仍需同 ROS_DOMAIN_ID + 同 RMW + 网络可达（有线千兆），
# 或两端部署同版本 zenoh-bridge-ros2dds。见 README.md。
# ============================================================
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def _load_m20_config():
    path = os.path.join(
        get_package_share_directory('robot_configs'), 'config', 'm20.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    cfg = _load_m20_config() or {}

    net = cfg.get('network', {}) or {}
    vel = cfg.get('velocity', {}) or {}
    topics = cfg.get('topics', {}) or {}
    tf_cfg = cfg.get('tf', {}) or {}
    recovery = cfg.get('error_recovery', {}) or {}

    bridge = Node(
        package='adapter_m20',
        executable='m20_cmd_vel_bridge',
        name='m20_cmd_vel_bridge',
        output='screen',
        # 原生类型参数（数字参数均为 int/float，不做字符串注入）
        parameters=[{
            'robot_ip': str(net.get('robot_ip', '10.21.31.103')),
            'port': int(net.get('port', 30000)),
            'cmd_frequency_hz': float(net.get('cmd_frequency_hz', 20.0)),
            'heartbeat_hz': float(net.get('heartbeat_hz', 1.0)),
            'vx_max': float(vel.get('vx_max', 0.5)),
            'vy_max': float(vel.get('vy_max', 0.5)),
            'wz_max': float(vel.get('wz_max', 0.6)),
            'cmd_vel_topic': str(topics.get('cmd_vel_topic', '/cmd_vel')),
            'status_topic': '/m20_bridge_status',
            # ErrorHalt 有界恢复（默认关 = 硬件不确定项；人工复位服务常开）
            'error_auto_recovery': bool(recovery.get('auto_recovery_enabled', False)),
            'error_max_recovery_attempts': int(recovery.get('max_recovery_attempts', 3)),
            'error_recovery_backoff_s': float(recovery.get('recovery_base_backoff_s', 5.0)),
            'error_clear_window_s': float(recovery.get('error_clear_window_s', 5.0)),
        }],
    )

    relay = Node(
        package='adapter_m20',
        executable='m20_topic_bridge',
        name='m20_topic_bridge',
        output='screen',
        parameters=[{
            'source_cloud_topic': '/LIDAR/POINTS',
            'source_imu_topic': '/IMU',
            'output_cloud_topic': str(topics.get('points_topic', '/points_raw')),
            'output_imu_topic': str(topics.get('imu_topic', '/imu_raw')),
            'lidar_frame': str(tf_cfg.get('lidar_frame', 'lidar')),
            'imu_frame': str(tf_cfg.get('imu_frame', 'imu')),
        }],
    )

    return LaunchDescription([bridge, relay])
