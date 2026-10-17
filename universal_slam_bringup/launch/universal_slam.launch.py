#!/usr/bin/env python3
# ============================================================
# universal_slam_bringup/launch/universal_slam.launch.py
#
#   ros2 launch universal_slam_bringup universal_slam.launch.py \
#       robot_config:=/abs/profile.yaml [use_bridge:=true] [use_nav:=true]
#
# Robot-agnostic entry point. It starts static TF, FAST-LIO2, the odometry adapter and
# (optionally) Nav2. The robot bridge is whatever package the profile names in
# bridge.package / bridge.launch; it is included generically with robot_config,
# use_sim_time and enable_control. Nothing here depends on a robot package.
# ============================================================
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# ---- extrinsic math (pure, unit-tested) ----
# Profile extrinsics are base->sensor poses; roll/pitch/yaw are in DEGREES.
# FAST-LIO extrinsic is lidar->IMU: p_IMU = extrinsic_R * p_LIDAR + extrinsic_T.

def _rpy_to_quat(roll, pitch, yaw):
    """RPY (rad) -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rpy_to_rot(roll, pitch, yaw):
    """RPY (rad) -> row-major 3x3 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = (1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr)
    ry = (cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp)
    rz = (cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0)
    return _mat_mul(_mat_mul(rz, ry), rx)


def _mat_mul(a, b):
    return tuple(
        sum(a[i * 3 + k] * b[k * 3 + j] for k in range(3))
        for i in range(3) for j in range(3))


def _mat_transpose(m):
    return tuple(m[j * 3 + i] for i in range(3) for j in range(3))


def _mat_vec(m, v):
    return tuple(sum(m[i * 3 + k] * v[k] for k in range(3)) for i in range(3))


def _quat_to_rot(q):
    x, y, z, w = q
    return (
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    )


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _tf_inverse(xyz, quat):
    x, y, z = xyz
    r_t = _mat_transpose(_quat_to_rot(quat))
    t_inv = _mat_vec(r_t, (-x, -y, -z))
    return t_inv, (-quat[0], -quat[1], -quat[2], quat[3])


def _tf_compose(xyz1, quat1, xyz2, quat2):
    x1, y1, z1 = xyz1
    t2_rot = _mat_vec(_quat_to_rot(quat1), xyz2)
    return (
        (x1 + t2_rot[0], y1 + t2_rot[1], z1 + t2_rot[2]),
        _quat_mul(quat1, quat2),
    )


def _ext_xyz_rpy(ext):
    xyz = (ext.get('x', 0.0), ext.get('y', 0.0), ext.get('z', 0.0))
    rpy = tuple(math.radians(ext.get(k, 0.0)) for k in ('roll', 'pitch', 'yaw'))
    return xyz, rpy


def lidar_to_imu_extrinsic(imu_ext, lidar_ext):
    """FAST-LIO extrinsic_T/R from base->imu and base->lidar poses.

      extrinsic_R = R_BI^T @ R_BL
      extrinsic_T = R_BI^T @ (t_BL - t_BI)
    """
    t_bi, rpy_bi = _ext_xyz_rpy(imu_ext)
    t_bl, rpy_bl = _ext_xyz_rpy(lidar_ext)
    r_bi_t = _mat_transpose(_rpy_to_rot(*rpy_bi))
    r = _mat_mul(r_bi_t, _rpy_to_rot(*rpy_bl))
    t = _mat_vec(r_bi_t, tuple(t_bl[k] - t_bi[k] for k in range(3)))
    return t, r


def load_profile(path):
    if not path:
        raise RuntimeError('robot_config:=/absolute/path/to/profile.yaml is required '
                           '(profiles live in their bridge package, <bridge>/config)')
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError('Robot profile must be a mapping')
    for key in ('tf', 'extrinsics', 'lidar'):
        if key not in cfg:
            raise RuntimeError(f'Robot profile is missing "{key}"')
    return cfg


def _config(context):
    return load_profile(context.launch_configurations.get('robot_config', ''))


def _flag(context, name):
    return context.launch_configurations.get(name, 'false').lower() == 'true'


def _ext_xyz_quat(ext):
    xyz, rpy = _ext_xyz_rpy(ext)
    if not all(math.isfinite(x) for x in (*xyz, *rpy)):
        raise RuntimeError('Non-finite extrinsic')
    return xyz, _rpy_to_quat(*rpy)


def _xyz_quat_args(ext):
    xyz, quat = _ext_xyz_quat(ext)
    return [str(v) for v in (*xyz, *quat)]


def frames(cfg):
    t = cfg.get('tf', {})
    return {k: t.get(k, 'uslam/' + default) for k, default in
            [('map_frame', 'map'), ('odom_frame', 'odom'), ('base_frame', 'base_link'),
             ('lidar_frame', 'lidar'), ('imu_frame', 'imu')]}


def build_static_tf_descriptions(cfg):
    ext = cfg['extrinsics']
    f = frames(cfg)
    if len(set(f.values())) != len(f) or any(not x or x.startswith('/') for x in f.values()):
        raise RuntimeError('TF frames must be unique, nonempty, without a leading slash')
    return [
        dict(name='map_to_odom', args=['0', '0', '0', '0', '0', '0', '1', f['map_frame'], f['odom_frame']]),
        dict(name='base_to_imu', args=_xyz_quat_args(ext['imu']) + [f['base_frame'], f['imu_frame']]),
        dict(name='base_to_lidar', args=_xyz_quat_args(ext['lidar']) + [f['base_frame'], f['lidar_frame']]),
    ]


def _make_static_tf_nodes(context):
    sim = _flag(context, 'use_sim_time')
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name=d['name'], arguments=d['args'], parameters=[{'use_sim_time': sim}])
            for d in build_static_tf_descriptions(_config(context))]


def _make_fastlio_launch(context):
    cfg = _config(context)
    lidar, topics = cfg['lidar'], cfg.get('topics', {})
    ext = cfg['extrinsics']
    t, r = lidar_to_imu_extrinsic(ext['imu'], ext['lidar'])
    extra = {k: str(lidar[k]) for k in ('scan_line', 'lidar_type', 'timestamp_unit', 'blind')}
    extra.update({
        'use_sim_time': context.launch_configurations.get('use_sim_time', 'false'),
        'scan_rate': str(lidar.get('scan_rate', 10)),
        'extrinsic_T': ','.join(map(str, t)), 'extrinsic_R': ','.join(map(str, r)),
        'points_topic': topics.get('points_topic', '/uslam/points_raw'),
        'imu_topic': topics.get('imu_topic', '/uslam/imu_raw'),
        'map_file_path': context.launch_configurations.get(
            'map_file_path', os.path.expanduser('~/.ros/universal_slam_map.pcd')),
    })
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_nav_core'), 'launch', 'fastlio.launch.py')),
        launch_arguments=extra.items())]


def bridge_launch_path(cfg):
    """Resolve the bridge launch file named by the profile (bridge.package / bridge.launch)."""
    bridge = cfg.get('bridge')
    if not isinstance(bridge, dict) or not bridge.get('package') or not bridge.get('launch'):
        raise RuntimeError('use_bridge:=true requires bridge.package and bridge.launch in the profile')
    if os.path.basename(bridge['launch']) != bridge['launch']:
        raise RuntimeError('bridge.launch must be a file name inside <package>/launch')
    return os.path.join(get_package_share_directory(bridge['package']), 'launch', bridge['launch'])


def _make_bridge(context):
    if not _flag(context, 'use_bridge'):
        return [LogInfo(msg='use_bridge=false: normalized sensor topics must be provided externally')]
    lc = context.launch_configurations
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch_path(_config(context))),
        launch_arguments={'robot_config': lc['robot_config'],
                          'use_sim_time': lc.get('use_sim_time', 'false'),
                          'enable_control': lc.get('enable_control', 'false')}.items())]


def _make_runtime(context):
    cfg = _config(context)
    sim = _flag(context, 'use_sim_time')
    nav = _flag(context, 'use_nav')
    control = _flag(context, 'enable_control')
    if (nav or control) and cfg.get('calibration_verified', False) is not True:
        raise RuntimeError('Navigation/control blocked: profile calibration is not verified')
    if control and not _flag(context, 'use_bridge'):
        raise RuntimeError('Control requires the robot bridge (use_bridge:=true)')
    f = frames(cfg)
    topics = cfg.get('topics', {})
    xyz, quat = _ext_xyz_quat(cfg['extrinsics']['imu'])
    velocity = cfg.get('velocity', {})
    actions = [Node(package='slam_nav_core', executable='odometry_adapter.py',
        parameters=[{'use_sim_time': sim, 'base_imu_xyz': list(map(float, xyz)),
                     'base_imu_quat': list(quat), 'odom_frame': f['odom_frame'],
                     'base_frame': f['base_frame'], 'input_topic': '/uslam/lio/odometry',
                     'output_topic': topics.get('odom_topic', '/uslam/odom'),
                     # Well above any commanded motion, far below a LIO blow-up.
                     'max_linear_speed': 3.0 * max(float(velocity.get('vx_max', .5)), float(velocity.get('vy_max', .5)), .5),
                     'max_angular_speed': 3.0 * max(float(velocity.get('wz_max', .6)), .6)}])]
    if nav:
        navigation = cfg.get('navigation', {})
        if 'base_height' not in navigation:
            raise RuntimeError('Navigation requires navigation.base_height (base_link height above the floor)')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_nav_core'), 'launch', 'nav_stack.launch.py')),
            launch_arguments={'use_sim_time': str(sim).lower(),
                              'robot_config': context.launch_configurations['robot_config']}.items()))
        actions.append(Node(package='slam_nav_core', executable='obstacle_filter.py', parameters=[{
            'use_sim_time': sim, 'base_height': float(navigation['base_height']),
            'odom_frame': f['odom_frame'], 'base_frame': f['base_frame'],
            'self_radius': float(navigation.get('robot_radius', .55)),
            'output_topic': topics.get('obstacles_topic', '/uslam/obstacles')}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value='',
                              description='Absolute path of the robot profile (owned by its bridge package)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_bridge', default_value='false'),
        DeclareLaunchArgument('use_nav', default_value='false'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('map_file_path', default_value=os.path.expanduser('~/.ros/universal_slam_map.pcd')),
        OpaqueFunction(function=_make_runtime),
        OpaqueFunction(function=_make_static_tf_nodes),
        OpaqueFunction(function=_make_fastlio_launch),
        OpaqueFunction(function=_make_bridge),
    ])
