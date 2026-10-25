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
             ('lidar_frame', 'lidar'), ('imu_frame', 'imu'), ('lio_frame', 'lio_world')]}


def build_static_tf_descriptions(cfg, static_map_odom=True):
    ext = cfg['extrinsics']
    f = frames(cfg)
    if len(set(f.values())) != len(f) or any(not x or x.startswith('/') for x in f.values()):
        raise RuntimeError('TF frames must be unique, nonempty, without a leading slash')
    descs = [
        dict(name='base_to_imu', args=_xyz_quat_args(ext['imu']) + [f['base_frame'], f['imu_frame']]),
        dict(name='base_to_lidar', args=_xyz_quat_args(ext['lidar']) + [f['base_frame'], f['lidar_frame']]),
    ]
    if static_map_odom:  # without the back end there is no correction: map == odom
        descs.insert(0, dict(name='map_to_odom', args=['0', '0', '0', '0', '0', '0', '1', f['map_frame'], f['odom_frame']]))
    return descs


def _make_static_tf_nodes(context):
    sim = _flag(context, 'use_sim_time')
    return [Node(package='tf2_ros', executable='static_transform_publisher',
                 name=d['name'], arguments=d['args'], parameters=[{'use_sim_time': sim}])
            for d in build_static_tf_descriptions(_config(context), not _flag(context, 'use_backend'))]


def backend_parameters(cfg, mode, map_dir, sim):
    """Parameters of uslam_backend; localization needs a map saved by a mapping session."""
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('slam_mode must be mapping or localization')
    if not map_dir or not os.path.isabs(map_dir):
        raise RuntimeError('map_dir must be an absolute directory')
    if mode == 'localization' and not os.path.isfile(os.path.join(map_dir, 'map.pcd')):
        raise RuntimeError(f'localization needs a saved map: {map_dir}/map.pcd not found')
    f = frames(cfg)
    topics = cfg.get('topics', {})
    return {'use_sim_time': sim, 'mode': mode, 'map_dir': map_dir,
            'map_frame': f['map_frame'], 'odom_frame': f['odom_frame'], 'base_frame': f['base_frame'],
            'imu_frame': f['imu_frame'], 'odom_topic': topics.get('odom_topic', '/uslam/odom'),
            'cloud_topic': '/uslam/lio/cloud_registered_body'}


def _make_backend(context):
    if not _flag(context, 'use_backend'):
        return [LogInfo(msg='use_backend=false: static map->odom, no loop closure or relocalization')]
    lc = context.launch_configurations
    params = backend_parameters(_config(context), lc.get('slam_mode', 'mapping'), lc.get('map_dir', ''),
                                _flag(context, 'use_sim_time'))
    defaults = os.path.join(get_package_share_directory('uslam_backend'), 'config', 'backend.yaml')
    return [Node(package='uslam_backend', executable='backend_node', name='uslam_backend', output='screen',
                 parameters=[defaults, params])]


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
        'pcd_save_en': 'false' if _flag(context, 'use_backend') else 'true',
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


def odometry_guard(cfg):
    """Divergence bounds for the odometry adapter.

    They describe what the platform can physically do under ANY driver (vendor remote while
    mapping included), not the Nav2 limits; deriving them from velocity.* latched correct
    odometry at 1.5 m/s during manual mapping. A LIO blow-up is typically far above these.
    """
    guard = {'max_linear_speed': 5.0, 'max_angular_speed': 6.0, 'jump_margin': 0.3}
    guard.update(cfg.get('odometry_guard') or {})
    guard = {k: float(v) for k, v in guard.items() if k in ('max_linear_speed', 'max_angular_speed', 'jump_margin')}
    if not all(math.isfinite(v) for v in guard.values()) or min(guard['max_linear_speed'],
                                                                guard['max_angular_speed']) <= 0 or guard['jump_margin'] < 0:
        raise RuntimeError('odometry_guard bounds must be finite, speeds positive')
    return guard


def static_map_path(lc):
    """Saved grid for the global static layer: only when localizing in a saved map that has one."""
    if lc.get('use_backend', 'true').lower() != 'true' or lc.get('slam_mode', 'mapping') != 'localization':
        return ''
    path = os.path.join(lc.get('map_dir', ''), 'map2d.yaml')
    return path if os.path.isfile(path) else ''


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
    actions = [Node(package='slam_nav_core', executable='odometry_adapter.py',
        parameters=[{'use_sim_time': sim, 'base_imu_xyz': list(map(float, xyz)),
                     'base_imu_quat': list(quat), 'odom_frame': f['odom_frame'],
                     'base_frame': f['base_frame'], 'lio_frame': f['lio_frame'],
                     'input_topic': '/uslam/lio/odometry', 'gravity_topic': '/uslam/lio/gravity',
                     'output_topic': topics.get('odom_topic', '/uslam/odom'),
                     **odometry_guard(cfg)}])]
    if nav:
        navigation = cfg.get('navigation', {})
        if 'base_height' not in navigation:
            raise RuntimeError('Navigation requires navigation.base_height (base_link height above the floor)')
        lc = context.launch_configurations
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_nav_core'), 'launch', 'nav_stack.launch.py')),
            launch_arguments={'use_sim_time': str(sim).lower(), 'robot_config': lc['robot_config'],
                              'static_map': static_map_path(lc)}.items()))
        actions.append(Node(package='slam_nav_core', executable='obstacle_filter.py', parameters=[{
            'use_sim_time': sim, 'base_height': float(navigation['base_height']),
            'odom_frame': f['odom_frame'], 'base_frame': f['base_frame'], 'lio_frame': f['lio_frame'],
            'self_radius': float(navigation.get('robot_radius', .55)),
            'output_topic': topics.get('obstacles_topic', '/uslam/obstacles'),
            'terrain_topic': topics.get('terrain_obstacles_topic', '/uslam/terrain_obstacles'),
            'clear_rays_topic': topics.get('clear_rays_topic', '/uslam/clear_rays'),
            'sensor_frame': f['lidar_frame']}]))
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
        DeclareLaunchArgument('use_backend', default_value='true',
                              description='Loop closure / relocalization back end; publishes map->odom'),
        DeclareLaunchArgument('slam_mode', default_value='mapping', description='mapping | localization'),
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/.ros/universal_slam_map'),
                              description='Saved-map directory (written by save_map, read in localization)'),
        OpaqueFunction(function=_make_runtime),
        OpaqueFunction(function=_make_static_tf_nodes),
        OpaqueFunction(function=_make_backend),
        OpaqueFunction(function=_make_fastlio_launch),
        OpaqueFunction(function=_make_bridge),
    ])
