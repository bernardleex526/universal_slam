#!/usr/bin/env python3
# ============================================================
# universal_slam_bringup/launch/universal_slam.launch.py
# 统一入口：
#   ros2 launch universal_slam_bringup universal_slam.launch.py robot:=m20|sw01 [use_adapter:=true]
# 启动：
#   1. 静态 TF（map→odom 恒等, body→base_link, base_link→lidar/imu，取自 robot_configs）
#   2. FAST-LIO2（fastlio.launch.py，/Odometry → /odom；数字参数原生 int/float）
#   3. Nav2 导航栈（nav_stack.launch.py）
#   4. 机型适配器（use_adapter:=true 时；缺 launch/可执行时 fail-fast）
#
# 诚实声明（当前能力边界，勿当作"一键全链成功"）：
#   - 本 launch 不发布 /map，不加载静态地图：地图保存/栅格化管线（D9，
#     save_map.sh / pcd2pgm）尚未实现，Nav2 static layer 保持禁用。
#     FAST-LIO2 持续建图 + 实时点云代价地图即为当前导航模式。
#   - M20 数据接入不是零配置：use_adapter:=true 仅启动 topic_bridge 整形
#     frame_id/QoS；跨主机仍需同 ROS_DOMAIN_ID + 同 RMW + 有线千兆可达
#     （或两端同版本 zenoh-bridge-ros2dds）。use_adapter:=false 时
#     /points_raw 与 /imu_raw 必须由外部提供，本文件仅打印提醒，不保证可用。
#   - SW01 适配器需专有 SDK（libASTRALL_SDK.so）构建；未构建时
#     adapter.launch.py 直接抛错（fail-fast），不会静默降级。
# ============================================================
import math
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ---- 外参数学（纯函数，可测；单测见 test/test_universal_slam_launch.py）----
# 约定：robot_configs 的 extrinsics.roll/pitch/yaw 单位为【度】，需实机标定（P4）；
#       未标定时保持 0（单位四元数 / 单位矩阵）。
# 约定：FAST-LIO extrinsic 为 lidar→IMU：
#       p_IMU = extrinsic_R * p_LIDAR + extrinsic_T（与锁定 commit 2fffc57
#       laserMapping.cpp 的 Lidar_R_wrt_IMU / Lidar_T_wrt_IMU 一致）。

def _rpy_to_quat(roll, pitch, yaw):
    """RPY(弧度) → 四元数 (x, y, z, w)（static_transform_publisher 参数顺序）。"""
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
    """RPY(弧度) → 3x3 旋转矩阵 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，行主序 9 元组。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = (1.0, 0.0, 0.0, 0.0, cr, -sr, 0.0, sr, cr)
    ry = (cp, 0.0, sp, 0.0, 1.0, 0.0, -sp, 0.0, cp)
    rz = (cy, -sy, 0.0, sy, cy, 0.0, 0.0, 0.0, 1.0)
    return _mat_mul(_mat_mul(rz, ry), rx)


def _mat_mul(a, b):
    """3x3 行主序矩阵乘。"""
    return tuple(
        sum(a[i * 3 + k] * b[k * 3 + j] for k in range(3))
        for i in range(3) for j in range(3))


def _mat_transpose(m):
    return tuple(m[j * 3 + i] for i in range(3) for j in range(3))


def _mat_vec(m, v):
    return tuple(sum(m[i * 3 + k] * v[k] for k in range(3)) for i in range(3))


def _quat_to_rot(q):
    """四元数 (x,y,z,w) → 3x3 旋转矩阵（行主序）。"""
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
    """TF 逆变换：inv(t, q) = (-R(q)^T·t, conj(q))。"""
    x, y, z = xyz
    r_t = _mat_transpose(_quat_to_rot(quat))
    t_inv = _mat_vec(r_t, (-x, -y, -z))
    return t_inv, (-quat[0], -quat[1], -quat[2], quat[3])


def _tf_compose(xyz1, quat1, xyz2, quat2):
    """TF 合成：T1 ∘ T2 → (t1 + R(q1)·t2, q1 ⊗ q2)。"""
    x1, y1, z1 = xyz1
    t2_rot = _mat_vec(_quat_to_rot(quat1), xyz2)
    return (
        (x1 + t2_rot[0], y1 + t2_rot[1], z1 + t2_rot[2]),
        _quat_mul(quat1, quat2),
    )


def _ext_xyz_rpy(ext):
    """外参 dict → ((x, y, z), (roll, pitch, yaw)[弧度])。"""
    xyz = (ext.get('x', 0.0), ext.get('y', 0.0), ext.get('z', 0.0))
    rpy = tuple(math.radians(ext.get(k, 0.0)) for k in ('roll', 'pitch', 'yaw'))
    return xyz, rpy


def lidar_to_imu_extrinsic(imu_ext, lidar_ext):
    """FAST-LIO extrinsic_T / extrinsic_R（lidar→IMU，纯函数）。

    base→imu: (R_BI, t_BI)；base→lidar: (R_BL, t_BL) ⇒
      extrinsic_R = R_BI^T @ R_BL
      extrinsic_T = R_BI^T @ (t_BL - t_BI)   # lidar 原点在 IMU 系中的坐标
    返回 (T3, R9)。配置值为初值，实机需标定（见 robot_configs 注释）。
    """
    t_bi, rpy_bi = _ext_xyz_rpy(imu_ext)
    t_bl, rpy_bl = _ext_xyz_rpy(lidar_ext)
    r_bi_t = _mat_transpose(_rpy_to_rot(*rpy_bi))
    r = _mat_mul(r_bi_t, _rpy_to_rot(*rpy_bl))
    t = _mat_vec(r_bi_t, tuple(t_bl[k] - t_bi[k] for k in range(3)))
    return t, r


def _load_robot_config(robot_name):
    import yaml
    path = os.path.join(get_package_share_directory('robot_configs'), 'config', f'{robot_name}.yaml')
    with open(path, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise RuntimeError('Robot profile must be a mapping')
    return cfg


def _config(context):
    import yaml
    path = context.launch_configurations.get('robot_config', '')
    if path:
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f)
    return _load_robot_config(context.launch_configurations.get('robot', 'm20'))


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
            [('map_frame','map'),('odom_frame','odom'),('base_frame','base_link'),
             ('lidar_frame','lidar'),('imu_frame','imu')]}


def build_static_tf_descriptions(robot_name, cfg):
    ext = cfg['extrinsics']
    f = frames(cfg)
    if len(set(f.values())) != len(f) or any(not x or x.startswith('/') for x in f.values()):
        raise RuntimeError('TF frames must be unique, nonempty, without a leading slash')
    return [
        dict(name='map_to_odom', args=['0','0','0','0','0','0','1', f['map_frame'], f['odom_frame']]),
        dict(name='base_to_imu', args=_xyz_quat_args(ext['imu'])+[f['base_frame'],f['imu_frame']]),
        dict(name='base_to_lidar', args=_xyz_quat_args(ext['lidar'])+[f['base_frame'],f['lidar_frame']]),
    ]


def _make_static_tf_nodes(context):
    cfg = _config(context)
    sim = context.launch_configurations.get('use_sim_time','false').lower() == 'true'
    return [Node(package='tf2_ros',executable='static_transform_publisher',
                 name=d['name'],arguments=d['args'],parameters=[{'use_sim_time':sim}])
            for d in build_static_tf_descriptions('',cfg)]


def _make_fastlio_launch(context):
    cfg = _config(context)
    lidar, topics = cfg['lidar'], cfg.get('topics',{})
    ext = cfg['extrinsics']
    t,r = lidar_to_imu_extrinsic(ext['imu'],ext['lidar'])
    extra = {k:str(lidar[k]) for k in ('scan_line','lidar_type','timestamp_unit','blind')}
    extra.update({
        'use_sim_time':context.launch_configurations.get('use_sim_time','false'),
        'scan_rate':str(lidar.get('scan_rate',10)),
        'extrinsic_T':','.join(map(str,t)), 'extrinsic_R':','.join(map(str,r)),
        'points_topic':topics.get('points_topic','/uslam/points_raw'),
        'imu_topic':topics.get('imu_topic','/uslam/imu_raw'),
        'map_file_path':context.launch_configurations.get('map_file_path',os.path.expanduser('~/.ros/universal_slam_map.pcd')),
    })
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_nav_core'),'launch','fastlio.launch.py')),
        launch_arguments=extra.items())]


def _adapter_status_log(context):
    if context.launch_configurations.get('use_adapter','false').lower() in ('false','0'):
        return [LogInfo(msg='use_adapter=false: provide normalized sensor data externally; use_adapter:=true enables sensors only by default')]
    return []


def _make_runtime(context):
    cfg = _config(context)
    sim = context.launch_configurations.get('use_sim_time','false').lower() == 'true'
    nav = context.launch_configurations.get('use_nav','false').lower() == 'true'
    control = context.launch_configurations.get('enable_control','false').lower() == 'true'
    adapter = context.launch_configurations.get('use_adapter','false').lower() == 'true'
    if (nav or control) and cfg.get('calibration_verified',False) is not True:
        raise RuntimeError('Navigation/control blocked: profile calibration is not verified')
    if control and (sim or not adapter):
        raise RuntimeError('Control prohibited during simulation or without adapter')
    if control and cfg.get('adapter','m20') != 'm20':
        raise RuntimeError('Only the audited M20 control backend may be enabled')
    f = frames(cfg)
    xyz,quat = _ext_xyz_quat(cfg['extrinsics']['imu'])
    actions = [Node(package='slam_nav_core',executable='odometry_adapter.py',
        parameters=[{'use_sim_time':sim,'base_imu_xyz':list(map(float,xyz)),
                     'base_imu_quat':list(quat),'odom_frame':f['odom_frame'],
                     'base_frame':f['base_frame'],'input_topic':'/uslam/lio/odometry',
                     'output_topic':cfg.get('topics',{}).get('odom_topic','/uslam/odom')}])]
    if nav:
        if 'ground_z' not in cfg.get('navigation',{}):
            raise RuntimeError('Navigation requires an explicit world-frame ground_z')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_nav_core'),'launch','nav_stack.launch.py')),
            launch_arguments={'use_sim_time':str(sim).lower(),
                              'robot_config':context.launch_configurations.get('robot_config','') or os.path.join(get_package_share_directory('robot_configs'),'config',context.launch_configurations.get('robot','m20')+'.yaml')}.items()))
        actions.append(Node(package='slam_nav_core',executable='obstacle_filter.py',parameters=[{
            'use_sim_time':sim,'ground_z':float(cfg['navigation']['ground_z']),
            'odom_frame':f['odom_frame'],'base_frame':f['base_frame'],
            'self_radius':float(cfg['navigation'].get('robot_radius',.55)),
            'output_topic':cfg.get('topics',{}).get('obstacles_topic','/uslam/obstacles')}]))
    actions.insert(0, SetLaunchConfiguration('adapter', cfg.get('adapter',context.launch_configurations.get('robot','m20'))))
    return actions


def generate_launch_description():
    # A profile may change sensor contracts, but never silently enables control.
    adapter_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(PythonExpression(['"adapter_" + "',LaunchConfiguration('adapter'),'"'])),
                              'launch','adapter.launch.py'])),condition=IfCondition(LaunchConfiguration('use_adapter')),
        launch_arguments={'robot_config':LaunchConfiguration('robot_config'),
                          'robot':LaunchConfiguration('robot'),
                          'use_sim_time':LaunchConfiguration('use_sim_time'),
                          'enable_control':LaunchConfiguration('enable_control')}.items())
    return LaunchDescription([
        DeclareLaunchArgument('robot',default_value='m20'),
        DeclareLaunchArgument('robot_config',default_value=''),
        DeclareLaunchArgument('adapter',default_value='m20'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('use_adapter',default_value='false'),
        DeclareLaunchArgument('use_nav',default_value='false'),
        DeclareLaunchArgument('enable_control',default_value='false'),
        DeclareLaunchArgument('map_file_path',default_value=os.path.expanduser('~/.ros/universal_slam_map.pcd')),
        OpaqueFunction(function=_make_runtime),
        OpaqueFunction(function=_make_static_tf_nodes),
        OpaqueFunction(function=_make_fastlio_launch),
        adapter_launch,OpaqueFunction(function=_adapter_status_log),
    ])
