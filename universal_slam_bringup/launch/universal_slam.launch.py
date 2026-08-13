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
                            LogInfo, OpaqueFunction)
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
    """读取 install 的 robot_configs/<robot>.yaml；失败返回 None（回退默认外参）。"""
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
    """按 robot:= 读取 lidar 参数并注入 fastlio.launch.py。

    数字参数在 _load_robot_config 后以 int()/float() 原生转换，
    再经 launch_arguments 传入；fastlio.launch.py 内部 build_fastlio_parameters
    保证到达 Node 前全部为原生 int/float（契约测试覆盖，禁止字符串注入）。
    外参（extrinsics.imu / extrinsics.lidar）经 lidar_to_imu_extrinsic 换算为
    FAST-LIO extrinsic_T/R 并注入；数值为配置初值，实机必须标定（P4）。
    """
    robot_name = context.launch_configurations.get('robot', 'm20')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
    cfg = _load_robot_config(robot_name) or {}
    lidar_cfg = cfg.get('lidar', {}) or {}

    extra_args = {'use_sim_time': use_sim_time}
    if 'scan_line' in lidar_cfg:
        extra_args['scan_line'] = str(int(lidar_cfg['scan_line']))
    if 'lidar_type' in lidar_cfg:
        extra_args['lidar_type'] = str(int(lidar_cfg['lidar_type']))
    if 'timestamp_unit' in lidar_cfg:
        extra_args['timestamp_unit'] = str(int(lidar_cfg['timestamp_unit']))
    if 'blind' in lidar_cfg:
        extra_args['blind'] = str(float(lidar_cfg['blind']))

    # 外参 → FAST-LIO extrinsic_T/R（lidar→IMU）；两份外参都缺失时不覆盖参数文件
    extrinsics = cfg.get('extrinsics', {}) or {}
    imu_ext = extrinsics.get('imu', {})
    lidar_ext = extrinsics.get('lidar', {})
    if imu_ext and lidar_ext:
        t, r = lidar_to_imu_extrinsic(imu_ext, lidar_ext)
        extra_args['extrinsic_T'] = ','.join(f'{v:.6f}' for v in t)
        extra_args['extrinsic_R'] = ','.join(f'{v:.6f}' for v in r)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('slam_nav_core'), 'launch', 'fastlio.launch.py'])),
            launch_arguments=extra_args.items(),
        )
    ]


def _ext_xyz_quat(ext):
    """外参 dict → ((x,y,z), (qx,qy,qz,qw))（浮点；RPY 单位为度）。"""
    xyz, (roll, pitch, yaw) = _ext_xyz_rpy(ext)
    return xyz, _rpy_to_quat(roll, pitch, yaw)


def _xyz_quat_args(ext):
    """外参 dict → static_transform_publisher 参数（xyz + RPY→四元数 x y z w，字符串）。"""
    xyz, quat = _ext_xyz_quat(ext)
    return [str(v) for v in (*xyz, *quat)]


def build_static_tf_descriptions(robot_name, cfg):
    """纯函数（可测）：机型配置 → 静态 TF 发布节点描述（name + 参数字符串列表）。

    review 修复（body=IMU 物理语义）：FAST-LIO 以 IMU 为 body 帧发布
    camera_init→body 动态 TF，故 body 与 imu 是同一物理系。
    body→base_link 必须是 (base_link→imu)⁻¹（逆变换），使
    body→base_link→imu 合成恒等 —— 外参只定义一次（extrinsics.imu），
    不再出现"body→base_link 恒等"与"base_link→imu 非零"的矛盾。
    """
    extrinsics = (cfg or {}).get('extrinsics', {}) or {}
    imu_ext = extrinsics.get('imu', {})
    lidar_ext = extrinsics.get('lidar', {})
    if not imu_ext:
        print(f'[universal_slam] 警告: robot_configs/{robot_name}.yaml 缺少 extrinsics.imu，使用默认外参')
    if not lidar_ext:
        print(f'[universal_slam] 警告: robot_configs/{robot_name}.yaml 缺少 extrinsics.lidar，使用默认外参')

    imu_xyz, imu_quat = _ext_xyz_quat(imu_ext)
    # body(=IMU)→base_link = (base_link→imu)⁻¹
    inv_xyz, inv_q = _tf_inverse(imu_xyz, imu_quat)
    body_args = [str(v) for v in (*inv_xyz, *inv_q)] + ['body', 'base_link']

    return [
        # map→odom 恒等（D9: 静态层与 FAST-LIO camera_init 帧对齐；偏移参数 P4 预留）
        dict(name=None, args=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']),
        # odom→camera_init：衔接 FAST-LIO camera_init 坐标系与 REP-105 odom 帧
        dict(name='odom_to_camera_init_tf',
             args=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init']),
        # body→base_link 静态补齐：body=IMU（FAST-LIO body 帧即 IMU 系），
        # 取 (base_link→imu) 的逆变换，保证合成恒等
        dict(name=None, args=body_args),
        # base_link→imu（外参初值，robot_configs/<robot>.yaml）
        dict(name=None, args=_xyz_quat_args(imu_ext) + ['base_link', 'imu']),
        # base_link→lidar（外参初值，robot_configs/<robot>.yaml）
        dict(name=None, args=_xyz_quat_args(lidar_ext) + ['base_link', 'lidar']),
    ]


def _make_static_tf_nodes(context):
    """按 robot:= 机型参数构建静态 TF 发布节点（从 yaml 读取外参）。"""
    robot_name = context.launch_configurations.get('robot', 'm20')
    cfg = _load_robot_config(robot_name) or {}
    return [
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=d['args'], name=d['name'] or None)
        for d in build_static_tf_descriptions(robot_name, cfg)
    ]


def _adapter_status_log(context):
    """use_adapter:=false 时明确提醒：数据源必须由外部提供（非零配置）。"""
    use_adapter = str(context.launch_configurations.get('use_adapter', 'false')).lower()
    robot = context.launch_configurations.get('robot', 'm20')
    if use_adapter in ('false', '0'):
        if robot == 'sw01':
            msg = ('[universal_slam] use_adapter=false 且 robot=sw01：SW01 无 ROS，'
                   '/points_raw 与 /imu_raw 没有外部来源，请使用 use_adapter:=true '
                   '（需先以 ASTRALL_SDK_DIR 构建 adapter_sw01）')
        else:
            msg = ('[universal_slam] use_adapter=false：/points_raw 与 /imu_raw 必须由外部提供'
                   '（M20 直连 DDS：同 ROS_DOMAIN_ID + 同 RMW + 有线千兆可达，'
                   '或两端同版本 zenoh-bridge-ros2dds），本 launch 不负责桥接配置')
        return [LogInfo(msg=msg)]
    return []


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

    # ---- 机型适配器（use_adapter:=true 时启动所选机型的 adapter.launch.py）----
    # 缺 launch 文件/缺可执行（SW01 未以 SDK 构建）→ FindPackageShare/launch 直接
    # 抛错（fail-fast），不会静默跳过适配器。
    adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(PythonExpression(['"adapter_" + "', robot, '"'])),
                'launch', 'adapter.launch.py'])),
        condition=IfCondition(use_adapter),
    )

    # ---- 静态 TF（OpaqueFunction: 运行时按 robot:= 读取 robot_configs yaml）----
    static_tf_action = OpaqueFunction(function=_make_static_tf_nodes)

    # ---- use_adapter:=false 时打印数据源提醒 ----
    adapter_status_log = OpaqueFunction(function=_adapter_status_log)

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='m20', choices=['m20', 'sw01'],
            description='目标机型（对应 robot_configs/config/<robot>.yaml）'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='是否使用仿真时钟'),
        DeclareLaunchArgument(
            'use_adapter', default_value='false',
            description='是否启动机型适配器（true 时包含 adapter_<robot>/launch/adapter.launch.py）'),

        static_tf_action,
        fastlio_action,
        nav_stack_launch,
        adapter_launch,
        adapter_status_log,
    ])
