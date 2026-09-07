#!/usr/bin/env python3
# ============================================================
# universal_slam_bringup / test/test_universal_slam_launch.py
# P0 契约测试：
#   1) use_adapter 真正控制所选机型 adapter launch 的包含（IfCondition）
#   2) 注入 FAST-LIO 的数字参数经 launch_arguments 后到达节点仍为原生
#      int/float（与 slam_nav_core 的 build_fastlio_parameters 端到端对拍）
#   3) use_adapter:=false 时产生明确提醒（非零配置声明）
#   4) 静态 TF 参数为数值字符串（build_static_tf_descriptions 纯函数）
# ============================================================
import importlib.util
import math
import os
import sys

import pytest


def _load_module(name, relpath):
    """从仓库根加载 launch 模块（relpath 相对仓库根）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, '..', '..'))
    path = os.path.normpath(os.path.join(root, relpath))
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


bringup = _load_module(
    'universal_slam_launch_under_test', 'universal_slam_bringup/launch/universal_slam.launch.py')
fastlio = _load_module(
    'fastlio_launch_for_bringup_test', 'slam_nav_core/launch/fastlio.launch.py')


def _ctx(**kwargs):
    from launch.launch_context import LaunchContext
    ctx = LaunchContext()
    ctx.launch_configurations.update(kwargs)
    return ctx


def test_generate_launch_description_ok():
    ld = bringup.generate_launch_description()
    entities = list(ld.entities)
    assert entities, 'LaunchDescription 不应为空'


def test_adapter_include_gated_by_use_adapter():
    """适配器 Include 必须受 IfCondition(use_adapter) 控制（真正接线）。"""
    from launch.actions import IncludeLaunchDescription
    from launch.conditions import IfCondition

    ld = bringup.generate_launch_description()
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    adapter_include = None
    for inc in includes:
        # PythonLaunchDescriptionSource.location 在本版本返回描述字符串
        if 'adapter.launch.py' in str(inc.launch_description_source.location):
            adapter_include = inc
            break
    assert adapter_include is not None, '缺少 adapter launch 的 IncludeLaunchDescription'
    assert isinstance(adapter_include.condition, IfCondition), \
        'adapter include 必须由 IfCondition(use_adapter) 门控'
    # condition 表达式引用了 use_adapter 变量（本版本无公开属性，检查内部存储）
    pred = adapter_include.condition._IfCondition__predicate_expression

    def _flatten(xs):
        for x in xs:
            if isinstance(x, (list, tuple)):
                yield from _flatten(x)
            else:
                yield x

    described = [getattr(x, 'describe', lambda: '')() for x in _flatten(pred)]
    assert any('use_adapter' in d for d in described), \
        'IfCondition 必须绑定 use_adapter'


def test_fastlio_injected_values_reach_node_as_native_types():
    """端到端：bringup 按 yaml 注入 → fastlio.build_fastlio_parameters → 原生类型。"""
    # 使用真实 m20.yaml 的 lidar 段
    cfg = bringup._load_robot_config('m20')
    assert cfg is not None, '无法加载 robot_configs/m20.yaml'
    lidar_cfg = cfg.get('lidar', {}) or {}
    assert 'scan_line' in lidar_cfg and 'lidar_type' in lidar_cfg

    ctx = _ctx(robot='m20', use_sim_time='false')
    actions = bringup._make_fastlio_launch(ctx)
    assert len(actions) == 1
    inc = actions[0]
    args = dict(inc.launch_arguments) if inc.launch_arguments else {}

    # launch_arguments 为字符串（框架边界），但必须与 yaml 原生值一致
    assert args['scan_line'] == str(int(lidar_cfg['scan_line']))
    assert args['lidar_type'] == str(int(lidar_cfg['lidar_type']))
    assert args['timestamp_unit'] == str(int(lidar_cfg['timestamp_unit']))
    assert args['blind'] == str(float(lidar_cfg['blind']))

    # 对拍 fastlio 侧转换：到达 Node 前的参数必须是原生类型（string abort 消失）
    params = fastlio.build_fastlio_parameters(args)
    assert type(params['preprocess.scan_line']) is int
    assert type(params['preprocess.lidar_type']) is int
    assert type(params['preprocess.timestamp_unit']) is int
    assert type(params['preprocess.blind']) is float
    assert type(params['use_sim_time']) is bool
    assert params['preprocess.scan_line'] == int(lidar_cfg['scan_line'])
    assert params['preprocess.blind'] == float(lidar_cfg['blind'])


def test_static_tf_descriptions_numeric_strings():
    cfg=bringup._load_robot_config('m20')
    descs=bringup.build_static_tf_descriptions('m20',cfg)
    assert len(descs)==3
    for d in descs:
        assert all(math.isfinite(float(v)) for v in d['args'][:7])
    assert descs[0]['args'][-2:]==['uslam/map','uslam/odom']
    assert descs[1]['args'][-2:]==['uslam/base_link','uslam/imu']
    assert descs[2]['args'][-2:]==['uslam/base_link','uslam/lidar']
    # base_link has only the dynamic odometry adapter as its parent.
    assert all(d['args'][-1]!='uslam/base_link' for d in descs)
    assert len(bringup._make_static_tf_nodes(_ctx(robot='m20')))==3


def test_navigation_and_control_reject_unverified_profile():
    for settings in ({'use_nav':'true'},{'enable_control':'true','use_adapter':'true'}):
        with pytest.raises(RuntimeError,match='calibration'):
            bringup._make_runtime(_ctx(robot='m20',**settings))



# ---- P1: RPY→四元数 与 lidar→IMU 外参换算（纯数学，实机值需标定）----

def _close(a, b, eps=1e-9):
    return all(abs(x - y) < eps for x, y in zip(a, b))


def test_rpy_to_quat_known_angles():
    q = bringup._rpy_to_quat
    # 全零 → 单位四元数
    assert _close(q(0, 0, 0), (0, 0, 0, 1))
    # 单轴 90°（yaw）→ (0, 0, sin45, cos45)
    s = math.sin(math.radians(45.0))
    assert _close(q(0, 0, math.radians(90.0)), (0, 0, s, s))
    # roll 90° → (sin45, 0, 0, cos45)
    assert _close(q(math.radians(90.0), 0, 0), (s, 0, 0, s))
    # pitch 90° → (0, sin45, 0, cos45)
    assert _close(q(0, math.radians(90.0), 0), (0, s, 0, s))
    # 四元数模长恒为 1
    qq = q(math.radians(10.0), math.radians(-20.0), math.radians(35.0))
    assert abs(sum(v * v for v in qq) - 1.0) < 1e-12


def test_xyz_quat_args_uses_rpy():
    args = bringup._xyz_quat_args(
        {'x': 1.0, 'y': 2.0, 'z': 3.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 90.0})
    s = math.sin(math.radians(45.0))
    assert float(args[0]) == 1.0 and float(args[1]) == 2.0 and float(args[2]) == 3.0
    assert _close([float(v) for v in args[3:7]], (0, 0, s, s))


def test_lidar_to_imu_extrinsic_identity_and_translation():
    # 恒等：两份外参相同 → T=0, R=I
    ext = {'x': 0.1, 'y': 0.2, 'z': 0.3, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    t, r = bringup.lidar_to_imu_extrinsic(ext, dict(ext))
    assert _close(t, (0, 0, 0), 1e-12)
    assert _close(r, (1, 0, 0, 0, 1, 0, 0, 0, 1), 1e-12)

    # SW01 手册值（identity 旋转）：T = t_lidar - t_imu
    imu = {'x': -0.00115, 'y': 0.0, 'z': 0.0595, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    lidar = {'x': 0.3732, 'y': 0.0, 'z': 0.080, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    t, r = bringup.lidar_to_imu_extrinsic(imu, lidar)
    assert _close(t, (0.37435, 0.0, 0.0205), 1e-9)
    assert _close(r, (1, 0, 0, 0, 1, 0, 0, 0, 1), 1e-12)


def test_lidar_to_imu_extrinsic_rotation():
    # imu 绕 z 转 90°（lidar 无旋转）：R_L2I = Rz(90)^T = Rz(-90)
    imu = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 90.0}
    lidar = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
    t, r = bringup.lidar_to_imu_extrinsic(imu, lidar)
    assert _close(t, (0, 0, 0), 1e-12)
    # Rz(-90): [[cos(-90),-sin(-90),0],[sin(-90),cos(-90),0],[0,0,1]] = [[0,1,0],[-1,0,0],[0,0,1]]
    assert _close(r, (0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0), 1e-12)


def _parse_tf_args(args):
    """静态 TF 描述参数 → ((x,y,z), (qx,qy,qz,qw)) 浮点。"""
    xyz = tuple(float(v) for v in args[0:3])
    quat = tuple(float(v) for v in args[3:7])
    return xyz, quat


def test_tf_inverse_preserves_arbitrary_rigid_transform():
    xyz=(1.5,-2.,.5)
    quat=bringup._rpy_to_quat(.2,-.4,1.1)
    t,q=bringup._tf_compose(*bringup._tf_inverse(xyz,quat),xyz,quat)
    assert _close(t,(0,0,0),1e-9)
    assert _close(q[:3],(0,0,0),1e-9) and abs(abs(q[3])-1)<1e-9



def test_fastlio_launch_injects_extrinsics_as_typed_lists():
    """端到端：sw01 config → _make_fastlio_launch 注入 extrinsic_T/R →
    fastlio.build_fastlio_parameters 解析为原生 float 列表。"""
    ctx = _ctx(robot='sw01', use_sim_time='false')
    actions = bringup._make_fastlio_launch(ctx)
    args = dict(actions[0].launch_arguments)
    assert 'extrinsic_T' in args and 'extrinsic_R' in args
    # sw01 手册初值换算结果（见 test_lidar_to_imu_extrinsic_identity_and_translation）
    t = [float(v) for v in args['extrinsic_T'].split(',')]
    assert _close(t, (0.37435, 0.0, 0.0205), 1e-3)
    r = [float(v) for v in args['extrinsic_R'].split(',')]
    assert len(r) == 9 and _close(r, (1, 0, 0, 0, 1, 0, 0, 0, 1), 1e-3)
    # fastlio 侧：原生 float 列表（非字符串）
    params = fastlio.build_fastlio_parameters(args)
    assert all(type(v) is float for v in params['mapping.extrinsic_T'])
    assert all(type(v) is float for v in params['mapping.extrinsic_R'])


# 注意：以上外参数值均为 robot_configs 初值/手册值 —— 实机标定（P4）前
# 不代表真实外参；本测试只锁定换算正确性与类型契约。


def test_adapter_status_log_reminds_when_disabled():
    from launch.actions import LogInfo

    # use_adapter=false → 提醒
    acts = bringup._adapter_status_log(_ctx(use_adapter='false', robot='m20'))
    assert len(acts) == 1 and isinstance(acts[0], LogInfo)
    texts = ''.join(getattr(s, 'text', '') for s in acts[0].msg)
    assert 'use_adapter=false' in texts
    # SW01 + false → 明确提示没有外部数据源
    acts = bringup._adapter_status_log(_ctx(use_adapter='false', robot='sw01'))
    sw01_text = ''.join(getattr(s, 'text', '') for s in acts[0].msg)
    assert 'use_adapter:=true' in sw01_text
    # use_adapter=true → 无提醒
    assert bringup._adapter_status_log(_ctx(use_adapter='true', robot='m20')) == []


def test_sw01_adapter_launch_fails_fast_without_executable():
    """SW01 adapter.launch.py：缺可执行文件必须抛 RuntimeError（fail-fast）。"""
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(
        here, '..', '..', 'robot_adapters', 'adapter_sw01', 'launch',
        'adapter.launch.py'))
    if not os.path.exists(path):
        pytest.skip('adapter_sw01 launch 文件不在预期位置（monorepo 布局）')
    spec = importlib.util.spec_from_file_location('sw01_adapter_launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    # 若 adapter_sw01 未构建（本环境无 ASTRALL_SDK_DIR），应抛 RuntimeError
    # 而不是静默返回空 LaunchDescription
    try:
        mod.generate_launch_description()
    except RuntimeError as e:
        assert 'sw01_adapter' in str(e)
    else:
        # 已构建的部署环境：必须返回包含 sw01_adapter Node 的 LD
        from launch_ros.actions import Node
        nodes = [e for e in mod.generate_launch_description().entities
                 if isinstance(e, Node)]
        assert len(nodes) == 1
        assert 'sw01_adapter' in str(nodes[0]._Node__node_name)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
