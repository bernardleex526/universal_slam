#!/usr/bin/env python3
# ============================================================
# slam_nav_core / test/test_fastlio_launch.py
# P0 契约测试：FAST-LIO 数字参数到达 Node 前必须为原生 int/float/bool。
#
# 背景（实测 string abort）：FAST-LIO 用
#   declare_parameter<int>("preprocess.scan_line") 等声明参数，
#   若 launch 传入字符串，rclcpp 参数类型不匹配 → 节点 abort。
#   本测试锁定该回归路径已消失。
# ============================================================
import importlib.util
import os
import sys

import pytest


def _load_module():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(here, '..', 'launch', 'fastlio.launch.py'))
    spec = importlib.util.spec_from_file_location('fastlio_launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


module = _load_module()

# FAST-LIO 侧参数声明的原生类型（见锁定 commit 2fffc57 laserMapping.cpp）
NUMERIC_KEYS = {
    'preprocess.scan_line': int,
    'preprocess.lidar_type': int,
    'preprocess.timestamp_unit': int,
    'preprocess.blind': float,
}
BOOL_KEYS = {'use_sim_time': bool}


def test_defaults_are_native_types():
    """默认 launch 配置（字符串）→ 原生类型参数（复现 string abort 的输入）。"""
    params = module.build_fastlio_parameters({})
    for key, expect in NUMERIC_KEYS.items():
        assert key in params, f'缺少参数 {key}'
        assert type(params[key]) is expect, f'{key} 应为 {expect}, 实际 {type(params[key])}'
        assert not isinstance(params[key], str), f'{key} 不得为字符串'
    for key in BOOL_KEYS:
        assert type(params[key]) is bool, f'{key} 应为 bool'


def test_explicit_string_config_converted():
    """bringup 通过 launch_arguments 传入的字符串值必须被转换为原生类型。"""
    params = module.build_fastlio_parameters({
        'use_sim_time': 'false',
        'scan_line': '64',
        'lidar_type': '2',
        'timestamp_unit': '0',
        'blind': '2.5',
    })
    assert params['use_sim_time'] is False
    assert params['preprocess.scan_line'] == 64 and type(params['preprocess.scan_line']) is int
    assert params['preprocess.lidar_type'] == 2 and type(params['preprocess.lidar_type']) is int
    assert params['preprocess.timestamp_unit'] == 0 and \
        type(params['preprocess.timestamp_unit']) is int
    assert params['preprocess.blind'] == 2.5 and type(params['preprocess.blind']) is float


def test_use_sim_time_true_variants():
    for raw in ('true', 'True', '1'):
        assert module.build_fastlio_parameters({'use_sim_time': raw})['use_sim_time'] is True
    for raw in ('false', '0'):
        assert module.build_fastlio_parameters({'use_sim_time': raw})['use_sim_time'] is False


def test_extrinsic_override_typed_lists():
    params = module.build_fastlio_parameters({
        'extrinsic_T': '0.1,0.2,0.3',
        'extrinsic_R': '1,0,0,0,1,0,0,0,1',
    })
    assert params['mapping.extrinsic_T'] == [0.1, 0.2, 0.3]
    assert params['mapping.extrinsic_R'] == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert all(type(v) is float for v in params['mapping.extrinsic_T'])
    assert all(type(v) is float for v in params['mapping.extrinsic_R'])

    # 空串 → 不覆盖参数文件默认值
    assert 'mapping.extrinsic_T' not in module.build_fastlio_parameters({})
    assert 'mapping.extrinsic_R' not in module.build_fastlio_parameters({})


def test_invalid_values_fail_fast():
    """非法值必须抛 RuntimeError（fail-fast），不得静默传入节点。"""
    for bad in (
        {'scan_line': 'abc'},
        {'scan_line': '9 6'},
        {'lidar_type': ''},
        {'timestamp_unit': 'NaN'},
        {'blind': 'not-a-float'},
        {'use_sim_time': 'maybe'},
        {'extrinsic_T': '1,2'},
        {'extrinsic_R': '1,2,3,4,5,6,7,8'},
        {'extrinsic_T': '1,x,3'},
    ):
        with pytest.raises(RuntimeError):
            module.build_fastlio_parameters(bad)


def test_opaque_function_builds_node_with_typed_parameters():
    """端到端：generate_launch_description + OpaqueFunction 构造的 Node
    其 parameters 中数字键全部为原生类型（string abort 路径必须消失）。"""
    from launch.launch_context import LaunchContext

    ld = module.generate_launch_description()
    opaque = None
    for entity in ld.entities:
        if type(entity).__name__ == 'OpaqueFunction':
            opaque = entity
            break
    assert opaque is not None, 'fastlio.launch.py 应包含 OpaqueFunction'

    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'params_file': '/dummy/fastlio_params.yaml',
        'use_sim_time': 'false',
        'scan_line': '96',
        'lidar_type': '2',
        'timestamp_unit': '2',
        'blind': '2.0',
    })
    # OpaqueFunction.execute(context) 调用注册的函数并返回实体列表
    nodes = opaque.execute(ctx)
    assert nodes and len(nodes) == 1

    # Node 内部保存参数元组：ParameterFile(参数文件) + 归一化后的参数 dict。
    # 归一化后 dict 的键为 substitutions 元组，值保持原生 int/float/bool。
    raw_params = nodes[0]._Node__parameters
    param_dicts = [p for p in raw_params if isinstance(p, dict)]
    assert param_dicts, 'Node 应包含参数 dict'

    def key_str(key):
        if isinstance(key, tuple):
            return ''.join(getattr(s, 'text', str(s)) for s in key)
        return str(key)

    # 合并所有 dict（可能有多个），断言数字键类型
    merged = {}
    for d in param_dicts:
        for k, v in d.items():
            merged[key_str(k)] = v
    for key, expect in NUMERIC_KEYS.items():
        assert type(merged[key]) is expect, f'Node 参数 {key} 应为 {expect}'
    assert type(merged['use_sim_time']) is bool
    # 直接回归断言：绝不出现字符串
    for key in list(NUMERIC_KEYS) + list(BOOL_KEYS):
        assert not isinstance(merged[key], str), \
            f'Node 参数 {key} 为字符串 —— string abort 回归！'


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
