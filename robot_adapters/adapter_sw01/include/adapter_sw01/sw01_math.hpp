// ============================================================
// adapter_sw01 / include/adapter_sw01/sw01_math.hpp
// 可测纯函数（P0 安全修复）：
//   1. cmd_vel → AstrallMove 归一化：在任何除法/归一化之前拒绝 NaN/Inf
//   2. 启动参数校验：有限正数 / 端口范围 / 控制频率下限，非法即 fail-fast
// 单测：test/test_sw01_math.cpp
// ============================================================
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace adapter_sw01 {

/// 归一化钳制 [-1, 1]
inline double clampUnit(double v) { return std::max(-1.0, std::min(1.0, v)); }

/// cmd_vel 归一化结果；valid=false 表示输入或限值含 NaN/Inf，调用方必须丢弃该指令。
struct NormalizedCmd
{
  double x{0.0}, y{0.0}, yaw{0.0};
  bool valid{false};
};

/// cmd_vel(m/s, rad/s) / 机型最大速度 → SDK 归一化 [-1,1]（不取整，AstrallMove 为 float）。
/// 任何输入（含限值）非有限 → 返回 invalid（不产生任何除法结果）。
inline NormalizedCmd normalizeCmdVel(
  double vx, double vy, double vw, double vx_max, double vy_max, double wz_max)
{
  NormalizedCmd out;
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vw) ||
    !std::isfinite(vx_max) || !std::isfinite(vy_max) || !std::isfinite(wz_max))
  {
    return out;  // 拒绝 NaN/Inf，除法前返回
  }
  out.valid = true;
  out.x = clampUnit(vx / vx_max);
  out.y = clampUnit(vy / vy_max);
  out.yaw = clampUnit(vw / wz_max);
  return out;
}

/// 启动参数：必须为有限正数，否则 fail-fast（构造节点时抛异常）。
inline double requireFinitePositive(double v, const std::string & name)
{
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::invalid_argument(
      "参数 '" + name + "' 必须为有限正数, 实际=" + std::to_string(v));
  }
  return v;
}

/// 启动参数：必须为有限正数且 ≥ min（SW01: SDK 契约要求 AstrallMove ≥20Hz，
/// 机器人侧 50ms 看门狗，低于 20Hz 会在边界处触发自动停止）。
inline double requireFiniteAtLeast(double v, const std::string & name, double min_v)
{
  requireFinitePositive(v, name);
  if (v < min_v) {
    throw std::invalid_argument(
      "参数 '" + name + "' 必须 ≥ " + std::to_string(min_v) + ", 实际=" + std::to_string(v));
  }
  return v;
}

/// 定时器频率上限校验（防止 1000/hz 取整为 0ms 导致空转定时器）。
inline double requireFrequencyHz(double v, const std::string & name, double max_hz)
{
  requireFinitePositive(v, name);
  if (v > max_hz) {
    throw std::invalid_argument(
      "参数 '" + name + "' 过大(>" + std::to_string(max_hz) + " Hz), 实际=" + std::to_string(v));
  }
  return v;
}

/// UDP 端口范围校验 [1, 65535]。
inline int requirePortRange(int v, const std::string & name)
{
  if (v < 1 || v > 65535) {
    throw std::invalid_argument(
      "参数 '" + name + "' 必须在 [1,65535], 实际=" + std::to_string(v));
  }
  return v;
}

}  // namespace adapter_sw01
