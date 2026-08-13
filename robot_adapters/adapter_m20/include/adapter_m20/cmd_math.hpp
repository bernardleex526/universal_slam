// ============================================================
// adapter_m20 / include/adapter_m20/cmd_math.hpp
// 可测纯函数（P0 安全修复）：
//   1. cmd_vel → 归一化轴指令：在任何除法/归一化之前拒绝 NaN/Inf
//   2. 启动参数校验：有限正数 / 端口范围，非法即 fail-fast
// 单测：test/test_cmd_math.cpp
// ============================================================
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace adapter_m20 {

/// 归一化钳制 [-1, 1]
inline double clampUnit(double v) { return std::max(-1.0, std::min(1.0, v)); }

/// 四舍五入保留 2 位小数（与官方 SDK 一致）
inline double round2(double v) { return std::round(v * 100.0) / 100.0; }

/// cmd_vel 归一化结果；valid=false 表示输入或限值含 NaN/Inf，调用方必须丢弃该指令。
struct NormalizedCmd
{
  double x{0.0}, y{0.0}, yaw{0.0};
  bool valid{false};
};

/// cmd_vel(m/s, rad/s) / 机型最大速度 → 归一化 [-1,1]（round2）。
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
  out.x = round2(clampUnit(vx / vx_max));
  out.y = round2(clampUnit(vy / vy_max));
  out.yaw = round2(clampUnit(vw / wz_max));
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

/// 整数范围校验 [min, max]。
inline int requireIntRange(int v, const std::string & name, int min_v, int max_v)
{
  if (v < min_v || v > max_v) {
    throw std::invalid_argument(
      "参数 '" + name + "' 必须在 [" + std::to_string(min_v) + "," +
      std::to_string(max_v) + "], 实际=" + std::to_string(v));
  }
  return v;
}

}  // namespace adapter_m20
