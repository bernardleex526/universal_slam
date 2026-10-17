// Startup parameter validation for the M20 Pro UDP backend; invalid values fail fast.
#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

namespace m20pro_bridge {

inline double requireFinitePositive(double v, const std::string & name)
{
  if (!std::isfinite(v) || v <= 0.0) {
    throw std::invalid_argument(
      "参数 '" + name + "' 必须为有限正数, 实际=" + std::to_string(v));
  }
  return v;
}

/// Upper bound keeps the wall-timer period from collapsing to zero.
inline double requireFrequencyHz(double v, const std::string & name, double max_hz)
{
  requireFinitePositive(v, name);
  if (v > max_hz) {
    throw std::invalid_argument(
      "参数 '" + name + "' 过大(>" + std::to_string(max_hz) + " Hz), 实际=" + std::to_string(v));
  }
  return v;
}

inline int requirePortRange(int v, const std::string & name)
{
  if (v < 1 || v > 65535) {
    throw std::invalid_argument(
      "参数 '" + name + "' 必须在 [1,65535], 实际=" + std::to_string(v));
  }
  return v;
}

}  // namespace m20pro_bridge
