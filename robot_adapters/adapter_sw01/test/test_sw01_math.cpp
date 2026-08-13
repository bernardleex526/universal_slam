// ============================================================
// adapter_sw01 / test/test_sw01_math.cpp
// P0 验收：cmd_vel 归一化在任何除法前拒绝 NaN/Inf；启动参数 fail-fast。
// ============================================================
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "adapter_sw01/sw01_math.hpp"

using namespace adapter_sw01;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// ---- 归一化：正常路径 ----
TEST(Sw01Math, NormalizeValid)
{
  const auto c = normalizeCmdVel(0.25, -0.5, 0.6, 0.5, 0.5, 0.6);
  ASSERT_TRUE(c.valid);
  EXPECT_DOUBLE_EQ(c.x, 0.5);
  EXPECT_DOUBLE_EQ(c.y, -1.0);
  EXPECT_DOUBLE_EQ(c.yaw, 1.0);
}

// ---- 归一化：超速钳制 ----
TEST(Sw01Math, NormalizeClamp)
{
  const auto c = normalizeCmdVel(5.0, -5.0, -5.0, 0.5, 0.5, 0.6);
  ASSERT_TRUE(c.valid);
  EXPECT_DOUBLE_EQ(c.x, 1.0);
  EXPECT_DOUBLE_EQ(c.y, -1.0);
  EXPECT_DOUBLE_EQ(c.yaw, -1.0);
}

// ---- 归一化：任何输入 NaN/Inf → invalid（除法前拒绝）----
TEST(Sw01Math, NormalizeRejectsNonFinite)
{
  // 命令输入非有限
  EXPECT_FALSE(normalizeCmdVel(kNaN, 0.0, 0.0, 0.5, 0.5, 0.6).valid);
  EXPECT_FALSE(normalizeCmdVel(kInf, 0.0, 0.0, 0.5, 0.5, 0.6).valid);
  EXPECT_FALSE(normalizeCmdVel(-kInf, 0.0, 0.0, 0.5, 0.5, 0.6).valid);
  EXPECT_FALSE(normalizeCmdVel(0.0, kNaN, 0.0, 0.5, 0.5, 0.6).valid);
  EXPECT_FALSE(normalizeCmdVel(0.0, 0.0, kNaN, 0.5, 0.5, 0.6).valid);
  // 限值非有限（构造时已被校验，双保险）
  EXPECT_FALSE(normalizeCmdVel(0.1, 0.0, 0.0, kNaN, 0.5, 0.6).valid);
  EXPECT_FALSE(normalizeCmdVel(0.1, 0.0, 0.0, 0.5, kInf, 0.6).valid);
  EXPECT_FALSE(normalizeCmdVel(0.1, 0.0, 0.0, 0.5, 0.5, -kInf).valid);
  // invalid 结果的输出必须为 0（调用方按零速处理）
  const auto c = normalizeCmdVel(kNaN, 0.0, 0.0, 0.5, 0.5, 0.6);
  EXPECT_EQ(c.x, 0.0);
  EXPECT_EQ(c.y, 0.0);
  EXPECT_EQ(c.yaw, 0.0);
}

// ---- clampUnit ----
TEST(Sw01Math, ClampUnit)
{
  EXPECT_DOUBLE_EQ(clampUnit(2.0), 1.0);
  EXPECT_DOUBLE_EQ(clampUnit(-2.0), -1.0);
  EXPECT_DOUBLE_EQ(clampUnit(0.3), 0.3);
}

// ---- 启动参数 fail-fast：有限正数 ----
TEST(Sw01Math, RequireFinitePositive)
{
  EXPECT_DOUBLE_EQ(requireFinitePositive(0.5, "vx_max"), 0.5);
  EXPECT_THROW(requireFinitePositive(0.0, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(-0.1, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(kNaN, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(kInf, "vx_max"), std::invalid_argument);
}

// ---- 启动参数 fail-fast：频率下限（SW01 SDK ≥20Hz 契约）----
TEST(Sw01Math, RequireFiniteAtLeast)
{
  EXPECT_DOUBLE_EQ(requireFiniteAtLeast(20.0, "cmd_frequency_hz", 20.0), 20.0);
  EXPECT_DOUBLE_EQ(requireFiniteAtLeast(30.0, "cmd_frequency_hz", 20.0), 30.0);
  EXPECT_THROW(requireFiniteAtLeast(19.9, "cmd_frequency_hz", 20.0), std::invalid_argument);
  EXPECT_THROW(requireFiniteAtLeast(kNaN, "cmd_frequency_hz", 20.0), std::invalid_argument);
}

// ---- 启动参数 fail-fast：频率上限 / 端口范围 ----
TEST(Sw01Math, RequireFrequencyAndPort)
{
  EXPECT_DOUBLE_EQ(requireFrequencyHz(20.0, "cmd_frequency_hz", 200.0), 20.0);
  EXPECT_THROW(requireFrequencyHz(201.0, "cmd_frequency_hz", 200.0), std::invalid_argument);
  EXPECT_THROW(requireFrequencyHz(kInf, "cmd_frequency_hz", 200.0), std::invalid_argument);
  EXPECT_THROW(requireFrequencyHz(0.0, "cmd_frequency_hz", 200.0), std::invalid_argument);

  EXPECT_EQ(requirePortRange(6100, "lidar_port"), 6100);
  EXPECT_EQ(requirePortRange(1, "lidar_port"), 1);
  EXPECT_EQ(requirePortRange(65535, "lidar_port"), 65535);
  EXPECT_THROW(requirePortRange(0, "lidar_port"), std::invalid_argument);
  EXPECT_THROW(requirePortRange(-1, "lidar_port"), std::invalid_argument);
  EXPECT_THROW(requirePortRange(65536, "lidar_port"), std::invalid_argument);
}
