// ============================================================
// adapter_m20 / test/test_cmd_math.cpp
// P0 验收：cmd_vel 归一化在任何除法前拒绝 NaN/Inf；启动参数 fail-fast。
// ============================================================
#include <gtest/gtest.h>

#include <limits>

#include "adapter_m20/cmd_math.hpp"

using namespace adapter_m20;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// ---- 归一化：正常路径（含 round2）----
TEST(CmdMath, NormalizeValid)
{
  const auto c = normalizeCmdVel(0.25, -0.5, 0.6, 0.5, 0.5, 0.6);
  ASSERT_TRUE(c.valid);
  EXPECT_DOUBLE_EQ(c.x, 0.5);
  EXPECT_DOUBLE_EQ(c.y, -1.0);
  EXPECT_DOUBLE_EQ(c.yaw, 1.0);
}

// ---- round2 行为 ----
TEST(CmdMath, Round2)
{
  EXPECT_DOUBLE_EQ(round2(0.123), 0.12);
  EXPECT_DOUBLE_EQ(round2(0.125), 0.13);
  EXPECT_DOUBLE_EQ(round2(-0.125), -0.13);
  EXPECT_DOUBLE_EQ(round2(1.0), 1.0);
}

// ---- 归一化：超速钳制 ----
TEST(CmdMath, NormalizeClamp)
{
  const auto c = normalizeCmdVel(5.0, -5.0, -5.0, 0.5, 0.5, 0.6);
  ASSERT_TRUE(c.valid);
  EXPECT_DOUBLE_EQ(c.x, 1.0);
  EXPECT_DOUBLE_EQ(c.y, -1.0);
  EXPECT_DOUBLE_EQ(c.yaw, -1.0);
}

// ---- 归一化：任何输入 NaN/Inf → invalid（除法前拒绝）----
TEST(CmdMath, NormalizeRejectsNonFinite)
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
TEST(CmdMath, ClampUnit)
{
  EXPECT_DOUBLE_EQ(clampUnit(2.0), 1.0);
  EXPECT_DOUBLE_EQ(clampUnit(-2.0), -1.0);
  EXPECT_DOUBLE_EQ(clampUnit(0.3), 0.3);
}

// ---- 启动参数 fail-fast：有限正数 ----
TEST(CmdMath, RequireFinitePositive)
{
  EXPECT_DOUBLE_EQ(requireFinitePositive(0.5, "vx_max"), 0.5);
  EXPECT_THROW(requireFinitePositive(0.0, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(-0.1, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(kNaN, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(kInf, "vx_max"), std::invalid_argument);
}

// ---- 启动参数 fail-fast：频率上限 / 端口范围 ----
TEST(CmdMath, RequireFrequencyAndPort)
{
  EXPECT_DOUBLE_EQ(requireFrequencyHz(20.0, "cmd_frequency_hz", 200.0), 20.0);
  EXPECT_DOUBLE_EQ(requireFrequencyHz(1.0, "heartbeat_hz", 100.0), 1.0);
  EXPECT_THROW(requireFrequencyHz(201.0, "cmd_frequency_hz", 200.0), std::invalid_argument);
  EXPECT_THROW(requireFrequencyHz(kInf, "cmd_frequency_hz", 200.0), std::invalid_argument);
  EXPECT_THROW(requireFrequencyHz(0.0, "cmd_frequency_hz", 200.0), std::invalid_argument);
  EXPECT_THROW(requireFrequencyHz(-1.0, "cmd_frequency_hz", 200.0), std::invalid_argument);

  EXPECT_EQ(requirePortRange(30000, "port"), 30000);
  EXPECT_EQ(requirePortRange(1, "port"), 1);
  EXPECT_EQ(requirePortRange(65535, "port"), 65535);
  EXPECT_THROW(requirePortRange(0, "port"), std::invalid_argument);
  EXPECT_THROW(requirePortRange(-1, "port"), std::invalid_argument);
  EXPECT_THROW(requirePortRange(65536, "port"), std::invalid_argument);
}
