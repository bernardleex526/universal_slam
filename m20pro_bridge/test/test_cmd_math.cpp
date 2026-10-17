#include <gtest/gtest.h>

#include <limits>

#include "m20pro_bridge/cmd_math.hpp"

using namespace m20pro_bridge;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

TEST(CmdMath, RequireFinitePositive)
{
  EXPECT_DOUBLE_EQ(requireFinitePositive(0.5, "vx_max"), 0.5);
  EXPECT_THROW(requireFinitePositive(0.0, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(-0.1, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(kNaN, "vx_max"), std::invalid_argument);
  EXPECT_THROW(requireFinitePositive(kInf, "vx_max"), std::invalid_argument);
}

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
