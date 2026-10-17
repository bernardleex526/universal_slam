// ============================================================
// m20pro_bridge / test/test_m20_protocol.cpp
// review 修复验收：
//   - 来源校验（伪造 IP/端口拒绝）
//   - 严格长度 n == 16 + asdu_len（截断/尾随拒绝）
//   - asdu_len=0 / 超上限 / 坏同步字 / 坏 JSON 拒绝
//   - 状态帧缺字段/数值越界拒绝；错误帧缺错误码拒绝
// ============================================================
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include <nlohmann/json.hpp>

#include "m20pro_bridge/m20_protocol.hpp"

using namespace m20pro_bridge;

namespace {

// Manufacturer envelope: sync + ASDU length + message ID + JSON marker + reserved.
std::vector<uint8_t> makeFrame(
  uint16_t type, uint16_t command, const nlohmann::json & body)
{
  const std::string s = nlohmann::json{{"PatrolDevice", {
    {"Type", type}, {"Command", command}, {"Items", body}}}}.dump();
  std::vector<uint8_t> v(kHeaderLen + s.size(), 0);
  v[0] = kSync0; v[1] = kSync1; v[2] = kSync2; v[3] = kSync3;
  uint16_t asdu = static_cast<uint16_t>(s.size());
  std::memcpy(v.data() + 4, &asdu, 2);
  v[6] = 7;  // Manufacturer message ID is independent of ASDU Type.
  v[8] = 1;  // JSON format marker.
  std::memcpy(v.data() + kHeaderLen, s.data(), s.size());
  return v;
}

}  // namespace

// ---- 合法状态帧 ----
TEST(M20Protocol, ValidStatusFrame)
{
  auto v = makeFrame(1002, 6, nlohmann::json{{"MotionState", 17}, {"ControlUsageMode", 0}});
  uint16_t t = 0, c = 0;
  nlohmann::json body;
  EXPECT_EQ(parseReportFrame(v.data(), v.size(), t, c, body), ReportParseResult::kOk);
  EXPECT_EQ(t, 1002); EXPECT_EQ(c, 6);
  StatusReport sr;
  ASSERT_TRUE(parseStatusReport(body, sr));
  EXPECT_EQ(sr.motion_state, 17);
  EXPECT_EQ(sr.control_usage_mode, 0u);
}

// ---- 截断（头不足 / 体被截断）----
TEST(M20Protocol, TruncatedRejected)
{
  auto v = makeFrame(1002, 6, nlohmann::json{{"MotionState", 1}, {"ControlUsageMode", 0}});
  uint16_t t = 0, c = 0;
  nlohmann::json body;
  EXPECT_EQ(parseReportFrame(v.data(), kHeaderLen - 1, t, c, body),
    ReportParseResult::kShortHeader);
  EXPECT_EQ(parseReportFrame(v.data(), kHeaderLen, t, c, body),
    ReportParseResult::kBadLength);       // asdu 声明有体但 n 只到头部
  EXPECT_EQ(parseReportFrame(v.data(), v.size() - 1, t, c, body),
    ReportParseResult::kBadLength);       // 体少 1 字节
}

// ---- 尾随（多余字节）----
TEST(M20Protocol, TrailingRejected)
{
  auto v = makeFrame(1002, 6, nlohmann::json{{"MotionState", 1}, {"ControlUsageMode", 0}});
  v.push_back(0x00);
  uint16_t t = 0, c = 0;
  nlohmann::json body;
  EXPECT_EQ(parseReportFrame(v.data(), v.size(), t, c, body),
    ReportParseResult::kBadLength);
}

// ---- asdu_len = 0 / 超上限 ----
TEST(M20Protocol, AsduBounds)
{
  auto v = makeFrame(1002, 6, nlohmann::json{{"MotionState", 1}, {"ControlUsageMode", 0}});
  v[4] = 0; v[5] = 0;   // asdu_len=0
  uint16_t t = 0, c = 0;
  nlohmann::json body;
  EXPECT_EQ(parseReportFrame(v.data(), v.size(), t, c, body),
    ReportParseResult::kAsduZero);
  // asdu_len 超大（> 上限）
  uint16_t big = static_cast<uint16_t>(kMaxAsduLen + 1);
  std::memcpy(v.data() + 4, &big, 2);
  EXPECT_EQ(parseReportFrame(v.data(), v.size(), t, c, body),
    ReportParseResult::kAsduTooLarge);
}

// ---- 坏同步字 / 坏 JSON ----
TEST(M20Protocol, BadSyncBadJson)
{
  auto v = makeFrame(1002, 6, nlohmann::json{{"MotionState", 1}, {"ControlUsageMode", 0}});
  v[2] = 0x00;
  uint16_t t = 0, c = 0;
  nlohmann::json body;
  EXPECT_EQ(parseReportFrame(v.data(), v.size(), t, c, body),
    ReportParseResult::kBadSync);

  auto v2 = makeFrame(1002, 6, nlohmann::json{{"x", 1}});
  v2[kHeaderLen] = 0xFF;   // 破坏 JSON
  EXPECT_EQ(parseReportFrame(v2.data(), v2.size(), t, c, body),
    ReportParseResult::kBadJson);
}

// ---- 来源校验（伪造 IP / 端口拒绝）----
TEST(M20Protocol, ValidatePeerRejectsForgedSource)
{
  const uint32_t robot_addr = 0x01020304;   // 网络序裸值
  const uint16_t robot_port = 30000;
  // 完全一致 → 接受
  EXPECT_TRUE(validatePeer(robot_addr, robot_port, robot_addr, robot_port));
  // 伪造 IP → 拒绝
  EXPECT_FALSE(validatePeer(0x01020305, robot_port, robot_addr, robot_port));
  // 伪造端口 → 拒绝
  EXPECT_FALSE(validatePeer(robot_addr, 30001, robot_addr, robot_port));
  // IP+端口都不同 → 拒绝
  EXPECT_FALSE(validatePeer(0xFFFFFFFF, 12345, robot_addr, robot_port));
}

// ---- 状态帧语义：缺字段 / 数值越界 → 拒绝 ----
TEST(M20Protocol, StatusReportValidation)
{
  StatusReport sr;
  // 缺 ControlUsageMode
  EXPECT_FALSE(parseStatusReport(nlohmann::json{{"MotionState", 17}}, sr));
  // 缺 MotionState
  EXPECT_FALSE(parseStatusReport(nlohmann::json{{"ControlUsageMode", 0}}, sr));
  // MotionState 越界（> int16）
  EXPECT_FALSE(parseStatusReport(
    nlohmann::json{{"MotionState", 32768}, {"ControlUsageMode", 0}}, sr));
  // ControlUsageMode 负值
  EXPECT_FALSE(parseStatusReport(
    nlohmann::json{{"MotionState", 1}, {"ControlUsageMode", -1}}, sr));
  // ControlUsageMode 越界（> uint16）
  EXPECT_FALSE(parseStatusReport(
    nlohmann::json{{"MotionState", 1}, {"ControlUsageMode", 65536}}, sr));
  // 合法边界值
  EXPECT_TRUE(parseStatusReport(
    nlohmann::json{{"MotionState", -32768}, {"ControlUsageMode", 65535}}, sr));
  EXPECT_EQ(sr.motion_state, -32768);
  EXPECT_EQ(sr.control_usage_mode, 65535u);
}

// ---- 错误帧语义：必须能提取错误码 ----
TEST(M20Protocol, ErrorReportValidation)
{
  uint32_t code = 0;
  EXPECT_TRUE(parseErrorReport(nlohmann::json{{"ErrorCode", 0x1234}}, code));
  EXPECT_EQ(code, 0x1234u);
  // 嵌套结构也能找到
  EXPECT_TRUE(parseErrorReport(
    nlohmann::json{{"PatrolDevice", {{"Items", {{"ErrorCode", 7}}}}}}, code));
  EXPECT_EQ(code, 7u);
  // 缺错误码 → 拒绝
  EXPECT_FALSE(parseErrorReport(nlohmann::json{{"SomeField", 1}}, code));
  EXPECT_FALSE(parseErrorReport(nlohmann::json::object(), code));
}

// ---- 数值类型健壮性：非整数不算有效字段 ----
TEST(M20Protocol, NonIntegerValuesRejected)
{
  StatusReport sr;
  // 字符串 "17" 不是整数（findInt 只认整数）→ 视为缺字段
  EXPECT_FALSE(parseStatusReport(
    nlohmann::json{{"MotionState", "17"}, {"ControlUsageMode", 0}}, sr));
  // 浮点值不是整数
  EXPECT_FALSE(parseStatusReport(
    nlohmann::json{{"MotionState", 17}, {"ControlUsageMode", 0.0}}, sr));
}
