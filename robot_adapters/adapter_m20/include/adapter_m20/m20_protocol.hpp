// ============================================================
// adapter_m20 / include/adapter_m20/m20_protocol.hpp
// M20 上报帧解析（review 修复，纯函数可测）：
//   - 严格长度：n == 16 + asdu_len（截断/尾随一律拒绝），asdu_len>0 且有上限
//   - 同步字校验、JSON 解析失败拒绝
//   - 来源校验 validatePeer：源 IP/端口必须与配置的机器人地址一致（防伪造/串流）
//   - 语义校验：状态帧必须含 MotionState+ControlUsageMode 且数值在界内；
//     错误帧必须能提取到错误码，否则拒绝（缺字段/越界数值不采信）
// 单测：test/test_m20_protocol.cpp（ASan/UBSan 构建运行）
// Manufacturer V1.2.1: [6:8] message ID, [8] JSON marker;
// Type/Command belong to the PatrolDevice JSON envelope, not the header.
// ============================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <nlohmann/json.hpp>

namespace adapter_m20 {

constexpr uint8_t kSync0 = 0xEB;
constexpr uint8_t kSync1 = 0x91;
constexpr uint8_t kSync2 = 0xEB;
constexpr uint8_t kSync3 = 0x90;
constexpr size_t kHeaderLen = 16;
constexpr size_t kMaxAsduLen = 4096;   // 上报 JSON 体上限（防御性）

enum class ReportParseResult {
  kOk,
  kShortHeader,    // n < 16
  kBadSync,        // 同步字不符
  kBadLength,      // n != 16 + asdu_len（截断或尾随）
  kAsduZero,       // asdu_len == 0
  kAsduTooLarge,   // asdu_len > 上限
  kBadJson,        // JSON 解析失败
  kBadFormat,
  kBadEnvelope,
};

/// 长度驱动的上报帧解析：只读取 [0, n)，严格长度匹配。
inline ReportParseResult parseReportFrame(
  const uint8_t * data, size_t n, uint16_t & type, uint16_t & command,
  nlohmann::json & body, size_t max_asdu = kMaxAsduLen)
{
  if (data == nullptr || n < kHeaderLen) { return ReportParseResult::kShortHeader; }
  if (data[0] != kSync0 || data[1] != kSync1 || data[2] != kSync2 || data[3] != kSync3) {
    return ReportParseResult::kBadSync;
  }
  const uint16_t asdu_len = static_cast<uint16_t>(data[4]) |
    (static_cast<uint16_t>(data[5]) << 8);
  if (asdu_len == 0) { return ReportParseResult::kAsduZero; }
  if (asdu_len > max_asdu) { return ReportParseResult::kAsduTooLarge; }
  // 严格：总长必须精确等于头 + ASDU（拒绝截断与尾随字节）
  if (n != kHeaderLen + static_cast<size_t>(asdu_len)) { return ReportParseResult::kBadLength; }

  if (data[8] != 0x01) { return ReportParseResult::kBadFormat; }
  try {
    body = nlohmann::json::parse(data + kHeaderLen, data + n);
  } catch (const std::exception &) {
    return ReportParseResult::kBadJson;
  }
  if (!body.is_object() || !body.contains("PatrolDevice")) {
    return ReportParseResult::kBadEnvelope;
  }
  const auto & p = body["PatrolDevice"];
  if (!p.is_object() || !p.contains("Items") || !p["Items"].is_object()) {
    return ReportParseResult::kBadEnvelope;
  }
  for (const auto & key : {"Type", "Command"}) {
    if (!p.contains(key) || !p[key].is_number_integer() ||
        p[key] < 0 || p[key] > 65535) {
      return ReportParseResult::kBadEnvelope;
    }
  }
  type = p["Type"].get<uint16_t>();
  command = p["Command"].get<uint16_t>();
  return ReportParseResult::kOk;
}

/// 来源校验：src 必须与配置的机器人地址（IP+端口）一致（防伪造/串流）。
/// 参数为网络序 sockaddr 中的裸字段（sin_addr.s_addr / sin_port）。
inline bool validatePeer(
  uint32_t src_addr, uint16_t src_port, uint32_t peer_addr, uint16_t peer_port)
{
  return src_addr == peer_addr && src_port == peer_port;
}

/// 在 JSON 对象/数组中递归查找指定 key 的整数值（健壮于未知嵌套）。
inline bool findInt(const nlohmann::json & j, const std::string & key, int64_t & out)
{
  if (j.is_object()) {
    auto it = j.find(key);
    if (it != j.end() && it->is_number_integer()) { out = it->get<int64_t>(); return true; }
    for (auto & kv : j.items()) { if (findInt(kv.value(), key, out)) return true; }
  } else if (j.is_array()) {
    for (auto & e : j) { if (findInt(e, key, out)) return true; }
  }
  return false;
}

struct StatusReport
{
  int16_t motion_state{0};
  uint32_t control_usage_mode{0};
};

/// 状态帧语义校验：两个字段必须齐全且数值在界内，否则拒绝（不采信缺字段/越界）。
inline bool parseStatusReport(const nlohmann::json & body, StatusReport & out)
{
  int64_t ms = 0, cm = 0;
  if (!findInt(body, "MotionState", ms) || !findInt(body, "ControlUsageMode", cm)) {
    return false;
  }
  if (ms < -32768 || ms > 32767) { return false; }
  if (cm < 0 || cm > 65535) { return false; }
  out.motion_state = static_cast<int16_t>(ms);
  out.control_usage_mode = static_cast<uint32_t>(cm);
  return true;
}

/// 错误帧语义校验：必须能提取错误码，否则拒绝。
inline bool parseErrorReport(const nlohmann::json & body, uint32_t & out_code)
{
  int64_t v = 0;
  for (const auto & key : {"ErrorCode", "ErrCode", "Error", "Code"}) {
    if (findInt(body, key, v)) {
      if (v < 0 || v > 4294967295LL) { return false; }
      out_code = static_cast<uint32_t>(v); return true;
    }
  }
  return false;
}

}  // namespace adapter_m20
