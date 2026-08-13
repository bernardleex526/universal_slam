// ============================================================
// adapter_sw01 / include/adapter_sw01/cloud_packet.hpp
// SW01 UDP 6100 点云协议解析（P0 重构 + review 修复）：
//   - 长度驱动、纯函数：只读取 [0, n) 范围内的字节，绝不依赖缓冲区旧内容
//   - 明确两种协议布局的接受规则（均要求包长精确匹配）：
//     * dynamic: 头部20B + posNum×28B 点 + tail 2B（默认）
//     * fixed:   完整固定结构体（50 点恒存在，posNum 仅计有效点）
//   - 截断包（长度不足）与残留包（多余字节）一律拒绝
//   - 点大小按 sizeof(AstrallPosCloudPosData) 推导 + static_assert：
//     手册 5.10 节 AstrallPosCloudPosData = int32 x,y,z + uint32 r,g,b,a = 28B
//     （review: 修复此前 kCloudPointBytes=20 与结构体 28B 不一致的问题）
//   - 字节布局为 packed 小端假设（手册未规定字节级 packing；自然对齐布局
//     为另一种可能，见 adapter_sw01/README.md「未决项」，实机抓包确认）
// 单测：test/test_cloud_packet.cpp（ASan/UBSan 构建运行）
// ============================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace adapter_sw01 {

constexpr uint16_t kCloudSof = 0xAA55;    // 数据头 (LIDARPUSH_DATA_SOF)
constexpr uint16_t kCloudTail = 0xFF00;   // 数据尾 (LIDARPUSH_DATA_TAIL)
constexpr uint32_t kCloudMaxPointsPerPacket = 50;  // 单包 ≤50 点 (LIDARPUSH_PUSH_POS_CLOUD_POS_MAX)

/// 单点 wire 结构（与 SW01 手册 5.10 节 AstrallPosCloudPosData 一致）：
/// int32 x,y,z + uint32 r,g,b,a（7×uint32）。sizeof 参与协议长度推导，
/// 与 wire 完全一致的打包布局（无 padding，主机小端读取）。
struct CloudPointRaw
{
  int32_t x, y, z;
  uint32_t r, g, b, a;
};
static_assert(sizeof(CloudPointRaw) == 28,
  "AstrallPosCloudPosData 必须为 28 字节 (int32 x,y,z + uint32 r,g,b,a)，见 SW01 手册 5.10 节");

// packed 协议字节布局（按手册结构体字段顺序；packed 小端假设）
constexpr size_t kHeadOff = 0;     // uint16 SOF
constexpr size_t kTsOff = 2;       // uint64 timestampNs
constexpr size_t kTotalOff = 10;   // uint32 当前帧总点数
constexpr size_t kIndexOff = 14;   // uint32 本包起始点索引
constexpr size_t kPosNumOff = 18;  // uint16 本包点数
constexpr size_t kPosOff = 20;     // 点数据起点
constexpr size_t kHeaderBytes = 20;                       // head..posNum
constexpr size_t kCloudPointBytes = sizeof(CloudPointRaw);  // 28（sizeof 推导，不硬编码）
constexpr size_t kFixedPacketBytes =                       // 固定布局：完整结构体大小
  kHeaderBytes + kCloudMaxPointsPerPacket * kCloudPointBytes + 2;
static_assert(kFixedPacketBytes == 20 + 50 * 28 + 2,
  "固定布局包长应为 1422 字节 (头20 + 50×28 点 + 尾2)");

/// 协议布局（由启动参数 cloud_packet_layout 选择）
enum class CloudPacketLayout { kDynamic, kFixed };

/// 解析结果（拒绝原因可测试断言）
enum class ParseResult {
  kOk,
  kNullData,
  kShortHeader,   // n < 20（连头部都不完整）
  kBadSof,        // 数据头 ≠ 0xAA55
  kBadPosNum,     // posNum > 50
  kBadLength,     // 长度与所选布局的精确要求不符（截断或残留）
  kBadTail,       // 数据尾 ≠ 0xFF00
};

/// 解析出的包视图（不拷贝点数据，指向调用方缓冲内的有效区段）
struct CloudPacketView
{
  uint64_t timestamp_ns{0};
  uint32_t total{0};
  uint32_t index{0};
  uint32_t pos_num{0};
  const uint8_t * pos{nullptr};
};

/// 长度驱动的纯解析函数：
///   - 只读取 data[0, n)；任何短包直接拒绝，不触碰 n 之外的陈旧字节
///   - dynamic: n 必须 == 20 + posNum*28 + 2
///   - fixed:   n 必须 == 1422
inline ParseResult parseCloudPacket(
  const uint8_t * data, size_t n, CloudPacketLayout layout, CloudPacketView & out)
{
  if (data == nullptr) { return ParseResult::kNullData; }
  if (n < kHeaderBytes) { return ParseResult::kShortHeader; }

  uint16_t head;
  std::memcpy(&head, data + kHeadOff, sizeof(head));
  if (head != kCloudSof) { return ParseResult::kBadSof; }

  uint16_t pos_num;
  std::memcpy(&pos_num, data + kPosNumOff, sizeof(pos_num));
  if (pos_num > kCloudMaxPointsPerPacket) { return ParseResult::kBadPosNum; }

  // 长度校验（tail 在所选布局下的精确位置）
  size_t expected;
  if (layout == CloudPacketLayout::kFixed) {
    expected = kFixedPacketBytes;
  } else {
    expected = kHeaderBytes + static_cast<size_t>(pos_num) * kCloudPointBytes + 2;
  }
  if (n != expected) { return ParseResult::kBadLength; }

  uint16_t tail;
  std::memcpy(&tail, data + expected - sizeof(tail), sizeof(tail));
  if (tail != kCloudTail) { return ParseResult::kBadTail; }

  std::memcpy(&out.timestamp_ns, data + kTsOff, sizeof(out.timestamp_ns));
  std::memcpy(&out.total, data + kTotalOff, sizeof(out.total));
  std::memcpy(&out.index, data + kIndexOff, sizeof(out.index));
  out.pos_num = pos_num;
  out.pos = data + kPosOff;
  return ParseResult::kOk;
}

/// 包与当前帧缓冲的一致性校验（越界防护，纯函数便于测试）：
/// total>0、pts>0、pts≤total 且 index ≤ total-pts。
inline bool packetFitsFrame(uint32_t total, uint32_t index, uint32_t pts)
{
  return total > 0 && pts > 0 && pts <= total && index <= total - pts;
}

/// 从包内读取第 i 个点（int32 x,y,z + uint32 r,g,b,a；主机小端）。
inline void readCloudPoint(const uint8_t * pos, uint32_t i, CloudPointRaw & pt)
{
  static_assert(sizeof(pt) == kCloudPointBytes,
    "读取宽度必须与协议点大小一致");
  std::memcpy(&pt, pos + static_cast<size_t>(i) * kCloudPointBytes, sizeof(pt));
}

}  // namespace adapter_sw01
