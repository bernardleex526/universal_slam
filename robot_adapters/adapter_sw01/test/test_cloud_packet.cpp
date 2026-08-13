// ============================================================
// adapter_sw01 / test/test_cloud_packet.cpp
// P0 重构验收：长度驱动解析、无旧缓冲依赖、两种布局精确长度规则。
// ============================================================
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "adapter_sw01/cloud_packet.hpp"

using namespace adapter_sw01;

namespace {

/// 构造一个合法的 dynamic 布局包（pts 个点，按字段写入）
std::vector<uint8_t> makeDynamicPacket(
  uint16_t head, uint64_t ts, uint32_t total, uint32_t index, uint16_t pts, uint16_t tail)
{
  const size_t n = kHeaderBytes + static_cast<size_t>(pts) * kCloudPointBytes + 2;
  std::vector<uint8_t> buf(n, 0);
  auto put = [&buf](size_t off, const void * src, size_t len) {
    std::memcpy(buf.data() + off, src, len);
  };
  put(kHeadOff, &head, 2);
  put(kTsOff, &ts, 8);
  put(kTotalOff, &total, 4);
  put(kIndexOff, &index, 4);
  put(kPosNumOff, &pts, 2);
  for (uint16_t i = 0; i < pts; ++i) {
    CloudPointRaw pt{i * 10 + 1, i * 10 + 2, i * 10 + 3, 100, 200, 255, 0};
    put(kPosOff + static_cast<size_t>(i) * kCloudPointBytes, &pt, sizeof(pt));
  }
  put(n - 2, &tail, 2);
  return buf;
}

}  // namespace

// ---- 协议尺寸契约（review：点大小必须与手册结构体一致）----
TEST(CloudPacket, WireSizeContract)
{
  // 手册 5.10 节 AstrallPosCloudPosData = int32 x,y,z + uint32 r,g,b,a = 28B
  EXPECT_EQ(sizeof(CloudPointRaw), 28u);
  EXPECT_EQ(kCloudPointBytes, 28u);
  // 固定布局完整结构体 = 头20 + 50×28 点 + 尾2 = 1422B
  EXPECT_EQ(kFixedPacketBytes, 1422u);
}

// ---- dynamic 布局：合法包 ----
TEST(CloudPacket, DynamicValid)
{
  auto buf = makeDynamicPacket(kCloudSof, 123456789ULL, 100, 3, 5, kCloudTail);
  CloudPacketView pkt;
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kOk);
  EXPECT_EQ(pkt.timestamp_ns, 123456789ULL);
  EXPECT_EQ(pkt.total, 100u);
  EXPECT_EQ(pkt.index, 3u);
  EXPECT_EQ(pkt.pos_num, 5u);
  CloudPointRaw pt;
  readCloudPoint(pkt.pos, 2, pt);
  EXPECT_EQ(pt.x, 21);
  EXPECT_EQ(pt.y, 22);
  EXPECT_EQ(pt.z, 23);
  EXPECT_EQ(pt.r, 100u);
}

// ---- dynamic 布局：截断包（头部缺失 / 尾部缺失 / 点数据缺失）----
TEST(CloudPacket, DynamicTruncated)
{
  auto buf = makeDynamicPacket(kCloudSof, 1, 100, 0, 5, kCloudTail);
  CloudPacketView pkt;
  // n=0 → null 长度不足；n=19 → 头部不足
  EXPECT_EQ(parseCloudPacket(buf.data(), 0, CloudPacketLayout::kDynamic, pkt),
    ParseResult::kShortHeader);
  EXPECT_EQ(parseCloudPacket(buf.data(), kHeaderBytes - 1, CloudPacketLayout::kDynamic, pkt),
    ParseResult::kShortHeader);
  // n=20（只有头部，无点无尾）→ 长度不符
  EXPECT_EQ(parseCloudPacket(buf.data(), kHeaderBytes, CloudPacketLayout::kDynamic, pkt),
    ParseResult::kBadLength);
  // 少 1 字节（尾被截断一半）→ 长度不符
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size() - 1, CloudPacketLayout::kDynamic, pkt),
    ParseResult::kBadLength);
}

// ---- dynamic 布局：残留包（多余字节）----
TEST(CloudPacket, DynamicResidual)
{
  auto buf = makeDynamicPacket(kCloudSof, 1, 100, 0, 5, kCloudTail);
  buf.push_back(0xDE);  // 多余 1 字节
  CloudPacketView pkt;
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kBadLength);
}

// ---- SOF / TAIL 错误 ----
TEST(CloudPacket, BadSofBadTail)
{
  auto good = makeDynamicPacket(kCloudSof, 1, 100, 0, 5, kCloudTail);
  CloudPacketView pkt;
  auto bad_sof = good;
  uint16_t wrong = 0x1234;
  std::memcpy(bad_sof.data() + kHeadOff, &wrong, 2);
  EXPECT_EQ(parseCloudPacket(bad_sof.data(), bad_sof.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kBadSof);

  auto bad_tail = good;
  std::memcpy(bad_tail.data() + bad_tail.size() - 2, &wrong, 2);
  EXPECT_EQ(parseCloudPacket(bad_tail.data(), bad_tail.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kBadTail);
}

// ---- posNum 越界 ----
TEST(CloudPacket, BadPosNum)
{
  auto buf = makeDynamicPacket(kCloudSof, 1, 100, 0, 51, kCloudTail);
  CloudPacketView pkt;
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kBadPosNum);
}

// ---- posNum=0：dynamic 下合法（空包，适配器跳过）----
TEST(CloudPacket, DynamicZeroPoints)
{
  auto buf = makeDynamicPacket(kCloudSof, 1, 100, 0, 0, kCloudTail);
  CloudPacketView pkt;
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kOk);
  EXPECT_EQ(pkt.pos_num, 0u);
}

// ---- fixed 布局：完整 1422B 合法；截断/残留拒绝 ----
TEST(CloudPacket, FixedLayoutExactLength)
{
  // 构造完整固定结构体（50×28B 点槽位，posNum=3 表示 3 个有效点，其余槽位为 0）
  std::vector<uint8_t> buf(kFixedPacketBytes, 0);
  auto put = [&buf](size_t off, const void * src, size_t len) {
    std::memcpy(buf.data() + off, src, len);
  };
  uint16_t head = kCloudSof, tail = kCloudTail, pos_num = 3;
  uint64_t ts = 42;
  uint32_t total = 200, index = 10;
  put(kHeadOff, &head, 2);
  put(kTsOff, &ts, 8);
  put(kTotalOff, &total, 4);
  put(kIndexOff, &index, 4);
  put(kPosNumOff, &pos_num, 2);
  put(kFixedPacketBytes - 2, &tail, 2);

  CloudPacketView pkt;
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size(), CloudPacketLayout::kFixed, pkt),
    ParseResult::kOk);
  EXPECT_EQ(pkt.total, 200u);
  EXPECT_EQ(pkt.index, 10u);
  EXPECT_EQ(pkt.pos_num, 3u);

  // 截断 1 字节 → 拒绝
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size() - 1, CloudPacketLayout::kFixed, pkt),
    ParseResult::kBadLength);
  // 残留 1 字节 → 拒绝
  buf.push_back(0);
  EXPECT_EQ(parseCloudPacket(buf.data(), buf.size(), CloudPacketLayout::kFixed, pkt),
    ParseResult::kBadLength);
}

// ---- full→short 陈旧缓冲：先解析合法长包，再以短长度解析同一缓冲 ----
// 若实现依赖缓冲旧内容（旧 bug：先 memcpy 整个结构体再校验），短长度时
// 会读到长包遗留的 tail/SOF 并误判成功。本测试固化"长度驱动、绝不读越界字节"。
TEST(CloudPacket, FullThenShortStaleBuffer)
{
  auto full = makeDynamicPacket(kCloudSof, 7, 300, 0, 50, kCloudTail);
  std::vector<uint8_t> buf(full.size() + 64, 0xFF);   // 故意填充陈旧字节（缓冲区 ≥ 完整包长）
  std::memcpy(buf.data(), full.data(), full.size());

  CloudPacketView pkt;
  // 1) 完整长度 → 成功
  EXPECT_EQ(parseCloudPacket(buf.data(), full.size(), CloudPacketLayout::kDynamic, pkt),
    ParseResult::kOk);

  // 2) 短长度（如仅头部+部分点）：n 之外的字节含合法 tail 残留，也必须拒绝
  for (size_t n : {size_t(0), size_t(10), kHeaderBytes,
                   kHeaderBytes + 3 * kCloudPointBytes,
                   full.size() - 2}) {
    EXPECT_NE(parseCloudPacket(buf.data(), n, CloudPacketLayout::kDynamic, pkt),
      ParseResult::kOk) << "短长度 n=" << n << " 不应解析成功";
  }

  // 3) 布局错配：短 dynamic 包（3 点）以 fixed 布局解析 → 长度不符拒绝
  //    （注：50 点 dynamic 包与 fixed 结构体字节布局完全相同，长度相等时
  //      两种布局解析结果一致，属预期行为，不在此断言）
  auto short_pkt = makeDynamicPacket(kCloudSof, 7, 300, 0, 3, kCloudTail);
  EXPECT_EQ(parseCloudPacket(short_pkt.data(), short_pkt.size(),
                             CloudPacketLayout::kFixed, pkt),
    ParseResult::kBadLength);
}

// ---- null 指针 ----
TEST(CloudPacket, NullData)
{
  CloudPacketView pkt;
  EXPECT_EQ(parseCloudPacket(nullptr, 100, CloudPacketLayout::kDynamic, pkt),
    ParseResult::kNullData);
}

// ---- 帧一致性（越界防护纯函数）----
TEST(CloudPacket, PacketFitsFrame)
{
  EXPECT_TRUE(packetFitsFrame(100, 0, 50));
  EXPECT_TRUE(packetFitsFrame(100, 50, 50));     // 边界 index+pts==total
  EXPECT_FALSE(packetFitsFrame(100, 51, 50));    // index 越界
  EXPECT_FALSE(packetFitsFrame(100, 0, 101));    // pts > total
  EXPECT_FALSE(packetFitsFrame(0, 0, 1));        // total==0
  EXPECT_FALSE(packetFitsFrame(100, 0, 0));      // pts==0
  EXPECT_FALSE(packetFitsFrame(100, 90, 11));    // index+pts > total
}
