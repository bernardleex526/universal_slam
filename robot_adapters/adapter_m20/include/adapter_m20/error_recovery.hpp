// ============================================================
// adapter_m20 / include/adapter_m20/error_recovery.hpp
// M20 ErrorHalt 门控与有界恢复决策（纯函数，可测）。
//
// 终审修复（P1）：错误门控改为时间窗口 —— 进入 ErrorHalt 后，"是否有新错误"
// 由 now - last_error_time_ < error_clear_window_s 判定，而非 sticky 的
// error_flag_（error_flag_ 仅作状态展示/历史）。此前 error_flag_ 永久置位导致
// gated_now 恒真、evaluateErrorHalt 从不被调用，error_auto_recovery=true 时
// 恢复路径不可达。修复后：窗口过期 + Regular 模式 + 退避到 → 真正进入 retry。
//
// 健康定义（全部满足才允许重试）：
//   - 收到过 (1002,6) 状态上报（have_status，无证据不重试）
//   - 最近上报 ControlUsageMode==0（Regular）
//   - error_clear_window_s 窗口内无新错误帧（now - last_error_time_ >= window）
//   - 自进入 ErrorHalt 起退避时间已到（base × 2^attempts，有界）
// 重试 = 重跑 AUTH+MODE 启动序列，次数有限（达上限后保持 halt，仅人工复位/重启）。
// ErrorHalt 期间节点每 tick 持续下发零速（由节点负责）。
// 单测：test/test_error_recovery.cpp（含真实 error_flag_ 生命周期集成序列）
// ============================================================
#pragma once

#include <cstdint>

namespace adapter_m20 {

struct ErrorRecoveryConfig
{
  bool auto_recovery{false};         // 默认关（硬件不确定项，见 README）
  int max_attempts{3};               // 有限重试上限（>0 才有自动恢复）
  double base_backoff_s{5.0};        // 指数退避基数：等待 base × 2^attempts 秒
  double error_clear_window_s{5.0};  // 错误证据窗口（秒）：窗口内出现新错误帧即门控
};

struct ErrorHaltInputs
{
  bool have_status{false};           // 收到过 (1002,6) 状态上报
  bool control_usage_regular{false}; // 最近上报 ControlUsageMode==0（Regular）
  double time_since_enter_s{0.0};    // 自进入 ErrorHalt 起的秒数
  double time_since_last_error_s{0.0};  // 距最近一次错误帧的秒数（无错误帧 = 大数）
  int attempts{0};                   // 已完成的重试次数（0=尚未重试）
};

enum class ErrorHaltAction { kHold, kRetryStartup };

/// 恢复决策（仅在"处于 ErrorHalt 且本 tick 无错误门控、模式 Regular"时调用）：
///   - 自动恢复关闭、max_attempts<=0、attempts>=max_attempts → kHold
///   - 证据不足（无状态上报/非 Regular/错误帧仍在窗口内）→ kHold（不猜协议）
///   - 退避未到（base × 2^attempts）→ kHold
///   - 其余 → kRetryStartup（重跑 AUTH+MODE 启动序列，由节点递增 attempts）
inline ErrorHaltAction evaluateErrorHalt(
  const ErrorRecoveryConfig & cfg, const ErrorHaltInputs & in)
{
  if (!cfg.auto_recovery) { return ErrorHaltAction::kHold; }
  if (cfg.max_attempts <= 0 || in.attempts >= cfg.max_attempts) {
    return ErrorHaltAction::kHold;
  }
  if (!in.have_status || !in.control_usage_regular) { return ErrorHaltAction::kHold; }
  if (in.time_since_last_error_s < cfg.error_clear_window_s) { return ErrorHaltAction::kHold; }
  const int shift = in.attempts < 0 ? 0 : (in.attempts > 30 ? 30 : in.attempts);
  const double required_s = cfg.base_backoff_s * static_cast<double>(uint32_t{1} << shift);
  if (in.time_since_enter_s < required_s) { return ErrorHaltAction::kHold; }
  return ErrorHaltAction::kRetryStartup;
}

/// 每 tick 的门控决策（节点 controlTimerCb 直接消费，与节点逻辑完全一致）：
///   - 错误门控 = 窗口内出现新错误帧（time_since_last_error_s < window）
///     或 本 tick 上报非 Regular（mode_gated），与 sticky error_flag_ 无关；
///   - 不在 halt 且门控 → enter_halt（本 tick 进入并持续零速）
///   - 在 halt 且门控 → stay_halt（持续零速，不做健康评估）
///   - 在 halt 且本 tick 无门控 → 按 evaluateErrorHalt 决定 retry 或 stay_halt
struct GateDecision
{
  bool enter_halt{false};
  bool stay_halt{false};
  bool retry{false};
};

inline GateDecision evaluateErrorGate(
  bool is_halt, bool mode_gated, double time_since_last_error_s,
  const ErrorRecoveryConfig & cfg, const ErrorHaltInputs & inputs)
{
  const bool error_gated = (time_since_last_error_s < cfg.error_clear_window_s);
  const bool gated_now = mode_gated || error_gated;

  GateDecision d;
  if (is_halt) {
    if (gated_now) {
      d.stay_halt = true;
    } else if (evaluateErrorHalt(cfg, inputs) == ErrorHaltAction::kRetryStartup) {
      d.retry = true;
    } else {
      d.stay_halt = true;
    }
  } else if (gated_now) {
    d.enter_halt = true;
    d.stay_halt = true;
  }
  return d;
}

}  // namespace adapter_m20
