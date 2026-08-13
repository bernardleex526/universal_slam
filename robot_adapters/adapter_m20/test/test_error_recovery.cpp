// ============================================================
// adapter_m20 / test/test_error_recovery.cpp
// P1 + review 修复验收：
//   - evaluateGate 状态机驱动（enter/stay/retry），修复不可达门控
//   - 健康才恢复：有状态上报 + Regular 模式 + 错误窗口过期 + 退避到
//   - 默认关闭仍安全：保持 halt（持续零速由节点负责）
//   - 有界：重试达上限后保持 halt
// ============================================================
#include <gtest/gtest.h>

#include "adapter_m20/error_recovery.hpp"

using namespace adapter_m20;

namespace {

ErrorRecoveryConfig autoCfg()
{
  ErrorRecoveryConfig cfg;
  cfg.auto_recovery = true;
  cfg.max_attempts = 3;
  cfg.base_backoff_s = 5.0;
  cfg.error_clear_window_s = 5.0;
  return cfg;
}

// 健康输入：有状态、Regular、错误窗口已过期、时间可调
ErrorHaltInputs healthyInputs(double since_enter_s, int attempts)
{
  ErrorHaltInputs in;
  in.have_status = true;
  in.control_usage_regular = true;
  in.time_since_enter_s = since_enter_s;
  in.time_since_last_error_s = 1e9;   // 无错误证据（很久以前）
  in.attempts = attempts;
  return in;
}

}  // namespace

// ---- 默认关闭：任何健康条件下都保持 halt ----
TEST(ErrorRecovery, DisabledAlwaysHolds)
{
  ErrorRecoveryConfig cfg;   // auto_recovery=false
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(1000.0, 0)), ErrorHaltAction::kHold);
  // 门控层面：默认关闭时即使健康也 stay_halt（窗口过期 + Regular + 退避到）
  const GateDecision d = evaluateErrorGate(true, false, 1e9, cfg, healthyInputs(1000.0, 0));
  EXPECT_TRUE(d.stay_halt);
  EXPECT_FALSE(d.retry);
}

// ---- 有界重试 ----
TEST(ErrorRecovery, BoundedRetries)
{
  auto cfg = autoCfg();   // max=3
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(1000.0, 0)), ErrorHaltAction::kRetryStartup);
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(1000.0, 2)), ErrorHaltAction::kRetryStartup);
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(1000.0, 3)), ErrorHaltAction::kHold);
  cfg.max_attempts = 0;
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(1000.0, 0)), ErrorHaltAction::kHold);
  cfg.max_attempts = -1;
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(1000.0, 0)), ErrorHaltAction::kHold);
}

// ---- 证据不足：无状态上报 / 非 Regular / 错误窗口未过期 → 不重试（不猜协议）----
TEST(ErrorRecovery, MissingEvidenceHolds)
{
  auto cfg = autoCfg();

  auto no_status = healthyInputs(1000.0, 0);
  no_status.have_status = false;
  EXPECT_EQ(evaluateErrorHalt(cfg, no_status), ErrorHaltAction::kHold);

  auto not_regular = healthyInputs(1000.0, 0);
  not_regular.control_usage_regular = false;
  EXPECT_EQ(evaluateErrorHalt(cfg, not_regular), ErrorHaltAction::kHold);

  auto recent_error = healthyInputs(1000.0, 0);
  recent_error.time_since_last_error_s = 4.9;   // < 5s 窗口
  EXPECT_EQ(evaluateErrorHalt(cfg, recent_error), ErrorHaltAction::kHold);
  recent_error.time_since_last_error_s = 5.0;
  EXPECT_EQ(evaluateErrorHalt(cfg, recent_error), ErrorHaltAction::kRetryStartup);
}

// ---- 退避计时 ----
TEST(ErrorRecovery, BackoffTiming)
{
  auto cfg = autoCfg();   // base=5.0s
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(4.9, 0)), ErrorHaltAction::kHold);
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(5.0, 0)), ErrorHaltAction::kRetryStartup);
  // 第 2 次重试退避翻倍：5×2=10s
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(9.9, 1)), ErrorHaltAction::kHold);
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(10.0, 1)), ErrorHaltAction::kRetryStartup);
  // 第 3 次：5×4=20s
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(19.9, 2)), ErrorHaltAction::kHold);
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(20.0, 2)), ErrorHaltAction::kRetryStartup);
}

// ---- 门控状态机（review：窗口化错误门控 + 修复不可达重试路径）----
TEST(ErrorRecovery, GateStateMachineTransitions)
{
  auto cfg = autoCfg();   // error_clear_window_s=5.0
  const double no_error = 1e9;

  // 1) 运行中（非 halt）且无错误门控 → 无动作
  {
    const GateDecision d = evaluateErrorGate(false, false, no_error, cfg, healthyInputs(0.0, 0));
    EXPECT_FALSE(d.enter_halt); EXPECT_FALSE(d.stay_halt); EXPECT_FALSE(d.retry);
  }
  // 2) 运行中收到错误帧（窗口内）→ enter + stay（本 tick 进入 ErrorHalt）
  {
    const GateDecision d = evaluateErrorGate(false, false, 0.1, cfg, healthyInputs(0.0, 0));
    EXPECT_TRUE(d.enter_halt); EXPECT_TRUE(d.stay_halt); EXPECT_FALSE(d.retry);
  }
  // 3) 运行中模式非 Regular → enter + stay
  {
    const GateDecision d = evaluateErrorGate(false, true, no_error, cfg, healthyInputs(0.0, 0));
    EXPECT_TRUE(d.enter_halt); EXPECT_TRUE(d.stay_halt); EXPECT_FALSE(d.retry);
  }
  // 4) 已在 halt 且错误仍在窗口内（即使模式 Regular）→ stay（不做健康评估）
  {
    const GateDecision d = evaluateErrorGate(true, false, 4.9, cfg, healthyInputs(1000.0, 0));
    EXPECT_FALSE(d.enter_halt); EXPECT_TRUE(d.stay_halt); EXPECT_FALSE(d.retry);
  }
  // 5) 已在 halt、窗口已过期 + Regular 但证据不足 → stay
  {
    auto in = healthyInputs(1000.0, 0);
    in.have_status = false;
    const GateDecision d = evaluateErrorGate(true, false, no_error, cfg, in);
    EXPECT_TRUE(d.stay_halt); EXPECT_FALSE(d.retry);
  }
  // 6) 已在 halt、窗口过期 + 健康且退避到 → retry（终审修复的可行路径）
  {
    const GateDecision d = evaluateErrorGate(true, false, no_error, cfg, healthyInputs(6.0, 0));
    EXPECT_FALSE(d.enter_halt); EXPECT_FALSE(d.stay_halt); EXPECT_TRUE(d.retry);
  }
  // 7) 重试达上限后（attempts=max）→ 即使健康也 stay（人工复位/重启才可退出）
  {
    const GateDecision d = evaluateErrorGate(true, false, no_error, cfg, healthyInputs(1000.0, 3));
    EXPECT_TRUE(d.stay_halt); EXPECT_FALSE(d.retry);
  }
}

// ---- 终审集成序列：真实 error_flag_ 生命周期 ----
// 模拟节点状态（in_halt / attempts / sticky error_flag_ 历史 / last_error_time_），
// 由 evaluateErrorGate 驱动 —— 与 controlTimerCb 使用同一决策函数。
TEST(ErrorRecovery, ErrorFlagLifecycleIntegration)
{
  auto cfg = autoCfg();   // window=5s, base=5s, max=3
  bool in_halt = false;
  bool error_flag_hist = false;   // sticky 历史（仅展示；门控不依赖它）
  double t_enter = 0.0;
  double last_error_t = -1e9;     // 负值哨兵 = 从未收到错误帧（now-(-1e9) 恒为大正数）
  int attempts = 0;

  auto error_frame = [&](double at) {
    error_flag_hist = true;
    last_error_t = at;
  };
  auto tick = [&](bool mode_gated, double t_now) {
    const double since_error = t_now - last_error_t;
    ErrorHaltInputs inputs{
      true,                      // have_status（一直有状态上报）
      !mode_gated,               // Regular
      t_now - t_enter,
      since_error,
      attempts};
    const GateDecision d = evaluateErrorGate(in_halt, mode_gated, since_error, cfg, inputs);
    if (d.enter_halt) { in_halt = true; t_enter = t_now; }
    if (d.retry) {
      ++attempts;
      in_halt = false;
      error_flag_hist = false;   // 重试时清除历史锁存（与节点一致）
    }
    return d;
  };

  // 健康运行 → 无动作
  EXPECT_FALSE(tick(false, 0.0).stay_halt);

  // 错误帧 (1002,3) → error_flag_ 历史置位 + 进入 ErrorHalt
  error_frame(1.0);
  {
    const GateDecision d = tick(false, 1.0);
    EXPECT_TRUE(d.enter_halt);
    EXPECT_TRUE(error_flag_hist);   // sticky 历史保留
  }
  // 窗口内（t=2..5.9）：即使模式 Regular 也保持 halt（错误证据未过期）
  for (double t = 2.0; t <= 5.9; t += 0.5) {
    const GateDecision d = tick(false, t);
    EXPECT_TRUE(d.stay_halt) << "t=" << t << " 错误窗口内应保持 halt";
    EXPECT_FALSE(d.retry);
  }
  // 窗口过期（t=6.0：距错误帧 5.0s）+ Regular + 退避到（自 t_enter=1.0 起 5s）
  // → 终审修复：真正进入 retry（此前 error_flag_ 恒真导致此处不可达）
  {
    const GateDecision d = tick(false, 6.0);
    EXPECT_TRUE(d.retry);
    EXPECT_FALSE(in_halt);
    EXPECT_EQ(attempts, 1);
    EXPECT_FALSE(error_flag_hist);   // 重试清除历史
  }
  // 恢复后正常（启动序列阶段无门控动作）
  EXPECT_FALSE(tick(false, 7.0).stay_halt);

  // 再次错误帧 → 再次进入 halt（t_enter=8.0）
  error_frame(8.0);
  EXPECT_TRUE(tick(false, 8.0).enter_halt);
  EXPECT_TRUE(tick(false, 9.0).stay_halt);   // 窗口内
  // 窗口过期 + 退避 10s（attempts=1 → base×2）：t_enter=8.0 → t=18.0
  EXPECT_TRUE(tick(false, 17.9).stay_halt);
  EXPECT_TRUE(tick(false, 18.0).retry);
  EXPECT_EQ(attempts, 2);

  // 第三次错误 → 第三次 halt；退避 20s（attempts=2 → base×4）
  error_frame(19.0);
  EXPECT_TRUE(tick(false, 19.0).enter_halt);
  EXPECT_TRUE(tick(false, 39.0).retry);
  EXPECT_EQ(attempts, 3);

  // 第四次错误 → attempts==max → 窗口过期 + 健康也不再重试（保持 halt）
  error_frame(40.0);
  EXPECT_TRUE(tick(false, 40.0).enter_halt);
  const GateDecision final_d = tick(false, 500.0);
  EXPECT_TRUE(final_d.stay_halt);
  EXPECT_FALSE(final_d.retry);
}

// ---- 终审：默认关闭时窗口过期也不自动恢复（安全 hold）----
TEST(ErrorRecovery, DefaultOffStaysHaltedAfterWindowExpiry)
{
  ErrorRecoveryConfig cfg;   // auto_recovery=false
  bool in_halt = false;
  double t_enter = 0.0;
  double last_error_t = -1e9;   // 负值哨兵 = 从未收到错误帧
  int attempts = 0;

  auto tick = [&](double t_now) {
    const double since_error = t_now - last_error_t;
    ErrorHaltInputs inputs{true, true, t_now - t_enter, since_error, attempts};
    const GateDecision d = evaluateErrorGate(in_halt, false, since_error, cfg, inputs);
    if (d.enter_halt) { in_halt = true; t_enter = t_now; }
    if (d.retry) { ++attempts; in_halt = false; }
    return d;
  };

  last_error_t = 1.0;                 // 错误帧
  EXPECT_TRUE(tick(1.0).enter_halt);
  // 窗口过期很久、Regular、状态齐全 —— 默认关闭 → 一直 stay_halt
  for (double t : {10.0, 100.0, 1000.0}) {
    const GateDecision d = tick(t);
    EXPECT_TRUE(d.stay_halt) << "t=" << t;
    EXPECT_FALSE(d.retry);
  }
  EXPECT_EQ(attempts, 0);
}

// ---- 防御性输入 ----
TEST(ErrorRecovery, DefensiveInputs)
{
  auto cfg = autoCfg();
  EXPECT_EQ(evaluateErrorHalt(cfg, healthyInputs(5.0, -1)), ErrorHaltAction::kRetryStartup);
  auto big = healthyInputs(1000.0, 31);   // 31 >= max → hold
  EXPECT_EQ(evaluateErrorHalt(cfg, big), ErrorHaltAction::kHold);
}
