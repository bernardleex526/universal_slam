// ============================================================
// adapter_m20 / src/m20_cmd_vel_bridge.cpp
// M20 cmd_vel 桥（完整实现, P2）
// ------------------------------------------------------------
// 功能：
//   - 订阅 Nav2 输出 /cmd_vel (geometry_msgs/Twist)
//   - 按 M20 UDP JSON 轴指令协议发往机器人（归一化 [-1,1]，20Hz）
//   - 独立 UDP 接收线程解析机器人上报（状态/速度/错误），驱动启动序列状态机
//   - 错误门控：(1002,3) 错误上报 或 ControlUsageMode!=0(非 Regular) → 停止发轴指令
// ------------------------------------------------------------
// 帧格式（发送）：
//   头 16 字节:
//     [0-3]  = 0xEB 0x91 0xEB 0x90  同步字
//     [4-5]  = ASDU 长度(小端)      （= JSON 体字节数；若实机为含头总长，P4 校准）
//     [6-7]  = 自增 msgID(小端, 0→65535 循环, 应答回显)
//     [8]    = 0x01 (JSON)
//     [9-15] = 7×0x00
//   JSON 体:
//     {"PatrolDevice":{"Type":T,"Command":C,"Time":"YYYY-MM-DD HH:MM:SS",
//                      "Items":{...}}}
// 收帧 [6-7] 语义为 Type、[8-9] 为 Command（与发帧 [6-7]=msgID 不同，按官方 SDK 惯例；
//   具体布局 P4 以实机抓包校准）。
// ============================================================
#define _POSIX_C_SOURCE 200809L   // localtime_r / recvfrom 等 POSIX 接口

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "adapter_m20/cmd_math.hpp"
#include "adapter_m20/error_recovery.hpp"
#include "adapter_m20/m20_protocol.hpp"

#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace adapter_m20 {

namespace {
// ---- 帧协议常量（收帧解析见 adapter_m20/m20_protocol.hpp）----
constexpr uint8_t  kJsonMarker = 0x01;

// 控制帧 (Type, Command)
constexpr uint16_t kHeartbeatType = 100;   // 心跳 (100,100) 1Hz
constexpr uint16_t kHeartbeatCmd  = 100;
constexpr uint16_t kAxisCmdType   = 2;     // 轴速度指令 Type=2 Command=21
constexpr uint16_t kAxisCmdCmd    = 21;
constexpr uint16_t kModeSetType   = 1101;  // 确保 Regular 模式 (1101,5,Mode=0)
constexpr uint16_t kModeSetCmd    = 5;
constexpr uint16_t kStandUpType   = 2;     // 起立 (2,22,MotionParam=1)
constexpr uint16_t kStandUpCmd    = 22;
constexpr uint16_t kGaitType      = 2;     // 切步态 (2,23,GaitParam=0x1001)
constexpr uint16_t kGaitCmd       = 23;

// 上报帧 (Type, Command)
constexpr uint16_t kRptType      = 1002;   // 主动上报
constexpr uint16_t kRptStatusCmd = 6;      // (1002,6) 状态: MotionState / ControlUsageMode
constexpr uint16_t kRptSpeedCmd  = 4;      // (1002,4) 速度反馈 10Hz
constexpr uint16_t kRptErrCmd    = 3;      // (1002,3) 错误上报 → 错误门控

constexpr int  kMotionStateRLControl = 17; // MotionState==17 → RL 控制就绪
constexpr int  kStartupTimeoutMs     = 15000;
constexpr int  kCmdStaleTimeoutMs    = 500;  // cmd_vel 超时 → 发零速
}  // namespace

/// cmd_vel 桥节点：/cmd_vel → UDP JSON 轴指令 + 状态机 + 错误门控。
class CmdVelBridgeNode : public rclcpp::Node
{
public:
  explicit CmdVelBridgeNode(const rclcpp::NodeOptions & options);
  ~CmdVelBridgeNode() override;

private:
  void declareParams();
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void heartbeatTimerCb();   // 1Hz 心跳 (100,100)
  void controlTimerCb();     // 20Hz 启动序列 + 轴指令 + 错误门控/恢复
  void udpRecvLoop();        // 收帧线程
  bool openSocket();
  bool sendJson(uint16_t type, uint16_t command, const nlohmann::json & items);
  void publishStatus();
  std::string stateName() const;
  void onResetError(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);  // 人工复位服务

  // 参数
  std::string robot_ip_;
  uint16_t port_{30000};
  double cmd_frequency_hz_{20.0};
  double heartbeat_hz_{1.0};
  double vx_max_{0.5};
  double vy_max_{0.5};
  double wz_max_{0.6};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string status_topic_{"/m20_bridge_status"};
  // ErrorHalt 有界恢复参数（默认关 → 保持 halt，仅人工复位/重启）
  ErrorRecoveryConfig recovery_cfg_;

  // 网络
  int sock_{-1};
  sockaddr_in dst_{};
  std::atomic<bool> recv_run_{true};
  std::thread recv_thread_;
  std::atomic<uint16_t> msg_id_{0};

  // cmd_vel 最新目标（订阅线程写 / 控制定时器读）
  std::mutex cmd_mutex_;
  bool has_cmd_{false};
  double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_vw_{0.0};   // 原始 m/s, rad/s
  std::chrono::steady_clock::time_point last_cmd_time_{};

  // 上报数据（收帧线程写 / 控制定时器读）
  std::mutex rpt_mutex_;
  int16_t motion_state_{0};
  uint32_t control_usage_mode_{0};   // 0=Regular, 非 0 视为非 Regular → 错误门控
  bool have_status_{false};          // 收到过 (1002,6) 才算有状态证据（恢复决策用）
  bool error_flag_{false};
  uint32_t error_code_{0};
  bool error_logged_{false};
  std::chrono::steady_clock::time_point last_error_time_{};  // 最近一次 (1002,3)

  // 启动序列状态机
  enum class State { StartRegular, StandUp, WaitRl, SetGait, Run, ErrorHalt };
  State state_{State::StartRegular};
  std::chrono::steady_clock::time_point state_enter_{};
  int recovery_attempts_{0};                          // 已完成的有界重试次数
  std::atomic<bool> manual_reset_requested_{false};   // 人工复位服务 → 定时器线程消费

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

CmdVelBridgeNode::CmdVelBridgeNode(const rclcpp::NodeOptions & options)
: Node("m20_cmd_vel_bridge", options)
{
  declareParams();

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10).reliable(),
    [this](geometry_msgs::msg::Twist::SharedPtr msg) { onCmdVel(msg); });

  status_pub_ = create_publisher<std_msgs::msg::String>(
    status_topic_, rclcpp::QoS(10).transient_local());

  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / heartbeat_hz_)),
    [this]() { heartbeatTimerCb(); });

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / cmd_frequency_hz_)),
    [this]() { controlTimerCb(); });

  // 人工复位服务：ErrorHalt（含自动恢复达上限后）只能由此或重启节点退出。
  // 用法: ros2 service call /m20_cmd_vel_bridge/reset_error std_srvs/srv/Trigger {}
  reset_service_ = create_service<std_srvs::srv::Trigger>(
    "~/reset_error",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      onResetError(req, resp);
    });

  if (!openSocket()) {
    RCLCPP_ERROR(this->get_logger(),
      "UDP socket 创建失败(ip=%s port=%u)，桥接不工作",
      robot_ip_.c_str(), static_cast<unsigned int>(port_));
    return;
  }
  recv_thread_ = std::thread(&CmdVelBridgeNode::udpRecvLoop, this);
}

CmdVelBridgeNode::~CmdVelBridgeNode()
{
  recv_run_ = false;
  if (recv_thread_.joinable()) { recv_thread_.join(); }
  if (sock_ >= 0) { close(sock_); sock_ = -1; }
}

void CmdVelBridgeNode::declareParams()
{
  // 与 robot_configs/config/m20.yaml 保持一致，P3 由 bringup 注入
  // 数字参数 fail-fast 校验：非法值直接抛异常 → 节点构造失败、进程退出
  robot_ip_ = declare_parameter<std::string>("robot_ip", "10.21.31.103");
  if (robot_ip_.empty()) {
    throw std::invalid_argument("参数 'robot_ip' 不能为空");
  }
  port_ = static_cast<uint16_t>(
    requirePortRange(declare_parameter<int>("port", 30000), "port"));
  cmd_frequency_hz_ = requireFrequencyHz(
    declare_parameter<double>("cmd_frequency_hz", 20.0), "cmd_frequency_hz", 200.0);
  heartbeat_hz_ = requireFrequencyHz(
    declare_parameter<double>("heartbeat_hz", 1.0), "heartbeat_hz", 100.0);
  vx_max_ = requireFinitePositive(declare_parameter<double>("vx_max", 0.5), "vx_max");
  vy_max_ = requireFinitePositive(declare_parameter<double>("vy_max", 0.5), "vy_max");
  wz_max_ = requireFinitePositive(declare_parameter<double>("wz_max", 0.6), "wz_max");
  // ErrorHalt 有界恢复（默认关 = 硬件不确定项：协议无错误清除帧，
  // 自动恢复仅在机器人自身上报 Regular 模式时才重跑启动序列）
  recovery_cfg_.auto_recovery = declare_parameter<bool>("error_auto_recovery", false);
  recovery_cfg_.max_attempts = requireIntRange(
    declare_parameter<int>("error_max_recovery_attempts", 3),
    "error_max_recovery_attempts", 0, 100);
  recovery_cfg_.base_backoff_s = requireFinitePositive(
    declare_parameter<double>("error_recovery_backoff_s", 5.0),
    "error_recovery_backoff_s");
  recovery_cfg_.error_clear_window_s = requireFinitePositive(
    declare_parameter<double>("error_clear_window_s", 5.0),
    "error_clear_window_s");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  status_topic_ = declare_parameter<std::string>("status_topic", "/m20_bridge_status");
}

bool CmdVelBridgeNode::openSocket()
{
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ < 0) return false;

  // 收帧超时 100ms，便于退出线程
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  memset(&dst_, 0, sizeof(dst_));
  dst_.sin_family = AF_INET;
  dst_.sin_port = htons(port_);
  return inet_pton(AF_INET, robot_ip_.c_str(), &dst_.sin_addr) == 1;
}

bool CmdVelBridgeNode::sendJson(uint16_t type, uint16_t command, const nlohmann::json & items)
{
  if (sock_ < 0) return false;

  // Time: 当前本地时间 YYYY-MM-DD HH:MM:SS
  std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  char time_str[32];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);

  nlohmann::json body;
  body["PatrolDevice"]["Type"] = type;
  body["PatrolDevice"]["Command"] = command;
  body["PatrolDevice"]["Time"] = time_str;
  body["PatrolDevice"]["Items"] = items;

  std::string body_str = body.dump();
  uint16_t asdu_len = static_cast<uint16_t>(body_str.size());

  // 16 字节帧头
  std::vector<uint8_t> frame(kHeaderLen);
  frame[0] = kSync0; frame[1] = kSync1; frame[2] = kSync2; frame[3] = kSync3;
  frame[4] = static_cast<uint8_t>(asdu_len & 0xFF);
  frame[5] = static_cast<uint8_t>((asdu_len >> 8) & 0xFF);
  uint16_t id = msg_id_.fetch_add(1) & 0xFFFF;   // 0→65535 循环，应答回显
  frame[6] = static_cast<uint8_t>(id & 0xFF);
  frame[7] = static_cast<uint8_t>((id >> 8) & 0xFF);
  frame[8] = kJsonMarker;                         // [9-15] 全 0
  frame.insert(frame.end(), body_str.begin(), body_str.end());

  ssize_t n = sendto(sock_, frame.data(), frame.size(), 0,
                     reinterpret_cast<const sockaddr *>(&dst_), sizeof(dst_));
  if (n < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "sendto 失败: %s", strerror(errno));
    return false;
  }
  return true;
}

void CmdVelBridgeNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // P0：任何除法/归一化前拒绝 NaN/Inf；不更新 last_cmd_time_ → 看门狗触发零速
  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
    !std::isfinite(msg->angular.z))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "忽略非有限 cmd_vel (NaN/Inf)：不更新指令时间戳，看门狗将下发零速");
    return;
  }
  std::lock_guard<std::mutex> lk(cmd_mutex_);
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_vw_ = msg->angular.z;
  has_cmd_ = true;
  last_cmd_time_ = std::chrono::steady_clock::now();
}

void CmdVelBridgeNode::heartbeatTimerCb()
{
  // 心跳 (Type=100, Command=100, Items={})，1Hz
  sendJson(kHeartbeatType, kHeartbeatCmd, nlohmann::json::object());
}

void CmdVelBridgeNode::controlTimerCb()
{
  const auto now = std::chrono::steady_clock::now();

  // ---- 人工复位（服务线程只置原子标志，状态全部在本定时器线程变更）----
  if (manual_reset_requested_.exchange(false)) {
    std::lock_guard<std::mutex> lk(rpt_mutex_);
    error_flag_ = false;
    error_code_ = 0;
    error_logged_ = false;
    recovery_attempts_ = 0;
    if (state_ == State::ErrorHalt) {
      state_ = State::StartRegular;
      state_enter_ = now;
      RCLCPP_INFO(this->get_logger(),
        "人工复位：清除错误锁存，重启启动序列 (AUTH+MODE)。若机器人仍上报非 Regular 模式，门控将再次生效");
    } else {
      RCLCPP_INFO(this->get_logger(), "人工复位：当前非 ErrorHalt，仅清除错误锁存");
    }
  }

  // ---- 错误门控（终审修复：门控用"窗口内新错误/非 Regular"，与 sticky
  //      error_flag_ 解耦；error_flag_ 仅作状态展示/历史）----
  bool stay_halt = false;
  {
    std::lock_guard<std::mutex> lk(rpt_mutex_);
    const bool is_halt = (state_ == State::ErrorHalt);
    const bool mode_gated = (control_usage_mode_ != 0);
    const double since_enter_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - state_enter_).count();
    const double since_error_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - last_error_time_).count();
    ErrorHaltInputs inputs{
      have_status_,
      control_usage_mode_ == 0,
      since_enter_s,
      since_error_s,
      recovery_attempts_};
    // 决策函数与节点逻辑逐字节一致（见 error_recovery.hpp，含集成测试）
    const GateDecision d = evaluateErrorGate(
      is_halt, mode_gated, since_error_s, recovery_cfg_, inputs);

    if (d.enter_halt) {
      if (!error_logged_) {
        RCLCPP_ERROR(this->get_logger(),
          "M20 错误门控触发: error_flag(历史)=%d error_code=0x%04X control_usage_mode=%u (非 Regular) -> ErrorHalt，持续零速",
          static_cast<int>(error_flag_), static_cast<unsigned int>(error_code_),
          static_cast<unsigned int>(control_usage_mode_));
        error_logged_ = true;
      }
      state_ = State::ErrorHalt;
      state_enter_ = now;
    }
    if (d.retry) {
      ++recovery_attempts_;   // 有界：达到 max_attempts 后不再自动重试
      error_flag_ = false;    // 历史锁存随重试清除（门控已不再依赖它）
      error_code_ = 0;
      error_logged_ = false;
      state_ = State::StartRegular;
      state_enter_ = now;
      RCLCPP_INFO(this->get_logger(),
        "ErrorHalt 有界恢复: 第 %d/%d 次重试，重跑启动序列 (AUTH+MODE)",
        recovery_attempts_, recovery_cfg_.max_attempts);
    }
    stay_halt = d.stay_halt;
  }
  if (stay_halt) {
    // ErrorHalt 期间每 tick 持续下发零速（不再只在进入时发一次；
    // 若单次零速帧被 UDP 丢失，机器人仍会执行上一非零指令 → 持续零速兜底）
    sendJson(kAxisCmdType, kAxisCmdCmd,
      nlohmann::json{{"X", 0.0}, {"Y", 0.0}, {"Z", 0.0},
                     {"Roll", 0.0}, {"Pitch", 0.0}, {"Yaw", 0.0}});
    publishStatus();
    return;
  }

  switch (state_) {
    case State::StartRegular:
      // (1101,5,Mode=0) 确保 Regular 模式
      sendJson(kModeSetType, kModeSetCmd, nlohmann::json{{"Mode", 0}});
      state_ = State::StandUp;
      state_enter_ = std::chrono::steady_clock::now();
      break;

    case State::StandUp:
      // (2,22,MotionParam=1) 起立
      sendJson(kStandUpType, kStandUpCmd, nlohmann::json{{"MotionParam", 1}});
      state_ = State::WaitRl;
      state_enter_ = std::chrono::steady_clock::now();
      break;

    case State::WaitRl: {
      // 等待上报 MotionState==17 (RL 控制)；15s 超时告警但不阻塞
      int16_t ms = 0;
      {
        std::lock_guard<std::mutex> lk(rpt_mutex_);
        ms = motion_state_;
      }
      bool ready = (ms == kMotionStateRLControl);
      bool timeout = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - state_enter_).count() >= kStartupTimeoutMs;
      if (!ready && !timeout) { publishStatus(); return; }
      if (timeout && !ready) {
        RCLCPP_WARN(this->get_logger(),
          "等待 MotionState==17 (RL 控制) 超时 %d ms, 当前=%d; 继续流程", kStartupTimeoutMs, ms);
      } else {
        RCLCPP_INFO(this->get_logger(), "M20 进入 RL 控制 (MotionState=%d)", ms);
      }
      // (2,23,GaitParam=0x1001) 切基础平地步态
      sendJson(kGaitType, kGaitCmd, nlohmann::json{{"GaitParam", 0x1001}});
      state_ = State::SetGait;
      state_enter_ = std::chrono::steady_clock::now();
      break;
    }

    case State::SetGait:
      // 步态切换已发出（上面 WaitRl 内发送），此处直接进入 RUN
      RCLCPP_INFO_ONCE(this->get_logger(), "M20 启动序列完成, 进入 RUN (轴指令 20Hz)");
      state_ = State::Run;
      state_enter_ = std::chrono::steady_clock::now();
      break;

    case State::Run: {
      // 轴指令: X=cmd.linear.x/vx_max, Y=cmd.linear.y/vy_max, Yaw=cmd.angular.z/wz_max
      // 归一化 clamp [-1,1] 保留 2 位小数；Z/Roll/Pitch 恒 0（Basic 步态仅 X/Y/Yaw 生效）
      // （纯函数：输入/限值非有限 → invalid → 零速，不做除法）
      double vx, vy, vyaw;
      {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - last_cmd_time_).count();
        if (!has_cmd_ || elapsed_ms >= kCmdStaleTimeoutMs) {
          vx = vy = vyaw = 0.0;   // cmd_vel 超时/未到 → 零速
        } else {
          const auto norm =
            normalizeCmdVel(cmd_vx_, cmd_vy_, cmd_vw_, vx_max_, vy_max_, wz_max_);
          if (norm.valid) {
            vx = norm.x;
            vy = norm.y;
            vyaw = norm.yaw;
          } else {
            vx = vy = vyaw = 0.0;
          }
        }
      }
      nlohmann::json items{
        {"X", vx}, {"Y", vy}, {"Z", 0.0},
        {"Roll", 0.0}, {"Pitch", 0.0}, {"Yaw", vyaw}};
      sendJson(kAxisCmdType, kAxisCmdCmd, items);
      break;
    }

    case State::ErrorHalt:
    default:
      break;   // 错误门控下不再发轴指令
  }
  publishStatus();
}

void CmdVelBridgeNode::udpRecvLoop()
{
  std::vector<uint8_t> buf(65536);
  const uint32_t peer_addr = dst_.sin_addr.s_addr;   // 网络序，与 recvfrom 一致
  const uint16_t peer_port = dst_.sin_port;
  while (recv_run_ && rclcpp::ok()) {
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(sock_, buf.data(), buf.size(), 0,
                         reinterpret_cast<sockaddr *>(&src), &slen);
    if (n <= 0) { continue; }   // 超时/错误：继续

    // review: 来源校验 —— 只接受配置的机器人 IP:端口（防伪造/串流）
    if (!validatePeer(src.sin_addr.s_addr, src.sin_port, peer_addr, peer_port)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "丢弃来源不符的上报帧 (src ip:port 与配置的 robot_ip:port 不一致)");
      continue;
    }

    // review: 严格解析 —— n == 16 + asdu_len（截断/尾随/缺字段/越界一律拒绝）
    uint16_t rtype = 0, rcmd = 0;
    nlohmann::json body;
    const ReportParseResult pr = parseReportFrame(
      buf.data(), static_cast<size_t>(n), rtype, rcmd, body);
    if (pr != ReportParseResult::kOk) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "丢弃上报帧 (parse=%d, n=%zd)", static_cast<int>(pr), static_cast<ssize_t>(n));
      continue;
    }

    if (rtype == kRptType && rcmd == kRptStatusCmd) {
      // (1002,6) 状态上报: MotionState / ControlUsageMode（两字段必须齐全且数值在界内）
      StatusReport sr;
      if (!parseStatusReport(body, sr)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "状态上报缺字段或数值越界，丢弃: %s", body.dump().c_str());
        continue;
      }
      {
        std::lock_guard<std::mutex> lk(rpt_mutex_);
        have_status_ = true;   // 收到状态上报 = 有证据（恢复决策的前提之一）
        motion_state_ = sr.motion_state;
        control_usage_mode_ = sr.control_usage_mode;
      }
      RCLCPP_DEBUG(this->get_logger(),
        "状态上报: MotionState=%d ControlUsageMode=%u",
        static_cast<int>(sr.motion_state), static_cast<unsigned int>(sr.control_usage_mode));
    } else if (rtype == kRptType && rcmd == kRptErrCmd) {
      // (1002,3) 错误上报 → 置错误标志并记录错误码（必须能提取到错误码，否则拒绝）
      uint32_t ec = 0;
      if (!parseErrorReport(body, ec)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "错误上报未找到错误码字段，丢弃: %s", body.dump().c_str());
        continue;
      }
      {
        std::lock_guard<std::mutex> lk(rpt_mutex_);
        error_flag_ = true;
        error_code_ = ec;
        error_logged_ = false;
        last_error_time_ = std::chrono::steady_clock::now();
      }
      RCLCPP_ERROR(this->get_logger(), "M20 错误上报 (1002,3): error_code=0x%04X",
        static_cast<unsigned int>(ec));
    } else if (rtype == kRptType && rcmd == kRptSpeedCmd) {
      // (1002,4) 速度反馈 10Hz，仅调试
      RCLCPP_DEBUG(this->get_logger(), "速度反馈帧: %s", body.dump().c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "未识别上报帧 Type=%u Command=%u: %s",
        rtype, rcmd, body.dump().c_str());
    }
  }
}

std::string CmdVelBridgeNode::stateName() const
{
  switch (state_) {
    case State::StartRegular: return "START_REGULAR";
    case State::StandUp:      return "STAND_UP";
    case State::WaitRl:       return "WAIT_RL";
    case State::SetGait:      return "SET_GAIT";
    case State::Run:          return "RUN";
    case State::ErrorHalt:    return "ERROR_HALT";
  }
  return "UNKNOWN";
}

void CmdVelBridgeNode::publishStatus()
{
  auto msg = std_msgs::msg::String();
  char ec_str[16];
  {
    std::lock_guard<std::mutex> lk(rpt_mutex_);
    snprintf(ec_str, sizeof(ec_str), "%04X", static_cast<unsigned int>(error_code_));
    msg.data = "state=" + stateName() +
               " motion_state=" + std::to_string(motion_state_) +
               " control_usage_mode=" + std::to_string(control_usage_mode_) +
               " error_flag=" + std::to_string(error_flag_ ? 1 : 0) +
               " error_code=0x" + ec_str +
               " have_status=" + std::to_string(have_status_ ? 1 : 0) +
               " recovery_attempts=" + std::to_string(recovery_attempts_) +
               "/" + std::to_string(recovery_cfg_.max_attempts);
  }
  status_pub_->publish(msg);
}

void CmdVelBridgeNode::onResetError(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  // 人工复位（显式、有界）：仅置原子标志，由控制定时器线程消费——
  // 所有 state_/error_* 变更保持在单一线程，避免引入锁竞争。
  manual_reset_requested_.store(true);
  resp->success = true;
  resp->message =
    "已请求人工复位：清除错误锁存并重启启动序列 (AUTH+MODE)。"
    "若机器人仍上报非 Regular 模式或再次报错，ErrorHalt 会重新生效。"
    "自动恢复已" + std::string(recovery_cfg_.auto_recovery ? "开启" : "关闭") +
    "（重试上限 " + std::to_string(recovery_cfg_.max_attempts) + " 次）。";
}

}  // namespace adapter_m20

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adapter_m20::CmdVelBridgeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
