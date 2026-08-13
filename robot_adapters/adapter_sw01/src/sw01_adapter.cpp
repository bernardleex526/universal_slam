// ============================================================
// adapter_sw01 / src/sw01_adapter.cpp
// SW01 适配器（完整实现, P2）—— 封装 Daedalus 专有 C 库 (libASTRALL_SDK.so)
// ------------------------------------------------------------
// 功能：
//   1. IMU:    AstrallSubscriptionData(IMU, 250Hz) → /imu_raw (sensor_msgs/Imu, imu 帧)
//   2. 点云:   UDP 6100 自定义协议 AstrallPosCloudData 重组 → /points_raw
//              (sensor_msgs/PointCloud2, lidar 帧, best_effort)
//   3. 控制:   订阅 /cmd_vel → AstrallMove(vx,vy,vyaw) ≥20Hz；50ms 看门狗 → 发 (0,0,0)
//   4. 心跳:   AstrallHeartbeat 10Hz；控制权 AstrallAuthControl 申请/丢失恢复（5s 重试）
//
// P0 安全修复：
//   - cmd_vel 非有限值（NaN/Inf）在除法/归一化前拒绝（watchdog 触发零速）
//   - 启动参数 fail-fast 校验（有限正数/端口/频率下限）
//   - UDP 解析重构为长度驱动纯函数 parseCloudPacket（见 cloud_packet.hpp）：
//     短包不再读到缓冲旧内容；dynamic/fixed 两种布局显式选择、包长精确校验
//
// ring/time 契约：SW01 协议不提供 ring 与逐点 time → /points_raw 仅 x,y,z,intensity
//   （不伪造数据）。锁定版 FAST-LIO lidar_type=2 在缺少 time 字段时走
//   yaw 角时序回退（given_offset_time=false），缺失 ring 时全部点落入 layer 0，
//   与现有字段真实兼容（详见 README.md「点云字段契约」）。
//
// 线程说明：SDK 回调（IMU/状态）在 SDK 线程执行，rclcpp publish 线程安全可直接调用；
// 点云 UDP 在独立线程；控制/心跳/申请在 rclcpp 定时器线程。
// ============================================================
#define _POSIX_C_SOURCE 200809L   // recvfrom / timeval 等 POSIX 接口

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "adapter_sw01/cloud_packet.hpp"
#include "adapter_sw01/sw01_math.hpp"

#include <interface.h>   // ASTRALL SDK（include 路径由 CMake 的 ASTRALL_SDK_DIR 提供）

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace adapter_sw01 {

namespace {
constexpr int    kHeartbeatPeriodMs = 100;      // 心跳 10Hz（SDK 约定，500ms 无心跳进失控保护）
constexpr int    kAuthRetryPeriodS  = 5;        // 控制权丢失后重试周期
constexpr int    kCmdStaleMs        = 50;       // 50ms 看门狗：超时未收到新指令 → 发 (0,0,0)
constexpr uint32_t kCloudMaxPoints  = (1U << 20);  // 单帧点云上限（防御性，防恶意包撑爆内存）

/// 发布用内部缓冲点（x,y,z,intensity 连续 4×float32，便于 memcpy）
struct CloudPoint
{
  float x, y, z, intensity;
};
static_assert(sizeof(CloudPoint) == 16, "CloudPoint 必须为 16 字节");
}  // namespace

// ============================================================
// AstrallSdkWrapper —— Daedalus SDK 封装
// ============================================================
class AstrallSdkWrapper
{
public:
  using ImuCallback    = std::function<void(const AstrallImuData & imu)>;
  using StatusCallback = std::function<void(bool link, bool ctrl_authority)>;

  AstrallSdkWrapper() = default;
  ~AstrallSdkWrapper() { deinit(); }

  bool init(const ImuCallback & imu_cb, const StatusCallback & status_cb, const rclcpp::Logger & logger);
  void deinit();

  bool inited() const { return inited_.load(); }
  bool link() const { return link_.load(); }
  bool ctrl_authority() const { return ctrl_authority_.load(); }

  uint16_t sendHeartbeat()      { return AstrallHeartbeat(); }
  uint16_t requestControl()     { return AstrallAuthControl(ASTRALL_AUTH_SDK); }
  uint16_t switchToMoveMode()   { return AstrallSportModeControl(ASTRALL_SPORT_MODE_CMD_MOVE); }
  uint16_t move(float vx, float vy, float vyaw) { return AstrallMove(vx, vy, vyaw); }

private:
  std::atomic<bool> inited_{false};
  std::atomic<bool> link_{false};
  std::atomic<bool> ctrl_authority_{false};
};

bool AstrallSdkWrapper::init(
  const ImuCallback & imu_cb, const StatusCallback & status_cb, const rclcpp::Logger & logger)
{
  if (inited_.load()) { return true; }

  AstrallConfig cfg;
  // 心跳回调：无需处理（节点定时器主动 AstrallHeartbeat）
  cfg.heartbeatCb = [](void *, uint16_t) {};
  // SDK 状态回调：SDK 线程调用；监控 link / ctrlAuthority 丢失
  cfg.sdkStatusCb = [this, status_cb](void * data, uint16_t) {
    if (!data) { return; }
    const auto * s = static_cast<const AstrallSdkStatus *>(data);
    link_.store(s->link);
    ctrl_authority_.store(s->ctrlAuthority);
    if (status_cb) { status_cb(s->link, s->ctrlAuthority); }
  };

  uint16_t res = AstrallSdkInit(cfg);
  if (res != ASTRALL_RES_SUCCESSED) {
    RCLCPP_ERROR(logger, "AstrallSdkInit 失败 res=0x%04X", static_cast<unsigned int>(res));
    return false;
  }
  inited_.store(true);

  // 订阅 IMU 250Hz（回调在 SDK 线程执行，rclcpp publish 线程安全可直接调用）
  res = AstrallSubscriptionData(
    ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_250HZ,
    [imu_cb](void * data, uint16_t len) {
      if (data && len == sizeof(AstrallImuData)) {
        const auto & imu = *static_cast<const AstrallImuData *>(data);
        if (imu_cb) { imu_cb(imu); }
      }
    });
  if (res != ASTRALL_RES_SUCCESSED) {
    RCLCPP_WARN(logger, "AstrallSubscriptionData(IMU) 失败 res=0x%04X", static_cast<unsigned int>(res));
    return false;
  }

  // 手册 5.10：订阅 ASTRALL_SUB_TOPIC_ID_LIDAR 成功后，机器人经 UDP 6100 向本机
  // 推送点云（数据不走此回调，由 cloudRecvLoop 在 UDP 6100 接收）。
  // 频率按手册"填除 0 外任意值"，此处用 25HZ 占位；订阅失败仅告警（UDP 推送
  // 是否依赖订阅需实机确认，见 README 未决项 6）。
  res = AstrallSubscriptionData(
    ASTRALL_SUB_TOPIC_ID_LIDAR, ASTRALL_SUB_FREQ_25HZ,
    [](void *, uint16_t) {});
  if (res != ASTRALL_RES_SUCCESSED) {
    RCLCPP_WARN(logger, "AstrallSubscriptionData(LIDAR, 触发 UDP 6100 推送) 失败 res=0x%04X",
      static_cast<unsigned int>(res));
  }
  return true;
}

void AstrallSdkWrapper::deinit()
{
  if (inited_.exchange(false)) {
    AstrallSdkDeinit();
  }
}

// ============================================================
// Sw01AdapterNode —— ROS2 节点
// ============================================================
class Sw01AdapterNode : public rclcpp::Node
{
public:
  explicit Sw01AdapterNode(const rclcpp::NodeOptions & options);
  ~Sw01AdapterNode() override;

private:
  void declareParams();
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void onImuData(const AstrallImuData & imu);
  void onSdkStatus(bool link, bool ctrl_authority);

  void controlLoop();    // ≥20Hz：cmd_vel → AstrallMove（50ms 看门狗）
  void heartbeatLoop();  // 10Hz
  void authRetryLoop();  // 5s：控制权丢失重试

  int openCloudSocket();
  void cloudRecvLoop();  // UDP 6100 点云重组线程
  void publishPointCloud();  // 需在持有 cloud_mutex_ 时调用

  // ---- 参数 ----
  uint16_t lidar_port_{6100};
  std::string imu_frame_{"imu"};
  std::string lidar_frame_{"lidar"};
  double point_scale_{0.001};   // int32 坐标 → 米（单位待实测，疑似 mm）
  double vx_max_{0.5}, vy_max_{0.5}, wz_max_{0.6};
  double cmd_frequency_hz_{20.0};
  CloudPacketLayout cloud_layout_{CloudPacketLayout::kDynamic};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string imu_topic_{"/imu_raw"};
  std::string points_topic_{"/points_raw"};

  // ---- SDK ----
  AstrallSdkWrapper sdk_;
  std::atomic<bool> auth_ok_{false};
  std::atomic<bool> move_mode_ok_{false};
  std::atomic<bool> was_authority_{false};

  // ---- cmd_vel ----
  std::mutex cmd_mutex_;
  bool has_cmd_{false};
  double cmd_vx_{0.0}, cmd_vy_{0.0}, cmd_vw_{0.0};
  std::chrono::steady_clock::time_point last_cmd_{};

  // ---- 点云重组（UDP 线程）----
  std::mutex cloud_mutex_;
  std::vector<CloudPoint> cloud_buf_;
  std::vector<uint8_t> cloud_filled_;
  uint32_t cloud_total_{0};
  uint32_t cloud_filled_count_{0};
  uint32_t cloud_last_index_{0};   // 当前帧已见的最大点索引（用于帧边界检测）
  uint64_t cloud_ts_ns_{0};
  int cloud_sock_{-1};
  std::atomic<bool> cloud_run_{true};
  std::thread cloud_thread_;

  // ---- ROS ----
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr auth_retry_timer_;
};

Sw01AdapterNode::Sw01AdapterNode(const rclcpp::NodeOptions & options)
: Node("sw01_adapter", options)
{
  declareParams();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::QoS(10).reliable());
  points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    points_topic_, rclcpp::QoS(5).best_effort());
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10).reliable(),
    [this](geometry_msgs::msg::Twist::SharedPtr msg) { onCmdVel(msg); });

  // 控制 ≥20Hz / 心跳 10Hz / 控制权重试 5s
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(1000.0 / cmd_frequency_hz_)),
    [this]() { controlLoop(); });
  heartbeat_timer_ = create_wall_timer(
    std::chrono::milliseconds(kHeartbeatPeriodMs),
    [this]() { heartbeatLoop(); });
  auth_retry_timer_ = create_wall_timer(
    std::chrono::seconds(kAuthRetryPeriodS),
    [this]() { authRetryLoop(); });

  // SDK 初始化 + IMU 订阅
  if (!sdk_.init(
        [this](const AstrallImuData & imu) { onImuData(imu); },
        [this](bool link, bool ctrl_authority) { onSdkStatus(link, ctrl_authority); },
        this->get_logger())) {
    RCLCPP_ERROR(this->get_logger(),
      "SW01 SDK 初始化失败，节点继续运行（心跳/点云将不可用）");
  }

  // 点云 UDP 接收线程
  cloud_sock_ = openCloudSocket();
  if (cloud_sock_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "点云 UDP 端口 %u 绑定失败，/points_raw 不可用",
      static_cast<unsigned int>(lidar_port_));
  } else {
    cloud_thread_ = std::thread(&Sw01AdapterNode::cloudRecvLoop, this);
    RCLCPP_INFO(this->get_logger(), "点云 UDP 监听 :%u 已启动",
      static_cast<unsigned int>(lidar_port_));
  }

  // ring/time 契约（一次性说明，勿伪造数据）：
  // SW01 协议无 ring/逐点 time → /points_raw 仅 x,y,z,intensity。
  // 锁定版 FAST-LIO lidar_type=2 缺 time 时走 yaw 角回退、缺 ring 时全部点归入 layer 0。
  RCLCPP_WARN_ONCE(this->get_logger(),
    "/points_raw 仅含 x,y,z,intensity（协议无 ring/逐点 time，不伪造）；"
    "FAST-LIO 须配置 lidar_type=2 使用 yaw 角时序回退，详见 adapter_sw01/README.md");
}

Sw01AdapterNode::~Sw01AdapterNode()
{
  // 先停 SDK（避免回调访问已析构成员），再停 UDP 线程
  sdk_.deinit();
  cloud_run_ = false;
  if (cloud_thread_.joinable()) { cloud_thread_.join(); }
  if (cloud_sock_ >= 0) { close(cloud_sock_); cloud_sock_ = -1; }
}

void Sw01AdapterNode::declareParams()
{
  // 与 robot_configs/config/sw01.yaml 保持一致，P3 由 bringup 注入
  // 数字参数 fail-fast 校验：非法值直接抛异常 → 节点构造失败、进程退出
  // 注意：无 robot_ip 参数 —— 控制经 SDK，点云为入站 UDP（监听 0.0.0.0）
  lidar_port_ = static_cast<uint16_t>(
    requirePortRange(declare_parameter<int>("lidar_port", 6100), "lidar_port"));
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
  point_scale_ = requireFinitePositive(
    declare_parameter<double>("point_scale", 0.001), "point_scale");
  vx_max_ = requireFinitePositive(declare_parameter<double>("vx_max", 0.5), "vx_max");
  vy_max_ = requireFinitePositive(declare_parameter<double>("vy_max", 0.5), "vy_max");
  wz_max_ = requireFinitePositive(declare_parameter<double>("wz_max", 0.6), "wz_max");
  // SDK 契约 ≥20Hz（50ms 看门狗边界），且防止频率过大导致定时器周期取整为 0ms。
  // P1: 默认提升到 30Hz（33ms 周期），为机器人侧 50ms 无指令自动停止留出调度余量。
  cmd_frequency_hz_ = requireFiniteAtLeast(
    declare_parameter<double>("cmd_frequency_hz", 30.0), "cmd_frequency_hz", 20.0);
  requireFrequencyHz(cmd_frequency_hz_, "cmd_frequency_hz", 200.0);

  const std::string layout =
    declare_parameter<std::string>("cloud_packet_layout", "dynamic");
  if (layout == "dynamic") {
    cloud_layout_ = CloudPacketLayout::kDynamic;
  } else if (layout == "fixed") {
    cloud_layout_ = CloudPacketLayout::kFixed;
  } else {
    throw std::invalid_argument(
      "参数 'cloud_packet_layout' 必须为 'dynamic' 或 'fixed', 实际=" + layout);
  }

  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_raw");
  points_topic_ = declare_parameter<std::string>("points_topic", "/points_raw");
}

void Sw01AdapterNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // P0：任何除法/归一化前拒绝 NaN/Inf；不更新 last_cmd_ → 50ms 看门狗触发零速
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
  last_cmd_ = std::chrono::steady_clock::now();
}

void Sw01AdapterNode::onImuData(const AstrallImuData & imu)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.frame_id = imu_frame_;

  // 时间戳：SDK 自带（疑似纳秒，单位待实测确认；若为微秒改除 1e6）。
  // 与 /points_raw 保持同一时间基（SDK 时间），便于 FAST-LIO2 插值对齐。
  if (imu.timestamp > 0) {
    msg.header.stamp.sec = static_cast<int32_t>(imu.timestamp / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(imu.timestamp % 1000000000LL);
  } else {
    msg.header.stamp = this->now();
  }

  msg.linear_acceleration.x = imu.accelerometer[0];
  msg.linear_acceleration.y = imu.accelerometer[1];
  msg.linear_acceleration.z = imu.accelerometer[2];
  msg.angular_velocity.x = imu.gyroscope[0];
  msg.angular_velocity.y = imu.gyroscope[1];
  msg.angular_velocity.z = imu.gyroscope[2];

  // 四元数顺序待实测：SDK 注释未明确 wxyz/xyzw。按常见惯例假定 [w,x,y,z]；
  // 若实机为 [x,y,z,w]，P4 调整赋值顺序。
  msg.orientation.w = imu.quaternion[0];
  msg.orientation.x = imu.quaternion[1];
  msg.orientation.y = imu.quaternion[2];
  msg.orientation.z = imu.quaternion[3];
  // 协方差未知，留 0（FAST-LIO2 只使用上述数值，不读协方差）

  imu_pub_->publish(msg);
}

void Sw01AdapterNode::onSdkStatus(bool /*link*/, bool ctrl_authority)
{
  bool prev = was_authority_.exchange(ctrl_authority);
  if (prev && !ctrl_authority) {
    RCLCPP_WARN(this->get_logger(),
      "SDK 控制权丢失（遥控器抢占等），停止移动，%d s 后重试申请控制权", kAuthRetryPeriodS);
  }
}

void Sw01AdapterNode::controlLoop()
{
  if (!sdk_.inited()) { return; }

  // 未持有控制权 / 未处于 MOVE 模式 → 不发移动指令（50ms 看门狗会让机器人自行停止）
  if (!auth_ok_.load() || !move_mode_ok_.load() || !sdk_.ctrl_authority()) { return; }

  double vx, vy, vyaw;
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_cmd_).count();
    if (!has_cmd_ || elapsed_ms >= kCmdStaleMs) {
      // 50ms 内未收到新指令 → 安全停止 (0,0,0)
      vx = vy = vyaw = 0.0;
    } else {
      // cmd_vel(m/s, rad/s) / 机型最大速度 → SDK 归一化 [-1,1]
      // （纯函数：输入/限值非有限 → invalid → 零速，不做除法）
      const auto norm = normalizeCmdVel(cmd_vx_, cmd_vy_, cmd_vw_, vx_max_, vy_max_, wz_max_);
      if (norm.valid) {
        vx = norm.x;
        vy = norm.y;
        vyaw = norm.yaw;
      } else {
        vx = vy = vyaw = 0.0;
      }
    }
  }

  uint16_t res = sdk_.move(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vyaw));
  if (res != ASTRALL_RES_SUCCESSED) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "AstrallMove 失败 res=0x%04X", static_cast<unsigned int>(res));
  }
}

void Sw01AdapterNode::heartbeatLoop()
{
  if (!sdk_.inited()) { return; }
  uint16_t res = sdk_.sendHeartbeat();
  if (res != ASTRALL_RES_SUCCESSED) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "AstrallHeartbeat 失败 res=0x%04X", static_cast<unsigned int>(res));
  }
}

void Sw01AdapterNode::authRetryLoop()
{
  if (!sdk_.inited()) { return; }

  if (!sdk_.ctrl_authority()) {
    // 无控制权（初始或丢失）→ 每 5s 申请一次
    if (auth_ok_.load()) {
      RCLCPP_WARN(this->get_logger(), "控制权已丢失，停止移动并重试申请");
      auth_ok_.store(false);
      move_mode_ok_.store(false);
    }
    uint16_t res = sdk_.requestControl();   // AstrallAuthControl(ASTRALL_AUTH_SDK)
    if (res == ASTRALL_RES_SUCCESSED) {
      RCLCPP_INFO(this->get_logger(), "获得控制权 (SDK)");
      auth_ok_.store(true);
      // 控制前须切换运动模式到 MOVE
      uint16_t mres = sdk_.switchToMoveMode();   // AstrallSportModeControl(MOVE)
      if (mres == ASTRALL_RES_SUCCESSED) {
        move_mode_ok_.store(true);
        RCLCPP_INFO(this->get_logger(), "运动模式已切换至 MOVE，允许移动");
      } else {
        move_mode_ok_.store(false);
        RCLCPP_WARN(this->get_logger(), "切换 MOVE 模式失败 res=0x%04X（下轮重试）",
          static_cast<unsigned int>(mres));
      }
    } else {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "AstrallAuthControl 申请控制权失败 res=0x%04X", static_cast<unsigned int>(res));
    }
  }
}

int Sw01AdapterNode::openCloudSocket()
{
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) { return -1; }
  int reuse = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(lidar_port_);
  if (bind(s, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0) {
    close(s);
    return -1;
  }
  // 收包超时 100ms，便于退出线程
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  return s;
}

void Sw01AdapterNode::cloudRecvLoop()
{
  std::vector<uint8_t> buf(65536);
  while (cloud_run_.load() && rclcpp::ok()) {
    sockaddr_in src{};
    socklen_t slen = sizeof(src);
    ssize_t n = recvfrom(cloud_sock_, buf.data(), buf.size(), 0,
                         reinterpret_cast<sockaddr *>(&src), &slen);
    if (n <= 0) { continue; }

    // P0 重构：长度驱动纯函数解析。只读取 [0, n) 字节——
    // 短包绝不触碰缓冲中的陈旧残留字节；长度与所选布局精确匹配才接受。
    CloudPacketView pkt;
    const ParseResult pr =
      parseCloudPacket(buf.data(), static_cast<size_t>(n), cloud_layout_, pkt);
    if (pr != ParseResult::kOk) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "点云包被拒 (parse=%d, n=%zd, layout=%s)",
        static_cast<int>(pr), static_cast<ssize_t>(n),
        cloud_layout_ == CloudPacketLayout::kFixed ? "fixed" : "dynamic");
      continue;
    }
    if (pkt.pos_num == 0) { continue; }   // 空包：无可组装点

    {
      std::lock_guard<std::mutex> lk(cloud_mutex_);
      if (pkt.total == 0) { continue; }
      if (pkt.total > kCloudMaxPoints) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "点云帧 total=%u 超上限 %u，丢弃", pkt.total, kCloudMaxPoints);
        continue;
      }
      // 帧边界检测：total 变化，或当前帧已见索引回退（新帧第一包 index=0）
      // 注：假设机器人顺序推送整帧（单帧内 UDP 一般保序）；乱序极端情况 P4 实测后
      //     可升级为按 timestampNs 分组/双缓冲。
      const bool new_frame =
        (pkt.total != cloud_total_) ||
        (cloud_total_ != 0 && pkt.index < cloud_last_index_);
      if (new_frame) {
        cloud_total_ = pkt.total;
        cloud_buf_.assign(cloud_total_, CloudPoint{});
        cloud_filled_.assign(cloud_total_, 0);
        cloud_filled_count_ = 0;
        cloud_last_index_ = 0;
      }
      // 越界防护（纯函数）：pts 必须落在 [0, total] 内
      if (!packetFitsFrame(cloud_total_, pkt.index, pkt.pos_num)) {
        RCLCPP_DEBUG(this->get_logger(),
          "点云包 index=%u pts=%u 越界 total=%u，丢弃本包",
          pkt.index, pkt.pos_num, cloud_total_);
        continue;
      }
      for (uint32_t i = 0; i < pkt.pos_num; ++i) {
        const uint32_t idx = pkt.index + i;
        CloudPointRaw raw;
        readCloudPoint(pkt.pos, i, raw);
        if (!cloud_filled_[idx]) {
          cloud_filled_[idx] = 1;
          ++cloud_filled_count_;
        }
        // int32 坐标 × point_scale（默认 0.001，单位待实测）
        CloudPoint & cp = cloud_buf_[idx];
        cp.x = static_cast<float>(raw.x) * static_cast<float>(point_scale_);
        cp.y = static_cast<float>(raw.y) * static_cast<float>(point_scale_);
        cp.z = static_cast<float>(raw.z) * static_cast<float>(point_scale_);
        // intensity = (r+g+b)/3，逐通道转 float 防 uint32 溢出
        cp.intensity = (static_cast<float>(raw.r) +
                        static_cast<float>(raw.g) +
                        static_cast<float>(raw.b)) / 3.0f;
      }
      cloud_last_index_ = std::max(cloud_last_index_, pkt.index + pkt.pos_num);
      cloud_ts_ns_ = pkt.timestamp_ns;

      // 整帧点收齐 → 发布并重置
      if (cloud_filled_count_ >= cloud_total_) {
        publishPointCloud();
        cloud_total_ = 0;
        cloud_buf_.clear();
        cloud_filled_.clear();
        cloud_filled_count_ = 0;
        cloud_last_index_ = 0;
      }
    }
  }
}

void Sw01AdapterNode::publishPointCloud()
{
  auto msg = sensor_msgs::msg::PointCloud2();
  msg.header.frame_id = lidar_frame_;

  // timestampNs 疑似纳秒 → sec / nanosec；为 0 时退回主机时钟
  if (cloud_ts_ns_ > 0) {
    msg.header.stamp.sec = static_cast<int32_t>(cloud_ts_ns_ / 1000000000ULL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(cloud_ts_ns_ % 1000000000ULL);
  } else {
    msg.header.stamp = this->now();
  }

  msg.height = 1;
  msg.width = cloud_total_;

  // 字段契约：仅 x,y,z,intensity（协议不提供 ring/time，不生成伪数据）。
  // FAST-LIO lidar_type=2 兼容路径见文件头注释与 README。
  msg.fields.resize(4);
  msg.fields[0].name = "x";         msg.fields[0].offset = 0;
  msg.fields[1].name = "y";         msg.fields[1].offset = 4;
  msg.fields[2].name = "z";         msg.fields[2].offset = 8;
  msg.fields[3].name = "intensity"; msg.fields[3].offset = 12;
  for (auto & f : msg.fields) {
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
  }
  msg.is_bigendian = false;
  msg.point_step = sizeof(CloudPoint);         // 16
  msg.row_step = msg.point_step * msg.width;
  msg.is_dense = true;

  msg.data.resize(msg.row_step);
  std::memcpy(msg.data.data(), cloud_buf_.data(), msg.row_step);

  points_pub_->publish(msg);
}

}  // namespace adapter_sw01

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adapter_sw01::Sw01AdapterNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
