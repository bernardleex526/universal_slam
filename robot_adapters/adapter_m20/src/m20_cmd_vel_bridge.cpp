// Explicitly armed UDP-axis backend. No automatic standing, mode changes or
// takeover.
#include "adapter_m20/cmd_math.hpp"
#include "adapter_m20/m20_protocol.hpp"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sys/socket.h>
#include <unistd.h>

namespace adapter_m20 {
using Clock = std::chrono::steady_clock;
class CmdVelBridgeNode : public rclcpp::Node {
public:
  CmdVelBridgeNode() : Node("m20_cmd_vel_bridge") {
    enabled_ = declare_parameter<bool>("enable_control", false);
    const bool calibrated =
        declare_parameter<bool>("calibration_confirmed", false);
    expected_gait_ = declare_parameter<int>("expected_gait", 0);
    if (enabled_ && expected_gait_ <= 0)
      throw std::invalid_argument("control requires calibrated expected_gait");
    const auto ip = declare_parameter<std::string>("robot_ip", "10.21.31.103");
    const auto port =
        requirePortRange(declare_parameter<int>("port", 30000), "port");
    const double hz =
        requireFrequencyHz(declare_parameter<double>("cmd_frequency_hz", 20.),
                           "cmd_frequency_hz", 100.);
    const double heartbeat_hz = requireFrequencyHz(
        declare_parameter<double>("heartbeat_hz", 1.), "heartbeat_hz", 10.);
    cmd_timeout_ = requireFinitePositive(
        declare_parameter<double>("cmd_timeout_s", .25), "cmd_timeout_s");
    status_timeout_ = requireFinitePositive(
        declare_parameter<double>("status_timeout_s", 1.5), "status_timeout_s");
    odom_timeout_ = requireFinitePositive(
        declare_parameter<double>("odom_timeout_s", .3), "odom_timeout_s");
    limits_ = {declare_parameter<double>("vx_max", .2),
               declare_parameter<double>("vy_max", .2),
               declare_parameter<double>("wz_max", .3)};
    scales_ = {declare_parameter<double>("vx_full_scale", 0.),
               declare_parameter<double>("vy_full_scale", 0.),
               declare_parameter<double>("wz_full_scale", 0.)};
    for (size_t i = 0; i < 3; ++i) {
      requireFinitePositive(limits_[i], "velocity limit");
      if (enabled_) {
        requireFinitePositive(scales_[i], "measured full scale");
        if (limits_[i] > scales_[i])
          throw std::invalid_argument("safe limit exceeds measured full scale");
      }
    }
    if (enabled_ && !calibrated)
      throw std::invalid_argument(
          "control requires verified full-scale calibration");
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &dst_.sin_addr) != 1)
      throw std::invalid_argument("invalid robot_ip");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    status_pub_ =
        create_publisher<std_msgs::msg::String>("m20_bridge_status", 1);
    arm_ = create_service<std_srvs::srv::SetBool>(
        "~/arm",
        [this](std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          if (!request->data) {
            disarm("operator disarmed");
            response->success = true;
          } else if (!enabled_ || !healthy()) {
            response->success = false;
          } else {
            armed_ = true;
            stopping_ = false;
            have_cmd_ = false;
            armed_time_ = Clock::now();
            response->success = true;
          }
          response->message =
              armed_ ? "Armed; waiting for a NEW command"
                     : "Disarmed; require enabled/calibrated, fresh "
                       "Regular/RL/HES/Direction/Charge status and odometry";
        });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel"),
        rclcpp::QoS(1), [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          const std::array<double, 3> v{msg->linear.x, msg->linear.y,
                                        msg->angular.z};
          if (!std::all_of(v.begin(), v.end(),
                           [](double x) { return std::isfinite(x); })) {
            disarm("non-finite command");
            return;
          }
          if (!armed_)
            return;
          cmd_ = v;
          last_cmd_ = Clock::now();
          have_cmd_ = true;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("odom_topic", "/odom"), rclcpp::QoS(5),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          const double age =
              (now() - rclcpp::Time(msg->header.stamp)).seconds();
          const auto &q = msg->pose.pose.orientation;
          const auto &p = msg->pose.pose.position;
          const auto &v = msg->twist.twist.linear;
          const auto &w = msg->twist.twist.angular;
          const std::array<double, 13> values{q.x, q.y, q.z, q.w, p.x, p.y, p.z,
                                              v.x, v.y, v.z, w.x, w.y, w.z};
          const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          if (msg->header.frame_id != odom_frame_ ||
              msg->child_frame_id != base_frame_ || age < -.1 ||
              age > odom_timeout_ || std::abs(norm - 1.) > .01 ||
              !std::all_of(values.begin(), values.end(),
                           [](double x) { return std::isfinite(x); })) {
            have_odom_ = false;
            disarm("invalid odometry");
            return;
          }
          last_odom_ = Clock::now();
          have_odom_ = true;
        });
    if (enabled_) {
      sock_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
      if (sock_ < 0)
        throw std::runtime_error("UDP socket creation failed");
      heartbeat_ = create_wall_timer(
          std::chrono::duration<double>(1. / heartbeat_hz),
          [this] { send(100, 100, nlohmann::json::object()); });
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / hz),
                               [this] { tick(); });
    RCLCPP_WARN(get_logger(),
                "Control %s. No automatic mode/stand/gait requests. Explicit "
                "~/arm required.",
                enabled_ ? "enabled but DISARMED" : "DISABLED");
  }
  ~CmdVelBridgeNode() override {
    disarm("shutdown");
    if (sock_ >= 0)
      close(sock_);
  }

private:
  double age(Clock::time_point t) const {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }
  bool healthy() const {
    return have_status_ && regular_rl_ && have_odom_ &&
           age(last_status_) < status_timeout_ &&
           age(last_odom_) < odom_timeout_;
  }
  bool send(uint16_t type, uint16_t command, const nlohmann::json &items) {
    if (!enabled_ || sock_ < 0)
      return false;
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm);
    const auto body = nlohmann::json{{"PatrolDevice",
                                      {{"Type", type},
                                       {"Command", command},
                                       {"Time", stamp},
                                       {"Items", items}}}}
                          .dump();
    std::vector<uint8_t> packet(16, 0);
    packet[0] = 0xeb;
    packet[1] = 0x91;
    packet[2] = 0xeb;
    packet[3] = 0x90;
    packet[4] = body.size() & 255;
    packet[5] = (body.size() >> 8) & 255;
    packet[6] = id_ & 255;
    packet[7] = (id_++ >> 8) & 255;
    packet[8] = 1;
    packet.insert(packet.end(), body.begin(), body.end());
    return sendto(sock_, packet.data(), packet.size(), 0,
                  reinterpret_cast<sockaddr *>(&dst_),
                  sizeof(dst_)) == static_cast<ssize_t>(packet.size());
  }
  void zero() {
    send(2, 21,
         {{"X", 0.},
          {"Y", 0.},
          {"Z", 0.},
          {"Roll", 0.},
          {"Pitch", 0.},
          {"Yaw", 0.}});
  }
  void disarm(const char *reason) {
    if (armed_) {
      stopping_ = true;
      zero();
      zero();
      zero();
      RCLCPP_WARN(get_logger(), "Disarmed, maintaining zero: %s", reason);
    }
    armed_ = false;
    have_cmd_ = false;
  }
  void receive() {
    if (sock_ < 0)
      return;
    std::array<uint8_t, 65536> bytes{};
    for (int i = 0; i < 32; ++i) {
      sockaddr_in src{};
      socklen_t len = sizeof(src);
      const auto n = recvfrom(sock_, bytes.data(), bytes.size(), 0,
                              reinterpret_cast<sockaddr *>(&src), &len);
      if (n <= 0)
        break;
      if (!validatePeer(src.sin_addr.s_addr, src.sin_port, dst_.sin_addr.s_addr,
                        dst_.sin_port))
        continue;
      uint16_t type = 0, command = 0;
      nlohmann::json body;
      if (parseReportFrame(bytes.data(), n, type, command, body) !=
          ReportParseResult::kOk)
        continue;
      if (type == 1002 && command == 6) {
        StatusReport status;
        int64_t hes = -1, direction = -1, charge = -1, gait = -1;
        if (!parseStatusReport(body, status)) {
          have_status_ = false;
          continue;
        }
        findInt(body, "HES", hes);
        findInt(body, "Direction", direction);
        findInt(body, "Charge", charge);
        findInt(body, "Gait", gait);
        regular_rl_ = status.control_usage_mode == 0 &&
                      status.motion_state == 17 && hes == 0 && direction == 0 &&
                      charge == 0 && gait == expected_gait_;
        have_status_ = true;
        last_status_ = Clock::now();
        if (status.control_usage_mode != 0) {
          disarm("explicit mode handoff");
          stopping_ =
              false; // Axis control is not authoritative in another mode.
        }
      } else if (type == 1002 && command == 3) {
        uint32_t error = 0;
        if (!parseErrorReport(body, error) || error != 0) {
          regular_rl_ = false;
          disarm("robot error report");
        }
      } else if ((type == 2 && command == 21) ||
                 (type == 100 && command == 100)) {
        const auto &items = body["PatrolDevice"]["Items"];
        const auto error = items.find("ErrorCode");
        if (error != items.end() && error->is_number_integer() && *error != 0) {
          regular_rl_ = false;
          disarm("command rejected");
        }
      }
    }
  }
  void tick() {
    receive();
    if (armed_) {
      if (!healthy())
        disarm("status/odometry unhealthy or stale");
      else if ((have_cmd_ && age(last_cmd_) >= cmd_timeout_) ||
               (!have_cmd_ && age(armed_time_) >= cmd_timeout_))
        disarm("command timeout");
      else if (!have_cmd_)
        zero();
      else {
        // SI safety limit and hardware full scale are different quantities. No
        // rounding up.
        std::array<double, 3> axis{};
        for (size_t i = 0; i < 3; ++i)
          axis[i] = std::clamp(cmd_[i], -limits_[i], limits_[i]) / scales_[i];
        if (!send(2, 21,
                  {{"X", axis[0]},
                   {"Y", axis[1]},
                   {"Z", 0.},
                   {"Roll", 0.},
                   {"Pitch", 0.},
                   {"Yaw", axis[2]}}))
          disarm("send failure");
      }
    }
    if (!armed_ && stopping_)
      zero();
    std_msgs::msg::String status;
    status.data = !enabled_   ? "DISABLED"
                  : armed_    ? "ARMED"
                  : stopping_ ? "STOPPING"
                              : "DISARMED";
    status_pub_->publish(status);
  }
  int sock_{-1};
  sockaddr_in dst_{};
  uint16_t id_{0};
  bool enabled_{false}, stopping_{false}, armed_{false}, have_cmd_{false},
      have_odom_{false}, have_status_{false}, regular_rl_{false};
  double cmd_timeout_, status_timeout_, odom_timeout_;
  int expected_gait_{0};
  std::string odom_frame_, base_frame_;
  std::array<double, 3> cmd_{}, limits_{}, scales_{};
  Clock::time_point last_cmd_{}, last_status_{}, last_odom_{}, armed_time_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_;
  rclcpp::TimerBase::SharedPtr timer_, heartbeat_;
};
} // namespace adapter_m20
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adapter_m20::CmdVelBridgeNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
