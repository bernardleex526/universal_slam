// RoboSense schema normalization; never infer frame geometry from a name.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdexcept>
#include <vector>

class TopicBridgeNode : public rclcpp::Node {
  struct Point {
    float x, y, z, intensity;
    uint16_t ring;
    double t;
  };

public:
  TopicBridgeNode() : Node("m20_topic_bridge") {
    source_frame_ =
        declare_parameter<std::string>("source_cloud_frame", "lidar_link");
    imu_source_frame_ =
        declare_parameter<std::string>("source_imu_frame", "imu");
    allow_empty_imu_ = declare_parameter<bool>("allow_empty_imu_frame", false);
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
    format_ =
        declare_parameter<std::string>("cloud_format", "robosense_absolute");
    if (format_ != "robosense_absolute" && format_ != "velodyne_relative")
      throw std::invalid_argument("unsupported cloud_format");
    scan_lines_ = declare_parameter<int>("scan_line", 192);
    max_scan_ = declare_parameter<double>("max_scan_duration_s", .25);
    if (scan_lines_ < 1 || scan_lines_ > 65536 || !std::isfinite(max_scan_) ||
        max_scan_ <= 0)
      throw std::invalid_argument("invalid scan limits");
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("output_cloud_topic", "/points_raw"),
        rclcpp::QoS(5).reliable());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
        declare_parameter<std::string>("output_imu_topic", "/imu_raw"),
        rclcpp::QoS(200).reliable());
    auto cloud_qos = rclcpp::SensorDataQoS();
    if (declare_parameter<bool>("source_cloud_reliable", false))
      cloud_qos.reliable();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        declare_parameter<std::string>("source_cloud_topic", "/LIDAR/POINTS"),
        cloud_qos,
        [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { cloud(*m); });
    auto imu_qos = rclcpp::QoS(200).best_effort();
    if (declare_parameter<bool>("source_imu_reliable", false))
      imu_qos.reliable();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        declare_parameter<std::string>("source_imu_topic", "/IMU"), imu_qos,
        [this](sensor_msgs::msg::Imu::SharedPtr m) { imu(*m); });
  }

private:
  uint32_t field(const sensor_msgs::msg::PointCloud2 &m, const char *name,
                 uint8_t type, size_t bytes) {
    auto f = std::find_if(m.fields.begin(), m.fields.end(),
                          [name](const auto &x) { return x.name == name; });
    if (f == m.fields.end() || f->datatype != type || f->count != 1 ||
        f->offset > m.point_step || bytes > m.point_step - f->offset)
      throw std::invalid_argument(std::string("invalid field: ") + name);
    return f->offset;
  }
  template <class T> T read(const uint8_t *p, uint32_t offset) {
    T v;
    std::memcpy(&v, p + offset, sizeof(v));
    return v;
  }
  void cloud(const sensor_msgs::msg::PointCloud2 &m) {
    try {
      if (m.header.frame_id != source_frame_)
        throw std::invalid_argument(
            "unexpected source cloud frame; refusing relabel");
      if (m.is_bigendian || m.width == 0 || m.height == 0 ||
          m.point_step == 0 || uint64_t(m.width) * m.point_step > m.row_step ||
          uint64_t(m.row_step) * m.height != m.data.size())
        throw std::invalid_argument("invalid cloud layout/endianness");
      const auto ox = field(m, "x", 7, 4), oy = field(m, "y", 7, 4),
                 oz = field(m, "z", 7, 4), oi = field(m, "intensity", 7, 4),
                 oring = field(m, "ring", 4, 2);
      const bool absolute = format_ == "robosense_absolute";
      const auto ot = field(m, absolute ? "timestamp" : "time",
                            absolute ? 8 : 7, absolute ? 8 : 4);
      std::vector<Point> points;
      points.reserve(size_t(m.width) * m.height);
      double first = std::numeric_limits<double>::infinity(), last = -first;
      const double header = rclcpp::Time(m.header.stamp).seconds();
      for (uint32_t row = 0; row < m.height; ++row)
        for (uint32_t col = 0; col < m.width; ++col) {
          const auto p = m.data.data() + size_t(row) * m.row_step +
                         size_t(col) * m.point_step;
          Point q{read<float>(p, ox),
                  read<float>(p, oy),
                  read<float>(p, oz),
                  read<float>(p, oi),
                  read<uint16_t>(p, oring),
                  absolute ? read<double>(p, ot) : header + read<float>(p, ot)};
          if (!std::isfinite(q.t) || q.t < 0 || q.ring >= scan_lines_)
            throw std::invalid_argument("invalid point timestamp/ring");
          if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
              !std::isfinite(q.z) || !std::isfinite(q.intensity))
            continue;
          first = std::min(first, q.t);
          last = std::max(last, q.t);
          points.push_back(q);
        }
      if (points.empty() || last - first > max_scan_ || first <= last_cloud_)
        throw std::invalid_argument("empty, oversized or non-monotonic scan");
      std::stable_sort(
          points.begin(), points.end(),
          [](const Point &a, const Point &b) { return a.t < b.t; });
      sensor_msgs::msg::PointCloud2 out;
      out.header.frame_id = lidar_frame_;
      out.header.stamp =
          rclcpp::Time(static_cast<int64_t>(std::llround(first * 1e9)));
      out.height = 1;
      out.width = points.size();
      out.point_step = 22;
      out.row_step = out.width * out.point_step;
      out.is_dense = true;
      for (const auto &spec :
           std::vector<std::pair<std::string, uint32_t>>{{"x", 0},
                                                         {"y", 4},
                                                         {"z", 8},
                                                         {"intensity", 12},
                                                         {"time", 16},
                                                         {"ring", 20}}) {
        sensor_msgs::msg::PointField f;
        f.name = spec.first;
        f.offset = spec.second;
        f.count = 1;
        f.datatype = f.name == "ring" ? 4 : 7;
        out.fields.push_back(f);
      }
      out.data.resize(out.row_step);
      for (size_t i = 0; i < points.size(); ++i) {
        auto p = out.data.data() + i * 22;
        const auto &q = points[i];
        const float time = static_cast<float>(q.t - first);
        std::memcpy(p, &q.x, 4);
        std::memcpy(p + 4, &q.y, 4);
        std::memcpy(p + 8, &q.z, 4);
        std::memcpy(p + 12, &q.intensity, 4);
        std::memcpy(p + 16, &time, 4);
        std::memcpy(p + 20, &q.ring, 2);
      }
      last_cloud_ = first;
      cloud_pub_->publish(out);
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Rejected cloud: %s", e.what());
    }
  }
  void imu(const sensor_msgs::msg::Imu &m) {
    const std::array<double, 6> values{
        m.angular_velocity.x,    m.angular_velocity.y,
        m.angular_velocity.z,    m.linear_acceleration.x,
        m.linear_acceleration.y, m.linear_acceleration.z};
    const double stamp = rclcpp::Time(m.header.stamp).seconds();
    if ((m.header.frame_id != imu_source_frame_ &&
         !(allow_empty_imu_ && m.header.frame_id.empty())) ||
        stamp <= last_imu_ ||
        !std::all_of(values.begin(), values.end(),
                     [](double v) { return std::isfinite(v); })) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected IMU: frame, timestamp or finite-data check failed");
      return;
    }
    auto out = m;
    out.header.frame_id = imu_frame_;
    const auto &q = out.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm) || std::abs(norm - 1.) > .01)
      out.orientation_covariance[0] = -1.;
    last_imu_ = stamp;
    imu_pub_->publish(out);
  }
  std::string source_frame_, imu_source_frame_, lidar_frame_, imu_frame_,
      format_;
  bool allow_empty_imu_;
  int scan_lines_;
  double max_scan_, last_cloud_{-1.}, last_imu_{-1.};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicBridgeNode>());
  rclcpp::shutdown();
}
