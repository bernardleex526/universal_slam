// ============================================================
// adapter_m20 / src/m20_topic_bridge.cpp
// M20 话题桥（完整实现, P2）
// ------------------------------------------------------------
// 功能：将机载 Foxy 话题桥接/整形为约定话题（ADAPTER_INTERFACE.md）：
//   /IMU           → /imu_raw      (sensor_msgs/Imu, imu 帧,   reliable)
//   /LIDAR/POINTS  → /points_raw   (sensor_msgs/PointCloud2, lidar 帧, best_effort)
//
// 部署场景：
//   - 同网段直连 DDS（同 DOMAIN_ID + 同 RMW）时话题级互通，本节点可省略
//     （直接由 launch/zenoh-bridge 完成桥接）。
//   - 本节点用于需要修正 frame_id / 跨网段转发 / WiFi 降采样兜底时（P4 启用）。
//
// 带宽提示：96 线点云 ≈ 240–720 Mbps，必须有线千兆；WiFi 需降采样（见 TODO）。
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace adapter_m20 {

/// 参数化中继节点：源/目标话题可配，QoS 按约定。
class TopicBridgeNode : public rclcpp::Node
{
public:
  explicit TopicBridgeNode(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  std::string src_cloud_topic_;
  std::string src_imu_topic_;
  std::string out_cloud_topic_;
  std::string out_imu_topic_;
  std::string lidar_frame_;
  std::string imu_frame_;
  // TODO(P4): voxel_downsample (bool, 默认 false) —— WiFi 场景用 pcl::VoxelGrid
  // 或隔帧发布降带宽；有线千兆直连无需开启。

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

TopicBridgeNode::TopicBridgeNode(const rclcpp::NodeOptions & options)
: Node("m20_topic_bridge", options)
{
  // 源话题（机载 Foxy 侧默认命名；与 deep-robotics-msg 文档一致）
  src_cloud_topic_ = declare_parameter<std::string>("source_cloud_topic", "/LIDAR/POINTS");
  src_imu_topic_ = declare_parameter<std::string>("source_imu_topic", "/IMU");
  // 目标话题（约定，勿改）
  out_cloud_topic_ = declare_parameter<std::string>("output_cloud_topic", "/points_raw");
  out_imu_topic_ = declare_parameter<std::string>("output_imu_topic", "/imu_raw");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");

  // 点云: best_effort（带宽优先，丢帧可容忍）；IMU: reliable
  points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    src_cloud_topic_, rclcpp::QoS(5).best_effort(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onPointCloud(msg); });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    src_imu_topic_, rclcpp::QoS(10).reliable(),
    [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(msg); });

  points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    out_cloud_topic_, rclcpp::QoS(5).best_effort());
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    out_imu_topic_, rclcpp::QoS(10).reliable());

  RCLCPP_INFO(this->get_logger(),
    "话题桥: %s→%s, %s→%s (frames: %s, %s)",
    src_cloud_topic_.c_str(), out_cloud_topic_.c_str(),
    src_imu_topic_.c_str(), out_imu_topic_.c_str(),
    lidar_frame_.c_str(), imu_frame_.c_str());
}

void TopicBridgeNode::onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 统一 frame_id 为约定 lidar 帧（拷贝后修改，避免影响原消息）
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
  out->header.frame_id = lidar_frame_;
  points_pub_->publish(*out);
}

void TopicBridgeNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // 统一 frame_id 为约定 imu 帧（拷贝后修改，避免影响原消息）
  auto out = std::make_shared<sensor_msgs::msg::Imu>(*msg);
  out->header.frame_id = imu_frame_;
  imu_pub_->publish(*out);
}

}  // namespace adapter_m20

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adapter_m20::TopicBridgeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
