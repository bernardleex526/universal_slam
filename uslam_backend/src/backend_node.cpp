// universal_slam back end: loop closure (mapping) and relocalization (localization).
//
// Frames: map (this node), odom (gravity-aligned, odometry adapter), base, imu (FAST-LIO "body").
// Inputs: odom_topic (T_odom_base) and cloud_topic (undistorted scan in the IMU/"body" frame),
// matched by identical stamps (both carry the LIO scan-end time).
// Output: the only map -> odom TF, stamped like the odometry it was computed from:
//   mapping:      T_map_odom = X_k* * T_odom_k^-1  (X_k* optimized pose of the newest keyframe)
//   localization: T_map_odom = T_map_base(scan-to-map GICP) * T_odom_base^-1
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "uslam_backend/map_io.hpp"
#include "uslam_backend/pose_graph.hpp"
#include "uslam_backend/registration.hpp"
#include "uslam_backend/scan_context.hpp"

namespace uslam_backend
{
namespace
{

double yawOf(const Eigen::Matrix3d & R) {return std::atan2(R(1, 0), R(0, 0));}

// Roll/pitch part of R: R = Rz(yaw) * levelling(R).
Eigen::Matrix3d levelling(const Eigen::Matrix3d & R)
{
  return Eigen::AngleAxisd(-yawOf(R), Eigen::Vector3d::UnitZ()).toRotationMatrix() * R;
}

double angleOf(const Eigen::Isometry3d & T) {return Eigen::AngleAxisd(T.rotation()).angle();}

Eigen::Isometry3d fromPose(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d T;
  tf2::fromMsg(p, T);
  return T;
}

}  // namespace

class BackendNode : public rclcpp::Node
{
  struct Frame
  {
    rclcpp::Time stamp;
    Eigen::Isometry3d T_odom_base;
    Cloud::Ptr cloud_base;
  };
  struct Keyframe
  {
    double stamp;
    Eigen::Isometry3d T_odom_base;
    Cloud::Ptr cloud;  // base frame, keyframe_voxel
    Cloud::Ptr reg;    // base frame, registration_voxel
    Eigen::MatrixXd sc;
    Eigen::VectorXd key;
  };
  enum class State {Initializing, Localized, Lost};

public:
  BackendNode()
  : Node("uslam_backend"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    mode_ = declare_parameter<std::string>("mode", "mapping");
    if (mode_ != "mapping" && mode_ != "localization") {
      throw std::invalid_argument("mode must be mapping or localization");
    }
    map_dir_ = declare_parameter<std::string>("map_dir", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu");
    kf_dist_ = declare_parameter<double>("keyframe_distance", 1.0);
    kf_angle_ = declare_parameter<double>("keyframe_angle_deg", 15.0) * M_PI / 180.0;
    kf_voxel_ = declare_parameter<double>("keyframe_voxel", 0.1);
    reg_voxel_ = declare_parameter<double>("registration_voxel", 0.25);
    map_voxel_ = declare_parameter<double>("map_voxel", 0.1);
    // 可视化用全局地图：低频、粗体素，避免大场景下每次重拼全部关键帧
    publish_period_ = declare_parameter<double>("map_publish_period_s", 10.0);
    publish_voxel_ = declare_parameter<double>("map_publish_voxel", 0.2);
    max_range_ = declare_parameter<double>("max_range", 40.0);
    loop_radius_ = declare_parameter<double>("loop_radius", 8.0);
    loop_time_gap_ = declare_parameter<double>("loop_min_time_gap_s", 20.0);
    loop_index_gap_ = declare_parameter<int>("loop_min_index_gap", 10);
    submap_half_ = declare_parameter<int>("loop_submap_half", 8);
    loop_cooldown_ = declare_parameter<int>("loop_cooldown_keyframes", 3);
    sc_threshold_ = declare_parameter<double>("scan_context_threshold", 0.25);
    odom_sigmas_ = {declare_parameter<double>("odom_sigma_rot", 0.01), declare_parameter<double>("odom_sigma_trans", 0.05)};
    loop_sigmas_ = {declare_parameter<double>("loop_sigma_rot", 0.02), declare_parameter<double>("loop_sigma_trans", 0.05)};
    cauchy_k_ = declare_parameter<double>("loop_cauchy_k", 10.0);
    drift_ratio_ = declare_parameter<double>("loop_max_drift_ratio", 0.05);
    drift_base_ = declare_parameter<double>("loop_max_drift_base_m", 0.5);
    loop_align_ = alignParams("loop", 1.5, 0.5, 0.55, 0.15);
    reloc_align_ = alignParams("reloc", 3.0, 0.6, 0.85, 0.15);
    track_align_ = alignParams("track", 0.8, 0.4, 0.55, 0.15);
    loc_period_ = declare_parameter<double>("localization_period_s", 0.5);
    loc_frames_ = declare_parameter<int>("localization_accumulate_frames", 5);
    reloc_candidates_ = declare_parameter<int>("relocalization_candidates", 15);
    reloc_spacing_ = declare_parameter<double>("relocalization_candidate_spacing_m", 1.0);
    max_jump_m_ = declare_parameter<double>("tracking_max_jump_m", 1.0);
    max_jump_rad_ = declare_parameter<double>("tracking_max_jump_deg", 20.0) * M_PI / 180.0;
    lost_timeout_ = declare_parameter<double>("lost_timeout_s", 5.0);
    try_origin_ = declare_parameter<bool>("try_map_origin_first", true);
    amb_dist_ = declare_parameter<double>("reloc_ambiguity_distance_m", 1.0);
    amb_angle_ = declare_parameter<double>("reloc_ambiguity_angle_deg", 20.0) * M_PI / 180.0;
    amb_margin_ = declare_parameter<double>("reloc_ambiguity_margin", 0.05);
    ScanContextParams scp;
    scp.max_radius = declare_parameter<double>("scan_context_max_radius", 20.0);
    sc_ = ScanContext(scp);
    for (double v : {kf_dist_, kf_angle_, kf_voxel_, reg_voxel_, map_voxel_, max_range_, loop_radius_, loc_period_,
        lost_timeout_, publish_period_, publish_voxel_})
    {
      if (!(v > 0.0) || !std::isfinite(v)) {
        throw std::invalid_argument("backend parameters must be finite and positive");
      }
    }

    io_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slow_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions io;
    io.callback_group = io_group_;
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(declare_parameter<std::string>("map_topic", "/uslam/backend/map"), latched);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(declare_parameter<std::string>("path_topic", "/uslam/backend/path"), latched);
    status_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter<std::string>("status_topic", "/uslam/backend/status"), latched);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(declare_parameter<std::string>("odom_topic", "/uslam/odom"),
      rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::SharedPtr m) {onOdom(m);}, io);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("cloud_topic", "/uslam/lio/cloud_registered_body"), rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {onCloud(m);}, io);

    if (mode_ == "mapping") {
      save_srv_ = create_service<std_srvs::srv::Trigger>("~/save_map",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {save(*res);},
        rmw_qos_profile_services_default, slow_group_);
      map_timer_ = create_wall_timer(std::chrono::duration<double>(publish_period_), [this] {publishMap();}, slow_group_);
      stats_timer_ = create_wall_timer(std::chrono::seconds(30), [this] {logStats();}, slow_group_);
      setStatus("MAPPING");
    } else {
      loadPriorMap();
      initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        declare_parameter<std::string>("initialpose_topic", "/initialpose"), rclcpp::QoS(1),
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {onInitialPose(*m);}, io);
      setStatus("INITIALIZING");
    }
    worker_ = std::thread([this] {run();});
    RCLCPP_INFO(get_logger(), "uslam_backend started in %s mode", mode_.c_str());
  }

  ~BackendNode() override
  {
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      stop_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  AlignParams alignParams(const std::string & prefix, double coarse, double fine, double ratio, double rmse)
  {
    AlignParams p;
    p.coarse_max_correspondence = declare_parameter<double>(prefix + "_coarse_correspondence", coarse);
    p.fine_max_correspondence = declare_parameter<double>(prefix + "_fine_correspondence", fine);
    p.min_inlier_ratio = declare_parameter<double>(prefix + "_min_inlier_ratio", ratio);
    p.max_rmse = declare_parameter<double>(prefix + "_max_rmse", rmse);
    return p;
  }

  void setStatus(const std::string & s)
  {
    if (s == status_) {
      return;
    }
    status_ = s;
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "status: %s", s.c_str());
  }

  // ---------------------------------------------------------------- input / sync
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr & m)
  {
    if (m->header.frame_id != odom_frame_ || m->child_frame_id != base_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "odometry frames do not match odom/base");
      return;
    }
    const Eigen::Isometry3d T = fromPose(m->pose.pose);
    publishMapOdom(m->header.stamp);
    const int64_t ns = rclcpp::Time(m->header.stamp).nanoseconds();
    odom_buf_[ns] = T;
    while (odom_buf_.size() > 200) {odom_buf_.erase(odom_buf_.begin());}
    match();
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & m)
  {
    cloud_buf_[rclcpp::Time(m->header.stamp).nanoseconds()] = m;
    while (cloud_buf_.size() > 20) {cloud_buf_.erase(cloud_buf_.begin());}
    match();
  }

  void match()
  {
    if (!have_base_imu_) {
      try {
        const auto t = tf_buffer_.lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
        T_base_imu_ = tf2::transformToEigen(t);
        have_base_imu_ = true;
      } catch (const tf2::TransformException &) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for TF %s <- %s", base_frame_.c_str(), imu_frame_.c_str());
        return;
      }
    }
    for (auto it = cloud_buf_.begin(); it != cloud_buf_.end(); ) {
      const auto odom = odom_buf_.find(it->first);
      if (odom == odom_buf_.end()) {
        ++it;
        continue;
      }
      // 点云在 FAST-LIO 的 IMU("body") 系：变换到 base 系并按量程裁剪
      Cloud raw;
      pcl::fromROSMsg(*it->second, raw);
      auto base = std::make_shared<Cloud>();
      base->reserve(raw.size());
      const Eigen::Matrix4f T = T_base_imu_.matrix().cast<float>();
      for (const auto & p : raw.points) {
        const Eigen::Vector4f q = T * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
        if (std::isfinite(q.x()) && q.head<3>().norm() < max_range_) {
          base->push_back(pcl::PointXYZ(q.x(), q.y(), q.z()));
        }
      }
      Frame f{rclcpp::Time(it->first, RCL_ROS_TIME), odom->second, base};
      {
        std::lock_guard<std::mutex> lock(queue_mtx_);
        frames_.push_back(f);
        if (frames_.size() > 10) {frames_.pop_front();}  // 后端跟不上时丢最旧帧，不阻塞里程计
      }
      queue_cv_.notify_one();
      it = cloud_buf_.erase(it);
    }
  }

  void publishMapOdom(const builtin_interfaces::msg::Time & stamp)
  {
    Eigen::Isometry3d T;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      if (!have_map_odom_) {return;}
      T = T_map_odom_;
    }
    geometry_msgs::msg::TransformStamped t = tf2::eigenToTransform(T);
    t.header.stamp = stamp;
    t.header.frame_id = map_frame_;
    t.child_frame_id = odom_frame_;
    tf_pub_->sendTransform(t);
  }

  void run()
  {
    while (true) {
      Frame f;
      {
        std::unique_lock<std::mutex> lock(queue_mtx_);
        queue_cv_.wait(lock, [this] {return stop_ || !frames_.empty();});
        if (stop_) {return;}
        f = frames_.front();
        frames_.pop_front();
      }
      try {
        if (mode_ == "mapping") {
          processMapping(f);
        } else {
          processLocalization(f);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "back end frame failed: %s", e.what());
      }
    }
  }

  Keyframe makeKeyframe(const Frame & f, const Eigen::Matrix3d & R_level_ref) const
  {
    Keyframe kf;
    kf.stamp = f.stamp.seconds();
    kf.T_odom_base = f.T_odom_base;
    kf.cloud = voxelize(*f.cloud_base, kf_voxel_);
    kf.reg = voxelize(*kf.cloud, reg_voxel_);
    describe(kf, R_level_ref);
    return kf;
  }

  // Scan Context on the cloud levelled with the roll/pitch of a gravity-aligned orientation.
  void describe(Keyframe & kf, const Eigen::Matrix3d & R_gravity_aligned) const
  {
    Eigen::Isometry3d L = Eigen::Isometry3d::Identity();
    L.linear() = levelling(R_gravity_aligned);
    kf.sc = sc_.describe(*transformed(*kf.reg, L));
    kf.key = ScanContext::ringKey(kf.sc);
  }

  // ---------------------------------------------------------------- mapping
  void processMapping(const Frame & f)
  {
    std::size_t k;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      if (kfs_.empty()) {
        kfs_.push_back(makeKeyframe(f, f.T_odom_base.rotation()));
        k = graph_.addFirst(f.T_odom_base, {1e-4, 1e-3});  // map == odom at the first keyframe
        T_map_odom_ = Eigen::Isometry3d::Identity();
        have_map_odom_ = true;
        map_dirty_ = true;
        return;
      }
      const Eigen::Isometry3d delta = kfs_.back().T_odom_base.inverse() * f.T_odom_base;
      if (delta.translation().norm() < kf_dist_ && angleOf(delta) < kf_angle_) {
        return;
      }
      kfs_.push_back(makeKeyframe(f, f.T_odom_base.rotation()));
      k = graph_.addOdometry(delta, odom_sigmas_);
    }
    const auto t0 = std::chrono::steady_clock::now();
    detectLoop(k);
    {
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      std::lock_guard<std::mutex> lock(stats_mtx_);
      loop_ms_sum_ += ms;
      loop_ms_max_ = std::max(loop_ms_max_, ms);
      ++loop_calls_;
    }
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      T_map_odom_ = graph_.pose(k) * kfs_[k].T_odom_base.inverse();
      map_dirty_ = true;
    }
  }

  void detectLoop(std::size_t k)
  {
    // 刚闭合过回环的区域，连续关键帧的回环大多冗余：间隔若干关键帧再尝试，节省 GICP 开销
    if (last_loop_k_ >= 0 && static_cast<long>(k) - last_loop_k_ < loop_cooldown_) {return;}
    std::vector<Eigen::Isometry3d> P;
    std::vector<double> stamps;
    std::vector<Eigen::Vector3d> odom_xyz;
    std::vector<Cloud::Ptr> regs;
    Eigen::MatrixXd sc_k;
    Eigen::VectorXd key_k;
    std::vector<Eigen::VectorXd> keys;
    std::vector<Eigen::MatrixXd> scs;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      P = graph_.poses();
      for (const auto & kf : kfs_) {
        stamps.push_back(kf.stamp);
        odom_xyz.push_back(kf.T_odom_base.translation());
        regs.push_back(kf.reg);
        keys.push_back(kf.key);
        scs.push_back(kf.sc);
      }
      sc_k = kfs_[k].sc;
      key_k = kfs_[k].key;
    }
    const auto eligible = [&](std::size_t j) {
        return j + static_cast<std::size_t>(loop_index_gap_) <= k && stamps[k] - stamps[j] >= loop_time_gap_;
      };
    // 1) 近邻候选（漂移小时最可靠）
    long best = -1;
    double best_d = loop_radius_;
    for (std::size_t j = 0; j < k; ++j) {
      const double d = (P[j].translation() - P[k].translation()).norm();
      if (eligible(j) && d < best_d) {
        best_d = d;
        best = static_cast<long>(j);
      }
    }
    Eigen::Isometry3d guess = P[k];
    std::string how = "proximity";
    if (best < 0) {
      // 2) Scan Context 兜底：环键预选 + 列移位距离，偏航由最佳移位给出
      std::vector<std::pair<double, std::size_t>> pre;
      for (std::size_t j = 0; j < k; ++j) {
        if (eligible(j)) {pre.push_back({(keys[j] - key_k).norm(), j});}
      }
      std::sort(pre.begin(), pre.end());
      double best_sc = sc_threshold_;
      int shift = 0;
      for (std::size_t i = 0; i < std::min<std::size_t>(pre.size(), 10); ++i) {
        const auto [d, s] = sc_.distance(sc_k, scs[pre[i].second]);
        if (d < best_sc) {
          best_sc = d;
          shift = s;
          best = static_cast<long>(pre[i].second);
        }
      }
      if (best < 0) {return;}
      const Eigen::Matrix3d R_k = P[k].rotation();
      guess.linear() = Eigen::AngleAxisd(yawOf(P[best].rotation()) + shift * sc_.sectorAngle(),
          Eigen::Vector3d::UnitZ()).toRotationMatrix() * levelling(R_k);
      guess.translation() = P[best].translation();
      how = "scan_context";
    }
    const std::size_t j = static_cast<std::size_t>(best);
    Cloud submap;
    const std::size_t lo = j > static_cast<std::size_t>(submap_half_) ? j - submap_half_ : 0;
    const std::size_t hi = std::min(j + submap_half_, k - static_cast<std::size_t>(loop_index_gap_) / 2);
    for (std::size_t i = lo; i <= hi; ++i) {
      submap += *transformed(*regs[i], P[i]);
    }
    Registration reg;
    try {
      reg.setTarget(voxelize(submap, reg_voxel_));
    } catch (const std::invalid_argument &) {
      return;
    }
    const AlignResult r = reg.align(*regs[k], guess, loop_align_);
    if (!r.accepted) {
      RCLCPP_DEBUG(get_logger(), "loop %zu-%zu (%s) rejected: inliers %.2f rmse %.3f", j, k, how.c_str(), r.inlier_ratio, r.rmse);
      return;
    }
    const Eigen::Isometry3d correction = P[k].inverse() * r.T_target_source;
    // 漂移合理性：修正量不应超过两帧之间轨迹长度允许的漂移，否则视为误匹配
    double travelled = 0.0;
    for (std::size_t i = j + 1; i <= k; ++i) {travelled += (odom_xyz[i] - odom_xyz[i - 1]).norm();}
    if (correction.translation().norm() > drift_ratio_ * travelled + drift_base_) {
      RCLCPP_WARN(get_logger(), "loop %zu-%zu (%s) rejected: correction %.2f m exceeds plausible drift over %.1f m",
        j, k, how.c_str(), correction.translation().norm(), travelled);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      graph_.addLoop(j, k, P[j].inverse() * r.T_target_source, loop_sigmas_, cauchy_k_);
      graph_.relax(3);
    }
    last_loop_k_ = static_cast<long>(k);
    RCLCPP_INFO(get_logger(), "loop %zu-%zu (%s): correction %.3f m %.2f deg, inliers %.2f rmse %.3f",
      j, k, how.c_str(), correction.translation().norm(), angleOf(correction) * 180 / M_PI, r.inlier_ratio, r.rmse);
  }

  Cloud::Ptr assemble(std::vector<Keyframe> & kfs_out, std::vector<Eigen::Isometry3d> & poses, std::size_t * loops = nullptr,
    double voxel = 0.0)
  {
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      kfs_out = kfs_;
      poses = graph_.poses();
      if (loops) {*loops = graph_.loops();}
    }
    Cloud all;
    for (std::size_t i = 0; i < kfs_out.size() && i < poses.size(); ++i) {
      all += *transformed(*kfs_out[i].cloud, poses[i]);
    }
    return voxelize(all, voxel > 0.0 ? voxel : map_voxel_);
  }

  void publishMap()
  {
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      if (!map_dirty_) {return;}
      map_dirty_ = false;
    }
    std::vector<Keyframe> kfs;
    std::vector<Eigen::Isometry3d> poses;
    const auto t0 = std::chrono::steady_clock::now();
    const auto map = assemble(kfs, poses, nullptr, publish_voxel_);
    {
      std::lock_guard<std::mutex> lock(stats_mtx_);
      publish_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      publish_points_ = map->size();
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    map_pub_->publish(msg);
    nav_msgs::msg::Path path;
    path.header = msg.header;
    for (const auto & T : poses) {
      geometry_msgs::msg::PoseStamped p;
      p.header = msg.header;
      p.pose = tf2::toMsg(T);
      path.poses.push_back(p);
    }
    path_pub_->publish(path);
  }

  void save(std_srvs::srv::Trigger::Response & res)
  {
    std::vector<Keyframe> kfs;
    std::vector<Eigen::Isometry3d> poses;
    std::size_t loops = 0;
    const auto map = assemble(kfs, poses, &loops);
    std::vector<MapKeyframe> out;
    for (std::size_t i = 0; i < kfs.size() && i < poses.size(); ++i) {
      out.push_back({kfs[i].stamp, poses[i], kfs[i].cloud});
    }
    std::string error;
    res.success = saveMap(map_dir_, out, *map, error);
    res.message = res.success ? "saved " + std::to_string(out.size()) + " keyframes, " +
      std::to_string(map->size()) + " points, " + std::to_string(loops) + " loops to " + map_dir_ : error;
    RCLCPP_INFO(get_logger(), "save_map: %s", res.message.c_str());
  }

  void logStats()
  {
    std::size_t kfs, loops;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      kfs = kfs_.size();
      loops = graph_.loops();
    }
    long pages = 0, rss_pages = 0;
    std::ifstream("/proc/self/statm") >> pages >> rss_pages;
    std::lock_guard<std::mutex> lock(stats_mtx_);
    RCLCPP_INFO(get_logger(), "stats: keyframes %zu loops %zu | loop detection avg %.1f ms max %.1f ms (%zu) | "
      "map publish %.0f ms %zu pts | rss %.0f MB", kfs, loops, loop_calls_ ? loop_ms_sum_ / loop_calls_ : 0.0,
      loop_ms_max_, loop_calls_, publish_ms_, publish_points_, rss_pages * 4096.0 / 1048576.0);
    loop_ms_sum_ = loop_ms_max_ = 0.0;
    loop_calls_ = 0;
  }

  // ---------------------------------------------------------------- localization
  void loadPriorMap()
  {
    std::vector<MapKeyframe> loaded;
    Cloud::Ptr global;
    std::string error;
    if (map_dir_.empty() || !loadMap(map_dir_, loaded, global, error)) {
      throw std::runtime_error("localization needs a saved map: " + (map_dir_.empty() ? "map_dir is empty" : error));
    }
    for (const auto & m : loaded) {
      Keyframe kf;
      kf.stamp = m.stamp;
      kf.T_odom_base = m.T_map_base;
      kf.cloud = m.cloud_base;
      kf.reg = voxelize(*m.cloud_base, reg_voxel_);
      describe(kf, m.T_map_base.rotation());
      prior_.push_back(kf);
    }
    prior_reg_.setTarget(voxelize(*global, reg_voxel_));
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*voxelize(*global, map_voxel_), msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    map_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "loaded map %s: %zu keyframes, %zu points", map_dir_.c_str(), prior_.size(), global->size());
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
  {
    if (m.header.frame_id != map_frame_) {
      RCLCPP_ERROR(get_logger(), "initial pose must be in %s", map_frame_.c_str());
      return;
    }
    Eigen::Isometry3d T = fromPose(m.pose.pose);
    // RViz 只给平面位姿：高度取最近关键帧
    double best = 1e9;
    for (const auto & kf : prior_) {
      const double d = (kf.T_odom_base.translation().head<2>() - T.translation().head<2>()).norm();
      if (d < best) {
        best = d;
        T.translation().z() = kf.T_odom_base.translation().z();
      }
    }
    std::lock_guard<std::mutex> lock(state_mtx_);
    pending_initial_ = T;
    have_pending_initial_ = true;
    state_ = State::Initializing;
    setStatus("INITIALIZING");
  }

  void processLocalization(const Frame & f)
  {
    local_.push_back(f);
    while (local_.size() > static_cast<std::size_t>(std::max(1, loc_frames_))) {local_.pop_front();}
    const double t = f.stamp.seconds();
    if (t - last_align_ < loc_period_) {return;}
    last_align_ = t;
    // 最近几帧在当前 base 系下拼接，提高稀疏雷达的配准稳定性
    Cloud acc;
    const Eigen::Isometry3d T_base_odom = f.T_odom_base.inverse();
    for (const auto & g : local_) {
      acc += *transformed(*g.cloud_base, T_base_odom * g.T_odom_base);
    }
    const auto source = voxelize(acc, reg_voxel_);

    State state;
    Eigen::Isometry3d T_map_odom;
    bool pending, have_estimate;
    Eigen::Isometry3d initial;
    {
      std::lock_guard<std::mutex> lock(state_mtx_);
      state = state_;
      T_map_odom = T_map_odom_;
      have_estimate = have_map_odom_;
      pending = have_pending_initial_;
      initial = pending_initial_;
      have_pending_initial_ = false;
    }
    if (state == State::Localized) {
      const Eigen::Isometry3d guess = T_map_odom * f.T_odom_base;
      const AlignResult r = prior_reg_.align(*source, guess, track_align_);
      const Eigen::Isometry3d jump = guess.inverse() * r.T_target_source;
      std::lock_guard<std::mutex> lock(state_mtx_);
      if (r.accepted && jump.translation().norm() < max_jump_m_ && angleOf(jump) < max_jump_rad_) {
        T_map_odom_ = r.T_target_source * f.T_odom_base.inverse();
        last_good_ = t;
      } else if (t - last_good_ > lost_timeout_) {
        state_ = State::Lost;
        setStatus("LOST");
      }
      return;
    }
    // 全局（重）定位：候选 = 人工初值 / 丢失前的估计 / 建图起点 / Scan Context 前 N 名
    std::vector<std::pair<std::string, Eigen::Isometry3d>> guesses;
    if (pending) {
      guesses.push_back({"initialpose", initial});
    } else {
      if (state == State::Lost && have_estimate) {
        guesses.push_back({"last_estimate", T_map_odom * f.T_odom_base});  // 重新进入建图区域时通常仍有效
      }
      if (try_origin_ && state == State::Initializing) {
        guesses.push_back({"map_origin", f.T_odom_base});
      }
      Keyframe query;
      query.reg = source;
      describe(query, f.T_odom_base.rotation());
      std::vector<std::tuple<double, std::size_t, int>> ranked;
      for (std::size_t j = 0; j < prior_.size(); ++j) {
        const auto [d, s] = sc_.distance(query.sc, prior_[j].sc);
        ranked.emplace_back(d, j, s);
      }
      std::sort(ranked.begin(), ranked.end());
      // 候选按描述子距离排序并在空间上去重：只取前几名时，对称/重复场景里孪生位置附近的多个关键帧会挤掉
      // 真实位置，歧义检查因没有对手而失效（仿真：对称房间 B 被定位到房间 A，误差 14 m）
      std::vector<Eigen::Vector3d> taken;
      for (const auto & [d, j, s] : ranked) {
        if (taken.size() >= static_cast<std::size_t>(reloc_candidates_)) {break;}
        const Eigen::Vector3d pos = prior_[j].T_odom_base.translation();
        if (std::any_of(taken.begin(), taken.end(), [&](const Eigen::Vector3d & q) {return (q - pos).head<2>().norm() < reloc_spacing_;})) {
          continue;
        }
        taken.push_back(pos);
        Eigen::Isometry3d g = Eigen::Isometry3d::Identity();
        g.linear() = Eigen::AngleAxisd(yawOf(prior_[j].T_odom_base.rotation()) + s * sc_.sectorAngle(),
            Eigen::Vector3d::UnitZ()).toRotationMatrix() * levelling(f.T_odom_base.rotation());
        g.translation() = prior_[j].T_odom_base.translation();
        guesses.push_back({"scan_context#" + std::to_string(j) + " d=" + std::to_string(d).substr(0, 5), g});
      }
    }
    std::vector<std::pair<AlignResult, std::string>> accepted;
    const auto t_start = std::chrono::steady_clock::now();
    for (const auto & [name, g] : guesses) {
      const AlignResult r = prior_reg_.align(*source, g, reloc_align_);
      if (r.accepted) {accepted.push_back({r, name});}
    }
    const double attempt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    if (accepted.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "relocalization: no candidate accepted (%zu tried, %.0f ms)",
        guesses.size(), attempt_ms);
      return;
    }
    std::sort(accepted.begin(), accepted.end(), [](const auto & a, const auto & b) {
        return a.first.inlier_ratio > b.first.inlier_ratio;
      });
    const AlignResult best = accepted.front().first;
    const std::string best_name = accepted.front().second;
    // 歧义检测：对称/重复场景中，若另一个位姿明显不同的候选几乎同样好，不急于给出定位（人工初值除外）
    if (!pending) {
      for (std::size_t i = 1; i < accepted.size(); ++i) {
        const Eigen::Isometry3d delta = best.T_target_source.inverse() * accepted[i].first.T_target_source;
        const bool distinct = delta.translation().norm() > amb_dist_ || angleOf(delta) > amb_angle_;
        if (distinct && best.inlier_ratio - accepted[i].first.inlier_ratio < amb_margin_) {
          const Eigen::Vector3d a = best.T_target_source.translation(), b = accepted[i].first.T_target_source.translation();
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "relocalization ambiguous: %s (%.2f, %.2f) inliers %.2f vs %s (%.2f, %.2f) inliers %.2f; "
            "waiting for more motion or /initialpose", best_name.c_str(), a.x(), a.y(), best.inlier_ratio,
            accepted[i].second.c_str(), b.x(), b.y(), accepted[i].first.inlier_ratio);
          std::lock_guard<std::mutex> lock(state_mtx_);
          setStatus("AMBIGUOUS");
          return;
        }
      }
    }
    std::lock_guard<std::mutex> lock(state_mtx_);
    T_map_odom_ = best.T_target_source * f.T_odom_base.inverse();
    have_map_odom_ = true;
    last_good_ = t;
    state_ = State::Localized;
    const Eigen::Vector3d p = best.T_target_source.translation();
    RCLCPP_INFO(get_logger(), "relocalized via %s at (%.2f, %.2f, %.2f) yaw %.1f deg, inliers %.2f rmse %.3f (%zu tried, %.0f ms)",
      best_name.c_str(), p.x(), p.y(), p.z(), yawOf(best.T_target_source.rotation()) * 180 / M_PI, best.inlier_ratio, best.rmse,
      guesses.size(), attempt_ms);
    setStatus("LOCALIZED");
  }

  // parameters
  std::string mode_, map_dir_, map_frame_, odom_frame_, base_frame_, imu_frame_;
  double kf_dist_, kf_angle_, kf_voxel_, reg_voxel_, map_voxel_, max_range_, publish_period_, publish_voxel_;
  double loop_radius_, loop_time_gap_, sc_threshold_, cauchy_k_, drift_ratio_, drift_base_;
  int loop_index_gap_, submap_half_, loop_cooldown_;
  long last_loop_k_ = -1;
  EdgeSigmas odom_sigmas_{}, loop_sigmas_{};
  AlignParams loop_align_, reloc_align_, track_align_;
  double loc_period_, max_jump_m_, max_jump_rad_, lost_timeout_, reloc_spacing_;
  int loc_frames_, reloc_candidates_;
  bool try_origin_;
  double amb_dist_, amb_angle_, amb_margin_;
  ScanContext sc_;

  // ROS
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::CallbackGroup::SharedPtr io_group_, slow_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr map_timer_, stats_timer_;
  std::mutex stats_mtx_;
  double loop_ms_sum_ = 0.0, loop_ms_max_ = 0.0, publish_ms_ = 0.0;
  std::size_t loop_calls_ = 0, publish_points_ = 0;

  // input sync (io callback group only)
  std::map<int64_t, Eigen::Isometry3d> odom_buf_;
  std::map<int64_t, sensor_msgs::msg::PointCloud2::SharedPtr> cloud_buf_;
  bool have_base_imu_ = false;
  Eigen::Isometry3d T_base_imu_ = Eigen::Isometry3d::Identity();

  // worker queue
  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<Frame> frames_;
  bool stop_ = false;
  std::thread worker_;

  // shared state (state_mtx_)
  std::mutex state_mtx_;
  Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
  bool have_map_odom_ = false;
  bool map_dirty_ = false;
  std::vector<Keyframe> kfs_;
  PoseGraph graph_;
  State state_ = State::Initializing;
  bool have_pending_initial_ = false;
  Eigen::Isometry3d pending_initial_ = Eigen::Isometry3d::Identity();
  double last_good_ = 0.0;
  std::string status_;

  // localization (worker thread only)
  std::vector<Keyframe> prior_;
  Registration prior_reg_;
  std::deque<Frame> local_;
  double last_align_ = -1e9;
};

}  // namespace uslam_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<uslam_backend::BackendNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
