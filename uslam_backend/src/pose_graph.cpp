#include "uslam_backend/pose_graph.hpp"
#include "uslam_backend/relative_pose_factor.hpp"

#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

namespace uslam_backend
{
namespace
{

gtsam::Pose3 toPose3(const Eigen::Isometry3d & T)
{
  return gtsam::Pose3(gtsam::Rot3(T.rotation()), gtsam::Point3(T.translation()));
}

gtsam::SharedNoiseModel diagonal(const EdgeSigmas & s)
{
  if (!(s.rot > 0.0) || !(s.trans > 0.0)) {
    throw std::invalid_argument("edge sigmas must be positive");
  }
  gtsam::Vector6 sigmas;
  sigmas << s.rot, s.rot, s.rot, s.trans, s.trans, s.trans;  // Pose3 tangent: rotation first
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::ISAM2Params isamParams()
{
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = 0.01;
  p.relinearizeSkip = 1;
  return p;
}

}  // namespace

PoseGraph::PoseGraph()
: isam_(isamParams()) {}

void PoseGraph::commit(const gtsam::NonlinearFactorGraph & factors, const gtsam::Values & values)
{
  isam_.update(factors, values);
  estimate_ = isam_.calculateEstimate();
}

std::size_t PoseGraph::addFirst(const Eigen::Isometry3d & T_map_base, const EdgeSigmas & prior)
{
  if (size_ != 0) {
    throw std::logic_error("pose graph already started");
  }
  gtsam::NonlinearFactorGraph f;
  gtsam::Values v;
  f.add(gtsam::PriorFactor<gtsam::Pose3>(0, toPose3(T_map_base), diagonal(prior)));
  v.insert(0, toPose3(T_map_base));
  commit(f, v);
  return size_++;
}

std::size_t PoseGraph::addOdometry(const Eigen::Isometry3d & T_prev_cur, const EdgeSigmas & sigmas)
{
  if (size_ == 0) {
    throw std::logic_error("addFirst must come first");
  }
  const auto k = size_;
  const gtsam::Pose3 z = toPose3(T_prev_cur);
  gtsam::NonlinearFactorGraph f;
  gtsam::Values v;
  f.add(RelativePoseFactor(k - 1, k, z, diagonal(sigmas)));
  // 初值：上一关键帧的当前优化估计 * 里程计增量
  v.insert(k, estimate_.at<gtsam::Pose3>(k - 1) * z);
  commit(f, v);
  return size_++;
}

void PoseGraph::addLoop(
  std::size_t i, std::size_t j, const Eigen::Isometry3d & T_i_j, const EdgeSigmas & sigmas, double cauchy_k)
{
  if (i >= size_ || j >= size_ || i == j) {
    throw std::out_of_range("loop edge references an unknown keyframe");
  }
  const auto noise = gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(cauchy_k), diagonal(sigmas));
  gtsam::NonlinearFactorGraph f;
  f.add(RelativePoseFactor(i, j, toPose3(T_i_j), noise));
  commit(f, gtsam::Values());
  ++loops_;
}

void PoseGraph::relax(int iterations)
{
  for (int k = 0; k < iterations; ++k) {
    isam_.update();
  }
  estimate_ = isam_.calculateEstimate();
}

Eigen::Isometry3d PoseGraph::pose(std::size_t k) const
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.matrix() = estimate_.at<gtsam::Pose3>(k).matrix();
  return T;
}

std::vector<Eigen::Isometry3d> PoseGraph::poses() const
{
  std::vector<Eigen::Isometry3d> out;
  out.reserve(size_);
  for (std::size_t k = 0; k < size_; ++k) {
    out.push_back(pose(k));
  }
  return out;
}

}  // namespace uslam_backend
