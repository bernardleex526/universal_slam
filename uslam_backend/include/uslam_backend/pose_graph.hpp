// Keyframe pose graph on GTSAM iSAM2.
//
// Variables X_k = T_map_base_k (gtsam::Pose3). GTSAM Pose3 uses right perturbation
// X (+) xi = X * Exp(xi), tangent order xi = (omega, v) - rotation first.
// Odometry and loop edges are RelativePoseFactor (exact Jacobians, see relative_pose_factor.hpp)
// with measurement Z ~ X_i^-1 X_j and residual e = Log(Z^-1 * X_i^-1 * X_j).
// Loop edges carry a Cauchy kernel (k in whitened units). k must exceed the whitened residual of a
// correct loop (accumulated drift / sigma), or true loops are treated as outliers and ignored.
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace uslam_backend
{

struct EdgeSigmas
{
  double rot;    // rad
  double trans;  // m
};

class PoseGraph
{
public:
  PoseGraph();

  std::size_t addFirst(const Eigen::Isometry3d & T_map_base, const EdgeSigmas & prior);
  // Edge from the previous keyframe; T_prev_cur = T_odom_prev^-1 * T_odom_cur.
  std::size_t addOdometry(const Eigen::Isometry3d & T_prev_cur, const EdgeSigmas & sigmas);
  // T_i_j = X_i^-1 * X_j as measured by scan registration.
  void addLoop(std::size_t i, std::size_t j, const Eigen::Isometry3d & T_i_j, const EdgeSigmas & sigmas,
    double cauchy_k);
  void relax(int iterations);

  std::size_t size() const {return size_;}
  std::size_t loops() const {return loops_;}
  Eigen::Isometry3d pose(std::size_t k) const;
  std::vector<Eigen::Isometry3d> poses() const;

private:
  void commit(const gtsam::NonlinearFactorGraph & factors, const gtsam::Values & values);

  gtsam::ISAM2 isam_;
  gtsam::Values estimate_;
  std::size_t size_ = 0;
  std::size_t loops_ = 0;
};

}  // namespace uslam_backend
