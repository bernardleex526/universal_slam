#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <functional>

#include "uslam_backend/pose_graph.hpp"
#include "uslam_backend/relative_pose_factor.hpp"

using uslam_backend::EdgeSigmas;
using uslam_backend::PoseGraph;

namespace
{

Eigen::Isometry3d pose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

// 10 x 10 m square, keyframe every 1 m, back at the start.
std::vector<Eigen::Isometry3d> squareTruth()
{
  std::vector<Eigen::Isometry3d> gt{pose(0, 0, 0, 0)};
  for (int side = 0; side < 4; ++side) {
    for (int i = 0; i < 10; ++i) {
      Eigen::Isometry3d step = pose(1.0, 0, 0, i == 9 ? M_PI / 2 : 0.0);
      gt.push_back(gt.back() * step);
    }
  }
  return gt;
}

// Odometry with a constant yaw bias and vertical drift per metre (LIO-like).
std::vector<Eigen::Isometry3d> driftedDeltas(const std::vector<Eigen::Isometry3d> & gt)
{
  std::vector<Eigen::Isometry3d> d;
  for (std::size_t k = 1; k < gt.size(); ++k) {
    d.push_back(gt[k - 1].inverse() * gt[k] * pose(0, 0, 0.01, 0.005));
  }
  return d;
}

double maxError(const PoseGraph & g, const std::vector<Eigen::Isometry3d> & gt)
{
  double e = 0.0;
  for (std::size_t k = 0; k < gt.size(); ++k) {
    e = std::max(e, (g.pose(k).translation() - gt[k].translation()).norm());
  }
  return e;
}

}  // namespace

TEST(PoseGraph, LoopEdgeRemovesAccumulatedDrift)
{
  const auto gt = squareTruth();
  const auto deltas = driftedDeltas(gt);
  PoseGraph g;
  const EdgeSigmas odom{0.01, 0.05};
  g.addFirst(gt[0], {1e-4, 1e-3});
  for (const auto & d : deltas) {g.addOdometry(d, odom);}
  const double before = maxError(g, gt);
  EXPECT_GT(before, 0.5) << "the drifted chain must be visibly wrong";
  // Registration measured the true relative pose between the last and the first keyframe.
  g.addLoop(0, gt.size() - 1, gt.front().inverse() * gt.back(), {0.01, 0.02}, 10.0);
  g.relax(5);
  EXPECT_LT((g.pose(gt.size() - 1).translation() - gt.back().translation()).norm(), 0.05);
  EXPECT_LT(maxError(g, gt), 0.25);
  EXPECT_EQ(g.loops(), 1u);
}

TEST(PoseGraph, CauchyKernelLimitsAWrongLoop)
{
  const auto gt = squareTruth();
  PoseGraph g;
  g.addFirst(gt[0], {1e-4, 1e-3});
  for (std::size_t k = 1; k < gt.size(); ++k) {g.addOdometry(gt[k - 1].inverse() * gt[k], {0.01, 0.05});}
  g.addLoop(0, gt.size() - 1, gt.front().inverse() * gt.back(), {0.01, 0.02}, 10.0);
  g.addLoop(5, 25, pose(5.0, 0, 0, 0), {0.01, 0.02}, 10.0);  // wrong by ~10 m
  g.relax(10);
  EXPECT_LT(maxError(g, gt), 0.3);
}

TEST(PoseGraph, RelativePoseFactorJacobiansMatchNumericalDifferences)
{
  // Large residuals on purpose: that is where gtsam::BetweenFactor's default Jacobians are off.
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 0.7);
  const auto random_pose = [&] {
      return gtsam::Pose3::Expmap((gtsam::Vector6() << n(rng), n(rng), n(rng), n(rng), n(rng), n(rng)).finished());
    };
  for (int trial = 0; trial < 50; ++trial) {
    const gtsam::Pose3 xi = random_pose(), xj = random_pose();
    const gtsam::Pose3 z = xi.between(xj) * random_pose();
    uslam_backend::RelativePoseFactor f(0, 1, z, gtsam::noiseModel::Unit::Create(6));
    gtsam::Matrix H1, H2;
    f.evaluateError(xi, xj, H1, H2);
    const std::function<gtsam::Vector(const gtsam::Pose3 &, const gtsam::Pose3 &)> err =
      [&](const gtsam::Pose3 & a, const gtsam::Pose3 & b) {return f.evaluateError(a, b);};
    const gtsam::Matrix N1 = gtsam::numericalDerivative21(err, xi, xj, 1e-6);
    const gtsam::Matrix N2 = gtsam::numericalDerivative22(err, xi, xj, 1e-6);
    EXPECT_LT((H1 - N1).cwiseAbs().maxCoeff(), 1e-5) << "trial " << trial;
    EXPECT_LT((H2 - N2).cwiseAbs().maxCoeff(), 1e-5) << "trial " << trial;
  }
}

TEST(PoseGraph, RejectsMisuse)
{
  PoseGraph g;
  EXPECT_THROW(g.addOdometry(pose(1, 0, 0, 0), {0.01, 0.05}), std::logic_error);
  g.addFirst(pose(0, 0, 0, 0), {1e-4, 1e-3});
  EXPECT_THROW(g.addLoop(0, 3, pose(0, 0, 0, 0), {0.01, 0.02}, 1.0), std::out_of_range);
  EXPECT_THROW(g.addOdometry(pose(1, 0, 0, 0), {0.0, 0.05}), std::invalid_argument);
}
