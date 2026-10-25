#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "uslam_backend/registration.hpp"

using uslam_backend::AlignParams;
using uslam_backend::Cloud;
using uslam_backend::Registration;

namespace
{

// Room with floor, walls and two boxes, sampled on its surfaces (map frame).
Cloud room()
{
  Cloud c;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto add = [&](double x, double y, double z) {
      c.push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
    };
  for (int i = 0; i < 60000; ++i) {
    const double a = u(rng), b = u(rng);
    switch (i % 7) {
      case 0: add(-6 + 20 * a, -7 + 14 * b, 0.0); break;
      case 1: add(-6, -7 + 14 * a, 2.5 * b); break;
      case 2: add(14, -7 + 14 * a, 2.5 * b); break;
      case 3: add(-6 + 20 * a, -7, 2.5 * b); break;
      case 4: add(-6 + 20 * a, 7, 2.5 * b); break;
      case 5: add(2 + a, -3, b); break;
      default: add(9, 4 + 2 * a, 1.2 * b); break;
    }
  }
  return *uslam_backend::voxelize(c, 0.1);
}

Eigen::Isometry3d pose(double x, double y, double z, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(x, y, z);
  return T;
}

}  // namespace

TEST(Registration, RecoversPoseFromPerturbedGuess)
{
  const auto target = std::make_shared<Cloud>(room());
  Registration reg;
  reg.setTarget(target);
  const Eigen::Isometry3d T_true = pose(3.0, 1.0, 0.47, 0.6);
  // Source = map points within 12 m of the sensor, expressed in the sensor frame.
  Cloud local;
  for (const auto & p : target->points) {
    if ((p.getVector3fMap().cast<double>() - T_true.translation()).norm() < 12.0) {local.push_back(p);}
  }
  const Cloud source = *uslam_backend::transformed(local, T_true.inverse());
  const Eigen::Isometry3d guess = T_true * pose(0.4, -0.3, 0.1, 8.0 * M_PI / 180.0);
  AlignParams p;
  p.coarse_max_correspondence = 2.0;
  const auto r = reg.align(*uslam_backend::voxelize(source, 0.25), guess, p);
  ASSERT_TRUE(r.accepted) << "inliers " << r.inlier_ratio << " rmse " << r.rmse;
  const Eigen::Isometry3d err = T_true.inverse() * r.T_target_source;
  EXPECT_LT(err.translation().norm(), 0.03);
  EXPECT_LT(Eigen::AngleAxisd(err.rotation()).angle() * 180.0 / M_PI, 0.5);
}

TEST(Registration, RejectsUnrelatedGeometry)
{
  Registration reg;
  reg.setTarget(std::make_shared<Cloud>(room()));
  Cloud noise;
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-10.0, 10.0);
  for (int i = 0; i < 3000; ++i) {
    noise.push_back(pcl::PointXYZ(static_cast<float>(u(rng)), static_cast<float>(u(rng)), static_cast<float>(u(rng))));
  }
  EXPECT_FALSE(reg.align(noise, pose(3, 1, 0.5, 0), AlignParams()).accepted);
}

TEST(Registration, RejectsTinyInputs)
{
  Registration reg;
  EXPECT_THROW(reg.setTarget(std::make_shared<Cloud>()), std::invalid_argument);
  reg.setTarget(std::make_shared<Cloud>(room()));
  EXPECT_FALSE(reg.align(Cloud(), Eigen::Isometry3d::Identity(), AlignParams()).accepted);
}
