#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include "uslam_backend/registration.hpp"
#include "uslam_backend/scan_context.hpp"

using uslam_backend::Cloud;
using uslam_backend::ScanContext;

namespace
{

// Asymmetric levelled scene around the origin: room walls, a box and a pillar, plus floor.
Cloud scene(const Eigen::Vector2d & centre)
{
  Cloud c;
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  auto add = [&](double x, double y, double z) {
      c.push_back(pcl::PointXYZ(static_cast<float>(x - centre.x()), static_cast<float>(y - centre.y()), static_cast<float>(z)));
    };
  for (int i = 0; i < 20000; ++i) {
    const double z = -0.47 + 2.2 * u(rng);
    const double t = u(rng);
    switch (i % 6) {
      case 0: add(-5.0, -4.0 + 10.0 * t, z); break;  // west wall
      case 1: add(7.0, -4.0 + 10.0 * t, z); break;   // east wall
      case 2: add(-5.0 + 12.0 * t, -4.0, z); break;  // south wall
      case 3: add(-5.0 + 12.0 * t, 6.0, z); break;   // north wall
      case 4: add(2.0 + 0.8 * t, 1.5, -0.47 + 0.7 * u(rng)); break;  // low box face
      default: add(-2.0 + 0.3 * std::cos(6.28 * t), 3.0 + 0.3 * std::sin(6.28 * t), z); break;  // pillar
    }
  }
  return c;
}

Cloud rotated(const Cloud & in, double yaw)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return *uslam_backend::transformed(in, T);
}

}  // namespace

TEST(ScanContext, RecoversYawWithDocumentedSign)
{
  const ScanContext sc;
  const Cloud cand = scene({0.0, 0.0});
  for (const double yaw_deg : {36.0, -72.0, 150.0}) {
    const double yaw = yaw_deg * M_PI / 180.0;
    // T_cand_query = Rz(yaw)  =>  p_query = Rz(-yaw) p_cand
    const Cloud query = rotated(cand, -yaw);
    const auto [d, shift] = sc.distance(sc.describe(query), sc.describe(cand));
    double est = shift * sc.sectorAngle();
    double err = std::remainder(est - yaw, 2.0 * M_PI);
    EXPECT_LT(d, 0.1) << "yaw " << yaw_deg;
    EXPECT_LT(std::abs(err), 1.5 * sc.sectorAngle()) << "yaw " << yaw_deg << " got " << est * 180 / M_PI;
    EXPECT_LT((ScanContext::ringKey(sc.describe(query)) - ScanContext::ringKey(sc.describe(cand))).norm(), 0.5);
  }
}

TEST(ScanContext, DifferentPlaceIsFartherThanSamePlace)
{
  const ScanContext sc;
  const auto here = sc.describe(scene({0.0, 0.0}));
  const auto same_rotated = sc.describe(rotated(scene({0.0, 0.0}), 1.0));
  const auto elsewhere = sc.describe(scene({4.0, 3.0}));
  EXPECT_LT(sc.distance(same_rotated, here).first + 0.05, sc.distance(elsewhere, here).first);
}

TEST(ScanContext, EmptyDescriptorIsMaximallyDistant)
{
  const ScanContext sc;
  const auto empty = sc.describe(Cloud());
  EXPECT_DOUBLE_EQ(sc.distance(empty, sc.describe(scene({0, 0}))).first, 1.0);
}
