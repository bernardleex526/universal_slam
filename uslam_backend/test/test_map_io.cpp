#include <gtest/gtest.h>

#include <filesystem>
#include <memory>

#include <unistd.h>

#include "uslam_backend/map_io.hpp"

namespace fs = std::filesystem;
using uslam_backend::Cloud;
using uslam_backend::MapKeyframe;

namespace
{

Cloud::Ptr box(double offset)
{
  auto c = std::make_shared<Cloud>();
  for (int i = 0; i < 100; ++i) {
    c->push_back(pcl::PointXYZ(static_cast<float>(i * 0.01 + offset), 1.0f, 2.0f));
  }
  return c;
}

}  // namespace

TEST(MapIo, RoundTripAndKeepsPreviousMap)
{
  const fs::path dir = fs::temp_directory_path() / ("uslam_map_io_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::remove_all(dir.string() + ".previous");
  std::vector<MapKeyframe> kfs(2);
  kfs[0].stamp = 1761200000.123456789;
  kfs[0].cloud_base = box(0.0);
  kfs[1].T_map_base.translate(Eigen::Vector3d(1.5, -2.0, 0.3));
  kfs[1].T_map_base.rotate(Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()));
  kfs[1].cloud_base = box(5.0);
  std::string error;
  ASSERT_TRUE(uslam_backend::saveMap(dir.string(), kfs, *box(1.0), error)) << error;
  ASSERT_TRUE(uslam_backend::saveMap(dir.string(), kfs, *box(2.0), error)) << error;
  EXPECT_TRUE(fs::exists(dir.string() + ".previous/map.pcd")) << "an overwritten map is kept, not deleted";

  std::vector<MapKeyframe> loaded;
  Cloud::Ptr global;
  ASSERT_TRUE(uslam_backend::loadMap(dir.string(), loaded, global, error)) << error;
  ASSERT_EQ(loaded.size(), 2u);
  EXPECT_NEAR(loaded[0].stamp, kfs[0].stamp, 1e-6);
  EXPECT_TRUE(loaded[1].T_map_base.isApprox(kfs[1].T_map_base, 1e-8));
  EXPECT_EQ(loaded[1].cloud_base->size(), 100u);
  EXPECT_NEAR(global->points[0].x, 2.0, 1e-6);
  fs::remove_all(dir);
  fs::remove_all(dir.string() + ".previous");
}

TEST(MapIo, RejectsRelativeOrEmptyInput)
{
  std::string error;
  std::vector<MapKeyframe> kfs(1);
  kfs[0].cloud_base = box(0.0);
  EXPECT_FALSE(uslam_backend::saveMap("relative/map", kfs, *box(0.0), error));
  EXPECT_FALSE(uslam_backend::saveMap("/tmp/uslam_empty", {}, *box(0.0), error));
  std::vector<MapKeyframe> loaded;
  Cloud::Ptr global;
  EXPECT_FALSE(uslam_backend::loadMap("/nonexistent/uslam", loaded, global, error));
}
