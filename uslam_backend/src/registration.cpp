#include "uslam_backend/registration.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace uslam_backend
{

Cloud::Ptr voxelize(const Cloud & in, double leaf)
{
  auto out = std::make_shared<Cloud>();
  if (in.empty()) {
    return out;
  }
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  grid.setInputCloud(std::make_shared<Cloud>(in));
  grid.filter(*out);
  return out;
}

Cloud::Ptr transformed(const Cloud & in, const Eigen::Isometry3d & T)
{
  auto out = std::make_shared<Cloud>();
  pcl::transformPointCloud(in, *out, T.matrix().cast<float>());
  return out;
}

void Registration::setTarget(const Cloud::ConstPtr & target)
{
  if (!target || target->size() < 50) {
    throw std::invalid_argument("registration target is empty");
  }
  target_ = target;
  tree_.setInputCloud(target_);
  // 目标协方差只在 setInputTarget 时重算；定位模式下全局地图不变，可复用
  gicp_.setInputTarget(target_);
}

AlignResult Registration::align(const Cloud & source, const Eigen::Isometry3d & guess, const AlignParams & params)
{
  AlignResult result;
  if (!target_ || source.size() < 50) {
    return result;
  }
  auto src = std::make_shared<Cloud>(source);
  Eigen::Matrix4f T = guess.matrix().cast<float>();
  Cloud aligned;
  for (const double max_corr : {params.coarse_max_correspondence, params.fine_max_correspondence}) {
    gicp_.setInputSource(src);
    gicp_.setMaxCorrespondenceDistance(max_corr);
    gicp_.setMaximumIterations(params.max_iterations);
    gicp_.setTransformationEpsilon(1e-6);
    gicp_.setEuclideanFitnessEpsilon(1e-6);
    gicp_.align(aligned, T);
    if (!gicp_.hasConverged()) {
      return result;
    }
    T = gicp_.getFinalTransformation();
  }
  result.converged = true;
  result.T_target_source.matrix() = T.cast<double>();
  // 独立的内点检验：对齐后的源点到目标最近邻距离
  std::vector<int> idx(1);
  std::vector<float> d2(1);
  std::size_t inliers = 0;
  double sum = 0.0;
  const double max_d2 = params.inlier_distance * params.inlier_distance;
  for (const auto & p : aligned.points) {
    if (tree_.nearestKSearch(p, 1, idx, d2) == 1 && d2[0] <= max_d2) {
      ++inliers;
      sum += d2[0];
    }
  }
  result.inlier_ratio = aligned.empty() ? 0.0 : static_cast<double>(inliers) / aligned.size();
  result.rmse = inliers ? std::sqrt(sum / inliers) : std::numeric_limits<double>::infinity();
  result.accepted = result.inlier_ratio >= params.min_inlier_ratio && result.rmse <= params.max_rmse;
  return result;
}

}  // namespace uslam_backend
