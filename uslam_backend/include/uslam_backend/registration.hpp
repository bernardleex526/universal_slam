// Scan-to-map registration (PCL GICP, coarse then fine) with an independent acceptance check.
//
// A result is accepted only if GICP converged AND, after alignment, at least min_inlier_ratio of
// the source points have a target neighbour within inlier_distance AND the inlier RMSE is at most
// max_rmse. GICP's own fitness score is not used for acceptance: it is dominated by outliers.
#pragma once

#include <memory>

#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace uslam_backend
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct AlignParams
{
  double coarse_max_correspondence = 1.0;
  double fine_max_correspondence = 0.5;
  int max_iterations = 40;
  double inlier_distance = 0.3;
  double min_inlier_ratio = 0.6;
  double max_rmse = 0.15;
};

struct AlignResult
{
  bool accepted = false;
  bool converged = false;
  Eigen::Isometry3d T_target_source = Eigen::Isometry3d::Identity();
  double inlier_ratio = 0.0;
  double rmse = 0.0;
};

class Registration
{
public:
  void setTarget(const Cloud::ConstPtr & target);
  bool hasTarget() const {return static_cast<bool>(target_);}
  // guess: initial T_target_source.
  AlignResult align(const Cloud & source, const Eigen::Isometry3d & guess, const AlignParams & params);

private:
  Cloud::ConstPtr target_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
};

Cloud::Ptr voxelize(const Cloud & in, double leaf);
Cloud::Ptr transformed(const Cloud & in, const Eigen::Isometry3d & T);

}  // namespace uslam_backend
