// Scan Context place descriptor (Kim & Kim, IROS 2018).
//
// Input: a gravity-levelled, base-centred cloud (x along the base heading, z up). Each polar
// cell (ring r, sector s) stores max(z + z_offset) of its points; empty cells are 0.
// Yaw convention (checked in test_scan_context): if rolling the query descriptor by `shift`
// columns matches the candidate, the query frame is the candidate frame rotated by
//   yaw = shift * sectorAngle(),  i.e.  T_cand_query = Rz(yaw).
#pragma once

#include <utility>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace uslam_backend
{

struct ScanContextParams
{
  int rings = 20;
  int sectors = 60;
  double max_radius = 20.0;
  double z_offset = 2.0;  // keeps floor-level returns (z ~ -base_height) positive
};

class ScanContext
{
public:
  explicit ScanContext(const ScanContextParams & params = ScanContextParams());

  Eigen::MatrixXd describe(const pcl::PointCloud<pcl::PointXYZ> & levelled) const;

  // Rotation-invariant ring key (mean of each ring), used to pre-select candidates.
  static Eigen::VectorXd ringKey(const Eigen::MatrixXd & descriptor);

  // Returns (distance in [0, 1], best column shift).
  std::pair<double, int> distance(const Eigen::MatrixXd & query, const Eigen::MatrixXd & candidate) const;

  double sectorAngle() const;

private:
  ScanContextParams params_;
};

}  // namespace uslam_backend
