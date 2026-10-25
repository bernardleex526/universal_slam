#include "uslam_backend/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace uslam_backend
{

ScanContext::ScanContext(const ScanContextParams & params)
: params_(params)
{
  if (params_.rings < 1 || params_.sectors < 4 || !(params_.max_radius > 0.0)) {
    throw std::invalid_argument("invalid Scan Context parameters");
  }
}

double ScanContext::sectorAngle() const {return 2.0 * M_PI / params_.sectors;}

Eigen::MatrixXd ScanContext::describe(const pcl::PointCloud<pcl::PointXYZ> & levelled) const
{
  Eigen::MatrixXd desc = Eigen::MatrixXd::Zero(params_.rings, params_.sectors);
  for (const auto & p : levelled.points) {
    const double range = std::hypot(p.x, p.y);
    if (!std::isfinite(range) || !std::isfinite(p.z) || range >= params_.max_radius || range < 1e-3) {
      continue;
    }
    // 方位角 [-pi, pi) 映射到扇区；旋转 query 等价于描述子按列循环移位
    const double azimuth = std::atan2(p.y, p.x) + M_PI;
    const int ring = std::min(params_.rings - 1, static_cast<int>(range / params_.max_radius * params_.rings));
    const int sector = std::min(params_.sectors - 1, static_cast<int>(azimuth / (2.0 * M_PI) * params_.sectors));
    desc(ring, sector) = std::max(desc(ring, sector), std::max(0.0, p.z + params_.z_offset));
  }
  return desc;
}

Eigen::VectorXd ScanContext::ringKey(const Eigen::MatrixXd & descriptor)
{
  return descriptor.rowwise().mean();
}

std::pair<double, int> ScanContext::distance(const Eigen::MatrixXd & query, const Eigen::MatrixXd & candidate) const
{
  if (query.rows() != candidate.rows() || query.cols() != candidate.cols()) {
    throw std::invalid_argument("Scan Context size mismatch");
  }
  const int n = static_cast<int>(query.cols());
  Eigen::VectorXd qn(n), cn(n);
  for (int j = 0; j < n; ++j) {
    qn(j) = query.col(j).norm();
    cn(j) = candidate.col(j).norm();
  }
  double best = 1.0;
  int best_shift = 0;
  for (int shift = 0; shift < n; ++shift) {
    double sum = 0.0;
    int valid = 0;
    for (int j = 0; j < n; ++j) {
      const int k = ((j - shift) % n + n) % n;  // query column that lands on candidate column j
      if (qn(k) < 1e-9 || cn(j) < 1e-9) {
        continue;
      }
      sum += 1.0 - query.col(k).dot(candidate.col(j)) / (qn(k) * cn(j));
      ++valid;
    }
    // 有效列太少时不可信：按最大距离处理
    const double d = valid >= n / 4 ? sum / valid : 1.0;
    if (d < best) {
      best = d;
      best_shift = shift;
    }
  }
  return {best, best_shift};
}

}  // namespace uslam_backend
