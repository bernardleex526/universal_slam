// Saved-map directory used for relocalization:
//   map.pcd                 global map, map frame (binary PCD, x y z)
//   keyframes.txt           "index stamp x y z qx qy qz qw" per keyframe, T_map_base
//   keyframes/NNNNNN.pcd    keyframe cloud in its base frame
// save writes into a temporary sibling directory and renames it into place; an existing map is
// kept as <dir>.previous instead of being deleted.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "uslam_backend/registration.hpp"

namespace uslam_backend
{

struct MapKeyframe
{
  double stamp = 0.0;
  Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
  Cloud::Ptr cloud_base;
};

bool saveMap(const std::string & dir, const std::vector<MapKeyframe> & keyframes, const Cloud & global,
  std::string & error);
bool loadMap(const std::string & dir, std::vector<MapKeyframe> & keyframes, Cloud::Ptr & global,
  std::string & error);

}  // namespace uslam_backend
