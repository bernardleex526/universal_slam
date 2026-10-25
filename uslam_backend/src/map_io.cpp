#include "uslam_backend/map_io.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace uslam_backend
{
namespace
{

std::string keyframeName(std::size_t k)
{
  std::ostringstream s;
  s << std::setw(6) << std::setfill('0') << k << ".pcd";
  return s.str();
}

}  // namespace

bool saveMap(const std::string & dir, const std::vector<MapKeyframe> & keyframes, const Cloud & global,
  std::string & error)
{
  try {
    const fs::path target(dir);
    if (dir.empty() || !target.is_absolute()) {
      error = "map_dir must be an absolute path";
      return false;
    }
    if (keyframes.empty() || global.empty()) {
      error = "no keyframes to save";
      return false;
    }
    const fs::path tmp = target.parent_path() / (target.filename().string() + ".tmp-" + std::to_string(getpid()));
    fs::remove_all(tmp);
    fs::create_directories(tmp / "keyframes");
    if (pcl::io::savePCDFileBinary((tmp / "map.pcd").string(), global) != 0) {
      error = "failed to write map.pcd";
      return false;
    }
    std::ofstream list(tmp / "keyframes.txt");
    list << "# index stamp x y z qx qy qz qw  (T_map_base)\n" << std::setprecision(10);
    for (std::size_t k = 0; k < keyframes.size(); ++k) {
      const auto & kf = keyframes[k];
      if (!kf.cloud_base || kf.cloud_base->empty() ||
        pcl::io::savePCDFileBinary((tmp / "keyframes" / keyframeName(k)).string(), *kf.cloud_base) != 0)
      {
        error = "failed to write keyframe " + std::to_string(k);
        return false;
      }
      const Eigen::Quaterniond q(kf.T_map_base.rotation());
      const Eigen::Vector3d t = kf.T_map_base.translation();
      list << k << ' ' << std::setprecision(17) << kf.stamp << std::setprecision(10) << ' ' << t.x() << ' ' <<
        t.y() << ' ' << t.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
    list.close();
    if (!list) {
      error = "failed to write keyframes.txt";
      return false;
    }
    if (fs::exists(target)) {
      const fs::path previous = target.parent_path() / (target.filename().string() + ".previous");
      fs::remove_all(previous);
      fs::rename(target, previous);
    }
    fs::rename(tmp, target);
    return true;
  } catch (const std::exception & e) {
    error = e.what();
    return false;
  }
}

bool loadMap(const std::string & dir, std::vector<MapKeyframe> & keyframes, Cloud::Ptr & global,
  std::string & error)
{
  try {
    const fs::path root(dir);
    global = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile((root / "map.pcd").string(), *global) != 0 || global->empty()) {
      error = "cannot read " + (root / "map.pcd").string();
      return false;
    }
    std::ifstream list(root / "keyframes.txt");
    if (!list) {
      error = "cannot read keyframes.txt";
      return false;
    }
    keyframes.clear();
    std::string line;
    while (std::getline(list, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream in(line);
      std::size_t k = 0;
      MapKeyframe kf;
      double x, y, z, qx, qy, qz, qw;
      if (!(in >> k >> kf.stamp >> x >> y >> z >> qx >> qy >> qz >> qw) || k != keyframes.size()) {
        error = "malformed keyframes.txt line: " + line;
        return false;
      }
      kf.T_map_base = Eigen::Isometry3d::Identity();
      kf.T_map_base.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
      kf.T_map_base.translation() = Eigen::Vector3d(x, y, z);
      kf.cloud_base = std::make_shared<Cloud>();
      if (pcl::io::loadPCDFile((root / "keyframes" / keyframeName(k)).string(), *kf.cloud_base) != 0) {
        error = "cannot read keyframe " + std::to_string(k);
        return false;
      }
      keyframes.push_back(kf);
    }
    if (keyframes.empty()) {
      error = "map has no keyframes";
      return false;
    }
    return true;
  } catch (const std::exception & e) {
    error = e.what();
    return false;
  }
}

}  // namespace uslam_backend
