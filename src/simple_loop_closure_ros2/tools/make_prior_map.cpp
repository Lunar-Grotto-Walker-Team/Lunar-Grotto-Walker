// make_prior_map — turn a /save_req dump into a prior-map asset.
//
//   make_prior_map --src DIR --out DIR [--translate DX DY]
//                  [--crop CX CY R] [--spacing 0.5]
//
// The source layout is what saveEachFrames() writes: poses.csv plus
// frames/NNNNNN.pcd (body-frame clouds).  The output has the same layout but
// is translated into the current base frame (--translate = new − old mouth
// coordinates), cropped to the cave mouth region (--crop, radius <= 0 keeps
// everything) and resampled in trajectory order (--spacing).
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "simple_loop_closure/prior_map.hpp"

namespace fs = std::filesystem;

namespace {

void printUsage() {
  std::cerr << "usage: make_prior_map --src DIR --out DIR [--translate DX DY]"
               " [--crop CX CY R] [--spacing S]\n";
}

std::string frameName(size_t index) {
  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(6) << index << ".pcd";
  return oss.str();
}

}  // namespace

int main(int argc, char** argv) {
  std::string src;
  std::string out;
  double dx = 0.0;
  double dy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double radius = 0.0;
  double spacing = 0.5;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto take = [&](double& v) -> bool {
      if (i + 1 >= argc) return false;
      v = std::atof(argv[++i]);
      return true;
    };
    if (arg == "--src" && i + 1 < argc) {
      src = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      out = argv[++i];
    } else if (arg == "--translate" && i + 2 < argc) {
      if (!take(dx) || !take(dy)) { printUsage(); return 2; }
    } else if (arg == "--crop" && i + 3 < argc) {
      if (!take(cx) || !take(cy) || !take(radius)) { printUsage(); return 2; }
    } else if (arg == "--spacing") {
      if (!take(spacing)) { printUsage(); return 2; }
    } else {
      printUsage();
      return 2;
    }
  }

  if (src.empty() || out.empty()) {
    printUsage();
    return 2;
  }

  std::ifstream csv((fs::path(src) / "poses.csv").string());
  if (!csv) {
    std::cerr << "cannot open " << (fs::path(src) / "poses.csv") << "\n";
    return 1;
  }

  std::string first_line;
  std::getline(csv, first_line);
  if (!simple_loop_closure::priorHeaderIsBodyFrame(first_line)) {
    std::cerr << "refusing: poses.csv does not declare '# frame: body' "
                 "(saved with mapped_cloud=false?)\n";
    return 1;
  }

  std::vector<simple_loop_closure::PriorPose> poses;
  if (!simple_loop_closure::parsePriorPoses(csv, poses)) {
    std::cerr << "no parsable poses in poses.csv\n";
    return 1;
  }

  const std::vector<size_t> kept = simple_loop_closure::selectPriorPoses(
      poses, dx, dy, cx, cy, radius, spacing);
  if (kept.empty()) {
    std::cerr << "selection is empty (crop centre/radius too tight?)\n";
    return 1;
  }

  const fs::path out_frames = fs::path(out) / "frames";
  fs::create_directories(out_frames);

  std::ofstream out_csv((fs::path(out) / "poses.csv").string());
  if (!out_csv) {
    std::cerr << "cannot write " << (fs::path(out) / "poses.csv") << "\n";
    return 1;
  }
  out_csv << "# frame: body\n";
  out_csv << "index, timestamp, x, y, z, qx, qy, qz, qw\n";

  size_t written = 0;
  for (size_t k = 0; k < kept.size(); ++k) {
    const auto& p = poses[kept[k]];
    const fs::path in_pcd = fs::path(src) / "frames" / frameName(p.index);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(in_pcd.string(), cloud) != 0) {
      std::cerr << "skip missing/unreadable " << in_pcd << "\n";
      continue;
    }
    const fs::path out_pcd = out_frames / frameName(written);
    if (pcl::io::savePCDFileBinary(out_pcd.string(), cloud) != 0) {
      std::cerr << "cannot write " << out_pcd << "\n";
      return 1;
    }
    out_csv << written << ", " << std::fixed << p.stamp << ", "
            << (p.t.x() + dx) << ", " << (p.t.y() + dy) << ", " << p.t.z()
            << ", " << p.q.x() << ", " << p.q.y() << ", " << p.q.z() << ", "
            << p.q.w() << "\n";
    ++written;
  }

  if (written == 0) {
    std::cerr << "no frames written\n";
    return 1;
  }

  std::cout << "prior map written: " << out << "  keyframes=" << written
            << " (selected " << kept.size() << " of " << poses.size() << ")\n";
  return 0;
}
