#include <pointcloud_compressor.hpp>

#include <unordered_map>

class XORVector3iHash {
public:
  size_t operator()(const Eigen::Vector3i& x) const {
    const size_t p1 = 73856093;
    const size_t p2 = 19349669;  // 19349663 was not a prime number
    const size_t p3 = 83492791;
    return static_cast<size_t>((x[0] * p1) ^ (x[1] * p2) ^ (x[2] * p3));
  }
};

std::shared_ptr<std::vector<Eigen::Vector4f>> PointCloudCompressorNode::downsample(std::shared_ptr<std::vector<Eigen::Vector4f>>& points) {
  if (downsample_resolution < 1e-6) {
    return points;
  }

  const double inv_resolution = 1.0 / downsample_resolution;

  std::unordered_map<Eigen::Vector3i, Eigen::Vector4f, XORVector3iHash> voxels;
  for (int i = 0; i < points->size(); i++) {
    const Eigen::Vector3i coord = (points->at(i) * inv_resolution).array().floor().cast<int>().head<3>();
    voxels[coord] = points->at(i);
  }

  points->clear();
  for (const auto& voxel : voxels) {
    points->emplace_back(voxel.second);
  }

  return points;
}