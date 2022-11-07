#include <pointcloud_compressor.hpp>

#define GLIM_ROS2
#include <glim/util/ros_cloud_converter.hpp>

std::shared_ptr<std::vector<Eigen::Vector4f>> PointCloudCompressorNode::extract(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points_msg) {
  const auto raw_points = glim::extract_raw_points(points_msg);

  auto points = std::make_shared<std::vector<Eigen::Vector4f>>();
  points->reserve(raw_points->size());

  for (int i = 0; i < raw_points->size(); i++) {
    const auto& pt = raw_points->points[i];
    const double time = raw_points->times[i];

    if (pt.x() < 0.0 || pt.head<3>().squaredNorm() < 1.0) {
      continue;
    }

    points->emplace_back(pt.x(), pt.y(), pt.z(), time);
  }

  return points;
}