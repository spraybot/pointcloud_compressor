#pragma once

#include <vector>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudCompressorNode : public rclcpp::Node {
public:
  PointCloudCompressorNode(const rclcpp::NodeOptions& options);
  ~PointCloudCompressorNode();

private:
  std::shared_ptr<std::vector<Eigen::Vector4f>> extract(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& points_msg);
  std::shared_ptr<std::vector<Eigen::Vector4f>> downsample(std::shared_ptr<std::vector<Eigen::Vector4f>>& points);
  std::shared_ptr<std::vector<char>> compress_draco(std::shared_ptr<std::vector<Eigen::Vector4f>>& points);

  void test(const std::vector<Eigen::Vector4f>& points, const std::vector<char>& bytes);

  void points_callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& points_msg);

private:
  double downsample_resolution;
  int encoding_speed;
  int decoding_speed;
  int pos_quantization_bits;
  int time_quantization_bits;

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> points_sub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_filtered_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_compressed_pub;
};