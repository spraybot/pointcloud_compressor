#include <pointcloud_compressor.hpp>

#include <iostream>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


PointCloudCompressorNode::PointCloudCompressorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("pointcloud_compressor", options) {
  downsample_resolution = this->get_parameter_or<double>("downsample_resolution", 0.0);
  encoding_speed = this->get_parameter_or<int>("encoding_speed", 5);
  decoding_speed = this->get_parameter_or<int>("decoding_speed", 2);
  pos_quantization_bits = this->get_parameter_or<int>("pos_quantization_bits", 11);
  time_quantization_bits = this->get_parameter_or<int>("time_quantization_bits", 8);

  std::cout << "downsample_resolution :" << downsample_resolution << std::endl;
  std::cout << "encoding_speed        :" << encoding_speed << std::endl;
  std::cout << "decoding_speed        :" << decoding_speed << std::endl;
  std::cout << "pos_quantization_bits :" << pos_quantization_bits << std::endl;
  std::cout << "time_quantization_bits:" << time_quantization_bits << std::endl;

  points_filtered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points/filtered", 10);
  points_compressed_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points/compressed", 10);
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { points_callback(msg); });
}

PointCloudCompressorNode::~PointCloudCompressorNode() {}

void PointCloudCompressorNode::points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& points_msg) {
  if(points_filtered_pub->get_subscription_count() == 0 && points_compressed_pub->get_subscription_count() == 0) {
    // return;
  }

  auto points = extract(points_msg);
  points = downsample(points);

  sensor_msgs::msg::PointCloud2::SharedPtr compressed_msg(new sensor_msgs::msg::PointCloud2);
  compressed_msg->header = points_msg->header;
  compressed_msg->width = points->size();
  compressed_msg->height = 1;

  compressed_msg->fields.resize(4);
  compressed_msg->fields[0].name = "x";
  compressed_msg->fields[0].offset = 0;
  compressed_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  compressed_msg->fields[0].count = 1;

  compressed_msg->fields[1].name = "y";
  compressed_msg->fields[1].offset = 4;
  compressed_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  compressed_msg->fields[1].count = 1;

  compressed_msg->fields[2].name = "z";
  compressed_msg->fields[2].offset = 8;
  compressed_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  compressed_msg->fields[2].count = 1;

  compressed_msg->fields[3].name = "time";
  compressed_msg->fields[3].offset = 12;
  compressed_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  compressed_msg->fields[3].count = 1;

  compressed_msg->is_dense = true;
  compressed_msg->is_bigendian = false;
  compressed_msg->point_step = sizeof(float) * 4;
  compressed_msg->row_step = compressed_msg->point_step * points->size();

  if(points_filtered_pub->get_subscription_count()) {
    compressed_msg->data.resize(sizeof(float) * 4 * points->size());
    memcpy(compressed_msg->data.data(), points->data(), sizeof(float) * 4 * points->size());
    points_filtered_pub->publish(*compressed_msg);
    std::cout << boost::format("raw:%d filtered  :%d compression_rate:%.3f") % points_msg->data.size() % compressed_msg->data.size() %
                   (static_cast<double>(compressed_msg->data.size()) / points_msg->data.size())
              << std::endl;
  }

  if (points_compressed_pub->get_subscription_count()) {
    auto bytes = compress_draco(points);
    compressed_msg->data.clear();
    compressed_msg->data.assign(bytes->begin(), bytes->end());
    points_compressed_pub->publish(*compressed_msg);
    std::cout << boost::format("raw:%d compressed:%d compression_rate:%.3f") % points_msg->data.size() % compressed_msg->data.size() %
                   (static_cast<double>(compressed_msg->data.size()) / points_msg->data.size())
              << std::endl;

    test(*points, *bytes);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PointCloudCompressorNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}