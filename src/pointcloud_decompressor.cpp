#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

class PointCloudDecompressorNode : public rclcpp::Node {
public:
  PointCloudDecompressorNode(const rclcpp::NodeOptions& options) : rclcpp::Node("pointcloud_decompressor", options) {
    points_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points/decompressed", 10);
    points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points/compressed", 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { points_callback(msg); });
  }
  ~PointCloudDecompressorNode() {}

private:
  void points_callback(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& points_msg) {
    draco::DecoderBuffer buffer;
    buffer.Init(reinterpret_cast<char*>(points_msg->data.data()), points_msg->data.size());

    draco::Decoder decoder;
    auto pc = decoder.DecodePointCloudFromBuffer(&buffer).value();

    if (pc->num_points() != points_msg->width * points_msg->height) {
      std::cerr << "error: num points do not match!! " << pc->num_points() << " " << points_msg->width * points_msg->height << std::endl;
      return;
    }
    if(pc->num_attributes() != 2) {
      std::cerr << "error: num attributes must be 1!!  " << pc->num_attributes() << std::endl;
      return;
    }

    auto pos_attribute = pc->attribute(0);
    auto time_attribute = pc->attribute(1);

    if(pos_attribute->byte_stride() != sizeof(float) * 3 || time_attribute->byte_stride() != sizeof(float)) {
      std::cerr << "error: size of attribute stride error!!" << std::endl;
    }

    points_msg->data.resize(sizeof(float) * 4 * pc->num_points());
    for (int i = 0; i < pc->num_points(); i++) {
      pos_attribute->GetValue(draco::AttributeValueIndex(i), points_msg->data.data() + sizeof(float) * 4 * i);
      time_attribute->GetValue(draco::AttributeValueIndex(i), points_msg->data.data() + sizeof(float) * 4 * i + sizeof(float) * 3);
    }

    points_pub->publish(*points_msg);
  }

private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> points_sub;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<PointCloudDecompressorNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}