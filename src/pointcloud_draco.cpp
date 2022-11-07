#include <pointcloud_compressor.hpp>

#include <draco/compression/decode.h>
#include <draco/compression/encode.h>
#include <draco/compression/expert_encode.h>
#include <draco/point_cloud/point_cloud_builder.h>


std::shared_ptr<std::vector<char>> PointCloudCompressorNode::compress_draco(std::shared_ptr<std::vector<Eigen::Vector4f>>& points) {
  draco::PointCloudBuilder builder;
  builder.Start(points->size());

  builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
  builder.AddAttribute(draco::GeometryAttribute::WEIGHTS, 1, draco::DT_FLOAT32);
  builder.SetAttributeValuesForAllPoints(0, points->front().data(), sizeof(Eigen::Vector4f));
  builder.SetAttributeValuesForAllPoints(1, points->front().data() + 3, sizeof(Eigen::Vector4f));

  auto pc = builder.Finalize(true);
  if (pc == nullptr) {
    std::cerr << "error: failed to finalize point cloud for draco" << std::endl;
    return nullptr;
  }

  draco::EncoderBuffer encode_buffer;

  draco::Encoder encoder;
  encoder.SetSpeedOptions(encoding_speed, decoding_speed);
  encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, pos_quantization_bits);
  encoder.SetAttributeQuantization(draco::GeometryAttribute::WEIGHTS, time_quantization_bits);
  // encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);

  draco::Status status = encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);
  if (status.code() != 0) {
    std::cerr << "error: failed to encode point cloud" << std::endl;
    return nullptr;
  }

  return std::make_shared<std::vector<char>>(encode_buffer.data(), encode_buffer.data() + encode_buffer.size());
}

void PointCloudCompressorNode::test(const std::vector<Eigen::Vector4f>& points, const std::vector<char>& bytes) {
  draco::DecoderBuffer buffer;
  buffer.Init(bytes.data(), bytes.size());

  draco::Decoder decoder;
  auto pc = decoder.DecodePointCloudFromBuffer(&buffer).value();
  auto pos_attribute = pc->attribute(0);
  auto time_attribute = pc->attribute(1);

  Eigen::Vector4d sum_errors = Eigen::Vector4d::Zero();
  Eigen::Vector4d max_errors = Eigen::Vector4d::Zero();

  std::vector<Eigen::Vector4f> decoded(pc->num_points());
  for (int i = 0; i < pc->num_points(); i++) {
    pos_attribute->GetValue(draco::AttributeValueIndex(i), decoded.data() + i);
    time_attribute->GetValue(draco::AttributeValueIndex(i), (decoded.data() + i)->data() + 3);

    Eigen::Vector4d error = (points[i] - decoded[i]).cast<double>();
    sum_errors += error;
    max_errors = max_errors.array().max(error.array());
  }

  std::cout << "max :" << max_errors.transpose() << std::endl;
  std::cout << "mean:" << sum_errors.transpose() / pc->num_points() << std::endl;
}