#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgcodecs.hpp>
#include <vector>

namespace traffic_light {
struct DecodableImage {
  void operator()(const sensor_msgs::CompressedImage& image, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    try {
      if (cv::imdecode(image.data, cv::IMREAD_COLOR).empty())
        issues.push_back(node_pipeline::Kind::Missing);
    } catch (const cv::Exception&) {
      issues.push_back(node_pipeline::Kind::Missing);
    }
  }
};
}  // namespace traffic_light
