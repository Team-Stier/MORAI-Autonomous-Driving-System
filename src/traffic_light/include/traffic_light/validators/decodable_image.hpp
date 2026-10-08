#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

namespace traffic_light {
// Core가 전제하는 압축 영상을 실제로 BGR 영상으로 디코딩해 본다. 디코딩할 수 없으면 Missing이다.
struct DecodableImage {
  void operator()(const sensor_msgs::CompressedImage& image, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    bool decodable = false;
    if (!image.data.empty()) {
      try {
        decodable = !cv::imdecode(image.data, cv::IMREAD_COLOR).empty();
      } catch (const cv::Exception&) {
        decodable = false;
      }
    }
    if (!decodable) issues.push_back(node_pipeline::Kind::Missing);
  }
};
}  // namespace traffic_light
