#include <traffic_light/libraries/yolo_decode.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace traffic_light {
std::vector<float> letterboxTensor(const cv::Mat& bgr, int input_width, int input_height) {
  const double scale = std::min(input_width / static_cast<double>(bgr.cols),
                                input_height / static_cast<double>(bgr.rows));
  const int resized_width = std::max(1, std::min(input_width, static_cast<int>(std::round(bgr.cols * scale))));
  const int resized_height = std::max(1, std::min(input_height, static_cast<int>(std::round(bgr.rows * scale))));
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(resized_width, resized_height));
  const int left = (input_width - resized_width) / 2;
  const int top = (input_height - resized_height) / 2;
  cv::Mat padded;
  cv::copyMakeBorder(resized, padded, top, input_height - resized_height - top, left,
                     input_width - resized_width - left, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  cv::Mat scaled;
  padded.convertTo(scaled, CV_32F, 1.0 / 255.0);
  const std::size_t plane = static_cast<std::size_t>(input_width) * static_cast<std::size_t>(input_height);
  std::vector<float> tensor(3 * plane);
  // split은 BGR 순서로 쓰므로 B·G·R 평면을 텐서의 2·1·0 채널에 직접 둔다.
  cv::Mat planes[3] = {cv::Mat(input_height, input_width, CV_32F, tensor.data() + 2 * plane),
                       cv::Mat(input_height, input_width, CV_32F, tensor.data() + plane),
                       cv::Mat(input_height, input_width, CV_32F, tensor.data())};
  cv::split(scaled, planes);
  return tensor;
}

std::vector<Detection> decodeYoloOutput(const YoloOutput& output, const std::vector<LampClass>& classes,
                                        float nms_iou) {
  const std::size_t class_count = std::min(classes.size(), output.channels >= 4 ? output.channels - 4 : 0);
  const std::size_t count = output.values.size() >= output.channels * output.count ? output.count : 0;
  const auto value = [&](std::size_t row, std::size_t channel) {
    return output.channels_first ? output.values[channel * output.count + row]
                                 : output.values[row * output.channels + channel];
  };
  std::vector<std::vector<cv::Rect2d>> boxes(class_count);
  std::vector<std::vector<float>> scores(class_count);
  for (std::size_t row = 0; row < count && class_count > 0; ++row) {
    std::size_t best = 0;
    float best_score = value(row, 4);
    for (std::size_t index = 1; index < class_count; ++index) {
      const float score = value(row, 4 + index);
      if (score > best_score) {
        best = index;
        best_score = score;
      }
    }
    const float cx = value(row, 0), cy = value(row, 1), width = value(row, 2), height = value(row, 3);
    if (!(best_score >= classes[best].confidence) || !(width > 0) || !(height > 0) ||
        !std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) || !std::isfinite(height))
      continue;
    boxes[best].emplace_back(cx - width / 2, cy - height / 2, width, height);
    scores[best].push_back(best_score);
  }
  std::vector<Detection> detections;
  for (std::size_t index = 0; index < class_count; ++index) {
    if (boxes[index].empty()) continue;
    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes[index], scores[index], classes[index].confidence, nms_iou, kept);
    for (const int candidate : kept) {
      const auto& box = boxes[index][static_cast<std::size_t>(candidate)];
      detections.push_back({cv::Rect2f(static_cast<float>(box.x), static_cast<float>(box.y),
                                       static_cast<float>(box.width), static_cast<float>(box.height)),
                            scores[index][static_cast<std::size_t>(candidate)], index});
    }
  }
  std::stable_sort(detections.begin(), detections.end(),
                   [](const Detection& a, const Detection& b) { return a.confidence > b.confidence; });
  return detections;
}

unsigned lampMask(const std::vector<Detection>& detections, const std::vector<LampClass>& classes) {
  unsigned lamps = 0;
  for (const auto& detection : detections)
    if (detection.class_index < classes.size()) lamps |= lampBit(classes[detection.class_index].lamp);
  return lamps;
}
}  // namespace traffic_light
