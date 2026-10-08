#pragma once
#include <traffic_light/libraries/lamp_state.hpp>
#include <traffic_light/libraries/yolo_decode.hpp>
#include <traffic_light/libraries/yolo_session.hpp>
#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/console.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace traffic_light {
struct YoloTrafficLightParameters {
  // ONNX 모델 바이트다.
  std::string model;
  InferenceDevice device = InferenceDevice::Cuda;
  int input_width = 1280;
  int input_height = 1280;
  // 모델 출력 클래스 인덱스 순서다.
  std::vector<LampClass> classes;
  float nms_iou = 0.45F;
  std::size_t history_size = 5;
  std::size_t minimum_votes = 3;
};

// camera_7 압축 영상을 YOLO Detect 모델로 추론한다. 검출된 클래스가 뜻하는 점등 등의 조합을
// 프레임 상태로 정하고, 최근 프레임 투표 결과를 TrafficLight.state로 반환한다.
// 세션이 없으면(생성 실패) UNKNOWN을 반환한다.
class YoloTrafficLight {
 public:
  explicit YoloTrafficLight(YoloTrafficLightParameters parameters)
      : session_(parameters.model, parameters.device, parameters.input_width, parameters.input_height,
                 4 + parameters.classes.size()),
        input_width_(parameters.input_width),
        input_height_(parameters.input_height),
        classes_(std::move(parameters.classes)),
        nms_iou_(parameters.nms_iou),
        voter_(parameters.history_size, parameters.minimum_votes) {}

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& image) {
    interfaces::TrafficLight light;
    light.state = interfaces::TrafficLight::UNKNOWN;
    if (!session_.ready()) return light;
    std::vector<float> tensor = letterboxTensor(cv::imdecode(image.data, cv::IMREAD_COLOR), input_width_,
                                                input_height_);
    const auto detections = decodeYoloOutput(session_.run(tensor), classes_, nms_iou_);
    const auto frame_state = combinationState(lampMask(detections, classes_));
    light.state = voter_.push(frame_state);
    ROS_DEBUG("traffic_light: %zu detections, frame state %u, voted state %u", detections.size(),
              static_cast<unsigned>(frame_state), static_cast<unsigned>(light.state));
    return light;
  }

 private:
  YoloSession session_;
  int input_width_;
  int input_height_;
  std::vector<LampClass> classes_;
  float nms_iou_;
  StateVoter voter_;
};
}  // namespace traffic_light
