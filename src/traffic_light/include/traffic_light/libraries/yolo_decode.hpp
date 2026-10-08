#pragma once
#include <traffic_light/libraries/lamp_state.hpp>
#include <opencv2/core.hpp>
#include <cstddef>
#include <string>
#include <vector>

namespace traffic_light {
// 모델 출력 클래스 하나다. 클래스 인덱스는 YoloTrafficLightParameters::classes의 위치다.
struct LampClass {
  std::string name;
  Lamp lamp = Lamp::Red;
  float confidence = 0.5F;
};

// YOLO Detect 출력 [1, channels, count](channels_first) 또는 [1, count, channels]의 값이다.
// channels = 4(cx, cy, w, h; 입력 픽셀) + 클래스 수다. 추론 결과가 없으면 count = 0이다.
struct YoloOutput {
  std::vector<float> values;
  std::size_t count = 0;
  std::size_t channels = 0;
  bool channels_first = true;
};

struct Detection {
  cv::Rect2f box;
  float confidence = 0.0F;
  std::size_t class_index = 0;
};

// BGR 영상을 비율을 유지해 input 크기로 줄이고 가운데에 놓아 114로 채운 뒤 RGB float32 [0, 1] NCHW로 반환한다.
std::vector<float> letterboxTensor(const cv::Mat& bgr, int input_width, int input_height);

// 후보마다 최고 점수 클래스를 고르고, 그 클래스의 confidence 미만과 크기가 양수가 아닌 상자를 버린 뒤
// 클래스별 NMS를 적용한다. 결과는 confidence 내림차순이다.
std::vector<Detection> decodeYoloOutput(const YoloOutput& output, const std::vector<LampClass>& classes,
                                        float nms_iou);

// 검출된 클래스가 뜻하는 점등 등의 합집합 비트(lampBit)다.
unsigned lampMask(const std::vector<Detection>& detections, const std::vector<LampClass>& classes);
}  // namespace traffic_light
