#pragma once
#include <traffic_light/libraries/yolo_decode.hpp>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace traffic_light {
enum class InferenceDevice { Cuda, Cpu };

// ONNX Runtime 세션 하나를 소유한다. 생성 시 지정한 장치로만 세션을 만들고 다른 장치로 바꾸지 않는다.
class YoloSession {
 public:
  // model은 ONNX 바이트다. 세션 생성이 실패하면 ROS ERROR를 남기고 ready()=false가 된다.
  // 생성된 모델의 입력이 [1,3,input_height,input_width] float32 하나가 아니거나 출력이
  // [1,channels,N] 또는 [1,N,channels] float32 하나가 아니면 std::invalid_argument를 던진다.
  YoloSession(const std::string& model, InferenceDevice device, int input_width, int input_height,
              std::size_t channels);
  ~YoloSession();
  YoloSession(YoloSession&&) noexcept;
  YoloSession& operator=(YoloSession&&) noexcept;
  YoloSession(const YoloSession&) = delete;
  YoloSession& operator=(const YoloSession&) = delete;

  bool ready() const;
  // input은 [1,3,input_height,input_width] RGB float32다. 추론 오류는 ROS ERROR로 남기고 count=0을 반환한다.
  YoloOutput run(std::vector<float>& input);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace traffic_light
