#pragma once

#include <traffic_light/detection.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <memory>

namespace traffic_light {

class YoloTrafficLightCore {
 public:
  explicit YoloTrafficLightCore(TrafficLightParameters parameters);
  ~YoloTrafficLightCore();
  YoloTrafficLightCore(YoloTrafficLightCore&&) noexcept;
  YoloTrafficLightCore& operator=(YoloTrafficLightCore&&) noexcept;
  YoloTrafficLightCore(const YoloTrafficLightCore&) = delete;
  YoloTrafficLightCore& operator=(const YoloTrafficLightCore&) = delete;

  interfaces::TrafficLight process(const sensor_msgs::CompressedImage& message);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace traffic_light
