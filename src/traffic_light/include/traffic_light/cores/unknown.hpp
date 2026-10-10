#pragma once

#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>

namespace traffic_light {

class Unknown {
 public:
  interfaces::TrafficLight process(const sensor_msgs::CompressedImage&) {
    interfaces::TrafficLight output;
    output.state = interfaces::TrafficLight::UNKNOWN;
    return output;
  }
};

}  // namespace traffic_light
