#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/SectionAnnotation.h>
#include <interfaces/TrafficLight.h>
#include <sensor_msgs/CompressedImage.h>
namespace traffic_light {
using Data = node_pipeline::Data<sensor_msgs::CompressedImage>;
using Message = interfaces::TrafficLight;
}  // namespace traffic_light
