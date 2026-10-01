#pragma once
#include <node_pipeline/components.hpp>
#include <interfaces/Objects.h>
#include <interfaces/SectionAnnotation.h>
#include <sensor_msgs/PointCloud2.h>
namespace object_detector {
using Data = node_pipeline::Data<sensor_msgs::PointCloud2>;
using Message = interfaces::Objects;
}  // namespace object_detector
