#pragma once
#include <node_pipeline/components.hpp>
#include <nav_msgs/Odometry.h>
#include <interfaces/SectionAnnotation.h>
namespace section_annotator {
using Data = node_pipeline::Data<nav_msgs::Odometry>;
using Message = interfaces::SectionAnnotation;
struct Checkpoint {
  double x_m = 0;
  double y_m = 0;
};
struct CheckpointParameters {
  std::array<Checkpoint, 16> checkpoints{};
  double radius_m = 10.0;
};
}  // namespace section_annotator
