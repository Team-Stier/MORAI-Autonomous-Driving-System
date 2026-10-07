#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <nav_msgs/Odometry.h>
#include <interfaces/SectionAnnotation.h>
namespace section_annotator {
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
