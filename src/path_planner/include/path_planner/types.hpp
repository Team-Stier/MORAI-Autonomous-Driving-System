#pragma once
#include <node_pipeline/components.hpp>
#include <lanelet2_core/primitives/Primitive.h>
#include <nav_msgs/Odometry.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Objects.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <tf2_msgs/TFMessage.h>
#include <array>
#include <memory>
namespace lanelet { class LaneletMap; }
namespace path_planner {
using Data = node_pipeline::Data<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects, tf2_msgs::TFMessage, interfaces::SectionAnnotation>;
using Message = interfaces::Path;
struct Checkpoint {
  double x_m = 0;
  double y_m = 0;
  double z_m = 0;
  double yaw_rad = 0;
  lanelet::Id lanelet_id = 0;
};
struct GlobalPathParameters {
  std::shared_ptr<lanelet::LaneletMap> map;
  std::array<Checkpoint, 16> checkpoints{};
};
struct LocalPathParameters {
  std::shared_ptr<lanelet::LaneletMap> map;
  double local_path_length_m = 20.0;
};
}  // namespace path_planner
