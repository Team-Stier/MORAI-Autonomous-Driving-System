#pragma once
#include <lanelet2_core/primitives/Primitive.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <nav_msgs/Odometry.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Objects.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <tf2_msgs/TFMessage.h>
#include <array>
#include <memory>
#include <vector>
namespace lanelet { class LaneletMap; }
namespace path_planner {
using Message = interfaces::Path;
using Route = lanelet::Optional<lanelet::routing::LaneletPath>;
using PredictedObjects = std::array<interfaces::Objects, 10>;
struct Checkpoint {
  double x_m = 0;
  double y_m = 0;
  double z_m = 0;
  double yaw_rad = 0;
  lanelet::Id lanelet_id = 0;
};
// 아직 경유하지 않은 체크포인트를 경유 순서대로 담는다. 마지막 원소가 목적지다.
using Checkpoints = std::vector<Checkpoint>;
}  // namespace path_planner
