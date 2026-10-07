#pragma once
#include <node_pipeline/components.hpp>
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
namespace lanelet { class LaneletMap; }
namespace path_planner {
// #DOWNSTREAM_CONTRACT
// 원본 Objects와 PredictedObjects는 perception이 실제 계산한 base_link_level을
// 기대한다. 차체 base_link와 원점·yaw는 같지만 roll/pitch가 0인 별도 frame이다.
// ROS 노드의 원본·예측 frame 검사에 이 값을 함께 사용하며, 필요 없이 객체를
// 역회전하거나 이름만 base_link로 바꾸지 않는다. 다른 좌표가 필요한 알고리즘은
// 자신의 입력 경계에서 관측 stamp에 맞는 실제 위치·방향·벡터 변환을 해야 한다.
constexpr const char* kObjectsFrame = "base_link_level";
using Data = node_pipeline::Data<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus, interfaces::Objects, tf2_msgs::TFMessage, interfaces::SectionAnnotation>;
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
struct GlobalPathParameters {
  std::shared_ptr<lanelet::LaneletMap> map;
  std::array<Checkpoint, 16> checkpoints{};
};
struct LocalPathParameters {
  std::shared_ptr<lanelet::LaneletMap> map;
  double local_path_length_m = 20.0;
};
}  // namespace path_planner
