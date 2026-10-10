#include <path_planner/libraries/map_parameters.hpp>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/core_path_planner.hpp>
#include <path_planner/cores/ego_compensated_prediction.hpp>
#include <path_planner/cores/local_path.hpp>
#include <path_planner/cores/prediction_pass_through.hpp>
#include <path_planner/parameters/local_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/validators/route_present.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <type_traits>

namespace pp = path_planner;
namespace {
std::string readFile(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open " + path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

double lengthM(const interfaces::Path& path) {
  double length_m = 0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& a = path.points[i - 1].pose.pose.position;
    const auto& b = path.points[i].pose.pose.position;
    length_m += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length_m;
}

void expectPayload(const interfaces::Path& path) {
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(lengthM(path), 20.0, 1e-6);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
  for (const auto& point : path.points) {
    const auto& q = point.pose.pose.orientation;
    EXPECT_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, 1e-9);
    EXPECT_TRUE(std::isfinite(point.pose.pose.position.z));
    EXPECT_TRUE(point.pose.header.stamp.isZero());
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    EXPECT_TRUE(std::isnan(point.velocity));
    EXPECT_DOUBLE_EQ(point.acceleration, 0.0);
  }
}

pp::SharedMap shortMap() {
  lanelet::Lanelets lanes;
  std::vector<lanelet::Point3d> left, right;
  for (int i = 0; i <= 10; ++i) {
    left.emplace_back(100 + 2*i, 5.0*i, 1, 10 + 0.5*i);
    right.emplace_back(101 + 2*i, 5.0*i, -1, 10 + 0.5*i);
  }
  for (int i = 0; i < 10; ++i) {
    lanelet::Lanelet lane(300 + i,
        lanelet::LineString3d(200 + 2*i, {left[i], left[i+1]}),
        lanelet::LineString3d(201 + 2*i, {right[i], right[i+1]}));
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    lanes.push_back(lane);
  }
  return pp::SharedMap({lanelet::utils::createMap(lanes)});
}

pp::Checkpoints shortCheckpoints() {
  pp::Checkpoints checkpoints(16);
  for (auto& checkpoint : checkpoints) {
    checkpoint.x_m = 1;
    checkpoint.lanelet_id = 300;
  }
  return checkpoints;
}

template<class Container> pp::Checkpoints remaining(const Container& checkpoints, std::size_t completed) {
  return pp::Checkpoints(checkpoints.begin() + completed, checkpoints.end());
}
}  // namespace

TEST(PredictionPassThrough, CopiesAllTenSamplesWithoutChangingObservation) {
  interfaces::Objects objects;
  objects.header.stamp.fromSec(10.0);
  objects.header.frame_id = "base_link";
  interfaces::Object object;
  object.id = 42;
  object.label = "vehicle";
  object.position.x = 3.0;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1.0;
  object.velocity.x = 2.0;
  object.velocity_available = true;
  for (std::uint32_t i = 0; i < 12; ++i) {
    object.id = 42 + i;
    objects.objects.push_back(object);
  }
  pp::PredictionPassThrough core;
  auto samples = std::get<6>(core.process(nav_msgs::Odometry{}, interfaces::CompetitionVehicleStatus{},
      objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  ASSERT_EQ(samples.size(), 10u);
  for (const auto& sample : samples) {
    ASSERT_EQ(sample.objects.size(), 12u);
    EXPECT_EQ(sample, objects);
  }
  samples[0].objects[0].position.x = 100.0;
  EXPECT_DOUBLE_EQ(objects.objects[0].position.x, 3.0);
  EXPECT_DOUBLE_EQ(samples[1].objects[0].position.x, 3.0);
}

TEST(PredictionPassThrough, PreservesEmptyObjectLists) {
  interfaces::Objects objects;
  objects.header.frame_id = "base_link";
  pp::PredictionPassThrough core;
  const auto samples = std::get<6>(core.process(nav_msgs::Odometry{}, interfaces::CompetitionVehicleStatus{},
      objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  for (const auto& sample : samples) EXPECT_EQ(sample, objects);
}

TEST(PathCores, ReturnsRouteWithPassThroughAndExtendsShortRoute) {
  auto shared = shortMap();
  const auto checkpoints = shortCheckpoints();
  pp::GlobalPath global(shared);
  pp::LocalPath local({20.0}, shared);
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(1);
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 2;
  ego.pose.pose.orientation.w = 1;
  interfaces::Objects objects;
  objects.header.frame_id = "base_link";
  interfaces::CompetitionVehicleStatus competition;
  competition.gear = 4;
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().child_frame_id = "base_link";
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  for (std::size_t completed : {0, 1, 1, 16, 0}) {
    const auto result = global.process(ego, competition, objects, tf, imu, remaining(checkpoints, completed));
    EXPECT_EQ(std::get<0>(result), ego);
    EXPECT_EQ(std::get<1>(result), competition);
    EXPECT_EQ(std::get<2>(result), objects);
    EXPECT_EQ(std::get<3>(result), tf);
    EXPECT_EQ(std::get<4>(result), imu);
    const auto& route = std::get<5>(result);
    ASSERT_TRUE(static_cast<bool>(route));
    ASSERT_EQ(route->size(), 1u);
    const auto path = local.process(ego, competition, objects, tf, imu, route, pp::PredictedObjects{});
    ASSERT_GE(path.points.size(), 2u);
    expectPayload(path);
    EXPECT_NEAR(path.points.front().pose.pose.position.x, 2.0, 1e-9);
    EXPECT_NEAR(path.points.back().pose.pose.position.x, 22.0, 1e-9);
    EXPECT_NEAR(path.points.front().pose.pose.position.z, 10.2, 1e-9);
    EXPECT_NEAR(path.points.back().pose.pose.position.z, 12.2, 1e-9);
    for (std::size_t i = 1; i < path.points.size(); ++i)
      EXPECT_GT(path.points[i].pose.pose.position.x, path.points[i - 1].pose.pose.position.x);
  }
}

TEST(PathCores, UsesPinnedLaneletInsteadOfRematchingCheckpointCoordinates) {
  auto shared = shortMap();
  auto checkpoints = shortCheckpoints();
  checkpoints.back().lanelet_id = 301;
  pp::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 2;
  ego.pose.pose.orientation.w = 1;
  const auto route = std::get<5>(global.process(ego, interfaces::CompetitionVehicleStatus{}, interfaces::Objects{}, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, checkpoints));
  ASSERT_TRUE(static_cast<bool>(route));
  ASSERT_EQ(route->size(), 2u);
  EXPECT_EQ(route->back().id(), 301);
}

TEST(PathCores, RejectsInvalidCheckpointLaneletIds) {
  const auto map = readFile(std::string(NP_PACKAGE_DIR "/../map") + "/kcity_lanelet2.osm");
  for (const std::string id : {"", "0", "-1", "100070.0", "100070x", "9223372036854775808", "9999999999"}) {
    std::string csv = "x,y,z,yaw,lanelet_id\n";
    for (int i = 0; i < 16; ++i) csv += "0,0,0,0," + id + "\n";
    EXPECT_ANY_THROW(pp::readCheckedLaneletMap(map, csv)) << id;
  }
}

TEST(PathCores, CoresFollowRemainingMapCheckpoints) {
  const auto csv = readFile(std::string(NP_PACKAGE_DIR "/../map") + "/checkpoints.csv");
  pp::SharedMap shared({pp::readCheckedLaneletMap(
      readFile(std::string(NP_PACKAGE_DIR "/../map") + "/kcity_lanelet2.osm"), csv)});
  const auto checkpoints = pp::readCheckpoints(csv);
  std::size_t completed = 0;
  pp::GlobalPath global(shared);
  pp::PredictionPassThrough prediction;
  pp::LocalPath local({20.0}, shared);
  auto ego = std::make_shared<nav_msgs::Odometry>();
  auto objects = std::make_shared<interfaces::Objects>();
  objects->header.stamp.fromSec(1.0);
  objects->header.frame_id = "base_link";
  ego->pose.pose.position.x = checkpoints.front().x_m;
  ego->pose.pose.position.y = checkpoints.front().y_m;
  ego->pose.pose.orientation.z = std::sin(checkpoints.front().yaw_rad / 2);
  ego->pose.pose.orientation.w = std::cos(checkpoints.front().yaw_rad / 2);
  const auto run = [&] {
    const auto route = std::get<5>(global.process(*ego, interfaces::CompetitionVehicleStatus{}, *objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, remaining(checkpoints, completed)));
    const auto predictions = std::get<6>(prediction.process(*ego, interfaces::CompetitionVehicleStatus{}, *objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, route));
    for (const auto& sample : predictions) EXPECT_EQ(sample, *objects);
    return local.process(*ego, interfaces::CompetitionVehicleStatus{}, *objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, route, predictions);
  };
  const auto route = std::get<5>(global.process(*ego, interfaces::CompetitionVehicleStatus{}, *objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, remaining(checkpoints, 0)));
  ASSERT_TRUE(static_cast<bool>(route));
  lanelet::Ids route_ids;
  for (const auto& lane : *route) route_ids.push_back(lane.id());
  for (const auto id : {100070, 110022})
    EXPECT_NE(std::find(route_ids.begin(), route_ids.end(), id), route_ids.end());
  for (const auto id : {200312, 140377})
    EXPECT_EQ(std::find(route_ids.begin(), route_ids.end(), id), route_ids.end());
  for (std::size_t i = 0; i < checkpoints.size(); ++i) {
    const auto& checkpoint = checkpoints[i];
    ego->pose.pose.position.x = checkpoint.x_m;
    ego->pose.pose.position.y = checkpoint.y_m;
    ego->pose.pose.orientation.z = std::sin(checkpoint.yaw_rad / 2);
    ego->pose.pose.orientation.w = std::cos(checkpoint.yaw_rad / 2);
    completed = i;
    expectPayload(run());
    completed = i + 1;
    const auto path = run();
    expectPayload(path);
    const auto repeated_path = run();
    EXPECT_EQ(repeated_path.header, path.header);
    ASSERT_EQ(repeated_path.points.size(), path.points.size());
    for (std::size_t j = 0; j < path.points.size(); ++j) {
      EXPECT_EQ(repeated_path.points[j].pose, path.points[j].pose);
      EXPECT_TRUE(std::isnan(repeated_path.points[j].velocity));
      EXPECT_TRUE(std::isnan(path.points[j].velocity));
      EXPECT_DOUBLE_EQ(repeated_path.points[j].acceleration, path.points[j].acceleration);
    }
  }
}

TEST(PredictionPassThrough, PassesAllInputsToTheLocalStage) {
  auto shared = shortMap();
  pp::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 2;
  ego.pose.pose.orientation.w = 1;
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = 2;
  interfaces::Objects objects;
  objects.objects.resize(1);
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  const auto route = std::get<5>(global.process(ego, competition, objects, tf, imu, shortCheckpoints()));
  const auto result = pp::PredictionPassThrough{}.process(ego, competition, objects, tf, imu, route);
  EXPECT_EQ(std::get<5>(result), route);
  EXPECT_EQ(std::get<0>(result), ego);
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), objects);
  EXPECT_EQ(std::get<3>(result), tf);
  EXPECT_EQ(std::get<4>(result), imu);
}

TEST(EgoCompensatedPrediction, PassesAllInputsWithoutChangingTheObservation) {
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = 2;
  interfaces::Objects objects;
  objects.objects.resize(1);
  objects.objects.front().bbox_pose.orientation.w = 1;
  tf2_msgs::TFMessage tf;
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  pp::Route route;
  const auto result = pp::EgoCompensatedPrediction{}.process(ego, competition, objects, tf, imu, route);
  EXPECT_EQ(std::get<0>(result), ego);
  EXPECT_EQ(std::get<1>(result), competition);
  EXPECT_EQ(std::get<2>(result), objects);
  EXPECT_EQ(std::get<3>(result), tf);
  EXPECT_EQ(std::get<4>(result), imu);
  EXPECT_EQ(std::get<5>(result), route);
  for (const auto& sample : std::get<6>(result)) EXPECT_EQ(sample, objects);
}

TEST(CorePathPlanner, UsesImuAccelerationAndReturnsMapPathPayload) {
  lanelet::Lanelet lane(1,
    lanelet::LineString3d(2, {lanelet::Point3d(4, -10, 5, 10), lanelet::Point3d(5, 100, 5, 10)}),
    lanelet::LineString3d(3, {lanelet::Point3d(6, -10, -5, 10), lanelet::Point3d(7, 100, -5, 10)}));
  pp::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  ego.header.stamp.fromSec(10.0);
  ego.twist.twist.linear.x = 2.0;
  interfaces::Objects objects;
  objects.header.stamp = ego.header.stamp;
  pp::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = 0.3;
  pp::PathParameters parameters;
  parameters.trajectory_horizon_m = 25.0;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1.0;
  pp::CorePathPlanner core(parameters);
  const auto path = core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
                                tf2_msgs::TFMessage{}, imu, route, predictions);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity, 2.0);
  EXPECT_DOUBLE_EQ(path.points.front().acceleration, 0.3);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
  for (const auto& point : path.points) {
    EXPECT_NEAR(point.pose.pose.position.z, 10.0, 1e-9);
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    EXPECT_TRUE(point.pose.header.stamp.isZero());
  }
}

TEST(CorePathPlanner, PitchedStandstillSensorRoundingStillProducesPath) {
  lanelet::Lanelet lane(1,
    lanelet::LineString3d(2, {lanelet::Point3d(4, -10, 5, 10), lanelet::Point3d(5, 100, 5, 10)}),
    lanelet::LineString3d(3, {lanelet::Point3d(6, -10, -5, 10), lanelet::Point3d(7, 100, -5, 10)}));
  const pp::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  const double pitch_rad = std::asin(0.032220431 / 9.81);
  ego.pose.pose.orientation.y = std::sin(pitch_rad / 2);
  ego.pose.pose.orientation.w = std::cos(pitch_rad / 2);
  ego.header.stamp.fromSec(10.0);
  interfaces::Objects objects;
  objects.header.stamp = ego.header.stamp;
  pp::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  imu.linear_acceleration.x = -0.032220431 - 0.00002;
  imu.linear_acceleration.z = 9.81 * std::cos(pitch_rad);
  pp::PathParameters parameters;
  parameters.trajectory_horizon_m = 25.0;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1.0;
  for (double speed_mps : {-0.00000314, 0.0, 0.00000314}) {
    SCOPED_TRACE(speed_mps);
    ego.twist.twist.linear.x = speed_mps;
    ego.twist.twist.angular.z = 0.0000005652157;
    pp::CorePathPlanner core(parameters);
    const auto path = core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
                                  tf2_msgs::TFMessage{}, imu, route, predictions);
    ASSERT_GE(path.points.size(), 2u);
    EXPECT_DOUBLE_EQ(path.points.front().velocity, 0.0);
    EXPECT_DOUBLE_EQ(path.points.front().acceleration, 0.0);
    EXPECT_GT(lengthM(path), 0.0);
  }
}

TEST(CorePathPlanner, GravityCompensationPreservesMovingDecelerationAndInfeasibleStop) {
  lanelet::Lanelet lane(1,
    lanelet::LineString3d(2, {lanelet::Point3d(4, -10, 5, 10), lanelet::Point3d(5, 100, 5, 10)}),
    lanelet::LineString3d(3, {lanelet::Point3d(6, -10, -5, 10), lanelet::Point3d(7, 100, -5, 10)}));
  const pp::Route route = lanelet::routing::LaneletPath({lane});
  nav_msgs::Odometry ego;
  const double pitch_rad = 0.03;
  ego.pose.pose.orientation.y = std::sin(pitch_rad / 2);
  ego.pose.pose.orientation.w = std::cos(pitch_rad / 2);
  ego.header.stamp.fromSec(10.0);
  interfaces::Objects objects;
  objects.header.stamp = ego.header.stamp;
  pp::PredictedObjects predictions;
  predictions.fill(objects);
  sensor_msgs::Imu imu;
  pp::PathParameters parameters;
  parameters.trajectory_horizon_m = 25.0;
  parameters.speed_beam_width = 16;
  parameters.speed_time_step_sec = 1.0;
  ego.twist.twist.linear.x = 2.0;
  imu.linear_acceleration.x = -0.3 - 9.81 * std::sin(pitch_rad);
  pp::CorePathPlanner core(parameters);
  const auto path = core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
                                tf2_msgs::TFMessage{}, imu, route, predictions);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_DOUBLE_EQ(path.points.front().velocity, 2.0);
  EXPECT_NEAR(path.points.front().acceleration, -0.3, 1e-12);
  for (const auto& state : {std::pair<double, double>{0.01, -1.0}, {2.0, -6.0}, {0.00000314, -0.01}}) {
    SCOPED_TRACE(state.first);
    ego.twist.twist.linear.x = state.first;
    imu.linear_acceleration.x = state.second - 9.81 * std::sin(pitch_rad);
    EXPECT_TRUE(core.process(ego, interfaces::CompetitionVehicleStatus{}, objects,
                            tf2_msgs::TFMessage{}, imu, route, predictions).points.empty());
  }
}

TEST(LocalPathStage, RoutePresentMarksEmptyRouteMissing) {
  const pp::RoutePresent validator;
  std::vector<node_pipeline::Kind> issues;
  validator(pp::Route{}, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}, issues);
  issues.clear();
  auto shared = shortMap();
  pp::GlobalPath global(shared);
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  validator(std::get<5>(global.process(ego, interfaces::CompetitionVehicleStatus{}, interfaces::Objects{}, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, shortCheckpoints())), node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
}

TEST(LocalPathStage, LoaderReadsPathLength) {
  EXPECT_DOUBLE_EQ(20.0, pp::loadLocalPathParameters(
      node_pipeline::ParamReader::fromString("local_path_length_m: 20.0\n")).local_path_length_m);
  EXPECT_THROW(pp::loadLocalPathParameters(node_pipeline::ParamReader::fromString("local_path_length_m: 0\n")),
               std::invalid_argument);
}

TEST(SharedMapBank, LoaderReadsMapOnceForBothPathCores) {
  pp::SharedMap shared(pp::loadSharedMapParameters(node_pipeline::ParamReader::fromString(
      "map_file: kcity_lanelet2.osm\ncheckpoints_file: checkpoints.csv\n", NP_PACKAGE_DIR "/../map")));
  ASSERT_TRUE(static_cast<bool>(shared.map));
  EXPECT_FALSE(shared.map->laneletLayer.empty());
  pp::GlobalPath global(shared);
  pp::LocalPath local({20.0}, shared);
  EXPECT_EQ(shared.map.use_count(), 3);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
