#include <path_planner/cores/frenet_speed_path.hpp>
#include <path_planner/cores/frenet_path.hpp>
#include <path_planner/parameters/frenet_speed_path.hpp>
#include <path_planner/validators/frenet_speed_path_inputs.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <string>

namespace pp = path_planner;
namespace {
pp::SharedMap straightMap() {
  std::vector<lanelet::Point3d> left, right;
  for (int i = 0; i <= 16; ++i) {
    left.emplace_back(100 + 2*i, 5.0*i, 6.0, 0.5*i);
    right.emplace_back(101 + 2*i, 5.0*i, -6.0, 0.5*i);
  }
  lanelet::Lanelets lanes;
  for (int i = 0; i < 16; ++i) {
    lanelet::Lanelet lane(300 + i,
        lanelet::LineString3d(200 + 2*i, {left[i], left[i + 1]}),
        lanelet::LineString3d(201 + 2*i, {right[i], right[i + 1]}));
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    lanes.push_back(lane);
  }
  return pp::SharedMap({lanelet::utils::createMap(lanes)});
}

const char* parametersYaml =
    "path_length_m: 20\nsample_step_m: 0.5\nreference_step_m: 2\n"
    "transition_lengths_m: [8, 12, 16]\nlateral_offsets_m: [-0.5, 0, 0.5]\n"
    "vehicle_front_m: 3.845008\nvehicle_rear_m: 0.79\nvehicle_half_width_m: 1.080418\n"
    "desired_speed_mps: 8\nmax_acceleration_mps2: 2\nmax_deceleration_mps2: 4\n"
    "max_jerk_mps3: 4\nmax_lateral_acceleration_mps2: 2\n"
    "time_step_sec: 0.25\ncommand_step_sec: 0.2\nbeam_width: 64\n"
    "collision_margin_m: 0.5\nprediction_uncertainty_mps: 0.2\n";

class FrenetSpeedPathTest : public testing::Test {
 protected:
  FrenetSpeedPathTest() : shared(straightMap()) {
    parameters.transition_lengths_m = {8.0};
    parameters.lateral_offsets_m = {0.0};
    ego.header.stamp.fromSec(10.0);
    ego.header.frame_id = "map";
    ego.pose.pose.position.x = 2.0;
    ego.pose.pose.orientation.w = 1.0;
    competition.header.frame_id = "base_link";
    objects.header.stamp = ego.header.stamp;
    objects.header.frame_id = "map";
    stamp = ego.header.stamp;
    predictions.fill(objects);
    lanelet::ConstLanelets lanes;
    for (int i = 0; i < 5; ++i) lanes.push_back(shared.map->laneletLayer.get(300 + i));
    route = lanelet::routing::LaneletPath(lanes);
  }

  void observe(double x_m, double y_m, double vx_mps = 0.0, double vy_mps = 0.0,
               double length_m = 4.0, double width_m = 2.0) {
    interfaces::Object object;
    object.id = 42;
    object.label = "vehicle";
    object.position.x = x_m;
    object.position.y = y_m;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = length_m;
    object.bbox_size.y = width_m;
    object.bbox_size.z = 1.5;
    objects.objects = {object};
    predictions.fill(objects);
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      auto& predicted = predictions[i].objects.front();
      predicted.position.x += vx_mps*0.5*(i + 1);
      predicted.position.y += vy_mps*0.5*(i + 1);
      predicted.bbox_pose.position = predicted.position;
    }
  }

  interfaces::Path plan(pp::FrenetSpeedPath& core) {
    competition.header.stamp = stamp;
    return core.process(ego, competition, objects, route, predictions, stamp);
  }

  pp::SharedMap shared;
  pp::FrenetSpeedPathParameters parameters;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  interfaces::Objects objects;
  pp::Route route;
  pp::PredictedObjects predictions;
  ros::Time stamp;
};
}  // namespace

TEST_F(FrenetSpeedPathTest, StartsFromRestWithFiniteProfileAndPayloadHeaders) {
  pp::FrenetSpeedPath core(parameters, shared);
  const auto path = plan(core);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GT(path.points.front().velocity, 0.0);
  EXPECT_LE(path.points.front().velocity,
      parameters.max_acceleration_mps2*parameters.command_step_sec);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 2.0, 1e-8);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 22.0, 1e-8);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
  for (const auto& point : path.points) {
    const auto& pose = point.pose.pose;
    const auto& q = pose.orientation;
    EXPECT_TRUE(std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
                std::isfinite(pose.position.z));
    EXPECT_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, 1e-8);
    EXPECT_TRUE(point.pose.header.stamp.isZero());
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    EXPECT_TRUE(std::isfinite(point.velocity));
    EXPECT_GE(point.velocity, 0.0);
    EXPECT_LE(point.velocity, parameters.desired_speed_mps);
    EXPECT_TRUE(std::isfinite(point.acceleration));
    EXPECT_GE(point.acceleration, -parameters.max_deceleration_mps2);
    EXPECT_LE(point.acceleration, parameters.max_acceleration_mps2);
  }
}

TEST_F(FrenetSpeedPathTest, UsesCompetitionSpeedRatherThanOdometryAsTheInitialState) {
  ego.twist.twist.linear.x = 100.0;
  competition.velocity_x_mps = 4.0;
  pp::FrenetSpeedPath core(parameters, shared);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GE(path.points.front().velocity,
      4.0 - parameters.max_deceleration_mps2*parameters.command_step_sec);
  EXPECT_LE(path.points.front().velocity,
      4.0 + parameters.max_acceleration_mps2*parameters.command_step_sec);
}

TEST_F(FrenetSpeedPathTest, PassesAnOccupiedPathAfterThePredictedObjectLeaves) {
  competition.velocity_x_mps = 2.0;
  observe(14.0, 0.0, 0.0, 16.0);
  pp::FrenetPathParameters geometric;
  geometric.transition_lengths_m = parameters.transition_lengths_m;
  geometric.lateral_offsets_m = parameters.lateral_offsets_m;
  pp::FrenetPath static_planner(geometric, shared);
  EXPECT_TRUE(static_planner.process(ego, objects, route).points.empty());
  pp::FrenetSpeedPath core(parameters, shared);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GT(path.points.front().velocity, 0.0);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 22.0, 1e-8);
  for (const auto& point : path.points) {
    EXPECT_TRUE(std::isfinite(point.velocity));
    EXPECT_NEAR(point.pose.pose.position.y, 0.0, 1e-8);
  }
}

TEST_F(FrenetSpeedPathTest, SlowsForAnInitiallyClearFutureCrossing) {
  competition.velocity_x_mps = 4.0;
  pp::FrenetSpeedPath core(parameters, shared);
  const auto free = plan(core);
  observe(14.0, 10.0, 0.0, -10.0);
  for (std::size_t i = 1; i < predictions.size(); ++i) {
    auto& predicted = predictions[i].objects.front();
    predicted.position.y = 0.0;
    predicted.bbox_pose.position = predicted.position;
  }
  const auto crossing = plan(core);
  ASSERT_FALSE(crossing.points.empty());
  ASSERT_EQ(crossing.points.size(), free.points.size());
  bool slower = false;
  for (std::size_t i = 0; i < crossing.points.size(); ++i) {
    EXPECT_TRUE(std::isfinite(crossing.points[i].velocity));
    slower = slower || crossing.points[i].velocity + 0.25 < free.points[i].velocity;
  }
  EXPECT_TRUE(slower);
}

TEST_F(FrenetSpeedPathTest, RequestsStopWhenEveryCandidateIsBlocked) {
  parameters.lateral_offsets_m = {-3.0, 0.0, 3.0};
  observe(4.0, 0.0, 0.0, 0.0, 8.0, 20.0);
  pp::FrenetSpeedPath core(parameters, shared);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  for (const auto& point : path.points) {
    EXPECT_DOUBLE_EQ(point.velocity, 0.0);
    EXPECT_DOUBLE_EQ(point.acceleration, -parameters.max_deceleration_mps2);
    EXPECT_NEAR(point.pose.pose.position.y, 0.0, 1e-8);
  }
}

TEST_F(FrenetSpeedPathTest, UsesMapObjectsWithTranslatedAndRotatedEgo) {
  lanelet::Lanelet lane(300,
      lanelet::LineString3d(200, {lanelet::Point3d(100, 94, 200, 0),
                                lanelet::Point3d(101, 94, 260, 0)}),
      lanelet::LineString3d(201, {lanelet::Point3d(102, 106, 200, 0),
                                lanelet::Point3d(103, 106, 260, 0)}));
  lane.attributes()["subtype"] = "road";
  lane.attributes()["location"] = "urban";
  lane.attributes()["one_way"] = "yes";
  shared.map = lanelet::utils::createMap({lane});
  route = lanelet::routing::LaneletPath({shared.map->laneletLayer.get(300)});
  ego.pose.pose.position.x = 100.0;
  ego.pose.pose.position.y = 202.0;
  ego.pose.pose.orientation.z = ego.pose.pose.orientation.w = std::sqrt(0.5);
  pp::FrenetSpeedPath core(parameters, shared);
  const auto free = plan(core);
  ASSERT_FALSE(free.points.empty());
  ASSERT_GT(free.points.front().velocity, 0.0);
  observe(100.0, 206.0, 0.0, 0.0, 20.0, 8.0);
  const auto occupied = plan(core);
  ASSERT_FALSE(occupied.points.empty());
  for (const auto& point : occupied.points) EXPECT_DOUBLE_EQ(point.velocity, 0.0);
}

TEST_F(FrenetSpeedPathTest, RejectsBaseLinkPredictions) {
  for (auto& sample : predictions) sample.header.frame_id = "base_link";
  std::vector<node_pipeline::Kind> issues;
  pp::FrenetSpeedPathInputs{}(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::FrameMismatch}), issues);
}

TEST_F(FrenetSpeedPathTest, RequestsStopAtTheDeadEndOnItsFirstCall) {
  ego.pose.pose.position.x = 80.0;
  ego.pose.pose.position.y = 0.0;
  competition.velocity_x_mps = 2.0;
  route = lanelet::routing::LaneletPath({shared.map->laneletLayer.get(315)});
  pp::FrenetSpeedPath core(parameters, shared);
  const auto path = plan(core);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    const auto& point = path.points[i];
    EXPECT_DOUBLE_EQ(point.velocity, 0.0);
    EXPECT_DOUBLE_EQ(point.acceleration, -parameters.max_deceleration_mps2);
    EXPECT_TRUE(point.pose.header.stamp.isZero());
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    if (i > 0) {
      const auto& previous = path.points[i - 1].pose.pose.position;
      const auto& position = point.pose.pose.position;
      EXPECT_GT(std::hypot(position.x - previous.x, position.y - previous.y), 0.0);
    }
  }
}

TEST_F(FrenetSpeedPathTest, SelectsAMovingLateralCandidateOverAStoppedCenterline) {
  parameters.transition_lengths_m = {8.0, 12.0};
  parameters.lateral_offsets_m = {-3.0, 0.0, 3.0};
  competition.velocity_x_mps = 2.0;
  observe(16.0, 0.0, 0.0, 0.0, 2.0, 1.0);
  pp::FrenetSpeedPath core(parameters, shared);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GT(path.points.front().velocity, 0.0);
  double lateral_m = 0.0;
  for (const auto& point : path.points) {
    EXPECT_TRUE(std::isfinite(point.velocity));
    lateral_m = std::max(lateral_m, std::abs(point.pose.pose.position.y));
  }
  EXPECT_GT(lateral_m, 2.0);
}

TEST_F(FrenetSpeedPathTest, AlignsThePlanningClockAndKeepsPeriodicAccelerationState) {
  competition.velocity_x_mps = 4.0;
  stamp.fromSec(10.5);
  pp::FrenetSpeedPath core(parameters, shared);
  const auto shifted = plan(core);
  ASSERT_FALSE(shifted.points.empty());
  EXPECT_NEAR(shifted.points.front().pose.pose.position.x, 4.0, 1e-8);
  stamp.fromSec(10.6);
  ego.header.stamp = stamp;
  ego.pose.pose.position.x = 4.4;
  competition.velocity_x_mps = 4.2;
  objects.header.stamp = stamp;
  predictions.fill(objects);
  const auto first = plan(core);
  const auto repeated = plan(core);
  ASSERT_FALSE(first.points.empty());
  ASSERT_EQ(first.points.size(), repeated.points.size());
  EXPECT_GT(first.points.front().velocity, competition.velocity_x_mps);
  EXPECT_DOUBLE_EQ(first.points.front().velocity, repeated.points.front().velocity);
  EXPECT_DOUBLE_EQ(first.points.front().acceleration, repeated.points.front().acceleration);
}

TEST_F(FrenetSpeedPathTest, LoadsParametersAndRejectsUnsafePlanningContracts) {
  const auto loaded = pp::loadFrenetSpeedPathParameters(
      node_pipeline::ParamReader::fromString(parametersYaml));
  EXPECT_DOUBLE_EQ(loaded.path_length_m, 20.0);
  EXPECT_DOUBLE_EQ(loaded.command_step_sec, 0.2);
  std::string invalid(parametersYaml);
  invalid.replace(invalid.find("time_step_sec: 0.25"), std::string("time_step_sec: 0.25").size(),
                  "time_step_sec: 0");
  EXPECT_THROW(pp::loadFrenetSpeedPathParameters(node_pipeline::ParamReader::fromString(invalid)),
               std::invalid_argument);
  const pp::FrenetSpeedPathInputs validator;
  std::vector<node_pipeline::Kind> issues;
  validator(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
  competition.velocity_x_mps = -1.0;
  validator(competition, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::OutRangeNegative}), issues);
  issues.clear();
  observe(12.0, 0.0);
  predictions[1].objects.front().id = 43;
  validator(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::OutRangePositive}), issues);
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
