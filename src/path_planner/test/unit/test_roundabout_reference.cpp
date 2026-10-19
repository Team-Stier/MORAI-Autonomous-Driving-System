#include <path_planner/cores/roundabout_reference.hpp>
#include <path_planner/parameters/roundabout_reference.hpp>
#include <path_planner/policies/roundabout_reference_inputs.hpp>
#include <gtest/gtest.h>
#include <pugixml.hpp>
#include <array>
#include <cmath>
#include <limits>
#include <fstream>
#include <chrono>
#include <map>
#include <string>

namespace pp = path_planner;
namespace {
double heading(const geometry_msgs::Quaternion& q) {
  return std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z);
}

double angle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

interfaces::Path geometryPlan(pp::RoundaboutReference& core, nav_msgs::Odometry ego) {
  ros::Time::init();
  interfaces::CompetitionVehicleStatus status;
  if (ego.header.stamp.isZero()) ego.header.stamp.fromSec(123.0);
  if (ego.pose.pose.orientation.w == 0 && ego.pose.pose.orientation.z == 0)
    ego.pose.pose.orientation.w = 1;
  status.header.stamp = ego.header.stamp;
  status.header.frame_id = "base_link";
  interfaces::Objects objects;
  objects.header = ego.header;
  pp::PredictedObjects predictions;
  predictions.fill(objects);
  return core.process(ego, status, objects, predictions, ego.header.stamp);
}


struct RoadPolygon {
  std::vector<std::array<double, 2>> points;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  bool contains(double x, double y) const {
    if (x < min_x || x > max_x || y < min_y || y > max_y) return false;
    bool inside = false;
    for (std::size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
      const auto& a = points[i];
      const auto& b = points[j];
      if ((a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0])
        inside = !inside;
    }
    return inside;
  }
};

std::vector<RoadPolygon> roadFixture() {
  pugi::xml_document document;
  if (!document.load_file((std::string(NP_PACKAGE_DIR) + "/../map/kcity_lanelet2.osm").c_str())) return {};
  std::map<std::string, std::array<double, 2>> nodes;
  std::map<std::string, std::vector<std::array<double, 2>>> ways;
  const auto root = document.child("osm");
  for (const auto node : root.children("node")) {
    std::array<double, 2> point{{0, 0}};
    for (const auto tag : node.children("tag")) {
      const std::string key = tag.attribute("k").value();
      if (key == "local_x") point[0] = tag.attribute("v").as_double();
      if (key == "local_y") point[1] = tag.attribute("v").as_double();
    }
    nodes[node.attribute("id").value()] = point;
  }
  for (const auto way : root.children("way"))
    for (const auto nd : way.children("nd"))
      ways[way.attribute("id").value()].push_back(nodes[nd.attribute("ref").value()]);
  std::vector<RoadPolygon> polygons;
  for (const auto relation : root.children("relation")) {
    std::string left_id, right_id;
    for (const auto member : relation.children("member")) {
      const std::string role = member.attribute("role").value();
      if (role == "left") left_id = member.attribute("ref").value();
      if (role == "right") right_id = member.attribute("ref").value();
    }
    if (left_id.empty() || right_id.empty()) continue;
    auto left = ways[left_id], right = ways[right_id];
    if (left.size() < 2 || right.size() < 2) continue;
    if (std::hypot(left.front()[0] - right.front()[0], left.front()[1] - right.front()[1]) >
        std::hypot(left.front()[0] - right.back()[0], left.front()[1] - right.back()[1]))
      std::reverse(right.begin(), right.end());
    RoadPolygon polygon;
    polygon.points = left;
    polygon.points.insert(polygon.points.end(), right.rbegin(), right.rend());
    for (const auto& point : polygon.points) {
      polygon.min_x = std::min(polygon.min_x, point[0]);
      polygon.max_x = std::max(polygon.max_x, point[0]);
      polygon.min_y = std::min(polygon.min_y, point[1]);
      polygon.max_y = std::max(polygon.max_y, point[1]);
    }
    if (polygon.max_x >= -115 && polygon.min_x <= 50 && polygon.max_y >= 245 && polygon.min_y <= 360)
      polygons.push_back(std::move(polygon));
  }
  return polygons;
}

class RoundaboutReferenceTest : public testing::Test {
 protected:
  RoundaboutReferenceTest()
      : parameters(pp::loadRoundaboutReferenceParameters(node_pipeline::ParamReader::fromFile(
            std::string(NP_PACKAGE_DIR) + "/config/path_planner/parameters/roundabout_reference.yaml",
            NP_PACKAGE_DIR))) {
    parameters.beam_width = 1;
    ego.header.frame_id = "map";
    ego.header.stamp.fromSec(123.0);
    ego.pose.pose.orientation.w = 1.0;
  }

  geometry_msgs::Point sourcePoint(double station_m) const {
    double traversed_m = 0.0;
    for (std::size_t i = 1; i < parameters.global_path.size(); ++i) {
      const auto& a = parameters.global_path[i - 1];
      const auto& b = parameters.global_path[i];
      const double length_m = std::hypot(b.x - a.x, b.y - a.y);
      if (length_m == 0.0) continue;
      if (traversed_m + length_m >= station_m) {
        const double t = (station_m - traversed_m) / length_m;
        geometry_msgs::Point point;
        point.x = a.x + t * (b.x - a.x);
        point.y = a.y + t * (b.y - a.y);
        point.z = a.z + t * (b.z - a.z);
        return point;
      }
      traversed_m += length_m;
    }
    return parameters.global_path.back();
  }

  pp::RoundaboutReferenceParameters parameters;
  nav_msgs::Odometry ego;
};
}  // namespace

TEST(RoundaboutReference, ProjectsOntoStraightReferenceAndHandlesRepeatedSourcePoints) {
  pp::RoundaboutReferenceParameters p;
  p.beam_width = 1;
  p.reference_start_m = 0.0;
  p.reference_end_m = 80.0;
  p.exit_shift_m = 0.0;
  for (int i = 0; i <= 80; ++i) {
    geometry_msgs::Point point;
    point.x = i;
    point.z = 10.0 + 0.1 * i;
    p.global_path.push_back(point);
    p.global_path.push_back(point);
  }
  pp::RoundaboutReference core(p);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 12.345;
  ego.pose.pose.position.y = 1.5;
  const auto path = geometryPlan(core, ego);
  ASSERT_EQ(path.points.size(), 161u);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 12.345, 1e-8);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 52.345, 1e-8);
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    const auto& point = path.points[i];
    EXPECT_NEAR(point.pose.pose.position.x, 12.345 + 0.25 * i, 1e-8);
    EXPECT_NEAR(point.pose.pose.position.y, 0.0, 1e-10);
    EXPECT_NEAR(point.pose.pose.position.z, 10.0 + 0.1 * point.pose.pose.position.x, 1e-8);
    EXPECT_NEAR(heading(point.pose.pose.orientation), 0.0, 1e-10);
    EXPECT_TRUE(std::isfinite(point.velocity));
    EXPECT_TRUE(std::isfinite(point.acceleration));
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    EXPECT_TRUE(point.pose.header.stamp.isZero());
  }
  EXPECT_TRUE(path.header.frame_id.empty());
  EXPECT_TRUE(path.header.stamp.isZero());
}

TEST_F(RoundaboutReferenceTest, RetainsOfficialCheckpointsEightAndNine) {
  pp::RoundaboutReference core(parameters);
  for (const auto station_m : {858.5023964183979, 930.6019068794656}) {
    ego.pose.pose.position = sourcePoint(station_m);
    const auto path = geometryPlan(core, ego);
    ASSERT_GE(path.points.size(), 2u);
    const auto& start = path.points.front().pose.pose.position;
    EXPECT_LT(std::hypot(start.x - ego.pose.pose.position.x, start.y - ego.pose.pose.position.y), 0.3);
    EXPECT_NEAR(start.z, ego.pose.pose.position.z, 0.02);
  }
}

TEST_F(RoundaboutReferenceTest, ProducesBoundedContinuousCurvatureAcrossEntryTurnAndExit) {
  pp::RoundaboutReference core(parameters);
  for (double source_m = 840.0; source_m <= 935.0; source_m += 2.5) {
    SCOPED_TRACE(source_m);
    ego.pose.pose.position = sourcePoint(source_m);
    const auto path = geometryPlan(core, ego);
    ASSERT_GE(path.points.size(), 160u);
    double length_m = 0.0;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto& point = path.points[i];
      const auto& q = point.pose.pose.orientation;
      EXPECT_NEAR(q.z * q.z + q.w * q.w, 1.0, 1e-12);
      EXPECT_TRUE(std::isfinite(point.pose.pose.position.z));
      EXPECT_TRUE(std::isfinite(point.velocity));
      if (i == 0) continue;
      const auto& previous = path.points[i - 1];
      const double step_m = std::hypot(point.pose.pose.position.x - previous.pose.pose.position.x,
                                      point.pose.pose.position.y - previous.pose.pose.position.y);
      ASSERT_GT(step_m, 0.0);
      EXPECT_LE(step_m, parameters.sample_step_m + 0.0001);
      const double curvature = angle(heading(q) - heading(previous.pose.pose.orientation)) / step_m;
      EXPECT_LT(std::abs(curvature), 1.0 / 5.87);
      EXPECT_LT(std::abs(std::atan(3.0 * curvature)), 40.0 * std::acos(-1.0) / 180.0);
      length_m += step_m;
    }
    EXPECT_NEAR(length_m, parameters.local_path_length_m, 0.002);
  }
}

TEST_F(RoundaboutReferenceTest, RepeatedInputsAreStableAndAdvancingEgoDoesNotJumpBranches) {
  pp::RoundaboutReference core(parameters);
  geometry_msgs::Point previous;
  double previous_heading = 0.0;
  bool first = true;
  for (double station_m = 848.5; station_m <= 931.0; station_m += 0.1) {
    ego.pose.pose.position = sourcePoint(station_m);
    const auto path = geometryPlan(core, ego);
    const auto repeated = geometryPlan(core, ego);
    ASSERT_FALSE(path.points.empty());
    ASSERT_EQ(path.points.size(), repeated.points.size());
    const auto& start = path.points.front().pose.pose.position;
    EXPECT_DOUBLE_EQ(start.x, repeated.points.front().pose.pose.position.x);
    EXPECT_DOUBLE_EQ(start.y, repeated.points.front().pose.pose.position.y);
    const double yaw = heading(path.points.front().pose.pose.orientation);
    if (!first) {
      EXPECT_LT(std::hypot(start.x - previous.x, start.y - previous.y), 0.13);
      EXPECT_LT(std::abs(angle(yaw - previous_heading)), 0.025);
    }
    previous = start;
    previous_heading = yaw;
    first = false;
  }
}

TEST_F(RoundaboutReferenceTest, KeepsVehiclePerimeterWithTenCentimetreMarginInsideMappedRoad) {
  const auto roads = roadFixture();
  ASSERT_FALSE(roads.empty());
  pp::RoundaboutReference core(parameters);
  const std::array<std::array<double, 2>, 4> corners{{
      {{3.945008, 1.180418}}, {{3.945008, -1.180418}},
      {{-0.89, -1.180418}}, {{-0.89, 1.180418}}}};
  for (const double station_m : {840.0, 865.0, 890.0, 915.0}) {
    ego.pose.pose.position = sourcePoint(station_m);
    const auto path = geometryPlan(core, ego);
    for (const auto& point : path.points) {
      const auto& center = point.pose.pose.position;
      const double yaw = heading(point.pose.pose.orientation);
      for (std::size_t edge = 0; edge < corners.size(); ++edge) {
        const auto& a = corners[edge];
        const auto& b = corners[(edge + 1) % corners.size()];
        const int count = static_cast<int>(std::ceil(std::hypot(b[0] - a[0], b[1] - a[1]) / 0.2));
        for (int i = 0; i <= count; ++i) {
          const double local_x = a[0] + (b[0] - a[0]) * i / count;
          const double local_y = a[1] + (b[1] - a[1]) * i / count;
          const double x = center.x + local_x * std::cos(yaw) - local_y * std::sin(yaw);
          const double y = center.y + local_x * std::sin(yaw) + local_y * std::cos(yaw);
          ASSERT_TRUE(std::any_of(roads.begin(), roads.end(),
              [x, y](const RoadPolygon& road) { return road.contains(x, y); })) << x << ", " << y;
        }
      }
    }
  }
}

TEST_F(RoundaboutReferenceTest, EndsAtReferenceEndpointWithoutRepeatingPoints) {
  pp::RoundaboutReference core(parameters);
  ego.pose.pose.position = sourcePoint(979.0);
  const auto path = geometryPlan(core, ego);
  ASSERT_GE(path.points.size(), 2u);
  ASSERT_LT(path.points.size(), 10u);
  const auto expected = sourcePoint(980.0);
  const auto& end = path.points.back().pose.pose.position;
  EXPECT_NEAR(end.x, expected.x, 1e-9);
  EXPECT_NEAR(end.y, expected.y, 1e-9);
  EXPECT_NEAR(end.z, expected.z, 1e-9);
  for (std::size_t i = 1; i < path.points.size(); ++i)
    EXPECT_GT(std::hypot(path.points[i].pose.pose.position.x - path.points[i - 1].pose.pose.position.x,
                         path.points[i].pose.pose.position.y - path.points[i - 1].pose.pose.position.y), 1e-10);
}

TEST_F(RoundaboutReferenceTest, RejectsInvalidIntervalsAndAssetsDuringParameterLoading) {
  std::ifstream configuration(std::string(NP_PACKAGE_DIR) +
      "/config/path_planner/parameters/roundabout_reference.yaml");
  const std::string base((std::istreambuf_iterator<char>(configuration)), std::istreambuf_iterator<char>());
  for (const auto replacement : {"reference_end_m: 800", "sample_step_m: 0", "control_step_m: .nan",
                                "exit_shift_m: .inf", "global_path_file: /dev/null",
                                "following_time_gap_sec: 0", "beam_width: 0", "time_step_sec: 0",
                                "max_deceleration_mps2: .nan", "entry_line_station_m: 800",
                                "entry_stop_margin_m: 0", "approach_speed_mps: 0",
                                "command_step_sec: 2", "entry_gap_confirmation_sec: 0", "curve_speed_ratio: 0"}) {
    std::string yaml = base;
    const std::string entry(replacement);
    const auto key = entry.substr(0, entry.find(':'));
    const auto begin = yaml.find(key + ':');
    yaml.replace(begin, yaml.find('\n', begin) - begin, entry);
    EXPECT_THROW(pp::loadRoundaboutReferenceParameters(
        node_pipeline::ParamReader::fromString(yaml, NP_PACKAGE_DIR)), std::invalid_argument);
  }
}

namespace {
class RoundaboutTrafficTest : public testing::Test {
 protected:
  RoundaboutTrafficTest() {
    ros::Time::init();
    parameters.entry_line_station_m = 0;
    parameters.reference_start_m = 0;
    parameters.reference_end_m = 100;
    parameters.entry_shift_m = parameters.exit_shift_m = 0;
    parameters.beam_width = 32;
    parameters.collision_margin_m = 0.7;
    for (int i = 0; i <= 100; ++i) {
      geometry_msgs::Point point;
      point.x = i;
      parameters.global_path.push_back(point);
    }
    ego.header.frame_id = "map";
    ego.header.stamp.fromSec(10);
    ego.pose.pose.position.x = 2;
    ego.pose.pose.orientation.w = 1;
    status.header.frame_id = "base_link";
    status.header.stamp = ego.header.stamp;
    objects.header = ego.header;
    predictions.fill(objects);
  }

  void observe(double x, double y, double vx = 0, double vy = 0, double yaw = 0) {
    interfaces::Object object;
    object.id = 42;
    object.bbox_pose.position.x = x;
    object.bbox_pose.position.y = y;
    object.position = object.bbox_pose.position;
    object.bbox_pose.orientation.z = std::sin(yaw/2);
    object.bbox_pose.orientation.w = std::cos(yaw/2);
    object.bbox_size.x = 4;
    object.bbox_size.y = 2;
    object.bbox_size.z = 1.5;
    objects.objects = {object};
    predictions.fill(objects);
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      auto& predicted = predictions[i].objects.front();
      predicted.position.x += vx*0.5*(i+1);
      predicted.position.y += vy*0.5*(i+1);
      predicted.bbox_pose.position = predicted.position;
    }
  }

  interfaces::Path plan(pp::RoundaboutReference& core) {
    return core.process(ego, status, objects, predictions, ego.header.stamp);
  }

  pp::RoundaboutReferenceParameters parameters;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus status;
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
};
}

TEST_F(RoundaboutTrafficTest, StartsFromRestAndPreservesFiniteSpeedLimits) {
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  EXPECT_GT(path.points.front().velocity, 0);
  for (const auto& point : path.points) {
    EXPECT_GE(point.velocity, 0);
    EXPECT_LE(point.velocity, parameters.desired_speed_mps + 1e-8);
    EXPECT_GE(point.acceleration, -parameters.max_deceleration_mps2 - 1e-8);
    EXPECT_LE(point.acceleration, parameters.max_acceleration_mps2 + 1e-8);
    EXPECT_TRUE(point.pose.header.stamp.isZero());
  }
}

TEST_F(RoundaboutTrafficTest, WaitsForBlockedEntryAndReleasesWhenNpcClears) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  observe(52, 0);
  pp::RoundaboutReference core(parameters);
  const auto blocked = plan(core);
  EXPECT_DOUBLE_EQ(blocked.points.front().velocity, 0);
  objects.objects.clear();
  predictions.fill(objects);
  const auto clear = plan(core);
  EXPECT_GT(clear.points.front().velocity, 0);
}

TEST_F(RoundaboutTrafficTest, CrossingNpcBetweenPredictionFramesChangesSpeedPlan) {
  pp::RoundaboutReference free_core(parameters);
  const auto free = plan(free_core);
  // Fast traffic crosses between 0.5 s knots; endpoint-only checks miss it.
  observe(9, -13, 0, 8, M_PI/2);
  pp::RoundaboutReference blocked_core(parameters);
  const auto blocked = plan(blocked_core);
  double difference = 0;
  for (std::size_t i = 0; i < free.points.size(); ++i)
    difference += std::abs(free.points[i].velocity - blocked.points[i].velocity);
  EXPECT_GT(difference, 0.5);
}

TEST_F(RoundaboutTrafficTest, SlowsForStoppedNpcAfterEntryWithoutPublishingStop) {
  status.velocity_x_mps = 4;
  observe(17, 0);
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  EXPECT_LT(path.points.front().velocity, status.velocity_x_mps);
  EXPECT_LT(path.points.front().acceleration, 0);
  for (const auto& point : path.points)
    EXPECT_GE(point.velocity, parameters.entered_min_speed_mps);
}

TEST_F(RoundaboutTrafficTest, SlowerLeadingNpcReducesSpeedWithoutForcingPermanentStop) {
  status.velocity_x_mps = 4;
  observe(17, 0, 2, 0);
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  pp::RoundaboutReference free_core(parameters);
  auto no_objects = objects;
  no_objects.objects.clear();
  pp::PredictedObjects no_predictions;
  no_predictions.fill(no_objects);
  const auto free = free_core.process(ego, status, no_objects, no_predictions, ego.header.stamp);
  EXPECT_LT(path.points.front().velocity, free.points.front().velocity);
  EXPECT_GT(path.points.front().velocity, 0);
  bool slowed = false;
  for (const auto& point : path.points)
    if (point.pose.pose.position.x > 10 && point.pose.pose.position.x < 20 &&
        point.velocity > 0 && point.velocity < 3) slowed = true;
  EXPECT_TRUE(slowed);
  const auto repeated = plan(core);
  EXPECT_DOUBLE_EQ(path.points.front().velocity, repeated.points.front().velocity);
}

TEST_F(RoundaboutTrafficTest, NoFeasibleMovingTrajectoryPublishesRequestedRollingTarget) {
  status.velocity_x_mps = 5;
  observe(5, 0);
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  EXPECT_DOUBLE_EQ(path.points.front().velocity, parameters.entered_min_speed_mps);
  EXPECT_LT(path.points.front().acceleration, 0);
  for (const auto& point : path.points) EXPECT_GE(point.velocity, parameters.entered_min_speed_mps);
}

TEST_F(RoundaboutTrafficTest, AlignsPredictionAgeWithPlanningStamp) {
  observe(6, 0, 20, 0);
  objects.objects.clear();
  pp::RoundaboutReference fresh(parameters);
  const auto occupied = plan(fresh);
  EXPECT_DOUBLE_EQ(occupied.points.front().velocity, parameters.entered_min_speed_mps);
  for (auto& prediction : predictions) prediction.header.stamp -= ros::Duration(0.4);
  pp::RoundaboutReference aged(parameters);
  const auto moved = plan(aged);
  EXPECT_GT(moved.points.front().velocity, occupied.points.front().velocity);
}

TEST_F(RoundaboutTrafficTest, ReplansFollowingWithFreshNpcObservations) {
  status.velocity_x_mps = 4;
  pp::RoundaboutReference core(parameters);
  double npc_x = 17;
  double minimum_speed = 4;
  for (int i = 0; i < 100; ++i) {
    observe(npc_x, 0, 2, 0);
    const auto path = plan(core);
    const double target = path.points.front().velocity;
    const double acceleration = std::max(-4.0, std::min(2.0,
        1.2*(target - status.velocity_x_mps)));
    const double speed = std::max(0.0, status.velocity_x_mps + acceleration*0.05);
    EXPECT_GT(speed, 0);
    EXPECT_GE(npc_x - 2 - ego.pose.pose.position.x - parameters.vehicle_front_m,
              parameters.collision_margin_m);
    minimum_speed = std::min(minimum_speed, speed);
    ego.pose.pose.position.x += 0.025*(status.velocity_x_mps + speed);
    status.velocity_x_mps = speed;
    ego.header.stamp += ros::Duration(0.05);
    status.header.stamp = ego.header.stamp;
    objects.header.stamp = ego.header.stamp;
    npc_x += 0.1;
  }
  EXPECT_LT(minimum_speed, 3);
}

TEST_F(RoundaboutTrafficTest, DistantTrafficKeepsTheSameCompleteSpeedPlan) {
  pp::RoundaboutReference empty_core(parameters);
  const auto expected = plan(empty_core);
  for (unsigned i = 0; i < 20; ++i) {
    interfaces::Object object;
    object.id = i + 1;
    object.position.x = 300 + i*10;
    object.position.y = 200;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1;
    object.bbox_size.x = 4;
    object.bbox_size.y = 2;
    object.bbox_size.z = 1.5;
    objects.objects.push_back(object);
  }
  predictions.fill(objects);
  for (std::size_t k = 0; k < predictions.size(); ++k)
    for (auto& object : predictions[k].objects) {
      object.position.x -= 10*0.5*(k + 1);
      object.bbox_pose.position = object.position;
    }
  pp::RoundaboutReference traffic_core(parameters);
  EXPECT_EQ(plan(traffic_core), expected);
}

TEST_F(RoundaboutTrafficTest, KeepsFastTrafficThatApproachesFromBehind) {
  observe(-6, 0, 12, 0);
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  for (const auto& point : path.points) EXPECT_GE(point.velocity, parameters.entered_min_speed_mps);
}

TEST_F(RoundaboutTrafficTest, IncludesObjectExtentWhenItsCenterIsOutsideThePath) {
  observe(20, 20);
  objects.objects.front().bbox_size.x = 50;
  objects.objects.front().bbox_size.y = 50;
  predictions.fill(objects);
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  for (const auto& point : path.points) EXPECT_GE(point.velocity, parameters.entered_min_speed_mps);
}

TEST_F(RoundaboutTrafficTest, ApproachesYieldTargetWhenTrafficBlocksTheEntryFarAhead) {
  parameters.entry_line_station_m = 50;
  observe(52, 0);
  pp::RoundaboutReference core(parameters);
  const auto path = plan(core);
  EXPECT_GT(path.points.front().velocity, 0.5);
  const double rear_target = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  for (const auto& point : path.points)
    if (point.pose.pose.position.x >= rear_target)
      EXPECT_DOUBLE_EQ(point.velocity, 0);
}

TEST_F(RoundaboutTrafficTest, WaitsBeforeTheLineThenLaunchesWhenItClears) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  observe(52, 0);
  pp::RoundaboutReference core(parameters);
  const auto waiting = plan(core);
  EXPECT_DOUBLE_EQ(waiting.points.front().velocity, 0);
  objects.objects.clear();
  predictions.fill(objects);
  const auto released = plan(core);
  EXPECT_GT(released.points.front().velocity, 0.5);
}

TEST_F(RoundaboutTrafficTest, ConfirmsGapUsingDistinctObservationTimes) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  observe(300, 300);
  pp::RoundaboutReference core(parameters);
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
  // A repeated observation cannot confirm the gap, even if calculation time advances.
  ego.header.stamp += ros::Duration(0.2);
  status.header.stamp = ego.header.stamp;
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
  ego.header.stamp += ros::Duration(0.2);
  status.header.stamp = ego.header.stamp;
  objects.header.stamp = ego.header.stamp;
  predictions.fill(objects);
  EXPECT_GT(plan(core).points.front().velocity, 0.5);
  // Before crossing the line, a new conflict closes the gate again.
  observe(52, 0);
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
}

TEST_F(RoundaboutTrafficTest, KeepsFollowingAfterCrossingAndRearmsAfterReturningUpstream) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 60;
  pp::RoundaboutReference core(parameters);
  EXPECT_GT(plan(core).points.front().velocity, 0.5);
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  observe(52, 0);
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
}

TEST_F(RoundaboutReferenceTest, ReservesCurveSpeedMarginAndProvidesAnActionableLaunchTarget) {
  ros::Time::init();
  parameters.beam_width = 32;
  ego.pose.pose.position = sourcePoint(886);
  const auto ahead = sourcePoint(886.1);
  const double yaw = std::atan2(ahead.y - ego.pose.pose.position.y, ahead.x - ego.pose.pose.position.x);
  ego.pose.pose.orientation.z = std::sin(yaw/2);
  ego.pose.pose.orientation.w = std::cos(yaw/2);
  for (double speed : {0.0, 4.0}) {
    pp::RoundaboutReference core(parameters);
    interfaces::CompetitionVehicleStatus status;
    status.header.stamp = ego.header.stamp;
    status.velocity_x_mps = speed;
    interfaces::Objects objects;
    objects.header = ego.header;
    pp::PredictedObjects predictions;
    predictions.fill(objects);
    const auto path = core.process(ego, status, objects, predictions, ego.header.stamp);
    EXPECT_GT(path.points.front().velocity, 0.5);
    // The launch command obeys nearby bends; departure can accelerate after
    // clearing them. Check the physical lateral bound along the whole path.
    double peak_after_bend = 0;
    for (std::size_t i = 1; i < path.points.size(); ++i) {
      const auto& a = path.points[i-1].pose.pose;
      const auto& b = path.points[i].pose.pose;
      const double ds = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
      const double k = std::abs(angle(heading(b.orientation)-heading(a.orientation)))/ds;
      const double v = path.points[i].velocity;
      EXPECT_LE(v*v*k, parameters.max_lateral_acceleration_mps2 + 0.03);
      if (i > 40) peak_after_bend = std::max(peak_after_bend, v);
    }
    if (speed == 0.0) EXPECT_GT(peak_after_bend, 3.8);
  }
}

TEST_F(RoundaboutTrafficTest, YieldsToCrossingNpcAndEntersAfterItsObservedGapClears) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  observe(53, -13, 0, 5, M_PI/2);
  pp::RoundaboutReference core(parameters);
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
  ego.header.stamp += ros::Duration(4);
  status.header.stamp = ego.header.stamp;
  objects.header.stamp = ego.header.stamp;
  observe(53, 12, 0, 5, M_PI/2);
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
  ego.header.stamp += ros::Duration(0.4);
  status.header.stamp = ego.header.stamp;
  objects.header.stamp = ego.header.stamp;
  observe(53, 14, 0, 5, M_PI/2);
  EXPECT_GT(plan(core).points.front().velocity, 0.5);
}

TEST_F(RoundaboutTrafficTest, FourSecondSearchStillChecksTheTerminalBrakingContinuation) {
  parameters.planning_horizon_sec = 4;
  status.velocity_x_mps = 5;
  // The NPC arrives at x=30 only after search end. Its prediction remains
  // available at 4.5 and 5.0 s and must constrain terminal continuation.
  observe(30, -50, 0, 10, M_PI/2);
  pp::RoundaboutReference occupied(parameters);
  const auto blocked = plan(occupied);
  observe(30, -100, 0, 10, M_PI/2);
  pp::RoundaboutReference clear(parameters);
  const auto free = plan(clear);
  double difference = 0;
  for (std::size_t i = 0; i < free.points.size(); ++i)
    difference += std::abs(free.points[i].velocity - blocked.points[i].velocity);
  EXPECT_GT(difference, 0.5);
}

TEST(RoundaboutReferenceInputPolicy, CorrectsOnlyStationaryNegativeNoiseAndPreservesSource) {
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps = -0.012;
  const auto source = std::make_shared<const interfaces::CompetitionVehicleStatus>(status);
  std::shared_ptr<const void> copy = source;
  node_pipeline::EditableInput input(copy, typeid(interfaces::CompetitionVehicleStatus));
  pp::RoundaboutReferenceInputs{}.correct(input, node_pipeline::Kind::OutRangeNegative);
  ASSERT_TRUE(copy);
  EXPECT_DOUBLE_EQ(std::static_pointer_cast<const interfaces::CompetitionVehicleStatus>(copy)->velocity_x_mps, 0);
  EXPECT_FLOAT_EQ(source->velocity_x_mps, -0.012f);
  status.velocity_x_mps = -0.1;
  copy = std::make_shared<const interfaces::CompetitionVehicleStatus>(status);
  pp::RoundaboutReferenceInputs{}.correct(input, node_pipeline::Kind::OutRangeNegative);
  EXPECT_FALSE(copy);
  copy = std::make_shared<const interfaces::Objects>();
  node_pipeline::EditableInput other(copy, typeid(interfaces::Objects));
  pp::RoundaboutReferenceInputs{}.correct(other, node_pipeline::Kind::OutRangeNegative);
  EXPECT_FALSE(copy);
}

TEST_F(RoundaboutTrafficTest, EntryBoundaryKeepsYieldingThenLatchesRollingTargets) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m - parameters.entry_stop_margin_m;
  observe(52, 0);
  pp::RoundaboutReference core(parameters);
  EXPECT_DOUBLE_EQ(plan(core).points.front().velocity, 0);
  ego.pose.pose.position.x = 50 - parameters.vehicle_front_m + 0.05;
  ego.header.stamp += ros::Duration(0.05);
  status.header.stamp = ego.header.stamp;
  objects.header.stamp = ego.header.stamp;
  observe(52, 0);
  const auto inside = plan(core);
  for (const auto& point : inside.points) EXPECT_GE(point.velocity, parameters.entered_min_speed_mps);
}

TEST_F(RoundaboutTrafficTest, AlternatingBlockedAndClearFramesCannotReintroduceStopAfterEntry) {
  parameters.entry_line_station_m = 50;
  ego.pose.pose.position.x = 60;
  pp::RoundaboutReference core(parameters);
  for (int frame = 0; frame < 20; ++frame) {
    ego.header.stamp += ros::Duration(0.05);
    status.header.stamp = ego.header.stamp;
    status.velocity_x_mps = frame % 3 == 0 ? 0.0 : 2.0;
    objects.header.stamp = ego.header.stamp;
    if (frame % 2 == 0) observe(64, 0);
    else { objects.objects.clear(); predictions.fill(objects); }
    const auto path = plan(core);
    EXPECT_GE(path.points.front().velocity, parameters.entered_min_speed_mps);
    for (const auto& point : path.points) {
      EXPECT_TRUE(std::isfinite(point.velocity));
      EXPECT_GE(point.velocity, parameters.entered_min_speed_mps);
      EXPECT_LE(point.velocity, parameters.desired_speed_mps + 1e-8);
    }
  }
}
