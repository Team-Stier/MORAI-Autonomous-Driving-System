#include <path_planner/cores/hybrid_astar.hpp>
#include <path_planner/parameters/hybrid_astar.hpp>
#include <path_planner/validators/hybrid_astar_inputs.hpp>
#include <path_planner/libraries/map_parameters.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <limits>

namespace pp = path_planner;
namespace {
constexpr double pi = 3.14159265358979323846;

lanelet::Lanelet road(lanelet::Id id, const lanelet::LineString3d& left,
                      const lanelet::LineString3d& right) {
  lanelet::Lanelet lane(id, left, right);
  lane.attributes()["subtype"] = "road";
  lane.attributes()["location"] = "urban";
  lane.attributes()["one_way"] = "yes";
  return lane;
}

pp::SharedMap straightMap(double half_width = 5, int segments = 16) {
  std::vector<lanelet::Point3d> left, right;
  for (int i = 0; i <= segments; ++i) {
    left.emplace_back(100 + 2*i, -10 + 5*i, half_width, 0);
    right.emplace_back(101 + 2*i, -10 + 5*i, -half_width, 0);
  }
  lanelet::Lanelets lanes;
  for (int i = 0; i < segments; ++i)
    lanes.push_back(road(300 + i,
        lanelet::LineString3d(200 + 2*i, {left[i], left[i + 1]}),
        lanelet::LineString3d(201 + 2*i, {right[i], right[i + 1]})));
  return pp::SharedMap({lanelet::utils::createMap(lanes)});
}

pp::Route route(const pp::SharedMap& map, std::initializer_list<lanelet::Id> ids) {
  lanelet::ConstLanelets lanes;
  for (auto id : ids) lanes.push_back(map.map->laneletLayer.get(id));
  return lanelet::routing::LaneletPath(lanes);
}

nav_msgs::Odometry egoAt(double x, double y = 0, double yaw = 0) {
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = x;
  ego.pose.pose.position.y = y;
  ego.pose.pose.orientation.z = std::sin(yaw / 2);
  ego.pose.pose.orientation.w = std::cos(yaw / 2);
  return ego;
}

interfaces::Objects obstacleAt(double x, double y, double length, double width,
                              double yaw = 0) {
  interfaces::Objects objects;
  objects.header.frame_id = "map";
  interfaces::Object object;
  object.id = 1;
  object.position.x = object.bbox_pose.position.x = x;
  object.position.y = object.bbox_pose.position.y = y;
  object.bbox_pose.orientation.z = std::sin(yaw / 2);
  object.bbox_pose.orientation.w = std::cos(yaw / 2);
  object.bbox_size.x = length;
  object.bbox_size.y = width;
  object.bbox_size.z = 1;
  objects.objects.push_back(object);
  return objects;
}

double heading(const geometry_msgs::Quaternion& q) {
  return std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
}

double pathLength(const interfaces::Path& path) {
  double length = 0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& a = path.points[i - 1].pose.pose.position;
    const auto& b = path.points[i].pose.pose.position;
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

using Corners = std::array<std::array<double, 2>, 4>;
Corners rectangle(double x, double y, double yaw, double front, double rear, double half_width) {
  Corners corners{{{{front, half_width}}, {{front, -half_width}},
                   {{-rear, -half_width}}, {{-rear, half_width}}}};
  for (auto& corner : corners) {
    const double local_x = corner[0], local_y = corner[1];
    corner[0] = x + local_x*std::cos(yaw) - local_y*std::sin(yaw);
    corner[1] = y + local_x*std::sin(yaw) + local_y*std::cos(yaw);
  }
  return corners;
}

bool overlap(const Corners& a, const Corners& b) {
  for (const auto* polygon : {&a, &b}) {
    for (std::size_t i = 0; i < 2; ++i) {
      const double axis_x = (*polygon)[i + 1][1] - (*polygon)[i][1];
      const double axis_y = (*polygon)[i][0] - (*polygon)[i + 1][0];
      double a_min = 1e100, a_max = -1e100, b_min = 1e100, b_max = -1e100;
      for (std::size_t j = 0; j < 4; ++j) {
        const double ap = a[j][0]*axis_x + a[j][1]*axis_y;
        const double bp = b[j][0]*axis_x + b[j][1]*axis_y;
        a_min = std::min(a_min, ap); a_max = std::max(a_max, ap);
        b_min = std::min(b_min, bp); b_max = std::max(b_max, bp);
      }
      if (a_max < b_min || b_max < a_min) return false;
    }
  }
  return true;
}

void expectGeometry(const interfaces::Path& path, const nav_msgs::Odometry& ego,
                    const pp::HybridAstarParameters& parameters) {
  ASSERT_GE(path.points.size(), 2u);
  const auto& start = path.points.front().pose.pose;
  EXPECT_NEAR(start.position.x, ego.pose.pose.position.x, 1e-8);
  EXPECT_NEAR(start.position.y, ego.pose.pose.position.y, 1e-8);
  EXPECT_NEAR(heading(start.orientation), heading(ego.pose.pose.orientation), 1e-8);
  EXPECT_TRUE(path.header.frame_id.empty());
  EXPECT_TRUE(path.header.stamp.isZero());
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    SCOPED_TRACE(i);
    const auto& point = path.points[i];
    const auto& pose = point.pose.pose;
    EXPECT_TRUE(std::isnan(point.velocity));
    EXPECT_TRUE(std::isnan(point.acceleration));
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    EXPECT_TRUE(point.pose.header.stamp.isZero());
    EXPECT_TRUE(std::isfinite(pose.position.x) && std::isfinite(pose.position.y));
    const auto& q = pose.orientation;
    EXPECT_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1, 1e-8);
    if (i == 0) continue;
    const auto& previous = path.points[i - 1].pose.pose;
    const double ds = std::hypot(pose.position.x - previous.position.x,
                                 pose.position.y - previous.position.y);
    EXPECT_GT(ds, 0);
    EXPECT_LE(ds, parameters.sample_step_m + 1e-3);
    const double dyaw = std::remainder(heading(q) - heading(previous.orientation), 2*pi);
    EXPECT_LE(std::abs(dyaw) / ds,
              std::tan(parameters.max_steer_rad) / parameters.wheelbase_m + 0.02);
  }
}

void expectClear(const interfaces::Path& path, const interfaces::Objects& objects,
                 const pp::HybridAstarParameters& parameters, double half_width) {
  for (const auto& point : path.points) {
    const auto& pose = point.pose.pose;
    const auto vehicle = rectangle(pose.position.x, pose.position.y, heading(pose.orientation),
        parameters.vehicle_front_m + parameters.collision_margin_m,
        parameters.vehicle_rear_m + parameters.collision_margin_m,
        parameters.vehicle_half_width_m + parameters.collision_margin_m);
    const auto road_vehicle = rectangle(pose.position.x, pose.position.y, heading(pose.orientation),
        parameters.vehicle_front_m, parameters.vehicle_rear_m,
        parameters.vehicle_half_width_m + parameters.road_margin_m);
    for (const auto& corner : road_vehicle) EXPECT_LE(std::abs(corner[1]), half_width + 1e-6);
    for (const auto& object : objects.objects) {
      const auto box = rectangle(object.bbox_pose.position.x, object.bbox_pose.position.y,
          heading(object.bbox_pose.orientation), object.bbox_size.x / 2,
          object.bbox_size.x / 2, object.bbox_size.y / 2);
      EXPECT_FALSE(overlap(vehicle, box));
    }
  }
}
}  // namespace

TEST(HybridAstar, FollowsCenterAndExtendsRouteAcrossSuccessors) {
  auto map = straightMap();
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto path = core.process(ego, interfaces::Objects{}, route(map, {300, 301, 302}),
                                 pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(pathLength(path), 50, 0.02);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 52, 0.02);
  for (const auto& point : path.points) EXPECT_NEAR(point.pose.pose.position.y, 0, 1e-8);
}

TEST(HybridAstar, SmoothlyMergesOffsetAndHeadingIntoCenter) {
  auto map = straightMap();
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2, 0.8, 0.08);
  const auto path = core.process(ego, interfaces::Objects{}, route(map, {302}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(pathLength(path), parameters.path_length_m, 0.1);
  EXPECT_NEAR(path.points.back().pose.pose.position.y, 0, 0.02);
  EXPECT_NEAR(heading(path.points.back().pose.pose.orientation), 0, 0.02);
  expectClear(path, interfaces::Objects{}, parameters, 5);
}

TEST(HybridAstar, FollowsCurvedLaneCenterWithoutObstacles) {
  std::vector<lanelet::Point3d> left, right, center;
  for (int i = -20; i <= 200; ++i) {
    const double s = i * 0.5, angle = std::max(0.0, s) / 50;
    const double x = s < 0 ? s : 50*std::sin(angle);
    const double y = s < 0 ? 0 : 50*(1 - std::cos(angle));
    const lanelet::Id id = 1000 + 3*(i + 20);
    left.emplace_back(id, x - 5*std::sin(angle), y + 5*std::cos(angle), 0);
    right.emplace_back(id + 1, x + 5*std::sin(angle), y - 5*std::cos(angle), 0);
    center.emplace_back(id + 2, x, y, 0);
  }
  auto lane = road(300, lanelet::LineString3d(200, left), lanelet::LineString3d(201, right));
  lane.setCenterline(lanelet::LineString3d(202, center));
  pp::SharedMap map({lanelet::utils::createMap({lane})});
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(0);
  const auto path = core.process(ego, interfaces::Objects{}, route(map, {300}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(pathLength(path), 50, 0.1);
  for (const auto& point : path.points) {
    const auto& p = point.pose.pose.position;
    EXPECT_NEAR(std::hypot(p.x, p.y - 50), 50, 0.03);
  }
  EXPECT_NEAR(heading(path.points.back().pose.pose.orientation), 1, 0.02);
}

TEST(HybridAstar, AvoidsRotatedStaticBoxAndRejoinsCenter) {
  auto map = straightMap(6);
  pp::HybridAstarParameters parameters;
  parameters.path_length_m = 40;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(20, 0, 3, 2, pi / 6);
  const auto path = core.process(ego, objects, route(map, {302, 303, 304}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GE(path.points.back().pose.pose.position.x, 35);
  EXPECT_NEAR(path.points.back().pose.pose.position.y, 0, 0.8);
  EXPECT_NEAR(heading(path.points.back().pose.pose.orientation), 0, 0.15);
  expectClear(path, objects, parameters, 6);
  double excursion = 0;
  for (const auto& point : path.points)
    excursion = std::max(excursion, std::abs(point.pose.pose.position.y));
  EXPECT_GT(excursion, 2);
}

TEST(HybridAstar, ReturnsMaximumSafeProgressBeforeFullWidthWall) {
  auto map = straightMap(3);
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(25, 0, 0.2, 20);
  const auto path = core.process(ego, objects, route(map, {302, 303, 304}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GT(path.points.back().pose.pose.position.x, 19);
  EXPECT_LT(path.points.back().pose.pose.position.x, 24);
  EXPECT_LT(pathLength(path), parameters.path_length_m);
  expectClear(path, objects, parameters, 3);
}

TEST(HybridAstar, ReturnsShortPathAtRoadEnd) {
  auto map = straightMap(3, 6);
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto path = core.process(ego, interfaces::Objects{}, route(map, {302, 303, 304, 305}),
                                 pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GT(path.points.back().pose.pose.position.x, 14);
  EXPECT_LT(pathLength(path), parameters.path_length_m);
  for (const auto& point : path.points) {
    const auto& pose = point.pose.pose;
    const auto corners = rectangle(pose.position.x, pose.position.y, heading(pose.orientation),
        parameters.vehicle_front_m + parameters.road_margin_m,
        parameters.vehicle_rear_m + parameters.road_margin_m,
        parameters.vehicle_half_width_m + parameters.road_margin_m);
    for (const auto& corner : corners) EXPECT_LE(corner[0], 20 + 1e-6);
  }
}

TEST(HybridAstar, BlocksSweptPredictionCrossingBetweenSnapshots) {
  auto map = straightMap(3);
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(20, -8, 1, 1);
  pp::PredictedObjects predictions;
  for (auto& prediction : predictions) prediction = obstacleAt(20, 8, 1, 1);
  const auto path = core.process(ego, objects, route(map, {302, 303, 304}), predictions);
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_LT(path.points.back().pose.pose.position.x, 19);
  interfaces::Objects sweep = obstacleAt(20, 0, 1, 17);
  expectClear(path, sweep, parameters, 3);
}

TEST(HybridAstar, ReturnsEmptyWhenInitialFootprintCollides) {
  auto map = straightMap();
  pp::HybridAstar core({}, map);
  const auto path = core.process(egoAt(2), obstacleAt(4, 0, 1, 1),
                                 route(map, {302}), pp::PredictedObjects{});
  EXPECT_TRUE(path.points.empty());
}

TEST(HybridAstar, ReturnsSafePartialPathWhenSearchBudgetEnds) {
  auto map = straightMap(6);
  pp::HybridAstarParameters parameters;
  parameters.max_expansions = 1;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(20, 0, 3, 2);
  const auto path = core.process(ego, objects, route(map, {302}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GT(pathLength(path), 0);
  EXPECT_LT(pathLength(path), parameters.path_length_m);
  expectClear(path, objects, parameters, 6);
}

TEST(HybridAstar, DetectsThinWallBetweenCoarseOutputSamples) {
  auto map = straightMap(3);
  pp::HybridAstarParameters parameters;
  parameters.sample_step_m = 2;
  parameters.primitive_length_m = 2;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(20.13, 0, 0.01, 20);
  const auto path = core.process(ego, objects, route(map, {302}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_FALSE(path.points.empty());
  EXPECT_LT(path.points.back().pose.pose.position.x, 20);
  expectClear(path, objects, parameters, 3);
}

TEST(HybridAstarInputs, RejectsMalformedGeometryWithoutRequiringMotionEstimates) {
  pp::HybridAstarInputs validator;
  const node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  auto objects = obstacleAt(20, 0, 3, 2);
  objects.objects.front().velocity.x = std::numeric_limits<double>::quiet_NaN();
  objects.objects.front().acceleration.x = std::numeric_limits<double>::quiet_NaN();
  validator(objects, limits, issues);
  EXPECT_TRUE(issues.empty());
  objects.objects.front().bbox_size.x = 0;
  validator(objects, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::OutRangeNegative), issues.end());
  issues.clear();
  auto ego = egoAt(2);
  ego.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  validator(ego, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::NaN), issues.end());
  issues.clear();
  objects = obstacleAt(20, 0, 3, 2);
  objects.objects.front().bbox_pose.orientation.w = 0;
  validator(objects, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::OutRangePositive), issues.end());
}

TEST(HybridAstarInputs, RequiresNonemptyRoute) {
  pp::HybridAstarInputs validator;
  const node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  validator(pp::Route{}, limits, issues);
  EXPECT_EQ(issues, (std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}));
  issues.clear();
  validator(pp::Route(lanelet::routing::LaneletPath(lanelet::ConstLanelets{})), limits, issues);
  EXPECT_EQ(issues, (std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}));
  issues.clear();
  auto map = straightMap();
  validator(route(map, {302}), limits, issues);
  EXPECT_TRUE(issues.empty());
}

TEST(HybridAstarInputs, AcceptsReorderedIdsAndRejectsPredictionIdentityOrHeaderChanges) {
  pp::HybridAstarInputs validator;
  const node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  auto objects = obstacleAt(20, 0, 3, 2);
  objects.header.stamp = ros::Time(10);
  auto other = obstacleAt(30, 1, 1, 1).objects.front();
  other.id = 2;
  objects.objects.push_back(other);
  pp::PredictedObjects predictions;
  for (auto& prediction : predictions) prediction = objects;
  std::reverse(predictions[4].objects.begin(), predictions[4].objects.end());
  validator(predictions, limits, issues);
  EXPECT_TRUE(issues.empty());
  predictions[9].objects.front().id = 3;
  validator(predictions, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::OutRangePositive), issues.end());
  issues.clear();
  predictions[9] = objects;
  predictions[7].header.stamp = ros::Time(11);
  validator(predictions, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::InvalidStamp), issues.end());
  issues.clear();
  predictions[7] = objects;
  predictions[2].header.frame_id = "base_link";
  validator(predictions, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::FrameMismatch), issues.end());
}

TEST(HybridAstarParameters, LoadsGeometryCoefficientsAndRejectsUnsafeValues) {
  const std::string yaml =
      "path_length_m: 50\n"
      "sample_step_m: 0.25\n"
      "primitive_length_m: 1\n"
      "grid_resolution_m: 0.5\n"
      "yaw_bins: 72\n"
      "max_expansions: 12000\n"
      "max_steer_rad: 0.698132\n"
      "steering_candidates_deg: [-19, -9, 0, 9, 19]\n"
      "wheelbase_m: 3\n"
      "vehicle_front_m: 3.845008\n"
      "vehicle_rear_m: 0.79\n"
      "vehicle_half_width_m: 1.080418\n"
      "collision_margin_m: 0.3\n"
      "road_margin_m: 0.05\n"
      "transition_lengths_m: [8, 12, 16]\n";
  const auto parameters = pp::loadHybridAstarParameters(node_pipeline::ParamReader::fromString(yaml));
  EXPECT_DOUBLE_EQ(parameters.path_length_m, 50);
  EXPECT_DOUBLE_EQ(parameters.sample_step_m, 0.25);
  EXPECT_EQ(parameters.max_expansions, 12000);
  EXPECT_EQ(parameters.steering_candidates_deg, (std::vector<double>{-19, -9, 0, 9, 19}));
  EXPECT_EQ(parameters.transition_lengths_m, (std::vector<double>{8, 12, 16}));
  const std::pair<std::string, std::string> invalid[] = {
      {"sample_step_m: 0.25", "sample_step_m: 0"},
      {"max_expansions: 12000", "max_expansions: 0"},
      {"collision_margin_m: 0.3", "collision_margin_m: .inf"},
      {"transition_lengths_m: [8, 12, 16]", "transition_lengths_m: []"},
      {"steering_candidates_deg: [-19, -9, 0, 9, 19]", "steering_candidates_deg: []"},
      {"steering_candidates_deg: [-19, -9, 0, 9, 19]", "steering_candidates_deg: [-41, 0, 41]"},
      {"steering_candidates_deg: [-19, -9, 0, 9, 19]", "steering_candidates_deg: [0, .inf]"},
      {"steering_candidates_deg: [-19, -9, 0, 9, 19]", "steering_candidates_deg: [0, 0, 19]"},
      {"steering_candidates_deg: [-19, -9, 0, 9, 19]", "steering_candidates_deg: [-19, 19]"}};
  for (const auto& entry : invalid) {
    SCOPED_TRACE(entry.second);
    auto bad_yaml = yaml;
    bad_yaml.replace(bad_yaml.find(entry.first), entry.first.size(), entry.second);
    EXPECT_THROW(pp::loadHybridAstarParameters(node_pipeline::ParamReader::fromString(bad_yaml)),
                 std::invalid_argument);
  }
}

TEST(HybridAstar, ChecksConnectorCurvatureDespiteCoarseConfiguredSamples) {
  auto map = straightMap(10);
  pp::HybridAstarParameters parameters;
  parameters.path_length_m = 30;
  parameters.sample_step_m = parameters.primitive_length_m = 5;
  parameters.transition_lengths_m = {8};
  parameters.max_expansions = 1;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2, 5);
  const auto path = core.process(ego, interfaces::Objects{}, route(map, {302}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_LT(pathLength(path), parameters.path_length_m);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& a = path.points[i - 1].pose.pose.position;
    const auto& b = path.points[i].pose.pose.position;
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), 0.251);
  }
  expectClear(path, interfaces::Objects{}, parameters, 10);
}

TEST(HybridAstar, ExtraSearchBudgetImprovesProgressWithoutPaddingLengthBehindWall) {
  auto map = straightMap(20);
  pp::HybridAstarParameters parameters;
  parameters.path_length_m = 30;
  parameters.max_expansions = 1;
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(18, 0, 0.2, 100);
  const auto global = route(map, {302});
  pp::HybridAstar small_budget(parameters, map);
  const auto baseline = small_budget.process(ego, objects, global, pp::PredictedObjects{});
  ASSERT_GE(baseline.points.size(), 2u);
  parameters.max_expansions = 500;
  pp::HybridAstar larger_budget(parameters, map);
  const auto path = larger_budget.process(ego, objects, global, pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GE(path.points.back().pose.pose.position.x,
            baseline.points.back().pose.pose.position.x - 1e-6);
  EXPECT_GT(path.points.back().pose.pose.position.x, 12);
  EXPECT_LT(pathLength(path), parameters.path_length_m - 1);
  expectClear(path, objects, parameters, 20);
}

TEST(HybridAstar, RejectsFootprintCrossingRoadNotchEvenWithAllCornersInsideRoad) {
  std::vector<lanelet::Point3d> left;
  const std::pair<double, double> vertices[] = {
      {-10, 5}, {3.9, 5}, {4, 0.2}, {4.1, 0.2}, {4.2, 5}, {70, 5}};
  for (std::size_t i = 0; i < 6; ++i)
    left.emplace_back(100 + i, vertices[i].first, vertices[i].second, 0);
  auto lane = road(300, lanelet::LineString3d(200, left), lanelet::LineString3d(201, {
      lanelet::Point3d(110, -10, -5, 0), lanelet::Point3d(111, 70, -5, 0)}));
  lane.setCenterline(lanelet::LineString3d(202, {
      lanelet::Point3d(112, -10, 0, 0), lanelet::Point3d(113, 70, 0, 0)}));
  pp::SharedMap map({lanelet::utils::createMap({lane})});
  pp::HybridAstar core({}, map);
  const auto path = core.process(egoAt(2), interfaces::Objects{}, route(map, {300}), pp::PredictedObjects{});
  EXPECT_TRUE(path.points.empty());
}

TEST(HybridAstar, StraightOnlyPrimitivesStopBeforeObstacleInsteadOfDetouring) {
  auto map = straightMap(6);
  pp::HybridAstarParameters parameters;
  parameters.path_length_m = 40;
  parameters.steering_candidates_deg = {0};
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(20, 0, 3, 2);
  const auto path = core.process(ego, objects, route(map, {302}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GT(path.points.back().pose.pose.position.x, 13);
  EXPECT_LT(path.points.back().pose.pose.position.x, 16);
  EXPECT_LT(pathLength(path), parameters.path_length_m);
  for (const auto& point : path.points) {
    EXPECT_NEAR(point.pose.pose.position.y, 0, 1e-8);
    EXPECT_NEAR(heading(point.pose.pose.orientation), 0, 1e-8);
  }
  expectClear(path, objects, parameters, 6);
}

TEST(HybridAstar, LocatesZeroSteeringWithReorderedPrimitiveList) {
  auto map = straightMap(6);
  pp::HybridAstarParameters parameters;
  parameters.steering_candidates_deg = {19, 0, -19};
  parameters.max_expansions = 1;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(20, 0, 3, 2);
  const auto path = core.process(ego, objects, route(map, {302}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_LT(pathLength(path), parameters.path_length_m);
  expectClear(path, objects, parameters, 6);
}

TEST(HybridAstar, StartsOnCurrentLaneWhenRouteIncludesPredecessor) {
  auto map = straightMap();
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(0.1);
  const auto path = core.process(ego, interfaces::Objects{}, route(map, {301, 302, 303}),
                                 pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 50.1, 0.02);
  for (std::size_t i = 1; i < path.points.size(); ++i)
    EXPECT_GE(path.points[i].pose.pose.position.x,
              path.points[i - 1].pose.pose.position.x - 1e-8);
}

TEST(HybridAstar, PreservesInvertedRouteDirectionOnBidirectionalLane) {
  auto lane = road(300, lanelet::LineString3d(200, {
      lanelet::Point3d(100, -10, 5, 0), lanelet::Point3d(101, 70, 5, 0)}),
      lanelet::LineString3d(201, {
      lanelet::Point3d(102, -10, -5, 0), lanelet::Point3d(103, 70, -5, 0)}));
  lane.attributes()["one_way"] = "no";
  lane.setCenterline(lanelet::LineString3d(202, {
      lanelet::Point3d(104, -10, 0, 0), lanelet::Point3d(105, 70, 0, 0)}));
  pp::SharedMap map({lanelet::utils::createMap({lane})});
  pp::HybridAstarParameters parameters;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(50, 0, pi);
  const pp::Route global = lanelet::routing::LaneletPath({lane.invert()});
  const auto path = core.process(ego, interfaces::Objects{}, global, pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 0, 0.02);
  for (std::size_t i = 1; i < path.points.size(); ++i)
    EXPECT_LE(path.points[i].pose.pose.position.x,
              path.points[i - 1].pose.pose.position.x + 1e-8);
}

TEST(HybridAstar, BlockedWideRoadDoesNotTurnBackTowardPredecessor) {
  auto map = straightMap(20);
  pp::HybridAstarParameters parameters;
  parameters.path_length_m = 30;
  parameters.max_expansions = 500;
  pp::HybridAstar core(parameters, map);
  const auto ego = egoAt(2);
  const auto objects = obstacleAt(18, 0, 0.2, 100);
  const auto path = core.process(ego, objects, route(map, {301, 302, 303}), pp::PredictedObjects{});
  expectGeometry(path, ego, parameters);
  ASSERT_GE(path.points.size(), 2u);
  expectClear(path, objects, parameters, 20);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    EXPECT_GE(path.points[i].pose.pose.position.x,
              path.points[i - 1].pose.pose.position.x - 1e-8);
    EXPECT_GE(std::cos(heading(path.points[i].pose.pose.orientation)), -1e-8);
  }
}

TEST(HybridAstar, DISABLED_MeasurePrimitiveBudget) {
  auto map = straightMap(6);
  const auto ego = egoAt(2, 0.8, 0.08);
  const auto global = route(map, {302, 303, 304});
  const std::vector<std::vector<double>> candidate_lists{
      {-19, -9, 0, 9, 19},
      {-40, -30, -20, -10, 0, 10, 20, 30, 40},
      {-40, -33, -26, -19, -12, -6, 0, 6, 12, 19, 26, 33, 40},
      {-40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40}};
  struct Case {
    const char* name;
    interfaces::Objects objects;
    pp::PredictedObjects predictions;
    interfaces::Objects collision_boxes;
  };
  std::vector<Case> cases{
      {"open", {}, {}, {}},
      {"detour", obstacleAt(20, 0, 3, 2, 0.25), {}, obstacleAt(20, 0, 3, 2, 0.25)},
      {"wall", obstacleAt(25, 0, 0.2, 30), {}, obstacleAt(25, 0, 0.2, 30)},
      {"crossing", obstacleAt(20, -8, 1, 1), {}, obstacleAt(20, 0, 1, 17)},
      {"80_offroad", {}, {}, {}}};
  for (auto& prediction : cases[3].predictions) prediction = obstacleAt(20, 8, 1, 1);
  for (int i = 0; i < 80; ++i) {
    auto object = obstacleAt(5 + i % 50, 20 + i / 50, 3, 2).objects.front();
    object.id = i + 1;
    cases[4].objects.objects.push_back(object);
  }
  cases[4].collision_boxes = cases[4].objects;
  std::cout << "primitive_count,max_expansions,case,worst_ms,progress_m,length_m,end_center_error_m\n";
  for (const auto& candidates : candidate_lists) {
    for (int budget : {200, 400, 800, 1600}) {
      pp::HybridAstarParameters parameters;
      parameters.steering_candidates_deg = candidates;
      parameters.max_expansions = budget;
      for (const auto& scenario : cases) {
        SCOPED_TRACE(std::string(scenario.name) + ", candidates=" +
                     std::to_string(candidates.size()) + ", budget=" + std::to_string(budget));
        pp::HybridAstar core(parameters, map);
        core.process(ego, scenario.objects, global, scenario.predictions);
        double worst_ms = 0;
        interfaces::Path path;
        for (int repeat = 0; repeat < 3; ++repeat) {
          const auto start = std::chrono::steady_clock::now();
          path = core.process(ego, scenario.objects, global, scenario.predictions);
          worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - start).count());
        }
        expectGeometry(path, ego, parameters);
        ASSERT_GE(path.points.size(), 2u);
        expectClear(path, scenario.collision_boxes, parameters, 6);
        const auto& end = path.points.back().pose.pose.position;
        EXPECT_GT(end.x, ego.pose.pose.position.x);
        EXPECT_LE(pathLength(path), parameters.path_length_m + 0.1);
        if (std::string(scenario.name) == "open" || std::string(scenario.name) == "80_offroad") {
          EXPECT_NEAR(pathLength(path), parameters.path_length_m, 0.1);
          EXPECT_NEAR(end.y, 0, 0.02);
        }
        std::cout << candidates.size() << ',' << budget << ',' << scenario.name << ','
                  << worst_ms << ',' << end.x - ego.pose.pose.position.x << ','
                  << pathLength(path) << ',' << std::abs(end.y) << '\n';
      }
    }
  }
}

TEST(HybridAstar, DISABLED_MeasureMapBudget) {
  std::ifstream map_file(std::string(NP_PACKAGE_DIR) + "/../map/kcity_lanelet2.osm");
  std::ifstream checkpoint_file(std::string(NP_PACKAGE_DIR) + "/../map/checkpoints.csv");
  ASSERT_TRUE(map_file.good() && checkpoint_file.good());
  const std::string map_data(std::istreambuf_iterator<char>(map_file), std::istreambuf_iterator<char>{});
  const std::string checkpoint_data(std::istreambuf_iterator<char>(checkpoint_file), std::istreambuf_iterator<char>{});
  auto map = pp::SharedMap({pp::readCheckedLaneletMap(map_data, checkpoint_data)});
  const auto lane_id = pp::readCheckpoints(checkpoint_data)[3].lanelet_id;
  const auto center = map.map->laneletLayer.get(lane_id).centerline();
  ASSERT_GE(center.size(), 2u);
  const auto i = (center.size() - 1) / 2;
  const double yaw = std::atan2(center[i + 1].y() - center[i].y(),
                               center[i + 1].x() - center[i].x());
  const auto ego = egoAt((center[i].x() + center[i + 1].x()) / 2,
                         (center[i].y() + center[i + 1].y()) / 2, yaw);
  const auto global = route(map, {lane_id});
  pp::HybridAstar reference_core({}, map);
  const auto reference = reference_core.process(ego, interfaces::Objects{}, global, pp::PredictedObjects{});
  ASSERT_GE(reference.points.size(), 2u);
  double station = 0;
  std::size_t obstacle_index = 1;
  for (; obstacle_index + 1 < reference.points.size() && station < 16; ++obstacle_index) {
    const auto& a = reference.points[obstacle_index - 1].pose.pose.position;
    const auto& b = reference.points[obstacle_index].pose.pose.position;
    station += std::hypot(b.x - a.x, b.y - a.y);
  }
  ASSERT_GE(station, 16);
  const auto& obstacle_pose = reference.points[obstacle_index].pose.pose;
  const auto obstacle = obstacleAt(obstacle_pose.position.x, obstacle_pose.position.y,
                                   1, 0.8, heading(obstacle_pose.orientation));
  const std::vector<std::vector<double>> candidate_lists{
      {-40, -30, -20, -10, 0, 10, 20, 30, 40},
      {-40, -33, -26, -19, -12, -6, 0, 6, 12, 19, 26, 33, 40},
      {-40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40}};
  std::cout << "kcity_primitive_count,max_expansions,obstacle,worst_ms,length_m,end_x,end_y\n";
  for (const auto& candidates : candidate_lists) for (int budget : {400, 800}) {
    pp::HybridAstarParameters parameters;
    parameters.steering_candidates_deg = candidates;
    parameters.max_expansions = budget;
    for (bool blocked : {false, true}) {
      SCOPED_TRACE(std::to_string(candidates.size()) + ", " + std::to_string(budget));
      const auto objects = blocked ? obstacle : interfaces::Objects{};
      pp::HybridAstar core(parameters, map);
      core.process(ego, objects, global, pp::PredictedObjects{});
      double worst_ms = 0;
      interfaces::Path path;
      for (int repeat = 0; repeat < 3; ++repeat) {
        const auto start = std::chrono::steady_clock::now();
        path = core.process(ego, objects, global, pp::PredictedObjects{});
        worst_ms = std::max(worst_ms, std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count());
      }
      expectGeometry(path, ego, parameters);
      ASSERT_GE(path.points.size(), 2u);
      if (blocked) for (const auto& point : path.points) {
        const auto& pose = point.pose.pose;
        EXPECT_FALSE(overlap(rectangle(pose.position.x, pose.position.y, heading(pose.orientation),
            parameters.vehicle_front_m + parameters.collision_margin_m,
            parameters.vehicle_rear_m + parameters.collision_margin_m,
            parameters.vehicle_half_width_m + parameters.collision_margin_m),
            rectangle(obstacle_pose.position.x, obstacle_pose.position.y,
                      heading(obstacle_pose.orientation), 0.5, 0.5, 0.4)));
      }
      const auto& end = path.points.back().pose.pose.position;
      std::cout << candidates.size() << ',' << budget << ',' << blocked << ',' << worst_ms
                << ',' << pathLength(path) << ',' << end.x << ',' << end.y << '\n';
    }
  }
}
