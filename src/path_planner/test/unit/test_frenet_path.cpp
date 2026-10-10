#include <path_planner/cores/frenet_path.hpp>
#include <path_planner/parameters/frenet_path.hpp>
#include <path_planner/validators/frenet_inputs.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace pp = path_planner;
namespace {
lanelet::Lanelet road(lanelet::Id id, const lanelet::LineString3d& left,
                      const lanelet::LineString3d& right) {
  lanelet::Lanelet lane(id, left, right);
  lane.attributes()["subtype"] = "road";
  lane.attributes()["location"] = "urban";
  lane.attributes()["one_way"] = "yes";
  return lane;
}

pp::SharedMap straightMap() {
  std::vector<lanelet::Point3d> left, right;
  for (int i = 0; i <= 12; ++i) {
    left.emplace_back(100 + 2*i, 5.0*i, 3, 0.5*i);
    right.emplace_back(101 + 2*i, 5.0*i, -3, 0.5*i);
  }
  lanelet::Lanelets lanes;
  for (int i = 0; i < 12; ++i)
    lanes.push_back(road(300 + i,
        lanelet::LineString3d(200 + 2*i, {left[i], left[i + 1]}),
        lanelet::LineString3d(201 + 2*i, {right[i], right[i + 1]})));
  return pp::SharedMap({lanelet::utils::createMap(lanes)});
}

pp::SharedMap parallelMap(bool legal) {
  lanelet::LineString3d separator(200, {
      lanelet::Point3d(100, 0, 2, 0), lanelet::Point3d(101, 60, 2, 0)});
  separator.attributes()["type"] = "line_thin";
  separator.attributes()["subtype"] = legal ? "dashed" : "solid";
  auto own = road(300, separator, lanelet::LineString3d(201, {
      lanelet::Point3d(102, 0, -2, 0), lanelet::Point3d(103, 60, -2, 0)}));
  auto neighbor = road(301, lanelet::LineString3d(202, {
      lanelet::Point3d(104, 0, 6, 0), lanelet::Point3d(105, 60, 6, 0)}), separator);
  auto before = road(400, lanelet::LineString3d(203, {
      lanelet::Point3d(106, -30, 3, 0), lanelet::Point3d(107, -10, 3, 0)}),
      lanelet::LineString3d(204, {
      lanelet::Point3d(108, -30, -3, 0), lanelet::Point3d(109, -10, -3, 0)}));
  auto after = road(401, lanelet::LineString3d(205, {
      lanelet::Point3d(110, 80, 3, 0), lanelet::Point3d(111, 100, 3, 0)}),
      lanelet::LineString3d(206, {
      lanelet::Point3d(112, 80, -3, 0), lanelet::Point3d(113, 100, -3, 0)}));
  return pp::SharedMap({lanelet::utils::createMap({own, neighbor, before, after})});
}

pp::SharedMap branchingMap() {
  lanelet::Point3d left_start(100, 0, 3, 0), left_junction(101, 10, 3, 0);
  lanelet::Point3d right_start(102, 0, -3, 0), right_junction(103, 10, -3, 0);
  auto incoming = road(300, lanelet::LineString3d(200, {left_start, left_junction}),
      lanelet::LineString3d(201, {right_start, right_junction}));
  auto straight = road(301, lanelet::LineString3d(202, {
      left_junction, lanelet::Point3d(104, 50, 3, 0)}),
      lanelet::LineString3d(203, {right_junction, lanelet::Point3d(105, 50, -3, 0)}));
  std::vector<lanelet::Point3d> left{left_junction}, right{right_junction};
  for (int i = 1; i <= 40; ++i) {
    const double y = 0.02*i*i;
    left.emplace_back(106 + 2*i, 10 + i, y + 3, 0);
    right.emplace_back(107 + 2*i, 10 + i, y - 3, 0);
  }
  auto curve = road(302, lanelet::LineString3d(204, left), lanelet::LineString3d(205, right));
  return pp::SharedMap({lanelet::utils::createMap({incoming, straight, curve})});
}

pp::Route route(const pp::SharedMap& shared, std::initializer_list<lanelet::Id> ids) {
  lanelet::ConstLanelets lanes;
  for (const auto id : ids) lanes.push_back(shared.map->laneletLayer.get(id));
  return lanelet::routing::LaneletPath(lanes);
}

nav_msgs::Odometry egoAt(double x, double y = 0) {
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = x;
  ego.pose.pose.position.y = y;
  ego.pose.pose.orientation.w = 1;
  return ego;
}

interfaces::Objects obstacleAt(double x, double y, double length, double width, double yaw = 0) {
  interfaces::Objects objects;
  interfaces::Object obstacle;
  obstacle.position.x = obstacle.bbox_pose.position.x = x;
  obstacle.position.y = obstacle.bbox_pose.position.y = y;
  obstacle.bbox_pose.orientation.z = std::sin(yaw / 2);
  obstacle.bbox_pose.orientation.w = std::cos(yaw / 2);
  obstacle.bbox_size.x = length;
  obstacle.bbox_size.y = width;
  obstacle.bbox_size.z = 1;
  objects.objects.push_back(obstacle);
  return objects;
}

const char* parametersYaml =
    "path_length_m: 20\n"
    "sample_step_m: 0.5\n"
    "transition_lengths_m: [8, 12, 16]\n"
    "lateral_offsets_m: [-0.5, 0, 0.5]\n"
    "vehicle_front_m: 3.845008\n"
    "vehicle_rear_m: 0.79\n"
    "vehicle_half_width_m: 1.080418\n"
    "obstacle_margin_m: 0.2\n"
    "max_curvature_inv_m: 0.2797\n";
}  // namespace

TEST(FrenetPath, StartsAtEgoBeyondPredecessorAndExtendsAcrossSuccessors) {
  auto shared = straightMap();
  pp::FrenetPath core({}, shared);
  const auto global = route(shared, {300, 301, 302});
  for (double x : {5.2, 7.0, 12.0}) {
    SCOPED_TRACE(x);
    const auto path = core.process(egoAt(x), interfaces::Objects{}, global);
    ASSERT_GE(path.points.size(), 2u);
    EXPECT_NEAR(path.points.front().pose.pose.position.x, x, 1e-8);
    EXPECT_NEAR(path.points.front().pose.pose.position.y, 0, 1e-8);
    EXPECT_NEAR(path.points.front().pose.pose.position.z, x / 10, 1e-8);
    EXPECT_NEAR(path.points.back().pose.pose.position.x, x + 20, 1e-8);
    EXPECT_NEAR(path.points.back().pose.pose.position.z, (x + 20) / 10, 1e-8);
    double length = 0;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto& point = path.points[i];
      EXPECT_TRUE(std::isnan(point.velocity));
      EXPECT_NEAR(point.pose.pose.position.y, 0, 1e-8);
      const auto& q = point.pose.pose.orientation;
      EXPECT_NEAR(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1, 1e-8);
      if (i == 0) continue;
      const auto& previous = path.points[i - 1].pose.pose.position;
      const auto& p = point.pose.pose.position;
      EXPECT_GT(p.x, previous.x);
      length += std::hypot(p.x - previous.x, p.y - previous.y);
    }
    EXPECT_NEAR(length, 20, 1e-8);
  }
}

TEST(FrenetPath, StartsInActualEgoLaneAndChangesToLegalGlobalNeighbor) {
  auto shared = parallelMap(true);
  pp::FrenetPathParameters parameters;
  parameters.lateral_offsets_m = {0};
  pp::FrenetPath core(parameters, shared);
  const auto path = core.process(egoAt(2), interfaces::Objects{}, route(shared, {301}));
  ASSERT_GE(path.points.size(), 3u);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 2, 1e-8);
  EXPECT_NEAR(path.points.front().pose.pose.position.y, 0, 1e-8);
  EXPECT_NEAR(path.points.back().pose.pose.position.y, 4, 1e-8);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& p = path.points[i].pose.pose.position;
    const auto& previous = path.points[i - 1].pose.pose.position;
    EXPECT_LT(std::hypot(p.x - previous.x, p.y - previous.y), 1.0);
  }
}

TEST(FrenetPath, SolidBoundaryDoesNotCreateEscapeCandidate) {
  auto shared = parallelMap(false);
  pp::FrenetPath core({}, shared);
  EXPECT_TRUE(core.process(egoAt(2), obstacleAt(10, 0, 1, 40),
      route(shared, {301})).points.empty());
}

TEST(FrenetPath, ReturnsEmptyWhenEveryCandidateIsBlocked) {
  auto shared = straightMap();
  pp::FrenetPath core({}, shared);
  EXPECT_TRUE(core.process(egoAt(2), obstacleAt(10, 0, 4, 40),
      route(shared, {300, 301, 302})).points.empty());
}

TEST(FrenetPath, FindsUnblockedFollowingBranch) {
  auto shared = branchingMap();
  pp::FrenetPath core({}, shared);
  const auto path = core.process(egoAt(2), obstacleAt(21, 0, 0.2, 0.2), route(shared, {300}));
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GT(path.points.back().pose.pose.position.y, 1.5);
}

TEST(FrenetPath, ChoosesFollowingBranchClosestToGlobalCentreline) {
  auto shared = branchingMap();
  pp::FrenetPath core({}, shared);
  const auto path = core.process(egoAt(2), interfaces::Objects{}, route(shared, {300, 302}));
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GT(path.points.back().pose.pose.position.y, 1.5);
}

TEST(FrenetPath, GlobalCostDoesNotJoinDisconnectedLaneletEndpoints) {
  auto shared = parallelMap(true);
  pp::FrenetPathParameters parameters;
  parameters.lateral_offsets_m = {0};
  pp::FrenetPath core(parameters, shared);
  const auto path = core.process(egoAt(2), interfaces::Objects{}, route(shared, {400, 401, 301}));
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.back().pose.pose.position.y, 4, 1e-8);
}

TEST(FrenetPath, DetectsThinObstacleBetweenOutputSamples) {
  auto shared = straightMap();
  pp::FrenetPathParameters parameters;
  parameters.sample_step_m = 10;
  parameters.lateral_offsets_m = {0};
  pp::FrenetPath core(parameters, shared);
  const auto global = route(shared, {300, 301, 302});
  EXPECT_FALSE(core.process(egoAt(2), interfaces::Objects{}, global).points.empty());
  EXPECT_TRUE(core.process(egoAt(2), obstacleAt(6, 0, 0.01, 40), global).points.empty());
}

TEST(FrenetPath, UsesObservedBoundingBoxRotation) {
  auto shared = straightMap();
  pp::FrenetPathParameters parameters;
  parameters.lateral_offsets_m = {0};
  pp::FrenetPath core(parameters, shared);
  const auto global = route(shared, {300, 301, 302});
  EXPECT_FALSE(core.process(egoAt(2), obstacleAt(10, 3, 8, 0.2), global).points.empty());
  EXPECT_TRUE(core.process(egoAt(2), obstacleAt(10, 3, 8, 0.2, std::acos(-1.0) / 4),
      global).points.empty());
}

TEST(FrenetPathInputs, RejectsInvalidGeometryAndAllowsZeroSizedObservation) {
  const pp::FrenetInputs validator;
  const node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  auto shared = straightMap();
  auto ego = egoAt(0);
  auto objects = obstacleAt(10, 0, 0, 0);
  objects.objects.front().bbox_size.z = 0;
  validator(ego, limits, issues);
  validator(objects, limits, issues);
  validator(route(shared, {300}), limits, issues);
  EXPECT_TRUE(issues.empty());
  ego.pose.pose.orientation.w = 0;
  validator(ego, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::OutRangePositive), issues.end());
  issues.clear();
  objects.objects.front().bbox_size.x = -1;
  validator(objects, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::OutRangeNegative), issues.end());
  issues.clear();
  objects.objects.front().bbox_size.x = std::numeric_limits<double>::infinity();
  validator(objects, limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::Inf), issues.end());
  issues.clear();
  validator(pp::Route{}, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}, issues);
  issues.clear();
  auto degenerate = road(500, lanelet::LineString3d(501, {
      lanelet::Point3d(502, 0, 1, 0), lanelet::Point3d(503, 0, 1, 0)}),
      lanelet::LineString3d(504, {
      lanelet::Point3d(505, 0, -1, 0), lanelet::Point3d(506, 0, -1, 0)}));
  validator(pp::Route(lanelet::routing::LaneletPath({degenerate})), limits, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::OutRangePositive), issues.end());
}

TEST(FrenetPathParameters, LoadsCoefficientsAndRejectsUnsafeSampling) {
  const auto parameters = pp::loadFrenetPathParameters(node_pipeline::ParamReader::fromString(parametersYaml));
  EXPECT_DOUBLE_EQ(parameters.path_length_m, 20);
  EXPECT_EQ(parameters.transition_lengths_m, (std::vector<double>{8, 12, 16}));
  EXPECT_EQ(parameters.lateral_offsets_m, (std::vector<double>{-0.5, 0, 0.5}));
  const std::pair<std::string, std::string> invalid[] = {
      {"sample_step_m: 0.5", "sample_step_m: 0"},
      {"obstacle_margin_m: 0.2", "obstacle_margin_m: -0.2"},
      {"transition_lengths_m: [8, 12, 16]", "transition_lengths_m: []"},
      {"transition_lengths_m: [8, 12, 16]", "transition_lengths_m: [21]"},
      {"lateral_offsets_m: [-0.5, 0, 0.5]", "lateral_offsets_m: []"}};
  for (const auto& entry : invalid) {
    SCOPED_TRACE(entry.second);
    std::string yaml(parametersYaml);
    yaml.replace(yaml.find(entry.first), entry.first.size(), entry.second);
    EXPECT_THROW(pp::loadFrenetPathParameters(node_pipeline::ParamReader::fromString(yaml)),
        std::invalid_argument);
  }
}
