#include <path_planner/cores/yielding_hybrid_astar.hpp>
#include <path_planner/parameters/yielding_hybrid_astar.hpp>
#include <path_planner/libraries/types.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iterator>
#include <string>

namespace pp = path_planner;
namespace {
pp::YieldingHybridAstarParameters configuredParameters() {
  return pp::loadYieldingHybridAstarParameters(node_pipeline::ParamReader::fromFile(
      std::string(NP_PACKAGE_DIR) + "/config/path_planner/parameters/yielding_hybrid_astar.yaml", NP_PACKAGE_DIR));
}

struct Scene {
  pp::SharedMap map{{}};
  pp::Route route;
  nav_msgs::Odometry ego;
};

Scene scene(double x = 0, double y = 0, double yaw = 0, double curvature = 0) {
  lanelet::Points3d left, right, center;
  for (int i = 0; i <= 80; ++i) {
    const double s = i - 10.0, angle = yaw + curvature*s;
    const double cx = x + (curvature == 0 ? s*std::cos(yaw) :
        (std::sin(angle) - std::sin(yaw))/curvature);
    const double cy = y + (curvature == 0 ? s*std::sin(yaw) :
        (std::cos(yaw) - std::cos(angle))/curvature);
    left.emplace_back(3*i + 1, cx - 10*std::sin(angle), cy + 10*std::cos(angle), 0);
    right.emplace_back(3*i + 2, cx + 10*std::sin(angle), cy - 10*std::cos(angle), 0);
    center.emplace_back(3*i + 3, cx, cy, 0);
  }
  lanelet::Lanelet lane(301, lanelet::LineString3d(302, left), lanelet::LineString3d(303, right));
  lane.attributes()["subtype"] = "road";
  lane.attributes()["location"] = "urban";
  lane.attributes()["one_way"] = "yes";
  lane.setCenterline(lanelet::LineString3d(304, center));
  Scene result;
  result.map = pp::SharedMap({lanelet::utils::createMap({lane})});
  result.route = lanelet::routing::LaneletPath({lane});
  result.ego.header.stamp = ros::Time(10);
  result.ego.pose.pose.position.x = x;
  result.ego.pose.pose.position.y = y;
  result.ego.pose.pose.orientation.z = std::sin(yaw/2);
  result.ego.pose.pose.orientation.w = std::cos(yaw/2);
  result.ego.twist.twist.linear.x = 9;
  result.ego.twist.twist.angular.z = 9*curvature;
  return result;
}

interfaces::Objects object(const Scene& s, double forward, double lateral, double lateral_speed,
                           bool have_velocity = true, double length = 0.5, double width = 0.4) {
  const auto& q = s.ego.pose.pose.orientation;
  const double yaw = 2*std::atan2(q.z, q.w), c = std::cos(yaw), sn = std::sin(yaw);
  interfaces::Objects objects;
  objects.header.frame_id = "map";
  objects.header.stamp = s.ego.header.stamp;
  interfaces::Object value;
  value.id = 1;
  value.label = "unknown";
  value.position.x = value.bbox_pose.position.x = s.ego.pose.pose.position.x + c*forward - sn*lateral;
  value.position.y = value.bbox_pose.position.y = s.ego.pose.pose.position.y + sn*forward + c*lateral;
  value.bbox_pose.orientation = q;
  value.bbox_size.x = length; value.bbox_size.y = width; value.bbox_size.z = 1.1;
  value.velocity_available = have_velocity;
  value.velocity.x = -sn*lateral_speed;
  value.velocity.y = c*lateral_speed;
  objects.objects.push_back(value);
  return objects;
}

void expectYield(const interfaces::Path& path) {
  ASSERT_EQ(2u, path.points.size());
  for (const auto& point : path.points) {
    EXPECT_DOUBLE_EQ(0, point.velocity);
    EXPECT_DOUBLE_EQ(0, point.acceleration);
    EXPECT_TRUE(std::isfinite(point.pose.pose.position.x));
    EXPECT_TRUE(std::isfinite(point.pose.pose.position.y));
  }
  const auto& a = path.points[0].pose.pose.position;
  const auto& b = path.points[1].pose.pose.position;
  EXPECT_GT(std::hypot(a.x - b.x, a.y - b.y), 0);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
}

void expectProceed(const interfaces::Path& path) {
  ASSERT_GT(path.points.size(), 2u);
  EXPECT_TRUE(std::isnan(path.points.front().velocity));
}
}  // namespace

TEST(YieldingHybridAstar, YieldsToUnknownCrossingDespiteStaticForecastsAndAnAvailableBypass) {
  auto s = scene();
  pp::YieldingHybridAstar core(configuredParameters(), s.map);
  const auto objects = object(s, 12, 3, -2);
  pp::PredictedObjects predictions;
  predictions.fill(objects);
  expectYield(core.process(s.ego, objects, s.route, predictions));
}

TEST(YieldingHybridAstar, NativeFirstObservationFixturesYieldWithoutMotionSubstitution) {
  for (int fixture = 0; fixture < 2; ++fixture) {
    auto s = fixture == 0 ? scene(-60.3415, -19.73245, 1.57627) :
                            scene(-61.59047, -23.30592, 1.537658);
    s.ego.twist.twist.linear.x = fixture == 0 ? 7.47143 : 6.65528;
    auto objects = object(s, 0, 0, 0, false);
    auto& pedestrian = objects.objects.front();
    pedestrian.position.x = pedestrian.bbox_pose.position.x = fixture == 0 ? -58.62554 : -57.57357;
    pedestrian.position.y = pedestrian.bbox_pose.position.y = fixture == 0 ? -11.25063 : -11.19768;
    pedestrian.bbox_size.x = fixture == 0 ? 0.441959 : 0.39397;
    pedestrian.bbox_size.y = fixture == 0 ? 0.172228 : 0.12689;
    pedestrian.bbox_size.z = fixture == 0 ? 1.100725 : 0.81053;
    pedestrian.bbox_pose.orientation.z = fixture == 0 ? -0.44835306590677204 : -0.5634020979722246;
    pedestrian.bbox_pose.orientation.w = fixture == 0 ? 0.8938565479382011 : 0.8261828344867108;
    const auto before = objects;
    pp::YieldingHybridAstar core(configuredParameters(), s.map);
    expectYield(core.process(s.ego, objects, s.route, {}));
    EXPECT_EQ(before, objects);
    EXPECT_FALSE(objects.objects.front().velocity_available);
  }
}

TEST(YieldingHybridAstar, RotatesAndTranslatesCrossingAndColdStartGeometry) {
  for (double yaw : {0.0, 0.7, -2.1}) {
    auto s = scene(100, -200, yaw);
    for (bool have_velocity : {false, true}) {
      pp::YieldingHybridAstar core(configuredParameters(), s.map);
      expectYield(core.process(s.ego, object(s, 12, 3, -2, have_velocity), s.route, {}));
    }
  }
}

TEST(YieldingHybridAstar, FollowsCurvedBrakingCorridor) {
  auto s = scene(100, -200, 0.7, 0.04);
  pp::YieldingHybridAstar core(configuredParameters(), s.map);
  // At s=10 the curved lane has moved almost 2 m laterally.
  expectYield(core.process(s.ego, object(s, 10, 4, -2), s.route, {}));
}

TEST(YieldingHybridAstar, DoesNotYieldToBehindDivergingOrDistantObjects) {
  auto s = scene();
  for (const auto& objects : {object(s, -8, 3, -2), object(s, 12, 3, 2), object(s, 40, 3, -2)}) {
    pp::YieldingHybridAstar core(configuredParameters(), s.map);
    expectProceed(core.process(s.ego, objects, s.route, {}));
  }
}

TEST(YieldingHybridAstar, KeepsStaticBoxBypassAndDoesNotYieldToColdLargeBox) {
  auto s = scene();
  for (bool have_velocity : {false, true}) {
    pp::YieldingHybridAstar core(configuredParameters(), s.map);
    expectProceed(core.process(s.ego, object(s, 20, 0, 0, have_velocity, 3, 2), s.route, {}));
  }
}

TEST(YieldingHybridAstar, HoldsAcrossBriefDropoutThenReleasesAndHandlesReappearance) {
  auto s = scene();
  pp::YieldingHybridAstar core(configuredParameters(), s.map);
  expectYield(core.process(s.ego, object(s, 12, 3, -2), s.route, {}));
  s.ego.header.stamp = ros::Time(10, 400000000);
  expectYield(core.process(s.ego, {}, s.route, {}));
  s.ego.header.stamp = ros::Time(11);
  expectProceed(core.process(s.ego, {}, s.route, {}));
  s.ego.header.stamp = ros::Time(12);
  expectYield(core.process(s.ego, object(s, 12, 3, 0, false), s.route, {}));
}

TEST(YieldingHybridAstar, KeepsApproachCorridorAfterStoppingUntilCrossingClears) {
  auto s = scene();
  pp::YieldingHybridAstar core(configuredParameters(), s.map);
  expectYield(core.process(s.ego, object(s, 12, 3, -2), s.route, {}));
  s.ego.twist.twist.linear.x = 0;
  s.ego.header.stamp = ros::Time(12);
  expectYield(core.process(s.ego, object(s, 12, 2, -2), s.route, {}));
  s.ego.header.stamp = ros::Time(12, 400000000);
  expectYield(core.process(s.ego, object(s, 12, -4, -2), s.route, {}));
  s.ego.header.stamp = ros::Time(13);
  expectProceed(core.process(s.ego, object(s, 12, -4, -2), s.route, {}));
}

TEST(YieldingHybridAstar, KeepsCurvedApproachCorridorAfterStopping) {
  auto s = scene(0, 0, 0, 0.06);
  auto objects = object(s, 11, 5, 0);
  objects.objects.front().velocity.x = 2;
  pp::YieldingHybridAstar core(configuredParameters(), s.map);
  expectYield(core.process(s.ego, objects, s.route, {}));
  s.ego.twist.twist.linear.x = 0;
  s.ego.twist.twist.angular.z = 0;
  s.ego.header.stamp = ros::Time(12);
  objects.header.stamp = s.ego.header.stamp;
  expectYield(core.process(s.ego, objects, s.route, {}));
}

TEST(YieldingHybridAstarParameters, LoadsConfiguredBoundsAndRejectsInvalidYieldCoefficients) {
  const auto parameters = configuredParameters();
  EXPECT_DOUBLE_EQ(0.4, parameters.collision_margin_m);
  EXPECT_DOUBLE_EQ(1, parameters.yield_reaction_sec);
  EXPECT_DOUBLE_EQ(3, parameters.yield_unknown_lateral_mps);
  EXPECT_DOUBLE_EQ(1.5, parameters.yield_compact_max_m);
  EXPECT_FALSE(parameters.checkpoints.empty());
  std::ifstream file(std::string(NP_PACKAGE_DIR) + "/config/path_planner/parameters/yielding_hybrid_astar.yaml");
  ASSERT_TRUE(file.good());
  const std::string yaml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  for (const char* key : {"yield_reaction_sec", "yield_unknown_lateral_mps", "yield_compact_max_m"}) {
    for (const char* value : {"0", "-.5", ".inf", ".nan"}) {
      auto invalid = yaml;
      const auto begin = invalid.find(std::string(key) + ": ");
      ASSERT_NE(std::string::npos, begin);
      const auto number = begin + std::string(key).size() + 2;
      invalid.replace(number, invalid.find('\n', number) - number, value);
      EXPECT_THROW(pp::loadYieldingHybridAstarParameters(
          node_pipeline::ParamReader::fromString(invalid, NP_PACKAGE_DIR)), std::invalid_argument);
    }
  }
}
