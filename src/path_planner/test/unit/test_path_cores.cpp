#include <path_planner/config.hpp>
#include <path_planner/core_set.hpp>
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
    EXPECT_DOUBLE_EQ(point.velocity, 0.0);
    EXPECT_DOUBLE_EQ(point.acceleration, 0.0);
  }
}

pp::GlobalPathParameters shortMap() {
  pp::GlobalPathParameters parameters;
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
  parameters.map = lanelet::utils::createMap(lanes);
  for (auto& checkpoint : parameters.checkpoints) {
    checkpoint.x_m = 1;
    checkpoint.lanelet_id = 300;
  }
  return parameters;
}
}  // namespace

TEST(PathCores, PassesOriginalMessagesAndExtendsShortRoute) {
  static_assert(std::is_same<std::tuple_element<2, pp::GlobalPath::Result>::type,
      lanelet::Optional<lanelet::routing::LaneletPath>>::value, "native optional path contract");
  const auto parameters = shortMap();
  pp::GlobalPath global(parameters);
  pp::LocalPath local({parameters.map, 20.0});
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(1);
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 2;
  ego.pose.pose.orientation.w = 1;
  interfaces::SectionAnnotation section;
  section.header = ego.header;
  for (int state : {0, 1, 1, 16, 0}) {
    section.state = state;
    const auto data = global.process(ego, section);
    EXPECT_EQ(&std::get<0>(data), &ego);
    EXPECT_EQ(&std::get<1>(data), &section);
    ASSERT_TRUE(static_cast<bool>(std::get<2>(data)));
    ASSERT_EQ(std::get<2>(data)->size(), 1u);
    const auto path = local.process(std::get<0>(data), std::get<1>(data), std::get<2>(data));
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
  auto parameters = shortMap();
  parameters.checkpoints.back().lanelet_id = 301;
  pp::GlobalPath global(parameters);
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = 2;
  ego.pose.pose.orientation.w = 1;
  interfaces::SectionAnnotation section;
  const auto route = std::get<2>(global.process(ego, section));
  ASSERT_TRUE(static_cast<bool>(route));
  ASSERT_EQ(route->size(), 2u);
  EXPECT_EQ(route->back().id(), 301);
}

TEST(PathCores, RejectsInvalidCheckpointLaneletIds) {
  const auto map = readFile(std::string(PATH_PLANNER_MAP_DIR) + "/kcity_lanelet2.osm");
  for (const std::string id : {"", "0", "-1", "100070.0", "100070x", "9223372036854775808", "9999999999"}) {
    std::string csv = "x,y,z,yaw,lanelet_id\n";
    for (int i = 0; i < 16; ++i) csv += "0,0,0,0," + id + "\n";
    EXPECT_ANY_THROW(pp::readGlobalPathParameters(map, csv)) << id;
  }
}

TEST(PathCores, RegisteredStagesHandleMapCheckpointsAndRepeatedSections) {
  const auto parameters = pp::readGlobalPathParameters(
      readFile(std::string(PATH_PLANNER_MAP_DIR) + "/kcity_lanelet2.osm"),
      readFile(std::string(PATH_PLANNER_MAP_DIR) + "/checkpoints.csv"));
  pp::CoreSet cores{pp::GlobalPath(parameters), pp::LocalPath({parameters.map, 20.0})};
  const pp::CoreIds ids{pp::FirstCoreId::GlobalPath, pp::SecondCoreId::LocalPath};
  const std::array<std::size_t, 2> required{{0, 4}};
  EXPECT_EQ(cores.requiredInputs(ids), required);
  pp::Data snapshot;
  EXPECT_FALSE(snapshot.hasInputs(required));
  auto ego = std::make_shared<nav_msgs::Odometry>();
  auto section = std::make_shared<interfaces::SectionAnnotation>();
  snapshot.get<0>().sample.message = ego;
  snapshot.get<4>().sample.message = section;
  ASSERT_TRUE(snapshot.hasInputs(required));
  ego->pose.pose.position.x = parameters.checkpoints.front().x_m;
  ego->pose.pose.position.y = parameters.checkpoints.front().y_m;
  ego->pose.pose.orientation.z = std::sin(parameters.checkpoints.front().yaw_rad / 2);
  ego->pose.pose.orientation.w = std::cos(parameters.checkpoints.front().yaw_rad / 2);
  const auto route = std::get<2>(cores.global_path.process(*ego, *section));
  ASSERT_TRUE(static_cast<bool>(route));
  lanelet::Ids route_ids;
  for (const auto& lane : *route) route_ids.push_back(lane.id());
  for (const auto id : {100070, 110022})
    EXPECT_NE(std::find(route_ids.begin(), route_ids.end(), id), route_ids.end());
  for (const auto id : {200312, 140377})
    EXPECT_EQ(std::find(route_ids.begin(), route_ids.end(), id), route_ids.end());
  for (std::size_t i = 0; i < parameters.checkpoints.size(); ++i) {
    const auto& checkpoint = parameters.checkpoints[i];
    ego->pose.pose.position.x = checkpoint.x_m;
    ego->pose.pose.position.y = checkpoint.y_m;
    ego->pose.pose.orientation.z = std::sin(checkpoint.yaw_rad / 2);
    ego->pose.pose.orientation.w = std::cos(checkpoint.yaw_rad / 2);
    section->state = i;
    expectPayload(cores.process(ids, snapshot));
    section->state = i + 1;
    const auto path = cores.process(ids, snapshot);
    expectPayload(path);
    EXPECT_EQ(cores.process(ids, snapshot), path);
    section->state = 0;
    EXPECT_EQ(cores.process(ids, snapshot), path);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
