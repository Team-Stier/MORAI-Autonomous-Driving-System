#include <path_planner/config.hpp>
#include <path_planner/components.hpp>
#include <path_planner/core_dispatch.hpp>
#include <path_planner/cores/prediction_pass_through.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <iterator>
#include <type_traits>

namespace pp = path_planner;
namespace np = node_pipeline;
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
  auto samples = core.process(objects, lanelet::Optional<lanelet::routing::LaneletPath>{});
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
  const auto samples = core.process(objects, lanelet::Optional<lanelet::routing::LaneletPath>{});
  for (const auto& sample : samples) EXPECT_EQ(sample, objects);
}

TEST(PathCores, ReturnsOnlyRouteAndExtendsShortRoute) {
  static_assert(std::is_same<pp::GlobalPath::Result,
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
    const auto route = global.process(ego, section);
    ASSERT_TRUE(static_cast<bool>(route));
    ASSERT_EQ(route->size(), 1u);
    const auto path = local.process(ego, section, route, pp::PredictedObjects{});
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
  const auto route = global.process(ego, section);
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
  pp::CoreSet cores{pp::GlobalPath(parameters), pp::PredictionPassThrough{}, pp::LocalPath({parameters.map, 20.0})};
  const std::array<std::size_t, 2> required{{0, 4}};
  pp::Data snapshot;
  EXPECT_FALSE(snapshot.hasInputs(required));
  auto ego = std::make_shared<nav_msgs::Odometry>();
  auto section = std::make_shared<interfaces::SectionAnnotation>();
  auto objects = std::make_shared<interfaces::Objects>();
  objects->header.stamp.fromSec(1.0);
  objects->header.frame_id = "base_link";
  snapshot.get<0>().sample.message = ego;
  snapshot.get<4>().sample.message = section;
  snapshot.get<2>().sample.message = objects;
  ASSERT_TRUE(snapshot.hasInputs(required));
  ego->pose.pose.position.x = parameters.checkpoints.front().x_m;
  ego->pose.pose.position.y = parameters.checkpoints.front().y_m;
  ego->pose.pose.orientation.z = std::sin(parameters.checkpoints.front().yaw_rad / 2);
  ego->pose.pose.orientation.w = std::cos(parameters.checkpoints.front().yaw_rad / 2);
  const auto run = [&] {
    np::StageData data; data.update(snapshot);
    np::CoreResult first, prediction, last;
    EXPECT_TRUE(pp::retainInputs(pp::processStage1)(cores, pp::FirstCoreId::GlobalPath, data, first, []{}));
    auto next = first(data);
    EXPECT_EQ(next.inputs.size(), 6u);
    for (std::size_t i = 0; i < data.inputs.size(); ++i) {
      EXPECT_EQ(next.inputs[i].message, data.inputs[i].message);
      EXPECT_EQ(next.inputs[i].stamp_ns, data.inputs[i].stamp_ns);
      EXPECT_EQ(next.inputs[i].receipt_steady_ns, data.inputs[i].receipt_steady_ns);
    }
    EXPECT_TRUE(next.inputs[0].has_stamp);
    EXPECT_FALSE(next.inputs[5].has_stamp);
    EXPECT_TRUE(pp::retainInputs(pp::processStage2)(cores, pp::SecondCoreId::PredictionPassThrough, next, prediction, []{}));
    auto predicted = prediction(next);
    EXPECT_EQ(predicted.inputs.size(), 7u);
    EXPECT_FALSE(predicted.inputs[6].has_stamp);
    EXPECT_FALSE(predicted.inputs[6].has_receipt);
    for (const auto& sample : predicted.message<pp::PredictedObjects>(6)) EXPECT_EQ(sample, *objects);
    EXPECT_TRUE(pp::processStage3(cores, pp::ThirdCoreId::LocalPath, predicted, last, []{}));
    return last(predicted).message<interfaces::Path>(0);
  };
  const auto route = cores.global_path.process(*ego, *section);
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
    expectPayload(run());
    section->state = i + 1;
    const auto path = run();
    expectPayload(path);
    EXPECT_EQ(run(), path);
    section->state = 0;
    EXPECT_EQ(run(), path);
  }
}

TEST(PathStages, MissingObjectsOrPredictionStopsBeforeCoreCall) {
  const auto parameters = shortMap();
  pp::CoreSet cores{pp::GlobalPath(parameters), pp::PredictionPassThrough{}, pp::LocalPath({parameters.map, 20.0})};
  pp::Stage3Data typed;
  typed.get<0>().sample.message = std::make_shared<nav_msgs::Odometry>();
  typed.get<4>().sample.message = std::make_shared<interfaces::SectionAnnotation>();
  typed.get<5>().sample.message = std::make_shared<pp::Route>();
  np::StageData data;
  data.update(typed);
  np::CoreResult result;
  bool called = false;
  const np::BeforeCore before = [&] { called = true; };
  EXPECT_FALSE(pp::processStage2(cores, pp::SecondCoreId::PredictionPassThrough, data, result, before));
  EXPECT_FALSE(called);
  EXPECT_FALSE(pp::processStage3(cores, pp::ThirdCoreId::LocalPath, data, result, before));
  EXPECT_FALSE(called);
}

namespace {
struct TestAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
  static std::int64_t stamp(const tf2_msgs::TFMessage& m) { return m.transforms.empty() ? 0 : m.transforms.front().header.stamp.toNSec(); }
  static std::string frame(const tf2_msgs::TFMessage& m) { return m.transforms.empty() ? "" : m.transforms.front().header.frame_id; }
};
using TestComponents = pp::Components<TestAccess>;
struct TestIO {
  using MessageType = interfaces::Path;
  std::shared_ptr<std::vector<interfaces::Path>> output;
  np::Clocks readClocks() const { return {1000000000, 1000000000}; }
  void publish(const interfaces::Path& message) { output->push_back(message); }
  void reportHealth(const np::StageHealth&, std::size_t) const {}
  void report(const np::ProcessReport&) const noexcept {}
};
struct TestHeaderInjector {
  std::shared_ptr<std::size_t> calls;
  void injectHeader(interfaces::Path& message, const np::Context<5>& context) const {
    ++*calls;
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = "map";
    for (auto& point : message.points) point.pose.header = message.header;
  }
};
}  // namespace

TEST(PathStages, ThreeStagePipelinePublishesOnceAndStopsWhenObjectsAreMissing) {
  const auto parameters = shortMap();
  auto selector = TestComponents::selector();
  selector.stage1.section_core_map[0] = pp::FirstCoreId::GlobalPath;
  selector.stage2.section_core_map[0] = pp::SecondCoreId::PredictionPassThrough;
  selector.stage3.section_core_map[0] = pp::ThirdCoreId::LocalPath;
  np::HealthLimits common;
  std::array<np::HealthLimits, 5> raw_limits;
  raw_limits.fill(common);
  raw_limits[1].frame = raw_limits[2].frame = "base_link";
  std::array<np::HealthLimits, 6> prediction_limits{{raw_limits[0], raw_limits[1], raw_limits[2], raw_limits[3], raw_limits[4], common}};
  std::array<np::HealthLimits, 7> local_limits{{raw_limits[0], raw_limits[1], raw_limits[2], raw_limits[3], raw_limits[4], common, common}};
  local_limits[6].frame = "base_link";
  auto output = std::make_shared<std::vector<interfaces::Path>>();
  auto headers = std::make_shared<std::size_t>(0);
  using Context = TestComponents::Buffer::ContextType;
  TestComponents::Node<TestIO, TestHeaderInjector> node({}, TestComponents::Buffer(8), {},
    {np::makeStage<pp::CoreSet, Context, pp::Data>(TestComponents::Checker(raw_limits, 1000000000),
       TestComponents::policy(common), selector.stage1, pp::retainInputs(pp::processStage1)),
     np::makeStage<pp::CoreSet, Context, pp::Stage2Data>(TestComponents::Stage2Checker(prediction_limits, 1000000000),
       TestComponents::Stage2Policy{{common.upper}, {common.lower}}, selector.stage2, pp::retainInputs(pp::processStage2)),
     np::makeStage<pp::CoreSet, Context, pp::Stage3Data>(TestComponents::Stage3Checker(local_limits, 1000000000),
       TestComponents::Stage3Policy{{common.upper}, {common.lower}}, selector.stage3, pp::processStage3)},
    pp::CoreSet{pp::GlobalPath(parameters), pp::PredictionPassThrough{}, pp::LocalPath({parameters.map, 20.0})},
    TestHeaderInjector{headers}, TestIO{output});
  nav_msgs::Odometry ego;
  ego.header.stamp.fromSec(1.0);
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 2.0;
  ego.pose.pose.orientation.w = 1.0;
  interfaces::SectionAnnotation section;
  section.header = ego.header;
  node.onInput(pp::InputTag<pp::Input::Ego>{}, ego);
  node.onInput(pp::InputTag<pp::Input::Section>{}, section);
  EXPECT_EQ(node.onPeriodic().status, np::ProcessStatus::NoCore);
  EXPECT_TRUE(output->empty());
  EXPECT_EQ(*headers, 0u);
  interfaces::Objects objects;
  objects.header.stamp = ego.header.stamp;
  objects.header.frame_id = "base_link";
  node.onInput(pp::InputTag<pp::Input::Objects>{}, objects);
  EXPECT_EQ(node.onPeriodic().status, np::ProcessStatus::Published);
  ASSERT_EQ(output->size(), 1u);
  EXPECT_EQ(*headers, 1u);
  EXPECT_EQ(output->front().header.stamp, ego.header.stamp);
  EXPECT_EQ(output->front().header.frame_id, "map");
  EXPECT_NEAR(lengthM(output->front()), 20.0, 1e-6);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
