#include <control/components.hpp>
#include <gtest/gtest.h>
#include <limits>
namespace {
control::tracking::Parameters parameters() {
  control::tracking::Parameters p;
  p.lateral_controller = "pure_pursuit";
  p.longitudinal_controller = "pid";
  p.mpc.solver = "projected_gradient";
  p.longitudinal_mpc.solver = "projected_gradient";
  p.pid.kp = 1;
  p.pid.ki = 0;
  p.pid.kd = 0;
  return p;
}
interfaces::Path path(double speed, double lateral = 0) {
  interfaces::Path result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  for (double x : {-10.0, 0.0, 10.0, 20.0}) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = x;
    point.pose.pose.position.y = lateral;
    point.pose.pose.orientation.w = 1;
    point.velocity = speed;
    result.points.push_back(point);
  }
  return result;
}
nav_msgs::Odometry ego() {
  nav_msgs::Odometry result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "map";
  result.child_frame_id = "base_link";
  result.pose.pose.orientation.w = 1;
  return result;
}
interfaces::CompetitionVehicleStatus status() {
  interfaces::CompetitionVehicleStatus result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  result.velocity_x_mps = 5;
  return result;
}
TEST(PathTrackingCore, UsesInjectedNearestSpeedAndNormalizedSteering) {
  control::PathTracking core(parameters());
  auto input = path(10, 2);
  input.points[1].velocity = 0;
  const auto command = core.process(input, ego(), status());
  EXPECT_EQ(command.longlCmdType, 1);
  EXPECT_DOUBLE_EQ(command.accel, 0);
  EXPECT_GT(command.brake, 0);
  EXPECT_GT(command.steering, 0);
  EXPECT_LE(command.steering, 1);
}
TEST(PathTrackingCore, RetainsPidStateAcrossCalls) {
  auto p = parameters();
  p.pid.kp = 0;
  p.pid.ki = 0.1;
  control::PathTracking core(p);
  auto input = path(6);
  const auto first = core.process(input, ego(), status());
  input.header.stamp = ros::Time(1, 50000000);
  const auto second = core.process(input, ego(), status());
  EXPECT_GT(second.accel, first.accel);
}
TEST(PathTrackingCore, RunsBothMpcSolversWithoutRosIo) {
  auto p = parameters();
  p.lateral_controller = "mpc";
  p.longitudinal_controller = "mpc";
  control::PathTracking core(p);
  const auto command = core.process(path(6), ego(), status());
  EXPECT_TRUE(std::isfinite(command.steering));
  EXPECT_LE(std::abs(command.steering), 1);
  EXPECT_GE(command.accel, 0);
  EXPECT_GE(command.brake, 0);
  EXPECT_FALSE(command.accel > 0 && command.brake > 0);
}
struct Access {
  template<class M> static std::int64_t stamp(const M& m) { return m.header.stamp.toNSec(); }
  template<class M> static const std::string& frame(const M& m) { return m.header.frame_id; }
};
struct IO {
  using MessageType = morai_msgs::CtrlCmd;
  int* outputs;
  node_pipeline::Clocks readClocks() const { return {1000000000, 1000000000}; }
  void publish(const morai_msgs::CtrlCmd&) { ++*outputs; }
  template<class H> void reportHealth(const H&, std::size_t) {}
  void report(const node_pipeline::ProcessReport&) noexcept {}
};
struct Header { void injectHeader(morai_msgs::CtrlCmd&, const node_pipeline::Context<3>&) {} };
TEST(PathTrackingPipeline, AllSectionsSelectCoreAndInvalidInputsAreRemoved) {
  namespace np = node_pipeline;
  using C = control::Components<Access>;
  auto selector = C::selector();
  for (int i = 0; i <= 16; ++i) selector.section_core_map[i] = control::CoreId::PathTracking;
  np::ExecutionConfig<control::Input> config;
  config.mode = np::ExecutionMode::OnInput;
  config.trigger_input = control::Input::Path;
  std::array<np::HealthLimits, 3> limits;
  limits[0].frame = "base_link"; limits[1].frame = "map"; limits[2].frame = "base_link";
  auto policy = C::policy(limits[0]); policy.priority.fill(100);
  int outputs = 0;
  C::Node<IO, Header> node(config, C::Buffer(8), {},
    std::vector<C::Stage>{np::makeStage<control::CoreSet, C::Buffer::ContextType, control::Data>(
      C::Checker(limits, 1000000000), policy, selector, control::processCore)},
    control::CoreSet(parameters()), {}, IO{&outputs});
  auto input = path(6);
  EXPECT_EQ(node.onInput(control::InputTag<control::Input::Path>{}, std::make_shared<const interfaces::Path>(input)).status, np::ProcessStatus::NoCore);
  node.onInput(control::InputTag<control::Input::Ego>{}, std::make_shared<const nav_msgs::Odometry>(ego()));
  node.onInput(control::InputTag<control::Input::Competition>{}, std::make_shared<const interfaces::CompetitionVehicleStatus>(status()));
  for (int i = 0; i <= 16; ++i) {
    node.onSection(np::Section{true, static_cast<std::uint8_t>(i), 1000000000, "map"});
    EXPECT_EQ(node.onInput(control::InputTag<control::Input::Path>{}, std::make_shared<const interfaces::Path>(input)).status, np::ProcessStatus::Published);
  }
  EXPECT_EQ(outputs, 17);
  auto invalid = status(); invalid.velocity_x_mps = std::numeric_limits<float>::quiet_NaN();
  node.onInput(control::InputTag<control::Input::Competition>{}, std::make_shared<const interfaces::CompetitionVehicleStatus>(invalid));
  EXPECT_EQ(node.onInput(control::InputTag<control::Input::Path>{}, std::make_shared<const interfaces::Path>(input)).status, np::ProcessStatus::NoCore);
  EXPECT_EQ(outputs, 17);
}
}
int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
