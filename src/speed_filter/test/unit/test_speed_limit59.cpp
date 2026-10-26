#include <speed_filter/cores/no_speed_limit.hpp>
#include <speed_filter/cores/speed_limit59.hpp>
#include <speed_filter/parameters/speed_limit59.hpp>
#include <speed_filter/validators/speed_limit59_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

namespace {
speed_filter::SpeedLimit59Parameters parameters() {
  std::ifstream file(std::string(NP_PACKAGE_DIR) + "/config/speed_filter/parameters/speed_limit59.yaml");
  const std::string yaml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  return speed_filter::loadSpeedLimit59Parameters(node_pipeline::ParamReader::fromString(yaml));
}

interfaces::CompetitionVehicleStatus status(double stamp_sec, double speed_kph) {
  interfaces::CompetitionVehicleStatus result;
  result.header.stamp.fromSec(stamp_sec);
  result.header.frame_id = "base_link";
  result.signed_velocity_mps = speed_kph / 3.6;
  return result;
}

void expectCommand(const morai_msgs::CtrlCmd& expected, const morai_msgs::CtrlCmd& actual) {
  EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
  EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
  EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
  EXPECT_DOUBLE_EQ(expected.steering, actual.steering);
  EXPECT_DOUBLE_EQ(expected.velocity, actual.velocity);
  EXPECT_DOUBLE_EQ(expected.acceleration, actual.acceleration);
}
}  // namespace

TEST(SpeedLimit59, BrakesBeforeSixtyAndLeavesTheHighwayCoreUnrestricted) {
  speed_filter::SpeedLimit59 core(parameters());
  speed_filter::NoSpeedLimit unlimited;
  morai_msgs::CtrlCmd command;
  command.longlCmdType = 1;
  command.accel = 0.4;
  command.steering = -0.25;
  command.velocity = 50;
  command.acceleration = 1.5;
  expectCommand(command, core.process(command, status(10, 58.9)));
  // Float32 cannot represent 59/3.6 exactly; test its adjacent message values.
  auto boundary = status(11, 59.0);
  boundary.signed_velocity_mps = std::nextafter(boundary.signed_velocity_mps, 0.0f);
  expectCommand(command, core.process(command, boundary));
  auto braking = command;
  braking.accel = 0;
  braking.brake = 1;
  boundary.signed_velocity_mps = std::nextafter(boundary.signed_velocity_mps, 100.0f);
  expectCommand(braking, core.process(command, boundary));
  for (double speed_kph : {59.01, 60.12, -60.12})
    expectCommand(braking, core.process(command, status(12, speed_kph)));
  expectCommand(command, unlimited.process(command, status(13, 100.0)));
}

TEST(SpeedLimit59, PreservesConfiguredStopAssistAndResetsItsTimers) {
  const auto config = parameters();
  EXPECT_DOUBLE_EQ(30.0, config.stop_tolerance_sec);
  EXPECT_DOUBLE_EQ(1.0, config.start_duration_sec);
  EXPECT_DOUBLE_EQ(0.2, config.start_accel);
  EXPECT_DOUBLE_EQ(0.1, config.stop_speed_mps);
  speed_filter::SpeedLimit59 core(config);
  morai_msgs::CtrlCmd command;
  command.brake = 0.4;
  command.steering = 0.25;
  auto kick = command;
  kick.accel = 0.2;
  kick.brake = 0;
  kick.steering = 0;
  expectCommand(command, core.process(command, status(10, 0)));
  expectCommand(command, core.process(command, status(39, 0)));
  expectCommand(kick, core.process(command, status(40, 0)));
  expectCommand(kick, core.process(command, status(40.5, 3.6)));
  expectCommand(command, core.process(command, status(41, 0)));
  auto braking = command;
  braking.brake = 1;
  expectCommand(braking, core.process(command, status(42, 60.12)));
  expectCommand(command, core.process(command, status(43, 0)));
  expectCommand(command, core.process(command, status(72, 0)));
  expectCommand(kick, core.process(command, status(73, 0)));
  command.accel = 0.1;
  expectCommand(command, core.process(command, status(74, 0)));
  command.accel = 0;
  expectCommand(command, core.process(command, status(75, 0)));
  expectCommand(command, core.process(command, status(104, 0)));
  expectCommand(kick, core.process(command, status(105, 0)));
  expectCommand(command, core.process(command, status(1, 0)));
  expectCommand(command, core.process(command, status(30, 0)));
  expectCommand(kick, core.process(command, status(31, 0)));
}

TEST(SpeedLimit59Parameters, RejectsUnsafeStaticCoefficients) {
  const std::string valid =
      "stop_tolerance_sec: 30.0\nstart_duration_sec: 1.0\nstart_accel: 0.2\nstop_speed_mps: 0.1\n";
  for (const auto& entry : {std::pair<std::string, std::string>{"stop_tolerance_sec: 30.0", "stop_tolerance_sec: 0"},
                           {"start_duration_sec: 1.0", "start_duration_sec: -1"},
                           {"start_accel: 0.2", "start_accel: 1.1"},
                           {"stop_speed_mps: 0.1", "stop_speed_mps: -0.1"}}) {
    auto yaml = valid;
    yaml.replace(yaml.find(entry.first), entry.first.size(), entry.second);
    EXPECT_THROW(speed_filter::loadSpeedLimit59Parameters(node_pipeline::ParamReader::fromString(yaml)),
                 std::invalid_argument);
  }
}

TEST(SpeedLimit59Inputs, RequiresFiniteSpeed) {
  speed_filter::SpeedLimit59Inputs validator;
  const node_pipeline::HealthLimits limits;
  std::vector<node_pipeline::Kind> issues;
  auto competition = status(10, 58.9);
  validator(competition, limits, issues);
  EXPECT_TRUE(issues.empty());
  competition.signed_velocity_mps = std::numeric_limits<float>::quiet_NaN();
  validator(competition, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::NaN}, issues);
  issues.clear();
  competition.signed_velocity_mps = std::numeric_limits<float>::infinity();
  validator(competition, limits, issues);
  EXPECT_EQ(std::vector<node_pipeline::Kind>{node_pipeline::Kind::Inf}, issues);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
