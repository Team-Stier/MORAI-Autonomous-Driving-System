#include <speed_filter/cores/no_speed_limit.hpp>
#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/parameters/speed_limit60.hpp>
#include <gtest/gtest.h>

namespace {
void expectCommand(const morai_msgs::CtrlCmd& expected, const morai_msgs::CtrlCmd& actual) {
  EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
  EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
  EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
  EXPECT_DOUBLE_EQ(expected.steering, actual.steering);
  EXPECT_DOUBLE_EQ(expected.velocity, actual.velocity);
  EXPECT_DOUBLE_EQ(expected.acceleration, actual.acceleration);
}

template<class Core> void checkFilter() {
  speed_filter::SpeedFilterParameters parameters{72.0, 2.0, 1.0, 0.3, 0.1};
  Core core(parameters);
  morai_msgs::CtrlCmd command;
  command.longlCmdType = 7;
  command.accel = 0.4;
  command.brake = 0.5;
  command.steering = -0.25;
  command.velocity = 50;
  command.acceleration = 1.5;
  interfaces::CompetitionVehicleStatus status;
  auto process = [&](double stamp_sec, float speed_mps) {
    status.header.stamp.fromSec(stamp_sec);
    status.signed_velocity_mps = speed_mps;
    return core.process(command, status);
  };
  expectCommand(command, process(10, 2));
  expectCommand(command, process(11, 20));
  auto expected = command;
  expected.accel = 0;
  expected.brake = 1;
  expectCommand(expected, process(12, 20.1));
  expectCommand(expected, process(13, -20.1));

  command.accel = 0;
  expectCommand(command, process(20, 0));
  expectCommand(command, process(21.9, -0.05));
  expected = command;
  expected.accel = 0.3;
  expected.brake = 0;
  expected.steering = 0;
  expectCommand(expected, process(22, 0));
  expectCommand(expected, process(22.2, 1));
  expectCommand(command, process(23, 0));
  expectCommand(command, process(24.9, 0));
  expectCommand(expected, process(25, 0));

  command.accel = 0.2;
  expectCommand(command, process(25.1, 0));
  command.accel = 0;
  expectCommand(command, process(25.2, 0));
  expectCommand(command, process(27.1, 0));
  expectCommand(expected, process(27.3, 0));
  auto braking = command;
  braking.brake = 1;
  expectCommand(braking, process(27.4, 21));
  expectCommand(command, process(28, 0));
  expectCommand(command, process(29, 0.2));
  expectCommand(command, process(30, 0));
  expectCommand(command, process(31.9, 0));
  expectCommand(expected, process(32, 0));
  expectCommand(command, process(1, 0));
  expectCommand(command, process(2.9, 0));
  expectCommand(expected, process(3, 0));
}
}  // namespace

TEST(SpeedFilter, BothCoresLimitAndRecoverWithIndependentParameters) {
  checkFilter<speed_filter::SpeedLimit60>();
  checkFilter<speed_filter::NoSpeedLimit>();
  speed_filter::NoSpeedLimit unlimited;
  speed_filter::SpeedLimit60 limited;
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromSec(10);
  status.signed_velocity_mps = 30;
  morai_msgs::CtrlCmd command;
  command.accel = 0.4;
  command.steering = 0.25;
  expectCommand(command, unlimited.process(command, status));
  command.accel = 0;
  command.brake = 1;
  expectCommand(command, limited.process(command, status));
}

TEST(SpeedFilter, LoaderReadsAllParametersAndRejectsInvalidAccel) {
  const auto parameters = speed_filter::loadSpeedLimit60Parameters(node_pipeline::ParamReader::fromString(
      "speed_limit_kph: 60.0\nstop_tolerance_sec: 30.0\nstart_duration_sec: 1.0\nstart_accel: 0.2\nstop_speed_mps: 0.1\n"));
  EXPECT_DOUBLE_EQ(60.0, parameters.speed_limit_kph);
  EXPECT_DOUBLE_EQ(30.0, parameters.stop_tolerance_sec);
  EXPECT_THROW(speed_filter::loadSpeedLimit60Parameters(node_pipeline::ParamReader::fromString(
      "speed_limit_kph: 60.0\nstop_tolerance_sec: 30.0\nstart_duration_sec: 1.0\nstart_accel: 1.5\nstop_speed_mps: 0.1\n")),
      std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
