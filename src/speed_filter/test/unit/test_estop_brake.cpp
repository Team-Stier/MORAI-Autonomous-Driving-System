#include <gtest/gtest.h>
#include <speed_filter/cores/estop_brake.hpp>
#include <speed_filter/cores/estop_pass.hpp>

namespace {
morai_msgs::CtrlCmd command() {
  morai_msgs::CtrlCmd message;
  message.longlCmdType = 1;
  message.accel = 0.6;
  message.brake = 0.0;
  message.steering = 0.1;
  return message;
}

std_msgs::Bool estop(bool value) {
  std_msgs::Bool message;
  message.data = value;
  return message;
}
}  // namespace

TEST(EstopBrake, TrueGivesFullBrakeAndKeepsSteering) {
  speed_filter::EstopBrake core;
  const auto output = core.process(command(), interfaces::CompetitionVehicleStatus{}, estop(true)).first;
  EXPECT_DOUBLE_EQ(output.accel, 0.0);
  EXPECT_DOUBLE_EQ(output.brake, 1.0);
  EXPECT_DOUBLE_EQ(output.steering, 0.1);
  EXPECT_EQ(output.longlCmdType, 1);
}

TEST(EstopBrake, FalsePassesTheCommand) {
  speed_filter::EstopBrake core;
  const auto output = core.process(command(), interfaces::CompetitionVehicleStatus{}, estop(false)).first;
  EXPECT_DOUBLE_EQ(output.accel, 0.6);
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
}

TEST(EstopPass, PassesTheCommand) {
  speed_filter::EstopPass core;
  const auto output = core.process(command(), interfaces::CompetitionVehicleStatus{}).first;
  EXPECT_DOUBLE_EQ(output.accel, 0.6);
}
