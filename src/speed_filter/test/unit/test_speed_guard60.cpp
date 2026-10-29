#include <gtest/gtest.h>
#include <speed_filter/cores/speed_guard60.hpp>

namespace {
interfaces::CompetitionVehicleStatus status(double stamp_sec, double speed_kph) {
  interfaces::CompetitionVehicleStatus message;
  message.header.stamp = ros::Time(stamp_sec);
  message.signed_velocity_mps = speed_kph / 3.6;
  return message;
}

morai_msgs::CtrlCmd command(double accel, double brake = 0.0, double steering = 0.1) {
  morai_msgs::CtrlCmd message;
  message.longlCmdType = 1;
  message.accel = accel;
  message.brake = brake;
  message.steering = steering;
  return message;
}
}  // namespace

TEST(SpeedGuard60, PassesCommandBelowTaper) {
  speed_filter::SpeedGuard60 core{speed_filter::SpeedGuard60Parameters{}};
  const auto output = core.process(command(0.5), status(10.0, 50.0));
  EXPECT_DOUBLE_EQ(output.accel, 0.5);
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
  EXPECT_DOUBLE_EQ(output.steering, 0.1);
}

TEST(SpeedGuard60, TapersAccelAtConstantSpeed) {
  speed_filter::SpeedGuard60 core{speed_filter::SpeedGuard60Parameters{}};
  core.process(command(0.5), status(10.0, 58.6));
  const auto output = core.process(command(0.5), status(10.1, 58.6));
  // (58.6 - 58.0) / (59.2 - 58.0) = 0.5 of the taper.
  EXPECT_NEAR(output.accel, 0.25, 1e-4);  // signed_velocity_mps is float32.
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
}

TEST(SpeedGuard60, CutsAccelEarlierWhileAccelerating) {
  speed_filter::SpeedGuard60 constant{speed_filter::SpeedGuard60Parameters{}};
  speed_filter::SpeedGuard60 accelerating{speed_filter::SpeedGuard60Parameters{}};
  morai_msgs::CtrlCmd constant_output, accelerating_output;
  for (int index = 0; index <= 30; ++index) {
    const double stamp_sec = 10.0 + index / 30.0;
    // 2.5 m/s^2 ramp ending at the same 57.9 km/h, below the taper start.
    const double speed_kph = 57.9 - (30 - index) * 2.5 * 3.6 / 30.0;
    constant_output = constant.process(command(0.5), status(stamp_sec, 57.9));
    accelerating_output = accelerating.process(command(0.5), status(stamp_sec, speed_kph));
  }
  EXPECT_DOUBLE_EQ(constant_output.accel, 0.5);
  EXPECT_DOUBLE_EQ(accelerating_output.accel, 0.0);
}

TEST(SpeedGuard60, BrakesGentlyNearLimit) {
  speed_filter::SpeedGuard60 core{speed_filter::SpeedGuard60Parameters{}};
  core.process(command(0.4), status(10.0, 59.7));
  const auto output = core.process(command(0.4), status(10.1, 59.7));
  EXPECT_DOUBLE_EQ(output.accel, 0.0);
  EXPECT_NEAR(output.brake, 0.15 * 0.5, 1e-4);  // signed_velocity_mps is float32.
}

TEST(SpeedGuard60, KeepsStrongerUpstreamBrake) {
  speed_filter::SpeedGuard60 core{speed_filter::SpeedGuard60Parameters{}};
  const auto output = core.process(command(0.0, 0.4), status(10.0, 59.8));
  EXPECT_DOUBLE_EQ(output.brake, 0.4);
}

TEST(SpeedGuard60, FullBrakeAboveLimit) {
  speed_filter::SpeedGuard60 core{speed_filter::SpeedGuard60Parameters{}};
  const auto output = core.process(command(0.6), status(10.0, 60.5));
  EXPECT_DOUBLE_EQ(output.accel, 0.0);
  EXPECT_DOUBLE_EQ(output.brake, 1.0);
}

TEST(SpeedGuard60, StartAssistAfterLongStop) {
  speed_filter::SpeedGuard60Parameters parameters;
  parameters.stop_tolerance_sec = 2.0;
  speed_filter::SpeedGuard60 core{parameters};
  core.process(command(0.0, 0.3), status(10.0, 0.0));
  EXPECT_DOUBLE_EQ(core.process(command(0.0, 0.3), status(11.0, 0.0)).brake, 0.3);
  const auto output = core.process(command(0.0, 0.3), status(12.0, 0.0));
  EXPECT_DOUBLE_EQ(output.accel, 0.2);
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
  EXPECT_DOUBLE_EQ(output.steering, 0.0);
  EXPECT_DOUBLE_EQ(core.process(command(0.0, 0.3), status(13.1, 0.0)).brake, 0.3);
}
