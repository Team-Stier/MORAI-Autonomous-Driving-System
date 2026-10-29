#include <gtest/gtest.h>
#include <speed_filter/cores/speed_guard60_hold.hpp>

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

TEST(SpeedGuard60Hold, PassesCommandBelowTaper) {
  speed_filter::SpeedGuard60Hold core{speed_filter::SpeedGuard60HoldParameters{}};
  const auto output = core.process(command(0.5), status(10.0, 50.0));
  EXPECT_DOUBLE_EQ(output.accel, 0.5);
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
  EXPECT_DOUBLE_EQ(output.steering, 0.1);
}

TEST(SpeedGuard60Hold, TapersAccelAtConstantSpeed) {
  speed_filter::SpeedGuard60Hold core{speed_filter::SpeedGuard60HoldParameters{}};
  core.process(command(0.5), status(10.0, 58.6));
  const auto output = core.process(command(0.5), status(10.1, 58.6));
  // (58.6 - 58.0) / (59.2 - 58.0) = 0.5 of the taper.
  EXPECT_NEAR(output.accel, 0.25, 1e-4);  // signed_velocity_mps is float32.
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
}

TEST(SpeedGuard60Hold, CutsAccelEarlierWhileAccelerating) {
  speed_filter::SpeedGuard60Hold constant{speed_filter::SpeedGuard60HoldParameters{}};
  speed_filter::SpeedGuard60Hold accelerating{speed_filter::SpeedGuard60HoldParameters{}};
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

TEST(SpeedGuard60Hold, BrakesGentlyNearLimit) {
  speed_filter::SpeedGuard60Hold core{speed_filter::SpeedGuard60HoldParameters{}};
  core.process(command(0.4), status(10.0, 59.7));
  const auto output = core.process(command(0.4), status(10.1, 59.7));
  EXPECT_DOUBLE_EQ(output.accel, 0.0);
  EXPECT_NEAR(output.brake, 0.15 * 0.5, 1e-4);  // signed_velocity_mps is float32.
}

TEST(SpeedGuard60Hold, KeepsStrongerUpstreamBrake) {
  speed_filter::SpeedGuard60Hold core{speed_filter::SpeedGuard60HoldParameters{}};
  const auto output = core.process(command(0.0, 0.4), status(10.0, 59.8));
  EXPECT_DOUBLE_EQ(output.brake, 0.4);
}

TEST(SpeedGuard60Hold, FullBrakeAboveLimit) {
  speed_filter::SpeedGuard60Hold core{speed_filter::SpeedGuard60HoldParameters{}};
  const auto output = core.process(command(0.6), status(10.0, 60.5));
  EXPECT_DOUBLE_EQ(output.accel, 0.0);
  EXPECT_DOUBLE_EQ(output.brake, 1.0);
}

TEST(SpeedGuard60Hold, KeepsHoldingAfterLongStop) {
  speed_filter::SpeedGuard60Hold core{speed_filter::SpeedGuard60HoldParameters{}};
  for (int second = 0; second <= 120; ++second) {
    const auto output = core.process(command(0.0, 0.3), status(10.0 + second, 0.0));
    EXPECT_DOUBLE_EQ(output.accel, 0.0);
    EXPECT_DOUBLE_EQ(output.brake, 0.3);
    EXPECT_DOUBLE_EQ(output.steering, 0.1);
  }
}
