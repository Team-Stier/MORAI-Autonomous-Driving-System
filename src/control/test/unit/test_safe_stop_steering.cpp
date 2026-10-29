#include <control/cores/safe_stop.hpp>
#include <gtest/gtest.h>
#include <cmath>
TEST(SafeStopSteering, KeepsTheRecordedRightTurnWhileApplyingFullBrake) {
  control::SafeStopParameters p;p.brake=1;control::SafeStop core(p);
  interfaces::CompetitionVehicleStatus status;status.steering_angle_rad=.24663884937763214;
  const auto command=core.process(status);
  EXPECT_DOUBLE_EQ(command.accel,0);EXPECT_DOUBLE_EQ(command.brake,1);
  EXPECT_NEAR(command.steering,-.353284128,1e-7);EXPECT_EQ(command.longlCmdType,1);
}
TEST(SafeStopSteering, SupportsBothDirectionsAndStraightStandstill) {
  control::SafeStopParameters p;p.brake=1;control::SafeStop core(p);
  for(double angle:{-.4,0.,.4}) {
    interfaces::CompetitionVehicleStatus status;status.steering_angle_rad=angle;
    const auto command=core.process(status);
    EXPECT_NEAR(command.steering,-angle/p.steering_limit_rad,1e-7);
    EXPECT_DOUBLE_EQ(command.brake,1);EXPECT_DOUBLE_EQ(command.accel,0);
  }
}
TEST(SafeStopSteering, PreservesTheNormalizedSteeringLimit) {
  control::SafeStopParameters p;p.brake=1;control::SafeStop core(p);
  for(double angle:{-1.,1.}) {
    interfaces::CompetitionVehicleStatus status;status.steering_angle_rad=angle;
    const auto command=core.process(status);
    EXPECT_DOUBLE_EQ(command.steering,angle>0 ? -1. : 1.);
  }
}
