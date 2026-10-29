#include <control/parameters/morai_mpc.hpp>
#include <gtest/gtest.h>

namespace {
struct Fixture {
  control::autoware_ai_mpc::Parameters parameters;
  interfaces::Path path;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus vehicle;
  interfaces::LocalizationStatus localization;

  explicit Fixture(double steering_deg = 15.0) {
    ros::Time::init();
    parameters = control::loadMoraiMpcParameters(node_pipeline::ParamReader::fromFile(
        std::string(NP_PACKAGE_DIR) + "/config/control/parameters/morai_mpc.yaml", NP_PACKAGE_DIR));
    path.header.stamp = ros::Time(100);
    path.header.frame_id = "base_link";
    ego.header.stamp = path.header.stamp;
    ego.header.frame_id = "map";
    ego.child_frame_id = "base_link";
    ego.pose.pose.orientation.w = 1.0;
    ego.twist.twist.linear.x = 6.2;
    vehicle.velocity_x_mps = 6.2;
    vehicle.steering_angle_rad = -steering_deg * M_PI / 180.0;
    localization.mode = interfaces::LocalizationStatus::GPS_FUSED;
    const double curvature = std::tan(steering_deg * M_PI / 180.0 *
        parameters.steering_prediction.gain) / parameters.lateral.vehicle_model_wheelbase;
    for (int i = 0; i <= 60; ++i) {
      const double station = i * 0.5, yaw = station * curvature;
      interfaces::PathPoint point;
      point.pose.header = path.header;
      point.pose.pose.position.x = curvature == 0 ? station : std::sin(yaw) / curvature;
      point.pose.pose.position.y = curvature == 0 ? 0 : (1 - std::cos(yaw)) / curvature;
      point.pose.pose.orientation.z = std::sin(yaw / 2);
      point.pose.pose.orientation.w = std::cos(yaw / 2);
      point.velocity = 6.2;
      path.points.push_back(point);
    }
  }

  morai_msgs::CtrlCmd call(control::MoraiMpc& core, double stamp) {
    path.header.stamp = ego.header.stamp = ros::Time(stamp);
    for (auto& point : path.points) point.pose.header = path.header;
    return core.process(path, ego, vehicle, localization);
  }
};
}  // namespace

TEST(MoraiMpc, FirstCommandStartsFromReportedSteeringInEitherDirection) {
  for (double angle_deg : {-15.0, 0.0, 15.3761739}) {
    Fixture f(angle_deg);
    control::MoraiMpc core(f.parameters);
    const auto command = f.call(core, 100);
    const double step_deg = f.parameters.lateral.morai_steer_rate_limit_deg *
        f.parameters.lateral.ctrl_period;
    EXPECT_LE(std::abs(command.steering * 40 - angle_deg), step_deg + 1e-5);
    EXPECT_TRUE(std::isfinite(command.accel));
    EXPECT_TRUE(std::isfinite(command.brake));
  }
}

TEST(MoraiMpc, ReactivationAndTimeRewindMatchFreshLateralState) {
  for (double stamp : {102.0, 99.0}) {
    Fixture f(-15);
    control::MoraiMpc resumed(f.parameters), fresh(f.parameters);
    f.call(resumed, 100);
    // Another controller has steered left while this Core was inactive.
    f.vehicle.steering_angle_rad = -15 * M_PI / 180.0;
    for (int i = 0; i < 8; ++i) {
      const double time = stamp + i * f.parameters.lateral.ctrl_period;
      EXPECT_NEAR(f.call(resumed, time).steering, f.call(fresh, time).steering, 1e-8);
    }
  }
}

TEST(MoraiMpc, ConsecutiveCallsRetainCommandHistory) {
  Fixture f;
  control::MoraiMpc core(f.parameters);
  const auto previous = f.call(core, 100);
  // A delayed status sample must not reseed the active controller every cycle.
  f.vehicle.steering_angle_rad = 15 * M_PI / 180.0;
  const auto next = f.call(core, 100 + f.parameters.lateral.ctrl_period);
  EXPECT_LE(std::abs(next.steering - previous.steering) * 40,
      f.parameters.lateral.morai_steer_rate_limit_deg * f.parameters.lateral.ctrl_period + 1e-5);
}
