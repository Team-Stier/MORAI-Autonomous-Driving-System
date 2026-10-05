#include <control/cores/morai_mpc.hpp>
#include <control/tracking/morai_mpc/steering_prediction.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <array>
#include <fstream>
#include <memory>
#include <sstream>

namespace {
interfaces::Path path(double target = 8.0, double offset = 0.0, double length = 45.0) {
  interfaces::Path result;
  result.header.frame_id = "base_link";
  result.header.stamp = ros::Time(1, 0);
  for (double x = -4.0; x <= length; x += 0.5) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = x;
    point.pose.pose.position.y = offset;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = target;
    result.points.push_back(point);
  }
  return result;
}
nav_msgs::Odometry ego(double speed = 5.0) {
  nav_msgs::Odometry result;
  result.header.frame_id = "map";
  result.child_frame_id = "base_link";
  result.pose.pose.orientation.w = 1.0;
  result.twist.twist.linear.x = speed;
  return result;
}
interfaces::CompetitionVehicleStatus status(double speed = 5.0) {
  interfaces::CompetitionVehicleStatus result;
  result.velocity_x_mps = speed;
  return result;
}

TEST(MoraiMpc, RunsOriginalSolversWithoutRosCommunication) {
  control::MoraiMpc core;
  const auto output = core.process(path(), ego(), status());
  EXPECT_EQ(output.longlCmdType, 1);
  EXPECT_NEAR(output.steering, 0.0, 1e-8);
  EXPECT_GT(output.accel, 0.0);
  EXPECT_LE(output.accel, 0.70);
  EXPECT_DOUBLE_EQ(output.brake, 0.0);
}

TEST(MoraiMpc, TracksNearestScalarSpeedInsteadOfFutureVelocityProfile) {
  auto reference = path(0.0);
  auto profile = reference;
  for (auto& point : profile.points) {
    if (point.pose.pose.position.x != 0.0) point.velocity = 16.0;
    point.acceleration = 2.0;
  }
  control::MoraiMpc first, second;
  const auto expected = first.process(reference, ego(), status());
  const auto actual = second.process(profile, ego(), status());
  EXPECT_GT(actual.brake, 0.0);
  EXPECT_DOUBLE_EQ(actual.accel, 0.0);
  EXPECT_NEAR(actual.brake, expected.brake, 1e-10);
  EXPECT_NEAR(actual.steering, expected.steering, 1e-10);
}

TEST(MoraiMpc, UsesLocalRearAxleOriginAndNormalizedSteering) {
  control::MoraiMpc left, right, translated;
  auto map_ego = ego();
  map_ego.pose.pose.position.x = 400.0;
  map_ego.pose.pose.position.y = -200.0;
  map_ego.pose.pose.orientation.z = std::sin(0.5);
  map_ego.pose.pose.orientation.w = std::cos(0.5);
  const auto a = left.process(path(8.0, 0.3), ego(), status());
  const auto b = right.process(path(8.0, -0.3), ego(), status());
  const auto c = translated.process(path(8.0, 0.3), map_ego, status());
  EXPECT_GT(a.steering, 0.0);
  EXPECT_LT(b.steering, 0.0);
  EXPECT_NEAR(a.steering, -b.steering, 1e-6);
  EXPECT_NEAR(a.steering, c.steering, 1e-10);
  // 60 deg/s * (1/30 s) / 40 deg = 0.05 normalized per first call.
  EXPECT_LE(std::abs(a.steering), 0.050001);
}

TEST(MoraiMpc, RetainsCommandAndOptimizerStateAcrossCalls) {
  control::MoraiMpc core;
  auto input = path(8.0, 0.3);
  const auto first = core.process(input, ego(), status());
  input.header.stamp = ros::Time(1, 33333333);
  const auto second = core.process(input, ego(), status());
  EXPECT_GT(second.accel, first.accel);
  EXPECT_LE(std::abs(second.steering - first.steering), 0.050001);
  EXPECT_TRUE(std::isfinite(second.steering));
}

TEST(MoraiMpc, KeepsOriginalLongitudinalHardSpeedGuard) {
  control::MoraiMpc core;
  const auto output = core.process(path(60.0 / 3.6), ego(17.0), status(17.0));
  EXPECT_DOUBLE_EQ(output.accel, 0.0);
  EXPECT_GE(output.brake, 0.25);
  EXPECT_LE(output.brake, 0.60);
}

TEST(MoraiMpc, HandlesForwardOnly45MetrePathAtCruiseAndStandstill) {
  for (double speed : {0.0, 60.0 / 3.6}) {
    auto input = path();
    input.points.erase(input.points.begin(), input.points.begin() + 8);
    control::MoraiMpc core;
    const auto output = core.process(input, ego(speed), status(speed));
    EXPECT_TRUE(std::isfinite(output.steering));
    EXPECT_LE(std::abs(output.steering), 1.0);
    EXPECT_FALSE(output.accel > 0.0 && output.brake > 0.0);
  }
}

TEST(MoraiMpc, ResamplesShortGeometryAndCoincidentPointsWithoutInventingPath) {
  auto input = path();
  input.points.erase(input.points.begin() + 2, input.points.end());
  input.points[0].pose.pose.position.x = 0.0;
  input.points[1].pose.pose.position.x = 0.1;
  input.points.insert(input.points.begin(), input.points.front());
  control::MoraiMpc core;
  const auto output = core.process(input, ego(0.0), status(0.0));
  EXPECT_TRUE(std::isfinite(output.steering));
  EXPECT_NEAR(output.steering, 0.0, 1e-8);
}

TEST(MoraiMpc, RecalculatesHeadingForTwoRealSamples) {
  auto input = path();
  input.points.resize(2);
  input.points[0].pose.pose.position.x = 0.0;
  input.points[1].pose.pose.position.x = 0.1;
  // Unit quaternions are valid input; enabled yaw recalculation must use XY.
  for (auto& point : input.points) {
    point.pose.pose.orientation.z = std::sin(control::morai_mpc::kPi / 4.0);
    point.pose.pose.orientation.w = std::cos(control::morai_mpc::kPi / 4.0);
  }
  control::MoraiMpc core;
  const auto output = core.process(input, ego(0.0), status(0.0));
  EXPECT_NEAR(output.steering, 0.0, 1e-8);
}

TEST(MoraiSteeringPrediction, IntegratesDelayAndFirstOrderResponseExactly) {
  control::morai_mpc::SteeringPrediction predictor(0.1, 0.2);
  predictor.sent(0.0, 0.3);
  EXPECT_DOUBLE_EQ(predictor.advance(0.1), 0.0);
  EXPECT_NEAR(predictor.advance(0.3), 0.1896361676485673, 1e-12);
  predictor.sent(0.3, -0.3);
  EXPECT_GT(predictor.advance(0.4), 0.0);
  EXPECT_NEAR(predictor.advance(5.0), -0.3, 1e-9);
}

TEST(MoraiMpc, RejectsInvalidTuningBeforeAllocatingSolverState) {
  control::morai_mpc::Parameters p;
  p.longitudinal.horizon_steps = -1;
  EXPECT_THROW(control::morai_mpc::validateParameters(p), std::invalid_argument);
  p = {};
  p.lateral.mpc_weight_lat_error = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(control::morai_mpc::validateParameters(p), std::invalid_argument);
  p = {};
  p.lateral.steer_lim_deg = 80.0;
  EXPECT_THROW(control::morai_mpc::validateParameters(p), std::invalid_argument);
}

TEST(MoraiMpc, MatchesOriginalControllerReferenceSequences) {
  // Recorded by the original d71cadc controller implementations, not by the port.
  std::ifstream input(std::string(MORAI_MPC_TEST_DATA_DIR) + "/morai_mpc_reference.csv");
  ASSERT_TRUE(input.good());
  std::string line;
  std::getline(input, line);
  control::morai_mpc::Parameters parameters;
  std::unique_ptr<control::morai_mpc::AutowareLateral> lateral;
  std::unique_ptr<control::morai_mpc::LongitudinalMpc> longitudinal;
  control::morai_mpc::autoware::Lane lane;
  int lateral_rows = 0, longitudinal_rows = 0;
  while (std::getline(input, line)) {
    std::istringstream row(line);
    std::array<double, 11> value{};
    for (auto& field : value) {
      std::string text;
      ASSERT_TRUE(static_cast<bool>(std::getline(row, text, ',')));
      field = std::stod(text);
    }
    SCOPED_TRACE(line);
    if (value[8] == 0.0) {
      if (value[3] == 0.0) {
        lateral.reset(new control::morai_mpc::AutowareLateral(
            parameters.lateral, parameters.steering_prediction));
        lane.waypoints.clear();
        const double curvature = value[2];
        for (double s = -4.0; s <= 45.0; s += 0.5) {
          control::morai_mpc::autoware::Waypoint w;
          w.pose.pose.position.x = curvature == 0.0 ? s : std::sin(s * curvature) / curvature;
          w.pose.pose.position.y = value[1] +
              (curvature == 0.0 ? 0.0 : (1.0 - std::cos(s * curvature)) / curvature);
          w.pose.pose.orientation.z = std::sin(s * curvature / 2.0);
          w.pose.pose.orientation.w = std::cos(s * curvature / 2.0);
          w.twist.twist.linear.x = std::max(0.5, value[0]);
          lane.waypoints.push_back(w);
        }
      }
      ASSERT_TRUE(lateral != nullptr);
      EXPECT_NEAR(lateral->calculate(lane, value[0], value[4], value[5]), value[6], 1e-9);
      ++lateral_rows;
    } else {
      if (value[3] == 0.0)
        longitudinal.reset(new control::morai_mpc::LongitudinalMpc(parameters.longitudinal));
      ASSERT_TRUE(longitudinal != nullptr);
      const auto result = longitudinal->update(value[7], value[8], 1.0 / 30.0);
      ASSERT_TRUE(result.valid) << result.error;
      EXPECT_NEAR(result.accel, value[9], 1e-10);
      EXPECT_NEAR(result.brake, value[10], 1e-10);
      ++longitudinal_rows;
    }
  }
  EXPECT_EQ(lateral_rows, 216);
  EXPECT_EQ(longitudinal_rows, 36);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
