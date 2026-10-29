#include <gtest/gtest.h>
#include <path_planner/cores/integrated_prediction.hpp>
#include <path_planner/parameters/integrated_prediction.hpp>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace {
const std::string kPackage = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/unit/"));

path_planner::IntegratedPredictionParameters configured() {
  return path_planner::loadIntegratedPredictionParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/path_planner/parameters/integrated_prediction.yaml", kPackage));
}

// An observed body with id, centre and square size; its velocity is not measured.
interfaces::Object body(std::uint32_t id, double x, double y, double size_m) {
  interfaces::Object o;
  o.id = id;
  o.label = "unknown";
  o.position.x = x;
  o.position.y = y;
  o.bbox_pose.position = o.position;
  o.bbox_pose.orientation.w = 1;
  o.bbox_size.x = o.bbox_size.y = size_m;
  o.bbox_size.z = 1.7;
  return o;
}

path_planner::PredictedObjects predict(path_planner::IntegratedPrediction& core, const std::vector<interfaces::Object>& list,
                                       double time) {
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  interfaces::Objects objects;
  objects.header.stamp = ros::Time(time);
  objects.header.frame_id = "map";
  objects.objects = list;
  tf2_msgs::TFMessage tf;
  sensor_msgs::Imu imu;
  path_planner::Route route;
  return std::get<6>(core.process(ego, competition, objects, tf, imu, route));
}

// A walker crossing at 3 m/s whose tracker velocity is still unknown.
void expectWalkerVelocity(double rate_hz) {
  path_planner::IntegratedPrediction core(configured());
  const double dt = 1 / rate_hz;
  for (int k = 0; k < 4; ++k) {
    const auto future = predict(core, {body(9, -62 + 3 * dt * k, 14.5, 0.5)}, 100 + dt * k);
    ASSERT_EQ(future[0].objects.size(), 1u);
    const auto& o = future[0].objects.front();
    if (k < 2) continue;
    ASSERT_TRUE(o.velocity_available) << rate_hz << " Hz frame " << k;
    EXPECT_NEAR(o.velocity.x, 3, 0.05);
    EXPECT_NEAR(future[0].objects.front().position.x, -62 + 3 * dt * k + 1.5, 0.05);
  }
}
}  // namespace

TEST(IntegratedPrediction, FitsWalkerVelocityFromThreeSparseFrames) {
  expectWalkerVelocity(3.4);
  expectWalkerVelocity(3.3);
}

// From 5 Hz up (frames up to 0.2 s) the output equals the fixed 0.3 s gap and 0.5 s window.
TEST(IntegratedPrediction, TenHertzMatchesFixedLimits) {
  path_planner::IntegratedPrediction widened(configured()), fixed{path_planner::IntegratedPredictionParameters{}};
  double t = 100;
  for (int k = 0; k < 60; ++k) {
    t += k % 4 == 0 ? 0.115 : 0.095;
    std::vector<interfaces::Object> list;
    // A walker that stops after 2 s, and a car hidden for 0.35 s from frame 20.
    const double walk = std::min(t - 100, 2.0);
    list.push_back(body(3, -60 + 1.4 * walk, 10, 0.5));
    if (k < 20 || k > 23) list.push_back(body(5, -50, -20 + 8 * (t - 100), 2.0));
    const auto a = predict(widened, list, t);
    const auto b = predict(fixed, list, t);
    for (std::size_t i = 0; i < a.size(); ++i) {
      ASSERT_EQ(a[i].objects.size(), b[i].objects.size()) << "frame " << k;
      for (std::size_t j = 0; j < a[i].objects.size(); ++j) {
        EXPECT_EQ(a[i].objects[j].velocity_available, b[i].objects[j].velocity_available) << "frame " << k;
        EXPECT_DOUBLE_EQ(a[i].objects[j].velocity.x, b[i].objects[j].velocity.x) << "frame " << k;
        EXPECT_DOUBLE_EQ(a[i].objects[j].position.y, b[i].objects[j].position.y) << "frame " << k;
      }
    }
  }
}

namespace {
// Velocity of a body after five 10 Hz frames moving at (vx, 0) with the given square size.
interfaces::Object fitted(const path_planner::IntegratedPredictionParameters& parameters, double vx, double size_m) {
  path_planner::IntegratedPrediction core(parameters);
  path_planner::PredictedObjects future;
  for (int k = 0; k < 5; ++k) future = predict(core, {body(4, -40 + vx * 0.1 * k, 20, size_m)}, 200 + 0.1 * k);
  return future[0].objects.front();
}
}  // namespace

// A standing box whose fragments drift its centre at 0.5 m/s keeps its velocity unknown.
TEST(IntegratedPrediction, LargeSlowBodyKeepsVelocityUnknown) {
  EXPECT_FALSE(fitted(configured(), 0.5, 2.0).velocity_available);
  EXPECT_TRUE(fitted(path_planner::IntegratedPredictionParameters{}, 0.5, 2.0).velocity_available);
}

// A car moving at 1.2 m/s and a walker at 0.5 m/s keep the fitted velocity.
TEST(IntegratedPrediction, MovingCarAndSlowWalkerKeepTheirVelocity) {
  const path_planner::IntegratedPredictionParameters fixed{};
  for (const auto& sample : std::vector<std::pair<double, double>>{{1.2, 4.5}, {0.5, 0.5}}) {
    const double vx = sample.first, size = sample.second;
    const auto a = fitted(configured(), vx, size), b = fitted(fixed, vx, size);
    ASSERT_TRUE(a.velocity_available) << size;
    EXPECT_DOUBLE_EQ(a.velocity.x, b.velocity.x) << size;
    EXPECT_NEAR(a.velocity.x, vx, 0.01) << size;
  }
}
