#include <gtest/gtest.h>
#include <object_detector/cores/tracking.hpp>
#include <object_detector/parameters/tracking.hpp>
#include <cmath>
#include <string>
#include <vector>

namespace {
const std::string kPackage = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/unit/"));

object_detector::TrackingParameters configured() {
  return object_detector::loadTrackingParameters(node_pipeline::ParamReader::fromFile(
      kPackage + "/config/object_detector/parameters/tracking.yaml", kPackage));
}

// A car-sized box at (x, y) heading `yaw_rad`, far from the roundabout ring.
interfaces::Object box(double x, double y, double yaw_rad, double length_m = 4.6, double width_m = 1.9) {
  interfaces::Object o;
  o.label = "unknown";
  o.position.x = x;
  o.position.y = y;
  o.bbox_pose.position = o.position;
  o.bbox_pose.orientation.z = std::sin(yaw_rad / 2);
  o.bbox_pose.orientation.w = std::cos(yaw_rad / 2);
  o.bbox_size.x = length_m;
  o.bbox_size.y = width_m;
  o.bbox_size.z = 1.6;
  return o;
}

interfaces::Objects frame(const std::vector<interfaces::Object>& list, double time) {
  interfaces::Objects objects;
  objects.header.stamp = ros::Time(time);
  objects.header.frame_id = "map";
  objects.objects = list;
  return objects;
}
}  // namespace

TEST(Tracking, ConfiguredGateGrowsOnlyBeyondTenHertzFrames) {
  const auto p = configured();
  EXPECT_DOUBLE_EQ(p.association_gate_m, 2.5);
  EXPECT_LE(p.association_max_speed_mps * 0.12 + p.association_speed_margin_m, p.association_gate_m + 1e-9);
  EXPECT_GE(std::min(p.association_max_gate_m, p.association_max_speed_mps * (1 / 3.4) + p.association_speed_margin_m),
            16.7 / 3.4 + 0.5);
}

// /objects at 3.4 Hz: a 60 km/h car moves 4.9 m per frame, beyond the 2.5 m fixed gate.
TEST(Tracking, FastCarKeepsIdAtSparseFrames) {
  object_detector::Tracking tracking(configured());
  const double dt = 1 / 3.4, speed = 16.7;
  std::uint32_t id = 0;
  for (int k = 0; k < 10; ++k) {
    const auto output = tracking.process(frame({box(500 + speed * dt * k, 0, 0)}, 100 + dt * k), ros::Time(100 + dt * k));
    ASSERT_EQ(output.objects.size(), 1u);
    if (k == 0) id = output.objects.front().id;
    EXPECT_EQ(output.objects.front().id, id) << "frame " << k;
    // Five observations span 1.18 s inside the 2.0 s window.
    if (k >= 4) {
      ASSERT_TRUE(output.objects.front().velocity_available) << "frame " << k;
      EXPECT_NEAR(output.objects.front().velocity.x, speed, 0.5);
    }
  }
}

// Opposite-direction cars in adjacent lanes pass within the widened gate.
TEST(Tracking, PassingCarsKeepTheirIdsAtSparseFrames) {
  object_detector::Tracking tracking(configured());
  const double dt = 1 / 3.4, speed = 16.7;
  std::uint32_t east = 0, west = 0;
  for (int k = 0; k < 10; ++k) {
    const double t = dt * k;
    const auto output = tracking.process(
        frame({box(500 + speed * t, 0, 0), box(520 - speed * t, 3.5, M_PI)}, 100 + t), ros::Time(100 + t));
    ASSERT_EQ(output.objects.size(), 2u);
    for (const auto& o : output.objects) {
      auto& id = o.position.y < 1.75 ? east : west;
      if (k == 0) id = o.id;
      EXPECT_EQ(o.id, id) << "frame " << k << " lane y " << o.position.y;
    }
  }
  EXPECT_NE(east, west);
}

// A walker appearing near a car that left view starts its own track at sparse frames.
TEST(Tracking, WalkerDoesNotInheritADepartedCarTrack) {
  object_detector::Tracking tracking(configured());
  const double dt = 1 / 3.4;
  const auto car = tracking.process(frame({box(500, 0, 0)}, 100), ros::Time(100));
  const auto walker = tracking.process(frame({box(504, 0, 0, 0.5, 0.5)}, 100 + dt), ros::Time(100 + dt));
  ASSERT_EQ(car.objects.size(), 1u);
  ASSERT_EQ(walker.objects.size(), 1u);
  EXPECT_NE(walker.objects.front().id, car.objects.front().id);
}

// At 10 Hz (frames up to 0.115 s) the output equals the fixed 2.5 m gate with 8 velocity observations.
TEST(Tracking, TenHertzMatchesFixedGate) {
  auto fixed_parameters = configured();
  fixed_parameters.association_max_speed_mps = 0;
  fixed_parameters.min_velocity_observations = 8;
  object_detector::Tracking widened(configured()), fixed(fixed_parameters);
  double t = 0;
  for (int k = 0; k < 40; ++k) {
    t += k % 3 == 0 ? 0.115 : 0.09;
    // A fast car, a walker crossing next to a parked car, and a car entering at frame 15.
    std::vector<interfaces::Object> list = {box(500 + 16.7 * t, 0, 0), box(530, 6, 0),
                                            box(528 + 1.4 * t, 2 + 1.4 * t, M_PI / 4, 0.5, 0.5)};
    if (k >= 15) list.push_back(box(560 - 12 * (t - 1.5), 3.5, M_PI));
    const auto a = widened.process(frame(list, 100 + t), ros::Time(100 + t));
    const auto b = fixed.process(frame(list, 100 + t), ros::Time(100 + t));
    ASSERT_EQ(a.objects.size(), b.objects.size()) << "frame " << k;
    for (std::size_t i = 0; i < a.objects.size(); ++i) {
      EXPECT_EQ(a.objects[i].id, b.objects[i].id) << "frame " << k;
      EXPECT_EQ(a.objects[i].velocity_available, b.objects[i].velocity_available) << "frame " << k;
      EXPECT_DOUBLE_EQ(a.objects[i].velocity.x, b.objects[i].velocity.x) << "frame " << k;
    }
  }
}
