#include <gtest/gtest.h>
#include <path_planner/cores/roundabout_rolling_entry.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <cmath>
#include <memory>

namespace {
using path_planner::PredictedObjects;
using path_planner::RoundaboutRollingEntry;
using path_planner::RoundaboutRollingEntryParameters;

// Straight approach along +x: lanelets 1 (0..40 m), 2 (40..80 m), 3 (80..160 m),
// yield line at x = 50 m, departure lanelet 3. Ring conflict zone along x = 55 m toward +y:
// upstream lanelet 4 (y -40..-3 m) and merged lanelet 5 (y -3..20 m).
lanelet::Lanelet lane(lanelet::Id id, double begin_m, double end_m) {
  lanelet::Point3d left_a(10 * id + 1, begin_m, 1.75, 0), left_b(10 * id + 2, end_m, 1.75, 0);
  lanelet::Point3d right_a(10 * id + 3, begin_m, -1.75, 0), right_b(10 * id + 4, end_m, -1.75, 0);
  return lanelet::Lanelet(id, lanelet::LineString3d(100 * id + 1, {left_a, left_b}),
                          lanelet::LineString3d(100 * id + 2, {right_a, right_b}));
}

lanelet::Lanelet ring(lanelet::Id id, double begin_m, double end_m) {
  lanelet::Point3d left_a(10 * id + 1, 53.25, begin_m, 0), left_b(10 * id + 2, 53.25, end_m, 0);
  lanelet::Point3d right_a(10 * id + 3, 56.75, begin_m, 0), right_b(10 * id + 4, 56.75, end_m, 0);
  return lanelet::Lanelet(id, lanelet::LineString3d(100 * id + 1, {left_a, left_b}),
                          lanelet::LineString3d(100 * id + 2, {right_a, right_b}));
}

struct Fixture {
  Fixture() {
    map.map = std::shared_ptr<lanelet::LaneletMap>(
        lanelet::utils::createMap({lane(1, 0, 40), lane(2, 40, 80), lane(3, 80, 160),
                                   ring(4, -40, -3), ring(5, -3, 20)}).release());
    parameters.reference_lanelet_ids = {1, 2, 3};
    parameters.departure_lanelet_id = 3;
    geometry_msgs::Point a, b;
    a.x = b.x = 50;
    a.y = -1.75;
    b.y = 1.75;
    parameters.stop_line = {a, b};
    parameters.desired_speed_mps = 6.0;
    parameters.max_lateral_acceleration_mps2 = 3.0;
    parameters.curve_speed_ratio = 0.9;
    parameters.collision_margin_m = 0.7;
    parameters.following_time_gap_sec = 0.8;
    parameters.entry_gap_confirmation_sec = 0.15;
    parameters.rolling_response_sec = 0.5;
    parameters.rolling_accept_distance_m = 6.0;
    parameters.conflict_lanelet_ids = {4};
    parameters.conflict_downstream_lanelet_id = 5;
  }
  path_planner::SharedMap map{path_planner::SharedMapParameters{}};
  RoundaboutRollingEntryParameters parameters;
};

nav_msgs::Odometry ego(double x_m, double time_sec) {
  nav_msgs::Odometry odometry;
  odometry.header.stamp = ros::Time(time_sec);
  odometry.header.frame_id = "map";
  odometry.pose.pose.position.x = x_m;
  odometry.pose.pose.orientation.w = 1;
  return odometry;
}

interfaces::CompetitionVehicleStatus moving(double speed_mps) {
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps = speed_mps;
  return status;
}

interfaces::Objects observed(double time_sec) {
  interfaces::Objects objects;
  objects.header.stamp = ros::Time(time_sec);
  objects.header.frame_id = "map";
  return objects;
}

// A car crossing the approach at x = 55 m toward +y; it reaches y = 0 at 2.5 s.
interfaces::Object crossing(double y_m) {
  interfaces::Object object;
  object.id = 7;
  object.position.x = 55;
  object.position.y = y_m;
  object.velocity.y = 8;
  object.velocity_available = true;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.z = std::sin(M_PI / 4);
  object.bbox_pose.orientation.w = std::cos(M_PI / 4);
  object.bbox_size.x = 4.5;
  object.bbox_size.y = 1.9;
  object.bbox_size.z = 1.5;
  return object;
}

// A ring car at (55, y) for which the detector reports zero velocity.
interfaces::Object ringCar(double y_m) {
  auto object = crossing(y_m);
  object.id = 9;
  object.velocity.y = 0;
  return object;
}

// A car following on the approach lane at x, driving +x at 6 m/s.
interfaces::Object follower(double x_m) {
  interfaces::Object object;
  object.id = 11;
  object.position.x = x_m;
  object.velocity.x = 6;
  object.velocity_available = true;
  object.bbox_pose.position = object.position;
  object.bbox_pose.orientation.w = 1;
  object.bbox_size.x = 4.5;
  object.bbox_size.y = 1.9;
  object.bbox_size.z = 1.5;
  return object;
}

PredictedObjects predict(const interfaces::Objects& objects) {
  PredictedObjects predictions;
  for (std::size_t i = 0; i < predictions.size(); ++i) {
    predictions[i] = objects;
    for (auto& object : predictions[i].objects) {
      object.position.x += object.velocity.x * 0.5 * (i + 1);
      object.position.y += object.velocity.y * 0.5 * (i + 1);
      object.bbox_pose.position = object.position;
    }
  }
  return predictions;
}

// Highest commanded speed on the path beyond the yield target (x > 45.4 m).
double speedPastYield(const interfaces::Path& path) {
  double speed = 0;
  for (const auto& point : path.points)
    if (point.pose.pose.position.x > 46.0) speed = std::max(speed, point.velocity);
  return speed;
}
}  // namespace

TEST(RoundaboutRollingEntry, EntersWithoutStoppingWhenApproachIsClear) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  const auto objects = observed(100.0);
  const auto path = core.process(ego(40, 100.0), moving(5.0), objects, predict(objects), ros::Time(100.05));
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_GT(speedPastYield(path), 3.0);
}

TEST(RoundaboutRollingEntry, DefersRollingEntryUntilNearTheYieldTarget) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  const auto objects = observed(100.0);
  const auto path = core.process(ego(30, 100.0), moving(5.0), objects, predict(objects), ros::Time(100.05));
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_DOUBLE_EQ(speedPastYield(path), 0.0);
}

TEST(RoundaboutRollingEntry, YieldsToCrossingTrafficWhileApproaching) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  for (int tick = 0; tick < 6; ++tick) {
    const double time = 100.0 + tick / 30.0;
    auto objects = observed(time);
    objects.objects.push_back(crossing(-20 + 8 * tick / 30.0));
    const auto path = core.process(ego(40, time), moving(5.0), objects, predict(objects), ros::Time(time + 0.05));
    ASSERT_GE(path.points.size(), 2u);
    EXPECT_DOUBLE_EQ(speedPastYield(path), 0.0) << "tick " << tick;
  }
}

TEST(RoundaboutRollingEntry, KeepsAcceptedEntryCommitted) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  const auto clear = observed(100.0);
  ASSERT_GT(speedPastYield(core.process(ego(40, 100.0), moving(5.0), clear, predict(clear), ros::Time(100.05))), 3.0);
  auto objects = observed(100.1);
  objects.objects.push_back(crossing(-20));
  const auto path = core.process(ego(40.5, 100.1), moving(5.0), objects, predict(objects), ros::Time(100.15));
  EXPECT_GT(speedPastYield(path), 3.0);
}

// A wrong zero velocity is predicted as standing still; the zone still blocks a car that moves on the ring.
TEST(RoundaboutRollingEntry, RingCarWithZeroReportedVelocityBlocksEntry) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  for (int tick = 0; tick < 30; ++tick) {
    const double time = 100.0 + tick / 10.0;
    auto objects = observed(time);
    objects.objects.push_back(ringCar(-20 + 3.5 * tick / 10.0));
    const auto path = core.process(ego(40, time), moving(5.0), objects, predict(objects), ros::Time(time + 0.05));
    ASSERT_GE(path.points.size(), 2u);
    EXPECT_DOUBLE_EQ(speedPastYield(path), 0.0) << "tick " << tick;
  }
}

// A ring object that stays in place stops blocking once it is confirmed stationary.
TEST(RoundaboutRollingEntry, StationaryRingObjectReleasesEntry) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  double speed = 0;
  for (int tick = 0; tick <= 35; ++tick) {
    const double time = 100.0 + tick / 10.0;
    auto objects = observed(time);
    objects.objects.push_back(ringCar(-8));
    speed = speedPastYield(core.process(ego(40, time), moving(5.0), objects, predict(objects), ros::Time(time + 0.05)));
    if (tick == 20) EXPECT_DOUBLE_EQ(speed, 0.0);
  }
  EXPECT_GT(speed, 3.0);
}

// A short roll past the yield target (for example a start assist after a long stop) does not commit the entry.
TEST(RoundaboutRollingEntry, ShortRollPastYieldTargetDoesNotCommit) {
  Fixture fixture;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  auto objects = observed(100.0);
  objects.objects.push_back(crossing(-20));
  ASSERT_DOUBLE_EQ(speedPastYield(core.process(ego(45.4, 100.0), moving(0.0), objects, predict(objects),
                                               ros::Time(100.05))), 0.0);
  objects = observed(100.1);
  objects.objects.push_back(crossing(-19.2));
  const auto path = core.process(ego(46.3, 100.1), moving(0.4), objects, predict(objects), ros::Time(100.15));
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_DOUBLE_EQ(speedPastYield(path), 0.0);
}

// At rest, the fast launch bound only removes acceptances: over crossing cars that pass the merge
// at different times, every entry accepted with it is also accepted by the slow bound alone, and
// some entry just behind a crossing car that the slow bound accepts is rejected.
TEST(RoundaboutRollingEntry, AtRestEntryAlsoClearsTheFastLaunch) {
  const auto accepts = [](bool fast, double crossing_sec) {
    Fixture fixture;
    fixture.parameters.conflict_clear_sec = 0;
    if (!fast) {
      fixture.parameters.fast_launch_response_sec = fixture.parameters.slow_launch_response_sec;
      fixture.parameters.fast_launch_acceleration_mps2 = fixture.parameters.slow_launch_acceleration_mps2;
      fixture.parameters.fast_launch_jerk_mps3 = fixture.parameters.slow_launch_jerk_mps3;
      fixture.parameters.fast_launch_command_step_sec = fixture.parameters.command_step_sec;
    }
    RoundaboutRollingEntry core(fixture.parameters, fixture.map);
    double speed = 0;
    for (int tick = 0; tick < 2; ++tick) {
      const double time = 100.0 + 0.2 * tick;
      auto objects = observed(time);
      objects.objects.push_back(crossing(-8 * crossing_sec + 8 * 0.2 * tick));
      speed = speedPastYield(core.process(ego(45.4, time), moving(0.0), objects, predict(objects), ros::Time(time + 0.05)));
    }
    return speed > 3.0;
  };
  int rejected_by_fast = 0;
  for (double crossing_sec = 0.6; crossing_sec < 2.65; crossing_sec += 0.1) {
    const bool with_fast = accepts(true, crossing_sec), slow_only = accepts(false, crossing_sec);
    EXPECT_FALSE(with_fast && !slow_only) << "crossing at " << crossing_sec << " s";
    if (slow_only && !with_fast) ++rejected_by_fast;
  }
  EXPECT_GT(rejected_by_fast, 0);
}

// A car 10 m behind the rear at 6 m/s never touches the accelerating entry, but it closes inside the
// 1 s rear time gap, so the rolling entry waits; without the rear gap the same entry is accepted.
TEST(RoundaboutRollingEntry, KeepsRearTimeGapToFollowingTraffic) {
  for (const double rear_gap_sec : {1.0, 0.0}) {
    Fixture fixture;
    fixture.parameters.conflict_clear_sec = 0;
    fixture.parameters.rear_time_gap_sec = rear_gap_sec;
    RoundaboutRollingEntry core(fixture.parameters, fixture.map);
    double speed = 0;
    for (int tick = 0; tick < 2; ++tick) {
      const double time = 100.0 + 0.2 * tick;
      auto objects = observed(time);
      objects.objects.push_back(follower(27 + 6 * 0.2 * tick));
      const auto path = core.process(ego(40 + 3 * 0.2 * tick, time), moving(3.0), objects, predict(objects),
                                     ros::Time(time + 0.05));
      ASSERT_GE(path.points.size(), 2u);
      speed = speedPastYield(path);
    }
    if (rear_gap_sec > 0) EXPECT_DOUBLE_EQ(speed, 0.0);
    else EXPECT_GT(speed, 3.0);
  }
}

// Crossing traffic that never clears the fast bound holds the at-rest entry until patience_sec;
// then the relaxed planned launch, which reaches the crossing after the car, accepts the same gap.
TEST(RoundaboutRollingEntry, RelaxesAtRestCheckAfterPatience) {
  Fixture fixture;
  fixture.parameters.conflict_clear_sec = 0;
  fixture.parameters.patience_sec = 2.0;
  RoundaboutRollingEntry core(fixture.parameters, fixture.map);
  double speed = 0;
  for (int tick = 0; tick <= 15; ++tick) {
    const double time = 100.0 + 0.2 * tick;
    auto objects = observed(time);
    objects.objects.push_back(crossing(-12.8));
    speed = speedPastYield(core.process(ego(45.4, time), moving(0.0), objects, predict(objects), ros::Time(time + 0.05)));
    if (tick < 10) EXPECT_DOUBLE_EQ(speed, 0.0) << "tick " << tick;
  }
  EXPECT_GT(speed, 3.0);
}
