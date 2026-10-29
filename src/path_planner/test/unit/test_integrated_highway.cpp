#include <gtest/gtest.h>
#include <path_planner/cores/integrated_highway.hpp>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/utility/Utilities.h>
#include <cmath>
#include <map>

namespace {
// HighwayMission logs with ROS_*_THROTTLE, which reads ros::Time::now().
struct RosTime : ::testing::Environment {
  void SetUp() override { ros::Time::init(); }
};
const auto* const ros_time = ::testing::AddGlobalTestEnvironment(new RosTime);

// One straight southbound lane from y=+50 to y=-250, sampled every metre.
lanelet::LaneletMapPtr southboundLane() {
  lanelet::LineString3d left(lanelet::utils::getId()), right(lanelet::utils::getId());
  for (int y = 50; y >= -250; --y) {
    left.push_back(lanelet::Point3d(lanelet::utils::getId(), 1.75, y, 0));
    right.push_back(lanelet::Point3d(lanelet::utils::getId(), -1.75, y, 0));
  }
  lanelet::Lanelet lane(lanelet::utils::getId(), left, right);
  lane.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return lanelet::utils::createMap({lane});
}

// Two southbound lanes from y=+50 to y=-250 sharing a dashed boundary; the right lane is centred
// on x=0 and the left lane on x=lane_width_m (left of travel is +x).
struct Road {
  lanelet::LaneletMapPtr map;
  lanelet::ConstLanelet right, left;
};
Road southboundRoad(double lane_width_m = 3.5) {
  lanelet::LineString3d outer_left(lanelet::utils::getId()), shared(lanelet::utils::getId()),
      outer_right(lanelet::utils::getId());
  for (int y = 50; y >= -250; --y) {
    outer_left.push_back(lanelet::Point3d(lanelet::utils::getId(), 1.5 * lane_width_m, y, 0));
    shared.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.5 * lane_width_m, y, 0));
    outer_right.push_back(lanelet::Point3d(lanelet::utils::getId(), -0.5 * lane_width_m, y, 0));
  }
  for (auto* line : {&outer_left, &shared, &outer_right})
    line->attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  outer_left.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Solid;
  shared.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Dashed;
  outer_right.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Solid;
  lanelet::Lanelet right(lanelet::utils::getId(), shared, outer_right), left(lanelet::utils::getId(), outer_left, shared);
  right.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  left.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return {lanelet::utils::createMap({right, left}), right, left};
}

// Two southbound lanes from y=+50 to y=-150 sharing a dashed boundary at x=1.75 (right lane on x=0,
// left lane on x=3.5). One lane tapers from 3.5 m at y=-40 to 0.5 m at y=-100, like the lanes merging
// into the toll-plaza lane; it is narrower than the car (2.16 m) from y=-66.8.
Road taperRoad(bool left_tapers) {
  lanelet::LineString3d outer_left(lanelet::utils::getId()), shared(lanelet::utils::getId()),
      outer_right(lanelet::utils::getId());
  for (int y = 50; y >= -150; --y) {
    const double narrowed = 3.0 * std::min(1.0, std::max(0.0, (-40.0 - y) / 60.0));
    outer_left.push_back(lanelet::Point3d(lanelet::utils::getId(), 5.25 - (left_tapers ? narrowed : 0.0), y, 0));
    shared.push_back(lanelet::Point3d(lanelet::utils::getId(), 1.75, y, 0));
    outer_right.push_back(lanelet::Point3d(lanelet::utils::getId(), -1.75 + (left_tapers ? 0.0 : narrowed), y, 0));
  }
  for (auto* line : {&outer_left, &shared, &outer_right})
    line->attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  outer_left.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Solid;
  shared.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Dashed;
  outer_right.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Solid;
  lanelet::Lanelet right(lanelet::utils::getId(), shared, outer_right), left(lanelet::utils::getId(), outer_left, shared);
  right.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  left.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return {lanelet::utils::createMap({right, left}), right, left};
}

// Toll-plaza shape: one southbound lane from y=+50 to y=-30, where a left lane opens beside it.
// The two lanes share a dashed boundary until y=-110 and a solid one to y=-250, so the left
// change window is 80 m long. Lanes are centred on x=0 (right) and x=3.5 (left).
struct ShortWindowRoad {
  lanelet::LaneletMapPtr map;
  lanelet::routing::LaneletPath route;  // single lane, right lane, then the change into the left lane
};
ShortWindowRoad shortWindowRoad() {
  // Pieces meeting at the same coordinate share the point, so the routing graph connects them.
  std::map<std::pair<int, int>, lanelet::Point3d> points;
  const auto line = [&points](double x, int from_y, int to_y, const char* subtype) {
    lanelet::LineString3d result(lanelet::utils::getId());
    for (int y = from_y; y >= to_y; --y) {
      const auto key = std::make_pair(static_cast<int>(std::lround(x * 100)), y);
      auto found = points.find(key);
      if (found == points.end())
        found = points.emplace(key, lanelet::Point3d(lanelet::utils::getId(), x, y, 0)).first;
      result.push_back(found->second);
    }
    result.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
    result.attributes()[lanelet::AttributeName::Subtype] = subtype;
    return result;
  };
  const auto window_shared = line(1.75, -30, -110, lanelet::AttributeValueString::Dashed);
  const auto plaza_shared = line(1.75, -110, -250, lanelet::AttributeValueString::Solid);
  lanelet::Lanelet entry(lanelet::utils::getId(), line(1.75, 50, -30, lanelet::AttributeValueString::Solid),
                         line(-1.75, 50, -30, lanelet::AttributeValueString::Solid));
  lanelet::Lanelet window_right(lanelet::utils::getId(), window_shared,
                                line(-1.75, -30, -110, lanelet::AttributeValueString::Solid));
  lanelet::Lanelet window_left(lanelet::utils::getId(), line(5.25, -30, -110, lanelet::AttributeValueString::Solid),
                               window_shared);
  lanelet::Lanelet plaza_right(lanelet::utils::getId(), plaza_shared,
                               line(-1.75, -110, -250, lanelet::AttributeValueString::Solid));
  lanelet::Lanelet plaza_left(lanelet::utils::getId(), line(5.25, -110, -250, lanelet::AttributeValueString::Solid),
                              plaza_shared);
  for (auto* lane : {&entry, &window_right, &window_left, &plaza_right, &plaza_left})
    lane->attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  return {lanelet::utils::createMap({entry, window_right, window_left, plaza_right, plaza_left}),
          lanelet::routing::LaneletPath(lanelet::ConstLanelets{entry, window_right, window_left, plaza_left})};
}

std::vector<double> stations(const path_planner::Message& path) {
  std::vector<double> result(path.points.size(), 0);
  for (std::size_t i = 1; i < result.size(); ++i) {
    const auto& a = path.points[i - 1].pose.pose.position;
    const auto& b = path.points[i].pose.pose.position;
    result[i] = result[i - 1] + std::hypot(b.x - a.x, b.y - a.y);
  }
  return result;
}

nav_msgs::Odometry southboundEgo(double x, double y, double stamp_sec) {
  nav_msgs::Odometry ego;
  ego.header.stamp = ros::Time(stamp_sec);
  ego.pose.pose.position.x = x;
  ego.pose.pose.position.y = y;
  ego.pose.pose.orientation.z = std::sin(-M_PI / 4);
  ego.pose.pose.orientation.w = std::cos(-M_PI / 4);
  return ego;
}

path_planner::HighwayMissionParameters parameters() {
  path_planner::HighwayMissionParameters result;
  result.reference_smoothing_enabled = false;
  result.mission_end_y_m = 1000;  // follow the given route without mission lane changes
  result.mission_change_count = 1;
  result.entry_speed_mps = 60.0 / 3.6;
  result.exit_speed_mps = 15.0;
  result.highway_end_y_m = -100;
  result.stopping_deceleration_mps2 = 3;
  return result;
}

// The merge mission (one left change) is active over the whole test road.
path_planner::HighwayMissionParameters mergeParameters() {
  auto result = parameters();
  result.mission_end_y_m = -1000;
  result.highway_end_y_m = -1000;
  return result;
}

// The route change of the short window at cruise speed: an 80 m change cannot run at the cruise
// design speed, so the waiting stop is reserved at the window start (y=-30).
path_planner::HighwayMissionParameters shortWindowParameters() {
  auto result = parameters();
  result.entry_speed_mps = result.cruise_speed_mps;  // no merge-entry cap on this road
  result.highway_end_y_m = -1000;
  return result;
}

struct Step {
  path_planner::Message path;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  interfaces::Objects objects;
};
Step step(path_planner::HighwayMission& mission, const path_planner::Route& route, double x, double y,
          double speed_mps, double stamp_sec, const std::vector<interfaces::Object>& objects = {}) {
  Step result;
  result.ego = southboundEgo(x, y, stamp_sec);
  result.competition.velocity_x_mps = speed_mps;
  result.objects.header.stamp = result.ego.header.stamp;
  result.objects.objects = objects;
  result.path = mission.process(result.ego, result.competition, result.objects, route);
  return result;
}

void expectSamePath(const path_planner::Message& actual, const path_planner::Message& expected) {
  ASSERT_EQ(actual.points.size(), expected.points.size());
  for (std::size_t i = 0; i < actual.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(actual.points[i].pose.pose.position.x, expected.points[i].pose.pose.position.x) << i;
    EXPECT_DOUBLE_EQ(actual.points[i].pose.pose.position.y, expected.points[i].pose.pose.position.y) << i;
    EXPECT_DOUBLE_EQ(actual.points[i].velocity, expected.points[i].velocity) << i;
  }
}

// Drives the merge: gap wait, commit, then completion on the left lane.
void completeMerge(path_planner::HighwayMission& mission, const path_planner::Route& route) {
  step(mission, route, 0, -20, 16, 10);
  const auto committed = step(mission, route, 0, -29.6, 16, 10.6);
  ASSERT_GT(committed.path.points.back().pose.pose.position.x, 3.0);  // the lane change was committed
  step(mission, route, 3.5, -100, 16, 15);
}

// A stopped southbound car, 4.5 m long and 2 m wide.
interfaces::Object stoppedCar(std::uint32_t id, double x, double y) {
  interfaces::Object car;
  car.id = id;
  car.position.x = car.bbox_pose.position.x = x;
  car.position.y = car.bbox_pose.position.y = y;
  car.bbox_pose.orientation.z = std::sin(-M_PI / 4);
  car.bbox_pose.orientation.w = std::cos(-M_PI / 4);
  car.bbox_size.x = 4.5;
  car.bbox_size.y = 2.0;
  car.bbox_size.z = 1.5;
  car.velocity_available = true;
  return car;
}
}  // namespace

TEST(IntegratedHighway, FirstCallWhileDrivingPlansFromEgo) {
  const auto road = southboundRoad();
  path_planner::HighwayMission mission(mergeParameters(), *road.map);
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{road.right});

  // Taking over from S10 mid-lane at speed: a moving path from the ego, no stop.
  const auto first = step(mission, route, 0, -20, 16, 10);
  ASSERT_GT(first.path.points.size(), 30u);
  const auto& front = first.path.points.front().pose.pose.position;
  EXPECT_NEAR(front.x, 0, 1e-6);
  EXPECT_NEAR(front.y, -20, 1e-6);
  EXPECT_NEAR(first.path.points.front().velocity, 16.0, 1e-6);
  for (const auto& point : first.path.points)
    if (point.pose.pose.position.y > -50) EXPECT_GT(point.velocity, 15.0) << "y=" << point.pose.pose.position.y;
}

TEST(IntegratedHighway, CheckpointReturnRestartsMergeMission) {
  const auto road = southboundRoad();
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{road.right});
  path_planner::HighwayMission returned(mergeParameters(), *road.map), fresh(mergeParameters(), *road.map);
  completeMerge(returned, route);

  // Back on the merge lane 80 m behind the last call: plan the merge again as a fresh Core would.
  const auto after_return = step(returned, route, 0, -20, 8, 20);
  const auto first_call = step(fresh, route, 0, -20, 8, 20);
  expectSamePath(after_return.path, first_call.path);
  EXPECT_LT(after_return.path.points.front().velocity, 60.0 / 3.6 + 1e-6);  // entry speed until the merge
}

TEST(IntegratedHighway, ClockRewindRestartsMergeMission) {
  const auto road = southboundRoad();
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{road.right});
  path_planner::HighwayMission rewound(mergeParameters(), *road.map), fresh(mergeParameters(), *road.map);
  completeMerge(rewound, route);

  // A new lap restarts the clock; the ego is not behind the last call.
  const auto after_rewind = step(rewound, route, 0, -120, 12, 2);
  const auto first_call = step(fresh, route, 0, -120, 12, 2);
  expectSamePath(after_rewind.path, first_call.path);
}

TEST(IntegratedHighway, PathPointsPreviewExitBrakingProfile) {
  auto map = southboundLane();
  path_planner::HighwayMission mission(parameters(), *map);
  nav_msgs::Odometry ego;
  ego.header.stamp = ros::Time(10);
  ego.pose.pose.position.y = -80;
  ego.pose.pose.orientation.z = std::sin(-M_PI / 4);
  ego.pose.pose.orientation.w = std::cos(-M_PI / 4);
  interfaces::CompetitionVehicleStatus competition;
  competition.velocity_x_mps = 16.0;
  interfaces::Objects objects;
  objects.header.stamp = ego.header.stamp;
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{*map->laneletLayer.begin()});

  const auto path = mission.process(ego, competition, objects, route);
  ASSERT_GT(path.points.size(), 30u);
  bool saw_exit_speed = false;
  for (const auto& point : path.points) {
    const double y = point.pose.pose.position.y;
    const double profile = std::sqrt(15.0 * 15.0 + 2 * 3 * std::max(0.0, y + 100));
    EXPECT_NEAR(point.velocity, std::min(16.0, profile), 1e-6) << "y=" << y;
    if (y <= -100) {
      EXPECT_NEAR(point.velocity, 15.0, 1e-6);
      saw_exit_speed = true;
    }
  }
  EXPECT_TRUE(saw_exit_speed);
  EXPECT_NEAR(path.points.front().velocity, 16.0, 1e-6);  // ego far enough before the exit
}

TEST(IntegratedHighway, FollowerInOwnLaneDoesNotHoldMerge) {
  // Toll-plaza lanes are narrow: a car queued 7 m behind in the ego lane falls inside the
  // target-lane corridor. It follows the ego, so the merge must not wait for it.
  const auto road = southboundRoad(2.2);
  path_planner::HighwayMission mission(mergeParameters(), *road.map);
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{road.right});
  const std::vector<interfaces::Object> follower{stoppedCar(18, 0, -13)};
  step(mission, route, 0, -20, 0, 10, follower);
  const auto second = step(mission, route, 0, -20, 0, 10.6, follower);
  EXPECT_GT(second.path.points.back().pose.pose.position.x, 1.8);  // committed into the left lane
}

TEST(IntegratedHighway, FastApproachToShortWindowSlowsOnlyToChangeSpeed) {
  // 26 m/s, 30 m before the window: a stop at the window start would need 11 m/s².
  const auto road = shortWindowRoad();
  path_planner::HighwayMission mission(shortWindowParameters(), *road.map);
  const path_planner::Route route = road.route;
  const auto first = step(mission, route, 0, 0, 26, 10);
  ASSERT_GT(first.path.points.size(), 30u);
  EXPECT_LT(first.path.points.back().pose.pose.position.x, 0.5);  // waiting in the right lane
  // The 80 m change runs at (80 m - stop buffer) / sqrt(6 * 3.5 m / 2 m/s²) = 23.8 m/s.
  const double change_speed = (80.0 - 3.0) / std::sqrt(6 * 3.5 / 2.0);
  // The 26 m/s braking distance (113 m) passes the last wait that leaves the shortest change
  // (sqrt(6 * 3.5 m / 0.12 /m) = 13.2 m) in the window: stop there, 3 m buffer before it.
  const double last_stop = 30.0 + 80.0 - std::sqrt(6 * 3.5 / 0.12) - 3.0;
  EXPECT_GT(first.path.points.front().velocity, 23.5);  // a stop at the window start allows 12.7 m/s
  const auto s = stations(first.path);
  bool saw_stop = false;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const double stopping = std::sqrt(2 * 3 * std::max(0.0, last_stop - s[i]));
    EXPECT_NEAR(first.path.points[i].velocity, std::min(change_speed, stopping), 1e-6) << "s=" << s[i];
    saw_stop = saw_stop || s[i] >= last_stop;
  }
  EXPECT_TRUE(saw_stop);
}

TEST(IntegratedHighway, SlowApproachToShortWindowKeepsWaitingStop) {
  // 8 m/s can still stop before the window start: wait there as designed.
  const auto road = shortWindowRoad();
  path_planner::HighwayMission mission(shortWindowParameters(), *road.map);
  const path_planner::Route route = road.route;
  const auto first = step(mission, route, 0, 0, 8, 10);
  ASSERT_GT(first.path.points.size(), 30u);
  const auto s = stations(first.path);
  bool saw_stop = false;
  for (std::size_t i = 0; i < s.size(); ++i)
    if (s[i] >= 30.0 - 3.0) {  // window start minus the stop buffer
      EXPECT_NEAR(first.path.points[i].velocity, 0.0, 1e-6) << "s=" << s[i];
      saw_stop = true;
    }
  EXPECT_TRUE(saw_stop);
}

// Commits the left change on the taper road from 10 m/s, and returns the y where the committed path
// first lies on the left lane centreline.
double taperHandoverY(const Road& road) {
  auto parameters = mergeParameters();
  parameters.entry_speed_mps = parameters.cruise_speed_mps;  // cruise-length changes before the taper rule
  path_planner::HighwayMission mission(parameters, *road.map);
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{road.right});
  step(mission, route, 0, 0, 10, 10);
  const auto committed = step(mission, route, 0, -6, 10, 10.6);
  EXPECT_GT(committed.path.points.back().pose.pose.position.x, 1.75);  // the change was committed
  for (const auto& point : committed.path.points) {
    const auto& p = point.pose.pose.position;
    if (lanelet::geometry::distance2d(road.left.centerline2d(), lanelet::BasicPoint2d(p.x, p.y)) < 0.05) return p.y;
  }
  return -1e9;
}

TEST(IntegratedHighway, TaperingSourceLaneKeepsChangeLength) {
  // Out of a lane that tapers into a full-width target, the blended path converges on the target:
  // keep the design change (90 m, ending near y=-96) rather than slowing to merge before the taper.
  EXPECT_LT(taperHandoverY(taperRoad(false)), -66.8);
}

TEST(IntegratedHighway, TaperingTargetLaneHandsOverBeforeItNarrows) {
  // Into a lane that tapers, hand over where it is still wider than the car.
  EXPECT_GT(taperHandoverY(taperRoad(true)), -66.8);
}

TEST(IntegratedHighway, TargetLaneCarBehindHoldsMerge) {
  const auto road = southboundRoad(2.2);
  path_planner::HighwayMission mission(mergeParameters(), *road.map);
  const path_planner::Route route = lanelet::routing::LaneletPath(lanelet::ConstLanelets{road.right});
  const std::vector<interfaces::Object> rear{stoppedCar(18, 2.2, -13)};
  step(mission, route, 0, -20, 0, 10, rear);
  const auto second = step(mission, route, 0, -20, 0, 10.6, rear);
  EXPECT_LT(second.path.points.back().pose.pose.position.x, 0.5);  // still waiting in the right lane
}
