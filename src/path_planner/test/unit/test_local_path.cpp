#include <path_planner/cores/local_path.hpp>
#include <gtest/gtest.h>

namespace pp = path_planner;
namespace {
pp::SharedMap shortMap() {
  std::vector<lanelet::Point3d> left, right;
  for (int i = 0; i <= 10; ++i) {
    left.emplace_back(100 + 2*i, 5.0*i, 1, 10 + 0.5*i);
    right.emplace_back(101 + 2*i, 5.0*i, -1, 10 + 0.5*i);
  }
  lanelet::Lanelets lanes;
  for (int i = 0; i < 10; ++i) {
    lanelet::Lanelet lane(300 + i,
        lanelet::LineString3d(200 + 2*i, {left[i], left[i + 1]}),
        lanelet::LineString3d(201 + 2*i, {right[i], right[i + 1]}));
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    lanes.push_back(lane);
  }
  return pp::SharedMap({lanelet::utils::createMap(lanes)});
}
}

TEST(LocalPath, StartsAtEgoProjectionAnywhereInRoute) {
  auto shared = shortMap();
  pp::LocalPath core({20.0}, shared);
  const pp::Route route = lanelet::routing::LaneletPath({
      shared.map->laneletLayer.get(300), shared.map->laneletLayer.get(301),
      shared.map->laneletLayer.get(302)});
  struct Case { double ego_x, start_z, end_x, end_z; };
  const Case cases[] = {{2.0, 10.2, 22.0, 12.2}, {7.0, 10.7, 27.0, 12.7},
                        {12.0, 11.2, 32.0, 13.2}};
  for (const auto& sample : cases) {
    SCOPED_TRACE(sample.ego_x);
    nav_msgs::Odometry ego;
    ego.pose.pose.position.x = sample.ego_x;
    ego.pose.pose.position.y = 0.5;
    ego.pose.pose.orientation.w = 1.0;
    const auto path = core.process(ego, interfaces::CompetitionVehicleStatus{}, interfaces::Objects{},
        tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, route, pp::PredictedObjects{});
    ASSERT_GE(path.points.size(), 2u);
    const auto& start = path.points.front().pose.pose.position;
    const auto& end = path.points.back().pose.pose.position;
    EXPECT_NEAR(start.x, sample.ego_x, 1e-9);
    EXPECT_NEAR(start.y, 0.0, 1e-9);
    EXPECT_NEAR(start.z, sample.start_z, 1e-9);
    EXPECT_NEAR(end.x, sample.end_x, 1e-9);
    EXPECT_NEAR(end.z, sample.end_z, 1e-9);
    double length_m = 0.0;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      const auto& point = path.points[i].pose.pose.position;
      EXPECT_GE(point.x, sample.ego_x);
      EXPECT_NEAR(point.y, 0.0, 1e-9);
      if (i == 0) continue;
      const auto& previous = path.points[i - 1].pose.pose.position;
      EXPECT_GT(point.x, previous.x);
      length_m += std::hypot(point.x - previous.x, point.y - previous.y);
    }
    EXPECT_NEAR(length_m, 20.0, 1e-9);
  }
}
