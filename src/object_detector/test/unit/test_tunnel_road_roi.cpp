#include <gtest/gtest.h>
#include <object_detector/cores/tunnel_road_roi.hpp>
#include <object_detector/parameters/road_roi.hpp>
#include <object_detector/parameters/tunnel_road_roi.hpp>
#include <cmath>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace {
const std::string kPackage = std::string(__FILE__).substr(0, std::string(__FILE__).rfind("/test/unit/"));

// Tunnel axis (localization tunnel_wall geometry): station along the bore, lateral to its left.
constexpr double kOriginX = -23.5, kOriginY = -519.2000122070312, kAxisYaw = -2.6434409392630127;

object_detector::TunnelRoadRoi& roi() {
  static object_detector::TunnelRoadRoi core(object_detector::loadTunnelRoadRoiParameters(
      node_pipeline::ParamReader::fromFile(kPackage + "/config/object_detector/parameters/tunnel_road_roi.yaml", kPackage)));
  return core;
}

struct Point { float x, y, z; };

// Leveled cloud in base_link_level with the vehicle at tunnel `station_m`, lateral -1.7 m (our lane), facing the exit.
struct Frame {
  sensor_msgs::PointCloud2 cloud;
  interfaces::CompetitionVehicleStatus competition;
  tf2_msgs::TFMessage tf;

  Frame(double station_m, const std::vector<Point>& points) {
    cloud.header.frame_id = "base_link_level";
    cloud.height = 1;
    cloud.width = points.size();
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    for (const char* name : {"x", "y", "z"}) {
      sensor_msgs::PointField field;
      field.name = name;
      field.offset = 4 * cloud.fields.size();
      field.datatype = sensor_msgs::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    geometry_msgs::TransformStamped level;
    level.header.frame_id = "map";
    level.child_frame_id = "base_link_level";
    level.transform.translation.x = kOriginX + station_m * std::cos(kAxisYaw) + 1.7 * std::sin(kAxisYaw);
    level.transform.translation.y = kOriginY + station_m * std::sin(kAxisYaw) - 1.7 * std::cos(kAxisYaw);
    level.transform.rotation.z = std::sin(kAxisYaw / 2);
    level.transform.rotation.w = std::cos(kAxisYaw / 2);
    tf.transforms.push_back(level);
  }
};

std::vector<Point> kept(const Frame& frame) {
  const auto output = std::get<0>(roi().process(frame.cloud, frame.competition, frame.tf));
  std::vector<Point> points(output.width);
  std::memcpy(points.data(), output.data.data(), points.size() * sizeof(Point));
  return points;
}

bool contains(const std::vector<Point>& points, const Point& point) {
  for (const auto& p : points)
    if (p.x == point.x && p.y == point.y && p.z == point.z) return true;
  return false;
}
}  // namespace

TEST(TunnelRoadRoi, KeepsWalkwayBodyAboveWalkwayHeightInsideTunnel) {
  // 15 m ahead: right walkway at tunnel lateral -4.3 m (2.6 m right of us), beyond the lane edge near -3.8 m.
  const Point torso{15.f, -2.6f, 0.5f}, curb{15.f, -2.6f, -0.25f};
  const auto points = kept(Frame(-30.0, {torso, curb}));
  EXPECT_TRUE(contains(points, torso));
  EXPECT_FALSE(contains(points, curb));
}

TEST(TunnelRoadRoi, KeepsLanePointsAtEveryHeight) {
  const Point low{15.f, 0.f, -0.3f}, high{15.f, 0.f, 1.0f};
  const auto points = kept(Frame(-30.0, {low, high}));
  EXPECT_TRUE(contains(points, low));
  EXPECT_TRUE(contains(points, high));
}

TEST(TunnelRoadRoi, DropsWallClearanceBehindAndBeyondPortal) {
  // Walls at -6.01 / +6.00 m keep 1.0 m clear: lateral -5.9 m is 4.2 m right of us, +5.35 m (a ledge along the
  // left wall) is 7.05 m left of us.
  const Point wall{15.f, -4.2f, 1.0f}, ledge{15.f, 7.05f, 0.5f}, behind{-2.f, -2.6f, 0.5f};
  const auto inside = kept(Frame(-30.0, {wall, ledge, behind}));
  EXPECT_FALSE(contains(inside, wall));
  EXPECT_FALSE(contains(inside, ledge));
  EXPECT_FALSE(contains(inside, behind));
  // 25 m before the entrance portal (station -58.6 m) the walkway band does not apply.
  const Point outside{15.f, -2.6f, 0.5f};
  EXPECT_FALSE(contains(kept(Frame(-98.0, {outside})), outside));
}

TEST(TunnelRoadRoi, MatchesRoadRoiOutsideTunnel) {
  // Before the entrance portal (station -58.6 m), with the grid ending short of it, and past the exit portal:
  // a grid of points ahead and to both sides at road, walkway and body heights keeps exactly road_roi's points.
  object_detector::RoadRoi road(object_detector::loadRoadRoiParameters(
      node_pipeline::ParamReader::fromFile(kPackage + "/config/object_detector/parameters/road_roi.yaml", kPackage)));
  std::vector<Point> grid;
  for (float x = -3.f; x <= 30.f; x += 1.5f)
    for (float y = -9.f; y <= 9.f; y += 0.5f)
      for (float z : {-0.3f, 0.5f, 1.5f}) grid.push_back({x, y, z});
  for (double station : {-110.0, -95.0, 59.0}) {
    const Frame frame(station, grid);
    const auto expected = std::get<0>(road.process(frame.cloud, frame.competition, frame.tf));
    const auto actual = std::get<0>(roi().process(frame.cloud, frame.competition, frame.tf));
    EXPECT_GT(expected.width, 0u) << station;
    EXPECT_EQ(actual.width, expected.width) << station;
    EXPECT_EQ(actual.data, expected.data) << station;
  }
}
