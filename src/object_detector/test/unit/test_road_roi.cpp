#include <object_detector/cores/road_roi.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <initializer_list>
#include <map>
#include <utility>
#include <vector>

namespace pkg = object_detector;
namespace {
using XY = std::pair<double, double>;

struct RoadMap {
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id next_id = 1;
  std::map<XY, lanelet::Point3d> points;
  std::map<std::vector<XY>, lanelet::LineString3d> lines;

  lanelet::LineString3d line(std::initializer_list<XY> coordinates, bool permitted = true) {
    const std::vector<XY> key(coordinates);
    const auto existing = lines.find(key);
    if (existing != lines.end()) return existing->second;
    lanelet::Points3d vertices;
    for (const auto& xy : coordinates) {
      auto found = points.find(xy);
      if (found == points.end())
        found = points.emplace(xy, lanelet::Point3d(next_id++, xy.first, xy.second, 0)).first;
      vertices.push_back(found->second);
    }
    lanelet::LineString3d boundary(next_id++, vertices);
    boundary.attributes()["type"] = "line_thin";
    boundary.attributes()["subtype"] = permitted ? "dashed" : "solid";
    boundary.attributes()["lane_change"] = permitted ? "yes" : "no";
    lines.emplace(key, boundary);
    return boundary;
  }

  lanelet::Lanelet lane(lanelet::LineString3d left, lanelet::LineString3d right) {
    lanelet::Lanelet result(next_id++, left, right);
    result.attributes()["subtype"] = "road";
    result.attributes()["location"] = "urban";
    result.attributes()["participant:vehicle"] = "yes";
    result.attributes()["one_way"] = "yes";
    map->add(result);
    return result;
  }

  void straight(double start, double end, double left, double right,
                bool allow_left = true, bool allow_right = true) {
    lane(line({{start, left}, {end, left}}, allow_left),
         line({{start, right}, {end, right}}, allow_right));
  }
};

tf2_msgs::TFMessage pose(double x = 0, double y = 0, double yaw = 0) {
  tf2_msgs::TFMessage result;
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = ros::Time(10);
  transform.child_frame_id = "base_link_level";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.rotation.z = std::sin(yaw / 2);
  transform.transform.rotation.w = std::cos(yaw / 2);
  result.transforms.push_back(transform);
  return result;
}

sensor_msgs::PointCloud2 cloud(std::initializer_list<std::array<float, 3>> points) {
  sensor_msgs::PointCloud2 result;
  result.header.frame_id = "base_link_level";
  result.header.stamp = ros::Time(10);
  result.height = 1;
  result.width = points.size();
  result.point_step = 16;
  result.row_step = result.width * result.point_step;
  result.data.resize(result.row_step);
  result.fields.resize(4);
  const char* names[] = {"x", "y", "z", "intensity"};
  for (std::size_t i = 0; i < 4; ++i) {
    result.fields[i].name = names[i];
    result.fields[i].offset = 4 * i;
    result.fields[i].count = 1;
    result.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
  }
  std::size_t i = 0;
  for (const auto& xyz : points) {
    auto* record = result.data.data() + i * result.point_step;
    std::memcpy(record, xyz.data(), 12);
    const float index = i++;
    std::memcpy(record + 12, &index, 4);
  }
  return result;
}

std::vector<float> indices(const sensor_msgs::PointCloud2& value) {
  std::vector<float> result;
  for (std::uint32_t i = 0; i < value.width; ++i) {
    float index;
    std::memcpy(&index, value.data.data() + i * value.point_step + 12, 4);
    result.push_back(index);
  }
  return result;
}

pkg::RoadRoiParameters parameters(const RoadMap& road, double distance = 50) {
  pkg::RoadRoiParameters result;
  result.map = road.map;
  result.forward_distance_m = distance;
  result.left_inflation_m = 0;
  result.right_inflation_m = 0;
  return result;
}
}  // namespace

TEST(RoadRoi, ExpandsAllowedNeighborsAtSuccessorsAndFollowsTheirSuccessors) {
  RoadMap road;
  road.straight(0, 10, 2, -2);
  road.straight(10, 20, 2, -2, true, false);
  road.straight(20, 40, 2, -2, true, false);
  road.straight(10, 20, 6, 2);
  road.straight(20, 40, 6, 2);
  road.straight(10, 20, 10, 6);
  road.straight(20, 40, 10, 6);
  road.straight(10, 20, -2, -6, false);
  road.straight(20, 40, -2, -6, false);
  pkg::RoadRoi core(parameters(road, 25));
  const auto input = cloud({{10, 4, 1}, {20, 4, 1}, {20, -4, 1},
                            {26, 4, 1}, {-1, 0, 1}, {20, 0, 1},
                            {10, 8, 1}, {20, 8, 1}, {26, 8, 1}});
  const auto output = std::get<0>(core.process(input, {}, pose(5)));
  EXPECT_EQ((std::vector<float>{0, 1, 5, 6, 7}), indices(output));
}

TEST(RoadRoi, ClipsForwardDistanceAlongCurvedRoadInsteadOfVehicleXAxis) {
  RoadMap road;
  auto curved = road.lane(road.line({{0, 2}, {8, 2}, {8, 10}}),
                          road.line({{0, -2}, {12, -2}, {12, 10}}));
  curved.setCenterline(road.line({{0, 0}, {10, 0}, {10, 10}}));
  pkg::RoadRoi core(parameters(road, 15));
  const auto output = std::get<0>(core.process(
    cloud({{6, 0, 1}, {10, 4, 1}, {10, 6, 1}, {14, 4, 1}}), {}, pose()));
  EXPECT_EQ((std::vector<float>{0, 1}), indices(output));
}

TEST(RoadRoi, AppliesIndependentLateralInflationHeightBoundsAndVehicleExclusion) {
  RoadMap road;
  road.straight(0, 40, 2, -2);
  auto config = parameters(road, 20);
  config.left_inflation_m = 1.5;
  config.right_inflation_m = 0.5;
  config.z_min_m = 0.2;
  config.z_max_m = 2;
  pkg::RoadRoi core(config);
  const auto input = cloud({{10, 3.4f, 1}, {10, 3.6f, 1}, {10, -2.4f, 1},
                            {10, -2.6f, 1}, {10, 0, 0.1f}, {10, 0, 2.1f},
                            {1, 0, 1}, {1, 0, 1.5f}, {10, 0, 0.2f}, {10, 0, 2}});
  EXPECT_EQ((std::vector<float>{0, 2, 7, 8, 9}),
            indices(std::get<0>(core.process(input, {}, pose()))));
}

TEST(RoadRoi, UsesMapPoseAndPreservesPointRecordsAndPassThroughMessages) {
  RoadMap road;
  road.lane(road.line({{98, 200}, {98, 240}}), road.line({{102, 200}, {102, 240}}));
  pkg::RoadRoi core(parameters(road, 12));
  const auto input = cloud({{8, 0, 1}, {8, 4, 1}, {13, 0, 1}});
  const auto tf = pose(100, 205, 1.5707963267948966);
  interfaces::CompetitionVehicleStatus status;
  status.header.seq = 17;
  status.link_id = "unrelated_link";
  status.velocity_x_mps = 7;
  const auto result = core.process(input, status, tf);
  const auto& output = std::get<0>(result);
  ASSERT_EQ(1u, output.width);
  EXPECT_EQ(input.header, output.header);
  EXPECT_EQ(input.fields, output.fields);
  EXPECT_EQ(input.point_step, output.point_step);
  EXPECT_EQ(16u, output.row_step);
  EXPECT_EQ((std::vector<std::uint8_t>(input.data.begin(), input.data.begin() + 16)), output.data);
  EXPECT_EQ(status.header, std::get<1>(result).header);
  EXPECT_EQ(status.link_id, std::get<1>(result).link_id);
  EXPECT_EQ(status.velocity_x_mps, std::get<1>(result).velocity_x_mps);
  EXPECT_EQ(tf.transforms, std::get<2>(result).transforms);
  EXPECT_TRUE(std::get<0>(core.process(cloud({}), status, tf)).data.empty());
}

TEST(RoadRoi, ExcludesBodyBoundsAfterRollPitchRotationIntoLevelFrame) {
  RoadMap road;
  road.straight(0, 40, 2, -2);
  auto config = parameters(road, 20);
  config.z_max_m = 3;
  pkg::RoadRoi core(config);
  interfaces::CompetitionVehicleStatus status;
  status.roll_rad = 0.4f;
  status.pitch_rad = -0.1f;
  const auto level_point = [&](double x, double y, double z) {
    const double rolled_y = std::cos(status.roll_rad) * y - std::sin(status.roll_rad) * z;
    const double rolled_z = std::sin(status.roll_rad) * y + std::cos(status.roll_rad) * z;
    return std::array<float, 3>{{
      static_cast<float>(std::cos(status.pitch_rad) * x + std::sin(status.pitch_rad) * rolled_z),
      static_cast<float>(rolled_y),
      static_cast<float>(-std::sin(status.pitch_rad) * x + std::cos(status.pitch_rad) * rolled_z)}};
  };
  const auto inside_body = level_point(3.6, 0.8, 1.2);
  const auto outside_body = level_point(1, 1.2, 0.5);
  const auto input = cloud({inside_body, outside_body});
  const auto output = std::get<0>(core.process(input, status, pose()));
  EXPECT_EQ((std::vector<float>{1}), indices(output));
  EXPECT_EQ((std::vector<std::uint8_t>(input.data.begin() + 16, input.data.end())), output.data);
}

TEST(RoadRoi, IncludesCurrentLanePrefixReachedThroughSuccessorCycle) {
  RoadMap road;
  auto a = road.lane(road.line({{1, 1}, {19, 1}}), road.line({{-1, -1}, {21, -1}}));
  auto b = road.lane(road.line({{19, 1}, {19, 9}}), road.line({{21, -1}, {21, 11}}));
  auto c = road.lane(road.line({{19, 9}, {1, 9}}), road.line({{21, 11}, {-1, 11}}));
  auto d = road.lane(road.line({{1, 9}, {1, 1}}), road.line({{-1, 11}, {-1, -1}}));
  a.setCenterline(road.line({{0, 0}, {20, 0}}));
  b.setCenterline(road.line({{20, 0}, {20, 10}}));
  c.setCenterline(road.line({{20, 10}, {0, 10}}));
  d.setCenterline(road.line({{0, 10}, {0, 0}}));
  pkg::RoadRoi core(parameters(road, 55));
  const auto output = std::get<0>(core.process(
    cloud({{-8, 0, 1}, {-3, 0, 1}, {6, 0, 1}, {10, 5, 1}}), {}, pose(10)));
  EXPECT_EQ((std::vector<float>{0, 2, 3}), indices(output));
}
