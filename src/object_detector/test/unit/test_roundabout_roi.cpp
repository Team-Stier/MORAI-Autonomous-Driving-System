#include <object_detector/parameters/roundabout_roi.hpp>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace od = object_detector;
namespace {
lanelet::LaneletMapPtr road() {
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id id = 1;
  const auto lane = [&](double x0, double x1, double y0, double y1) {
    const lanelet::Point3d a(id++, x0, y1), b(id++, x1, y1);
    const lanelet::Point3d c(id++, x0, y0), d(id++, x1, y0);
    lanelet::LineString3d left(id++, {a, b});
    lanelet::LineString3d right(id++, {c, d});
    lanelet::Lanelet value(id++, left, right);
    value.attributes()["subtype"] = "road";
    value.attributes()["location"] = "urban";
    value.attributes()["participant:vehicle"] = "yes";
    map->add(value);
  };
  lane(-10, 60, -2, 2);
  lane(-10, 60, 2, 6);
  lane(-20, -10, 10, 14);
  return map;
}

sensor_msgs::PointCloud2 cloud(const std::vector<std::array<float, 3>>& xyz) {
  sensor_msgs::PointCloud2 value;
  value.header.frame_id = "base_link_level";
  value.header.stamp = ros::Time(10);
  value.height = 1;
  value.width = xyz.size();
  value.point_step = 16;
  value.row_step = value.width * value.point_step;
  value.data.resize(value.row_step);
  const char* names[] = {"x", "y", "z", "intensity"};
  for (unsigned i = 0; i < 4; ++i) {
    sensor_msgs::PointField field;
    field.name = names[i]; field.offset = 4*i;
    field.datatype = sensor_msgs::PointField::FLOAT32; field.count = 1;
    value.fields.push_back(field);
  }
  for (std::size_t i = 0; i < xyz.size(); ++i) {
    std::memcpy(value.data.data() + 16*i, xyz[i].data(), 12);
    const float label = i;
    std::memcpy(value.data.data() + 16*i + 12, &label, 4);
  }
  return value;
}

std::vector<float> labels(const sensor_msgs::PointCloud2& value) {
  std::vector<float> result;
  for (unsigned i = 0; i < value.width; ++i) {
    float label;
    std::memcpy(&label, value.data.data() + i*value.point_step + 12, 4);
    result.push_back(label);
  }
  return result;
}

tf2_msgs::TFMessage pose(double x = 0, double y = 0, double yaw = 0) {
  tf2_msgs::TFMessage result;
  result.transforms.resize(1);
  auto& t = result.transforms.front();
  t.header.frame_id = "map"; t.header.stamp = ros::Time(10);
  t.child_frame_id = "base_link_level";
  t.transform.translation.x = x; t.transform.translation.y = y;
  t.transform.rotation.z = std::sin(yaw/2); t.transform.rotation.w = std::cos(yaw/2);
  return result;
}

od::RoundaboutRoiParameters parameters() {
  od::RoundaboutRoiParameters p;
  p.map = road();
  return p;
}
}  // namespace

TEST(RoundaboutRoi, RejectsRoadsideRailAndKeepsVehiclePointsAndCrossTraffic) {
  od::RoundaboutRoi core(parameters());
  const auto input = cloud({{10, -2.4f, 0.9f}, {10, -2.8f, 0.4f},
                            {10, -1.5f, 0.4f}, {11, -1.4f, 1.2f},
                            {-15, 12, 0.8f}, {10, 2, 0.8f}, {10, 6.5f, 0.8f}});
  EXPECT_EQ((std::vector<float>{2, 3, 4, 5}), labels(std::get<0>(core.process(input, {}, pose()))));
}

TEST(RoundaboutRoi, RejectsOuterRoadEdgeReturnsAndPreservesSharedLaneBoundary) {
  od::RoundaboutRoi core(parameters());
  const auto input = cloud({{10, -1.8f, 0.9f}, {10, 2, 0.9f}, {10, 5.8f, 0.9f}});
  EXPECT_EQ((std::vector<float>{1}), labels(std::get<0>(core.process(input, {}, pose()))));
}

TEST(RoundaboutRoi, UsesRoadBeyondSensingRangeToCheckFootprintAtRangeBoundary) {
  auto p = parameters();
  p.map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Id id = 100;
  for (const auto& extent : std::vector<std::pair<double,double>>{{-10,50.1},{50.1,60}}) {
    const lanelet::Point3d a(id++, extent.first, 2), b(id++, extent.second, 2);
    const lanelet::Point3d c(id++, extent.first, -2), d(id++, extent.second, -2);
    lanelet::LineString3d left(id++, {a,b}), right(id++, {c,d});
    lanelet::Lanelet lane(id++, left, right);
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["participant:vehicle"] = "yes";
    p.map->add(lane);
  }
  od::RoundaboutRoi core(p);
  const auto result = std::get<0>(core.process(cloud({{49.9f,0,1},{50.1f,0,1}}), {}, pose()));
  EXPECT_EQ((std::vector<float>{0}), labels(result));
}

TEST(RoundaboutRoi, UsesMapPoseAndPreservesOrganizedPaddedPointRecordsAndMessages) {
  od::RoundaboutRoi core(parameters());
  auto input = cloud({{10, 0, 1}, {10, -2.5f, 1}, {12, 0, 1}, {12, 7, 1}});
  const auto bytes = input.data;
  input.height = input.width = 2; input.row_step = 36;
  input.data.resize(72, 0x7a);
  std::memcpy(input.data.data()+36, bytes.data()+32, 32);
  interfaces::CompetitionVehicleStatus status;
  status.link_id = "kept"; status.header.seq = 7;
  const auto result = core.process(input, status, pose());
  EXPECT_EQ((std::vector<float>{0, 2}), labels(std::get<0>(result)));
  EXPECT_EQ(1u, std::get<0>(result).height);
  EXPECT_EQ(32u, std::get<0>(result).row_step);
  EXPECT_EQ(input.header, std::get<0>(result).header);
  EXPECT_EQ(input.fields, std::get<0>(result).fields);
  EXPECT_EQ(status, std::get<1>(result));
  EXPECT_EQ(pose().transforms, std::get<2>(result).transforms);
  const double half_pi = 1.5707963267948966;
  EXPECT_EQ((std::vector<float>{0}), labels(std::get<0>(core.process(
      cloud({{1, -8, 1}, {1, -8, 2.1f}}), {}, pose(2, 0, half_pi)))));
  EXPECT_TRUE(std::get<0>(core.process(cloud({}), {}, pose())).data.empty());
}

TEST(RoundaboutRoi, AppliesInclusiveRangeHeightBoundsAndTiltedEgoExclusion) {
  auto p = parameters(); p.range_m = 20;
  od::RoundaboutRoi core(p);
  auto result = std::get<0>(core.process(cloud({{20, 0, .2f}, {20, 0, 2},
      {20.01f, 0, 1}, {19, 0, .19f}, {1, 0, 1}, {1, 1.2f, .5f}}), {}, pose()));
  EXPECT_EQ((std::vector<float>{0, 1, 5}), labels(result));
  interfaces::CompetitionVehicleStatus status;
  status.roll_rad = .4f; status.pitch_rad = -.1f;
  const double cr = std::cos(status.roll_rad), sr = std::sin(status.roll_rad);
  const double cp = std::cos(status.pitch_rad), sp = std::sin(status.pitch_rad);
  const auto level = [&](double x, double y, double z) {
    const double yy = cr*y-sr*z, zz = sr*y+cr*z;
    return std::array<float,3>{{float(cp*x+sp*zz), float(yy), float(-sp*x+cp*zz)}};
  };
  result = std::get<0>(core.process(cloud({level(3.6,.8,1.2), level(1,1.2,.5)}), status, pose()));
  EXPECT_EQ((std::vector<float>{1}), labels(result));
}

TEST(RoundaboutRoiParameters, LoadsNativeMapAndRejectsInvalidConfiguration) {
  const std::string package = NP_PACKAGE_DIR;
  const auto p = od::loadRoundaboutRoiParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/object_detector/parameters/roundabout_roi.yaml", package));
  ASSERT_TRUE(p.map);
  EXPECT_EQ(50, p.range_m);
  EXPECT_DOUBLE_EQ(0.5, p.boundary_margin_m);
  EXPECT_FALSE(p.map->laneletLayer.empty());
  EXPECT_NO_THROW(od::RoundaboutRoi{p});
  const std::string valid = "map_file: ../map/kcity_lanelet2.osm\nrange_m: 50\n"
      "boundary_margin_m: 0.35\nz_min_m: 0.2\nz_max_m: 2\n";
  for (const auto& change : std::vector<std::pair<std::string, std::string>>{
      {"range_m: 50", "range_m: 0"}, {"range_m: 50", "range_m: .inf"},
      {"range_m: 50", "range_m: .nan"}, {"boundary_margin_m: 0.35", "boundary_margin_m: -1"},
      {"boundary_margin_m: 0.35", "boundary_margin_m: .inf"},
      {"z_min_m: 0.2", "z_min_m: .nan"}, {"z_max_m: 2", "z_max_m: .inf"},
      {"z_min_m: 0.2", "z_min_m: 3"}}) {
    auto invalid = valid;
    invalid.replace(invalid.find(change.first), change.first.size(), change.second);
    SCOPED_TRACE(invalid);
    EXPECT_THROW(od::loadRoundaboutRoiParameters(node_pipeline::ParamReader::fromString(
        invalid, package)), std::invalid_argument);
  }
}
