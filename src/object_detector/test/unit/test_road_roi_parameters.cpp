#include <object_detector/parameters/road_roi.hpp>
#include <object_detector/validators/road_roi_input.hpp>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace od = object_detector;
namespace {
const std::string package = NP_PACKAGE_DIR;
const std::string valid_parameters =
    "map_file: ../map/kcity_lanelet2.osm\nforward_distance_m: 50\n"
    "left_inflation_m: 1\nright_inflation_m: 1\nz_min_m: 0.2\nz_max_m: 2\n";

tf2_msgs::TFMessage validTf() {
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(1);
  tf.transforms.front().header.frame_id = "map";
  tf.transforms.front().child_frame_id = "base_link_level";
  tf.transforms.front().transform.rotation.w = 1;
  return tf;
}
}  // namespace

TEST(RoadRoiParameters, LoadsConfiguredMapInLocalCoordinates) {
  const auto config = od::loadRoadRoiParameters(node_pipeline::ParamReader::fromFile(
      package + "/config/object_detector/parameters/road_roi.yaml", package));
  ASSERT_TRUE(config.map);
  EXPECT_FALSE(config.map->laneletLayer.empty());
  EXPECT_DOUBLE_EQ(50, config.forward_distance_m);
  const auto& point = *config.map->pointLayer.begin();
  EXPECT_DOUBLE_EQ(point.attribute("local_x").asDouble().value(), point.x());
  EXPECT_DOUBLE_EQ(point.attribute("local_y").asDouble().value(), point.y());
  EXPECT_NO_THROW(od::RoadRoi{config});
}

TEST(RoadRoiParameters, RejectsNonfiniteInvalidAndReversedBounds) {
  for (const auto& replacement : std::vector<std::pair<std::string, std::string>>{
           {"forward_distance_m: 50", "forward_distance_m: 0"},
           {"forward_distance_m: 50", "forward_distance_m: .inf"},
           {"left_inflation_m: 1", "left_inflation_m: -1"},
           {"right_inflation_m: 1", "right_inflation_m: .nan"},
           {"z_min_m: 0.2", "z_min_m: .nan"},
           {"z_max_m: 2", "z_max_m: .inf"},
           {"z_min_m: 0.2", "z_min_m: 3"}}) {
    auto yaml = valid_parameters;
    yaml.replace(yaml.find(replacement.first), replacement.first.size(), replacement.second);
    SCOPED_TRACE(yaml);
    EXPECT_THROW(od::loadRoadRoiParameters(node_pipeline::ParamReader::fromString(yaml, package)),
                 std::invalid_argument);
  }
}

TEST(RoadRoiInput, RejectsMissingCloudFields) {
  sensor_msgs::PointCloud2 cloud;
  cloud.height = 1;
  cloud.point_step = 12;
  std::vector<node_pipeline::Kind> issues;
  od::RoadRoiInput{}(cloud, {}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
}

TEST(RoadRoiInput, AcceptsRequiredTransformAndRejectsWrongFrameNonunitOrMissing) {
  std::vector<node_pipeline::Kind> issues;
  auto tf = validTf();
  od::RoadRoiInput{}(tf, {}, issues);
  EXPECT_TRUE(issues.empty());
  tf.transforms.front().header.frame_id = "odom";
  od::RoadRoiInput{}(tf, {}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::FrameMismatch}), issues);
  issues.clear();
  tf = validTf();
  tf.transforms.front().transform.rotation.w = 2;
  od::RoadRoiInput{}(tf, {}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
  issues.clear();
  od::RoadRoiInput{}(tf2_msgs::TFMessage{}, {}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Missing}), issues);
}

TEST(RoadRoiInput, RejectsNonfiniteVehicleAttitude) {
  interfaces::CompetitionVehicleStatus competition;
  std::vector<node_pipeline::Kind> issues;
  od::RoadRoiInput{}(competition, {}, issues);
  EXPECT_TRUE(issues.empty());
  competition.roll_rad = std::numeric_limits<float>::quiet_NaN();
  od::RoadRoiInput{}(competition, {}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::NaN}), issues);
  issues.clear();
  competition.roll_rad = 0;
  competition.pitch_rad = std::numeric_limits<float>::infinity();
  od::RoadRoiInput{}(competition, {}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::Inf}), issues);
}
