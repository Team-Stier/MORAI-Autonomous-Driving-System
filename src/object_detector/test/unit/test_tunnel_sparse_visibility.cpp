#include <object_detector/cores/object_clustering.hpp>
#include <object_detector/parameters/object_clustering.hpp>
#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/serialization.h>
#include <fstream>
#include <iterator>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

TEST(TunnelSparseVisibility, KeepsSingleRingObstacleAboveRoadWithoutKeepingGround) {
  ros::Time::init();
  object_detector::Cloud points;
  for(int x=0;x<=60;++x)for(int y=-6;y<=6;++y)
    points.push_back(pcl::PointXYZ(4+x*.5f,y*.5f,-.4f));
  // The recorded stopped scene exposes a thin front stripe, not the full box.
  for(int i=0;i<12;++i)
    points.push_back(pcl::PointXYZ(19.87f,-.38f+i*.07f,-.19045f+i*.00031f));
  pcl::PointCloud<pcl::PointXYZI> timed;
  for(const auto& p:points) {
    pcl::PointXYZI q;q.x=p.x;q.y=p.y;q.z=p.z;q.intensity=-.05f;timed.push_back(q);
  }
  sensor_msgs::PointCloud2 cloud;pcl::toROSMsg(timed,cloud);
  for(auto& field:cloud.fields)if(field.name=="intensity")field.name="time";
  cloud.header.frame_id="base_link_level";cloud.header.stamp=ros::Time(100);
  object_detector::ObjectClustering old_core({});
  EXPECT_TRUE(std::get<0>(old_core.process(cloud,{},{})).empty());
  const auto config=object_detector::loadObjectClusteringParameters(node_pipeline::ParamReader::fromFile(
      std::string(NP_PACKAGE_DIR)+"/config/object_detector/parameters/object_clustering.yaml",NP_PACKAGE_DIR));
  object_detector::ObjectClustering core(config);
  const auto clusters=std::get<0>(core.process(cloud,{},{}));
  ASSERT_EQ(clusters.size(),1u);ASSERT_GE(clusters[0].size(),6u);
  for(const auto& point:clusters[0]) {
    EXPECT_GT(point.z,-.2);EXPECT_LT(point.z,-.18);
  }
}

// ROI output from 2026-10-29-17-49-50-gt.bag, scan 1793264053.3935804.
class TunnelApproachGroundSeeds : public ::testing::Test {
 protected:
  void SetUp() override {
    ros::Time::init();
    std::ifstream fixture(std::string(NP_PACKAGE_DIR) + "/test/data/tunnel_approach_ground_seed.bin",
                          std::ios::binary);
    ASSERT_TRUE(fixture.good());
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(fixture)), {});
    ASSERT_FALSE(bytes.empty());
    ros::serialization::IStream stream(bytes.data(), bytes.size());
    ros::serialization::deserialize(stream, cloud);
  }
  object_detector::ObjectClusteringParameters parameters() const {
    return object_detector::loadObjectClusteringParameters(node_pipeline::ParamReader::fromFile(
        std::string(NP_PACKAGE_DIR) + "/config/object_detector/parameters/object_clustering.yaml",
        NP_PACKAGE_DIR));
  }
  sensor_msgs::PointCloud2 cloud;
};

TEST_F(TunnelApproachGroundSeeds, PreservesRecordedElevatedSingleRing) {
  object_detector::ObjectClustering core(parameters());
  const auto clusters = std::get<0>(core.process(cloud, {}, {}));
  ASSERT_EQ(clusters.size(), 1u);
  ASSERT_EQ(clusters[0].size(), 7u);
  for (const auto& point : clusters[0]) {
    EXPECT_GT(point.x, 31.0); EXPECT_LT(point.x, 31.6);
    EXPECT_GT(point.z, -.18); EXPECT_LT(point.z, -.15);
  }
}

TEST_F(TunnelApproachGroundSeeds, StillRemovesRecordedRoadWithoutTheObstacle) {
  std::uint32_t retained = 0;
  for (std::uint32_t i = 0; i < cloud.width; ++i) {
    const auto* record = cloud.data.data() + i * cloud.point_step;
    float xyz[3];
    std::memcpy(xyz, record, sizeof(xyz));
    if (xyz[0] > 31.0 && xyz[0] < 31.6 && std::abs(xyz[1]) < .6 &&
        xyz[2] > -.24 && xyz[2] < -.1) continue;
    std::memmove(cloud.data.data() + retained * cloud.point_step, record, cloud.point_step);
    ++retained;
  }
  cloud.width = retained; cloud.row_step = retained * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  object_detector::ObjectClustering core(parameters());
  EXPECT_TRUE(std::get<0>(core.process(cloud, {}, {})).empty());
}

TEST(TunnelSparseVisibility, RemovesFlatAndGradedRoadWithProductionParameters) {
  ros::Time::init();
  const auto config = object_detector::loadObjectClusteringParameters(node_pipeline::ParamReader::fromFile(
      std::string(NP_PACKAGE_DIR) + "/config/object_detector/parameters/object_clustering.yaml",
      NP_PACKAGE_DIR));
  for (const auto slope : {std::pair<double, double>{0, 0}, {.03, 0}, {-.03, 0},
                            {0, .03}, {.08, 0}, {-.08, 0}}) {
    SCOPED_TRACE(::testing::Message() << "slope=" << slope.first << "," << slope.second);
    pcl::PointCloud<pcl::PointXYZI> points;
    for (int x = 0; x <= 60; ++x) for (int y = -6; y <= 6; ++y) {
      pcl::PointXYZI point;
      point.x = 4 + x * .5f; point.y = y * .5f;
      point.z = -.4 + slope.first * x * .5 + slope.second * point.y;
      point.intensity = -.05f; points.push_back(point);
    }
    sensor_msgs::PointCloud2 cloud; pcl::toROSMsg(points, cloud);
    for (auto& field : cloud.fields) if (field.name == "intensity") field.name = "time";
    cloud.header.stamp = ros::Time(100); cloud.header.frame_id = "base_link_level";
    object_detector::ObjectClustering core(config);
    EXPECT_TRUE(std::get<0>(core.process(cloud, {}, {})).empty());
  }
}
