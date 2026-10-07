#include <object_detector/cores/cluster_match.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>
#include <cmath>

namespace pkg = object_detector;
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

sensor_msgs::PointCloud2 message(const Cloud& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 17;
  cloud.header.stamp.fromNSec(1234567890);
  cloud.header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& point : points) {
    *x = point.x; *y = point.y; *z = point.z;
    ++x; ++y; ++z;
  }
  return cloud;
}

void addBox(Cloud& points, float x_min, float y_min, float z_min,
            int x_steps, int y_steps, int z_steps, float step) {
  for (int x = 0; x <= x_steps; ++x)
    for (int y = 0; y <= y_steps; ++y)
      for (int z = 0; z <= z_steps; ++z)
        if (x == 0 || x == x_steps || y == 0 || y == y_steps || z == 0 || z == z_steps)
          points.emplace_back(x_min + step * x, y_min + step * y, z_min + step * z);
}

TEST(ClusterMatch, SeparatesClustersAndReturnsUnknownGeometryWithoutMotion) {
  Cloud points;
  addBox(points, 5, -0.2, 0, 10, 4, 5, 0.1);
  addBox(points, 10, -0.2, 0, 10, 4, 5, 0.1);
  pkg::ClusterMatch core({}, {});
  const auto output = std::get<0>(core.process(message(points), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(2u, output.objects.size());
  for (const auto& object : output.objects) {
    EXPECT_EQ("unknown", object.label);
    EXPECT_EQ(0u, object.id);
    EXPECT_FALSE(object.velocity_available);
    EXPECT_FALSE(object.acceleration_available);
    EXPECT_DOUBLE_EQ(0, object.velocity.x);
    EXPECT_DOUBLE_EQ(0, object.velocity.y);
    EXPECT_DOUBLE_EQ(0, object.velocity.z);
    EXPECT_DOUBLE_EQ(0, object.acceleration.x);
    EXPECT_DOUBLE_EQ(0, object.acceleration.y);
    EXPECT_DOUBLE_EQ(0, object.acceleration.z);
    EXPECT_NEAR(1.0, object.bbox_size.x, 1e-5);
    EXPECT_NEAR(0.4, object.bbox_size.y, 1e-5);
    EXPECT_NEAR(0.5, object.bbox_size.z, 1e-5);
    EXPECT_DOUBLE_EQ(object.position.x, object.bbox_pose.position.x);
    EXPECT_DOUBLE_EQ(object.position.y, object.bbox_pose.position.y);
    EXPECT_DOUBLE_EQ(object.position.z, object.bbox_pose.position.z);
    const auto& q = object.bbox_pose.orientation;
    EXPECT_NEAR(1, q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1e-6);
  }
  EXPECT_TRUE(output.header.frame_id.empty());
  EXPECT_TRUE(output.header.stamp.isZero());
}

TEST(ClusterMatch, RemovesRoadPlaneWhileKeepingLowObstacle) {
  Cloud points;
  for (int x = 0; x <= 50; ++x)
    for (int y = -20; y <= 20; ++y)
      points.emplace_back(4 + x * 0.1f, y * 0.1f, -0.38f);
  addBox(points, 6, -0.2, -0.36, 10, 10, 5, 0.04);
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.02;
  pkg::ClusterMatch core(parameters, {});
  const auto output = std::get<0>(core.process(message(points), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_LT(output.objects.front().bbox_size.x, 0.6);
  EXPECT_LT(output.objects.front().bbox_size.y, 0.6);
  EXPECT_GT(output.objects.front().bbox_size.z, 0.08);
  EXPECT_LT(output.objects.front().position.z, 0);
}

TEST(ClusterMatch, EmptyCloudReturnsNoObjects) {
  pkg::ClusterMatch core({}, {});
  EXPECT_TRUE(std::get<0>(core.process(message({}), interfaces::CompetitionVehicleStatus{})).objects.empty());
}

TEST(ClusterMatch, RegionalRoadHeightDoesNotMergeIntoAnObstacle) {
  Cloud points;
  for (int x = -40; x <= 80; ++x)
    for (int y = -60; y <= 60; ++y) {
      const float px = 0.5f * x, py = 0.5f * y;
      if (px < 12 || px > 28 || std::abs(py) > 8)
        points.emplace_back(px, py, -0.48f);
    }
  for (int x = 0; x <= 80; ++x)
    for (int y = -40; y <= 40; ++y)
      points.emplace_back(12 + 0.2f * x, 0.2f * y, -0.36f);
  addBox(points, 20, 0, -0.28f, 6, 6, 6, 0.04f);
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.02;
  pkg::ClusterMatch core(parameters, {});
  const auto output = std::get<0>(core.process(message(points), interfaces::CompetitionVehicleStatus{}));
  const auto obstacle = std::find_if(output.objects.begin(), output.objects.end(),
    [](const interfaces::Object& object) {
      return object.position.z + object.bbox_size.z * 0.5 > -0.1;
    });
  ASSERT_NE(output.objects.end(), obstacle);
  EXPECT_LT(obstacle->bbox_size.x, 1.0);
  EXPECT_LT(obstacle->bbox_size.y, 1.0);
}

pkg::ModelTemplate asymmetricModel() {
  pkg::ModelTemplate model;
  model.label = "fixture_asymmetric";
  for (int latitude = 1; latitude < 20; ++latitude) {
    const double theta = latitude * 3.141592653589793 / 20;
    for (int longitude = 0; longitude < 48; ++longitude) {
      const double phi = longitude * 2 * 3.141592653589793 / 48;
      const double bulge = 1 + 0.18 * std::sin(theta) * std::cos(3 * phi);
      model.points.emplace_back(0.9 * std::sin(theta) * std::cos(phi) * bulge,
        0.35 * std::sin(theta) * std::sin(phi),
        0.45 * std::cos(theta) + 0.08 * std::sin(theta) * std::cos(phi));
    }
  }
  model.collider_available = true;
  model.collider_pose.position.x = 0.2;
  model.collider_pose.position.y = -0.1;
  model.collider_pose.position.z = 0.15;
  model.collider_pose.orientation.z = std::sin(0.25 / 2);
  model.collider_pose.orientation.w = std::cos(0.25 / 2);
  model.collider_size.x = 1.8;
  model.collider_size.y = 0.7;
  model.collider_size.z = 0.8;
  return model;
}

Cloud partialObservation(const pkg::ModelTemplate& model) {
  Cloud points;
  for (const auto& point : model.points)
    if (point.x < 0.15)
      points.emplace_back(8 + std::cos(0.6) * point.x - std::sin(0.6) * point.y,
        3 + std::sin(0.6) * point.x + std::cos(0.6) * point.y, 0.6 + point.z);
  return points;
}

pkg::ClusterMatchParameters matchingParameters() {
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.03;
  parameters.normal_radius_m = 0.18;
  parameters.feature_radius_m = 0.3;
  parameters.registration_iterations = 3000;
  parameters.correspondence_distance_m = 0.1;
  parameters.min_inlier_fraction = 0.8;
  parameters.max_rmse_m = 0.025;
  return parameters;
}

TEST(ClusterMatch, MatchesPartialSurfaceAndTransformsColliderOffsetAndOrientation) {
  const auto model = asymmetricModel();
  pkg::ClusterMatch core(matchingParameters(), {model});
  const auto output = std::get<0>(core.process(message(partialObservation(model)), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  ASSERT_EQ("fixture_asymmetric", object.label);
  EXPECT_NEAR(8.2215313703, object.position.x, 0.025);
  EXPECT_NEAR(3.030394932, object.position.y, 0.025);
  EXPECT_NEAR(0.75, object.position.z, 0.025);
  EXPECT_NEAR(std::sin(0.85 / 2), object.bbox_pose.orientation.z, 0.025);
  EXPECT_NEAR(std::cos(0.85 / 2), object.bbox_pose.orientation.w, 0.025);
  EXPECT_DOUBLE_EQ(1.8, object.bbox_size.x);
  EXPECT_DOUBLE_EQ(0.7, object.bbox_size.y);
  EXPECT_DOUBLE_EQ(0.8, object.bbox_size.z);
  EXPECT_EQ(0u, object.id);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
}

TEST(ClusterMatch, AmbiguousModelsKeepUnknownObservedBox) {
  const auto model = asymmetricModel();
  auto duplicate = model;
  duplicate.label = "fixture_duplicate";
  pkg::ClusterMatch core(matchingParameters(), {model, duplicate});
  const auto output = std::get<0>(core.process(message(partialObservation(model)), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("unknown", output.objects.front().label);
  EXPECT_LT(output.objects.front().bbox_size.x, 1.5);
}

TEST(ClusterMatch, ClassifiedModelWithUnverifiedColliderKeepsObservedBox) {
  auto model = asymmetricModel();
  model.collider_available = false;
  pkg::ClusterMatch core(matchingParameters(), {model});
  const auto output = std::get<0>(core.process(message(partialObservation(model)), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("fixture_asymmetric", output.objects.front().label);
  EXPECT_LT(output.objects.front().bbox_size.x, 1.5);
}

TEST(ClusterMatch, DenseLowObstacleTopIsNotSelectedAsRoad) {
  Cloud points;
  for (int x = 0; x <= 50; ++x)
    for (int y = -20; y <= 20; ++y) {
      const float px = 4 + x * 0.1f, py = y * 0.1f;
      if (!(px >= 6 && px <= 7 && std::abs(py) < 0.25))
        points.emplace_back(px, py, -0.38f);
    }
  for (int x = 0; x <= 100; ++x)
    for (int y = 0; y <= 50; ++y) {
      points.emplace_back(6 + x * 0.01f, -0.25f + y * 0.01f, -0.26f);
      if (x == 0 || x == 100 || y == 0 || y == 50)
        for (int z = 1; z < 12; ++z)
          points.emplace_back(6 + x * 0.01f, -0.25f + y * 0.01f, -0.38f + z * 0.01f);
    }
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.005;
  pkg::ClusterMatch core(parameters, {});
  const auto output = std::get<0>(core.process(message(points), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  EXPECT_LT(object.bbox_size.x, 1.2);
  EXPECT_LT(object.bbox_size.y, 0.7);
  EXPECT_GT(object.position.z + object.bbox_size.z * 0.5, -0.28);
}

TEST(ClusterMatch, NearPlanarPartialSupportDoesNotInventFullColliderPose) {
  auto model = asymmetricModel();
  for (auto& point : model.points) point.z *= 0.02;
  auto parameters = matchingParameters();
  parameters.model_max_tilt_rad = 3.141592653589793;
  pkg::ClusterMatch core(parameters, {model});
  const auto output = std::get<0>(core.process(message(partialObservation(model)), interfaces::CompetitionVehicleStatus{}));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("unknown", output.objects.front().label);
  EXPECT_LT(output.objects.front().bbox_size.z, 0.02);
}

TEST(ClusterMatch, ReturnsTheNativeStatusForTheNextCoreWithoutChangingIt) {
  pkg::ClusterMatch core({}, {});
  interfaces::CompetitionVehicleStatus status;
  status.header.seq = 43;
  status.header.stamp.fromNSec(1234000000);
  status.header.frame_id = "base_link";
  status.velocity_x_mps = 7;
  status.yaw_rad = 0.25;
  const auto result = core.process(message({}), status);
  EXPECT_TRUE(std::get<0>(result).objects.empty());
  EXPECT_EQ(status.header, std::get<1>(result).header);
  EXPECT_FLOAT_EQ(status.velocity_x_mps, std::get<1>(result).velocity_x_mps);
  EXPECT_FLOAT_EQ(status.yaw_rad, std::get<1>(result).yaw_rad);
}
