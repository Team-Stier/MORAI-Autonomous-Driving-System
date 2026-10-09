// Synthetic acceptance tests for the DBSCAN calculation core.
#include <object_detector/cores/dbscan.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace {
using Center = std::array<float, 3>;

sensor_msgs::PointCloud2 makeCloud(const std::vector<Center>& centers) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.fromSec(12.0);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(27 * centers.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& center : centers) {
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <= 1; ++iy) {
        for (int iz = -1; iz <= 1; ++iz, ++x, ++y, ++z) {
          *x = center[0] + 0.1f * ix;
          *y = center[1] + 0.1f * iy;
          *z = center[2] + 0.1f * iz;
        }
      }
    }
  }
  return cloud;
}

sensor_msgs::PointCloud2 makePoints(const std::vector<Center>& points) {
  auto cloud = makeCloud({});
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& point : points) {
    *x = point[0];
    *y = point[1];
    *z = point[2];
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

object_detector::DbscanParameters testParameters() {
  object_detector::DbscanParameters p;
  // Synthetic test settings, not tuned competition defaults.
  p.roi_x_min = 0.0;
  p.roi_x_max = 20.0;
  p.roi_y_min = -5.0;
  p.roi_y_max = 5.0;
  p.roi_z_min = 0.2;
  p.roi_z_max = 2.0;
  p.voxel_leaf_size = 0.04;
  p.dbscan_eps = 0.18;
  p.dbscan_min_points = 4;  // Includes the query point itself.
  p.min_cluster_size = 5;
  p.max_cluster_size = 1000;
  return p;
}

void expectBox(const interfaces::Object& object, const Center& center) {
  EXPECT_NEAR(center[0], object.bbox_pose.position.x, 1e-4);
  EXPECT_NEAR(center[1], object.bbox_pose.position.y, 1e-4);
  EXPECT_NEAR(center[2], object.bbox_pose.position.z, 1e-4);
  EXPECT_NEAR(0.2, object.bbox_size.x, 1e-4);
  EXPECT_NEAR(0.2, object.bbox_size.y, 1e-4);
  EXPECT_NEAR(0.2, object.bbox_size.z, 1e-4);
  EXPECT_NEAR(object.bbox_pose.position.x, object.position.x, 1e-6);
  EXPECT_NEAR(object.bbox_pose.position.y, object.position.y, 1e-6);
  EXPECT_NEAR(object.bbox_pose.position.z, object.position.z, 1e-6);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
  EXPECT_EQ("unknown", object.label);
  EXPECT_DOUBLE_EQ(0.0, object.bbox_pose.orientation.x);
  EXPECT_DOUBLE_EQ(0.0, object.bbox_pose.orientation.y);
  // Float extrema of this nominal square can make either axis the longer one.
  // Preserve its geometry assertions and require a canonical axis-aligned OBB.
  const auto& q = object.bbox_pose.orientation;
  const double yaw = 2.0 * std::atan2(q.z, q.w);
  EXPECT_NEAR(1.0, q.z*q.z + q.w*q.w, 1e-12);
  EXPECT_TRUE(std::abs(yaw) < 1e-12 || std::abs(yaw + std::acos(-1.0)/2.0) < 1e-12);
  EXPECT_GE(object.bbox_size.x, object.bbox_size.y);
}
}  // namespace

TEST(Dbscan, KeepsOneObjectAsOneCluster) {
  const Center center{{4.0f, 0.0f, 0.6f}};
  object_detector::Dbscan core(testParameters());
  const auto result = core.process(makeCloud({center}));
  ASSERT_EQ(1u, result.objects.size());
  expectBox(result.objects.front(), center);
  EXPECT_TRUE(result.header.stamp.isZero());
  EXPECT_TRUE(result.header.frame_id.empty());
}

TEST(Dbscan, SeparatesThreeDistantObjects) {
  const std::vector<Center> centers{
    {{4.0f, -2.0f, 0.6f}}, {{7.0f, 0.0f, 0.6f}}, {{10.0f, 2.0f, 0.6f}}};
  object_detector::Dbscan core(testParameters());
  auto result = core.process(makeCloud(centers));
  ASSERT_EQ(3u, result.objects.size());
  // Cluster enumeration order is not an object identity contract.
  std::sort(result.objects.begin(), result.objects.end(),
    [](const interfaces::Object& a, const interfaces::Object& b) {
      return a.position.x < b.position.x;
    });
  for (std::size_t i = 0; i < centers.size(); ++i) expectBox(result.objects[i], centers[i]);
}

TEST(Dbscan, ExcludesObjectsOutsideRoi) {
  object_detector::Dbscan core(testParameters());
  const Center inside{{4.0f, 0.0f, 0.6f}};
  const auto result = core.process(makeCloud({inside,
    {{-1.0f, 0.0f, 0.6f}}, {{21.0f, 0.0f, 0.6f}},
    {{4.0f, -6.0f, 0.6f}}, {{4.0f, 6.0f, 0.6f}},
    {{4.0f, 0.0f, 0.0f}}, {{4.0f, 0.0f, 3.0f}}}));
  ASSERT_EQ(1u, result.objects.size());
  expectBox(result.objects.front(), inside);
}

TEST(Dbscan, LeavesIsolatedPointsAsNoise) {
  auto parameters = testParameters();
  parameters.dbscan_eps = 0.05;
  parameters.min_cluster_size = 1;
  object_detector::Dbscan core(parameters);
  EXPECT_TRUE(core.process(makeCloud({{{4.0f, 0.0f, 0.6f}}})).objects.empty());
}

TEST(Dbscan, CountsTheQueryPointInMinPoints) {
  auto parameters = testParameters();
  parameters.dbscan_min_points = 1;
  parameters.min_cluster_size = 1;
  const auto cloud = makePoints({{{4.0f, 0.0f, 0.6f}}});
  object_detector::Dbscan one_point_core(parameters);
  ASSERT_EQ(1u, one_point_core.process(cloud).objects.size());
  parameters.dbscan_min_points = 2;
  object_detector::Dbscan two_point_core(parameters);
  EXPECT_TRUE(two_point_core.process(cloud).objects.empty());
}

TEST(Dbscan, IncludesEarlierNoiseAsBorderWithoutDuplicates) {
  auto parameters = testParameters();
  parameters.dbscan_eps = 0.16;
  parameters.dbscan_min_points = 3;
  parameters.min_cluster_size = 4;
  parameters.max_cluster_size = 4;
  object_detector::Dbscan core(parameters);
  // The leftmost point has only two neighbors including itself; the next is core.
  const auto result = core.process(makePoints({{{1.0f, 0.0f, 0.6f}},
    {{1.15f, 0.0f, 0.6f}}, {{1.20f, 0.0f, 0.6f}}, {{1.25f, 0.0f, 0.6f}}}));
  ASSERT_EQ(1u, result.objects.size());
  EXPECT_NEAR(1.125, result.objects.front().position.x, 1e-6);
  EXPECT_NEAR(0.25, result.objects.front().bbox_size.x, 1e-6);
}

TEST(Dbscan, AppliesInclusiveClusterSizeLimits) {
  const auto cloud = makeCloud({{{4.0f, 0.0f, 0.6f}}});
  auto parameters = testParameters();
  parameters.min_cluster_size = 27;
  parameters.max_cluster_size = 27;
  object_detector::Dbscan exact_size(parameters);
  EXPECT_EQ(1u, exact_size.process(cloud).objects.size());
  parameters.min_cluster_size = 28;
  parameters.max_cluster_size = 100;
  object_detector::Dbscan too_small(parameters);
  EXPECT_TRUE(too_small.process(cloud).objects.empty());
  parameters.min_cluster_size = 1;
  parameters.max_cluster_size = 26;
  object_detector::Dbscan too_large(parameters);
  EXPECT_TRUE(too_large.process(cloud).objects.empty());
}

namespace {
constexpr double kPi = 3.14159265358979323846;

// Six faces of a 4 x 2 x 1.5 m cuboid, including all extrema.
std::vector<Center> vehicleSurface(double yaw, float cx = 5.0f, float cy = 0.0f) {
  std::vector<Center> points;
  for (int ix = 0; ix <= 8; ++ix) {
    for (int iy = 0; iy <= 4; ++iy) {
      for (int iz = 0; iz <= 2; ++iz) {
        if (ix != 0 && ix != 8 && iy != 0 && iy != 4 && iz != 0 && iz != 2) continue;
        const double x = -2.0 + 0.5 * ix, y = -1.0 + 0.5 * iy;
        points.push_back({{static_cast<float>(cx + std::cos(yaw) * x - std::sin(yaw) * y),
                          static_cast<float>(cy + std::sin(yaw) * x + std::cos(yaw) * y),
                          static_cast<float>(0.25 + 0.75 * iz)}});
      }
    }
  }
  return points;
}

interfaces::Objects fitPoints(const std::vector<Center>& points) {
  auto p = testParameters();
  p.roi_x_min = -30.0;
  p.roi_x_max = 30.0;
  p.roi_y_min = -30.0;
  p.roi_y_max = 30.0;
  p.roi_z_min = -30.0;
  p.roi_z_max = 30.0;
  p.voxel_leaf_size = 0.02;
  p.dbscan_eps = 1.1;
  p.dbscan_min_points = 1;
  p.min_cluster_size = 1;
  return object_detector::Dbscan(p).process(makePoints(points));
}

double boxYaw(const interfaces::Object& box) {
  return 2.0 * std::atan2(box.bbox_pose.orientation.z, box.bbox_pose.orientation.w);
}

void expectSemantics(const interfaces::Object& box) {
  EXPECT_EQ(box.position, box.bbox_pose.position);
  EXPECT_EQ("unknown", box.label);
  EXPECT_FALSE(box.velocity_available);
  EXPECT_FALSE(box.acceleration_available);
  EXPECT_EQ(0u, box.id);  // No invented tracking identity.
  const auto& q = box.bbox_pose.orientation;
  EXPECT_DOUBLE_EQ(0.0, q.x);
  EXPECT_DOUBLE_EQ(0.0, q.y);
  EXPECT_NEAR(1.0, q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1e-12);
  for (double value : {box.position.x, box.position.y, box.position.z,
                      box.bbox_size.x, box.bbox_size.y, box.bbox_size.z, q.z, q.w})
    EXPECT_TRUE(std::isfinite(value));
  EXPECT_GE(box.bbox_size.x, box.bbox_size.y);
  EXPECT_GE(box.bbox_size.y, 0.0);
  EXPECT_GE(boxYaw(box), -kPi/2.0);
  EXPECT_LT(boxYaw(box), kPi/2.0);
}

void expectVehicle(const interfaces::Object& box, double yaw, double cx = 5.0, double cy = 0.0) {
  expectSemantics(box);
  EXPECT_NEAR(cx, box.position.x, 2e-5);
  EXPECT_NEAR(cy, box.position.y, 2e-5);
  EXPECT_NEAR(1.0, box.position.z, 1e-6);
  EXPECT_NEAR(4.0, box.bbox_size.x, 2e-5);
  EXPECT_NEAR(2.0, box.bbox_size.y, 2e-5);
  EXPECT_NEAR(1.5, box.bbox_size.z, 1e-6);
  EXPECT_NEAR(0.0, std::remainder(boxYaw(box) - yaw, kPi), 2e-5);
}
}  // namespace

TEST(DbscanObb, AxisAlignedVehicle) {
  const auto output = fitPoints(vehicleSurface(0.0));
  ASSERT_EQ(1u, output.objects.size());
  expectVehicle(output.objects.front(), 0.0);
  EXPECT_NEAR(8.0, output.objects.front().bbox_size.x * output.objects.front().bbox_size.y, 1e-5);
}

TEST(DbscanObb, RotatedVehicleIsTighterThanAabb) {
  for (double degrees : {30.0, 45.0, -30.0, 75.0}) {
    SCOPED_TRACE(degrees);
    const double yaw = degrees * kPi / 180.0;
    const auto points = vehicleSurface(yaw);
    const auto output = fitPoints(points);
    ASSERT_EQ(1u, output.objects.size());
    const auto& box = output.objects.front();
    expectVehicle(box, yaw);
    float min_x = points.front()[0], max_x = min_x;
    float min_y = points.front()[1], max_y = min_y;
    for (const auto& point : points) {
      min_x = std::min(min_x, point[0]); max_x = std::max(max_x, point[0]);
      min_y = std::min(min_y, point[1]); max_y = std::max(max_y, point[1]);
    }
    const double aabb_area = (max_x - min_x) * (max_y - min_y);
    const double obb_area = box.bbox_size.x * box.bbox_size.y;
    EXPECT_NEAR(8.0, obb_area, 1e-4);
    EXPECT_GT(aabb_area, obb_area + 4.0);
    // All original points lie inside the predicted local box.
    const double c = std::cos(boxYaw(box)), s = std::sin(boxYaw(box));
    for (const auto& point : points) {
      const double dx = point[0] - box.position.x, dy = point[1] - box.position.y;
      EXPECT_LE(std::abs(c*dx + s*dy), box.bbox_size.x/2.0 + 1e-5);
      EXPECT_LE(std::abs(-s*dx + c*dy), box.bbox_size.y/2.0 + 1e-5);
    }
  }
}

TEST(DbscanObb, TranslationChangesOnlyCenter) {
  for (const Center center : {Center{{5.0f, 0.0f, 1.0f}}, Center{{15.0f, 3.0f, 1.0f}}}) {
    const auto output = fitPoints(vehicleSurface(kPi/6.0, center[0], center[1]));
    ASSERT_EQ(1u, output.objects.size());
    expectVehicle(output.objects.front(), kPi/6.0, center[0], center[1]);
  }
}

TEST(DbscanObb, UsesZExtremaRatherThanCentroid) {
  auto points = vehicleSurface(kPi/4.0);
  for (auto& point : points) point[2] += 4.0f;
  // Distinct interior points survive voxelization and bias the centroid downward.
  for (int i = 0; i < 20; ++i) points.push_back({{4.6f + 0.04f*i, 0.0f, 4.4f}});
  double centroid_z = 0.0;
  for (const auto& point : points) centroid_z += point[2]/points.size();
  EXPECT_GT(std::abs(centroid_z - 5.0), 0.05);
  const auto output = fitPoints(points);
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_NEAR(5.0, output.objects.front().position.z, 1e-6);
  EXPECT_NEAR(1.5, output.objects.front().bbox_size.z, 1e-6);
}

TEST(DbscanObb, CanonicalizesHalfTurnAndPointOrder) {
  const auto reference = fitPoints(vehicleSurface(kPi/18.0));
  ASSERT_EQ(1u, reference.objects.size());
  for (double yaw : {kPi/18.0, kPi/18.0 + kPi, kPi/18.0 - kPi}) {
    auto points = vehicleSurface(yaw);
    std::reverse(points.begin(), points.end());
    const auto output = fitPoints(points);
    ASSERT_EQ(1u, output.objects.size());
    expectVehicle(output.objects.front(), kPi/18.0);
    EXPECT_NEAR(boxYaw(reference.objects.front()), boxYaw(output.objects.front()), 1e-6);
  }
}

TEST(DbscanObb, DegeneratePointLineAndNearlyZeroAreaAreFinite) {
  const std::vector<std::vector<Center>> cases{
    {{{2.0f, 3.0f, 0.5f}}},
    {{{0.0f, 0.0f, 0.5f}}, {{0.5f, 0.5f, 0.5f}}},
    {{{0.0f, 0.0f, 0.25f}}, {{0.5f, 0.5f, 1.0f}}, {{1.0f, 1.0f, 1.75f}}},
    {{{0.0f, 0.0f, 0.5f}}, {{0.5f, 1e-8f, 0.5f}}, {{1.0f, 0.0f, 0.5f}}}};
  for (std::size_t i = 0; i < cases.size(); ++i) {
    SCOPED_TRACE(i);
    const auto output = fitPoints(cases[i]);
    ASSERT_EQ(1u, output.objects.size());
    const auto& box = output.objects.front();
    expectSemantics(box);
    auto reversed = cases[i];
    std::reverse(reversed.begin(), reversed.end());
    const auto again = fitPoints(reversed);
    ASSERT_EQ(1u, again.objects.size());
    EXPECT_EQ(box, again.objects.front());
    if (i == 0) {
      EXPECT_DOUBLE_EQ(0.0, box.bbox_size.x);
      EXPECT_DOUBLE_EQ(0.0, box.bbox_size.y);
      EXPECT_DOUBLE_EQ(0.0, boxYaw(box));
    } else if (i < 3) {
      EXPECT_NEAR(kPi/4.0, boxYaw(box), 1e-6);
      EXPECT_NEAR(i == 1 ? std::sqrt(0.5) : std::sqrt(2.0), box.bbox_size.x, 1e-6);
      EXPECT_NEAR(0.0, box.bbox_size.y, 1e-12);
    } else {
      EXPECT_NEAR(1.0, box.bbox_size.x, 1e-6);
      EXPECT_NEAR(0.0, boxYaw(box), 1e-6);
      EXPECT_NEAR(1e-8, box.bbox_size.y, 1e-14);
      const double c = std::cos(boxYaw(box)), s = std::sin(boxYaw(box));
      for (const auto& point : cases[i]) {
        const double dx = point[0]-box.position.x, dy = point[1]-box.position.y;
        EXPECT_LE(std::abs(c*dx+s*dy), box.bbox_size.x/2.0 + 1e-14);
        EXPECT_LE(std::abs(-s*dx+c*dy), box.bbox_size.y/2.0 + 1e-14);
      }
    }
  }
}

TEST(DbscanObb, AabbHelperRemainsAvailableForSameClusterComparison) {
  for (double yaw : {0.0, kPi/6.0, kPi/4.0}) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(makePoints(vehicleSurface(yaw)), cloud);
    std::vector<int> indices(cloud.size());
    std::iota(indices.begin(), indices.end(), 0);
    const auto aabb = object_detector::bbox::makeAabb(cloud, indices);
    const auto obb = object_detector::bbox::makeObb(cloud, indices);
    EXPECT_DOUBLE_EQ(1.0, aabb.bbox_pose.orientation.w);
    EXPECT_DOUBLE_EQ(0.0, aabb.bbox_pose.orientation.z);
    EXPECT_EQ(aabb.position, aabb.bbox_pose.position);
    const double aabb_area = aabb.bbox_size.x * aabb.bbox_size.y;
    const double obb_area = obb.bbox_size.x * obb.bbox_size.y;
    EXPECT_GE(aabb_area + 1e-10, obb_area);
    if (yaw == 0.0) EXPECT_NEAR(aabb_area, obb_area, 1e-10);
    else EXPECT_GT(aabb_area, obb_area + 8.0);
  }
}

TEST(DbscanObb, UnevenTriangleUsesHullEdgesRatherThanPrincipalAxes) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(makePoints({{{0.0f, 0.0f, 0.0f}}, {{4.0f, 0.0f, 0.0f}},
                             {{0.0f, 1.0f, 2.0f}}, {{0.1f, 0.1f, 0.0f}}}), cloud);
  const auto obb = object_detector::bbox::makeObb(cloud, {0, 1, 2, 3});
  expectSemantics(obb);
  EXPECT_NEAR(2.0, obb.position.x, 1e-12);
  EXPECT_NEAR(0.5, obb.position.y, 1e-12);
  EXPECT_NEAR(1.0, obb.position.z, 1e-12);
  EXPECT_NEAR(4.0, obb.bbox_size.x, 1e-12);
  EXPECT_NEAR(1.0, obb.bbox_size.y, 1e-12);
  EXPECT_NEAR(0.0, boxYaw(obb), 1e-12);
}

TEST(DbscanObb, SquareTiesDuplicatesAndVerticalLineAreDeterministic) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(makePoints({{{-1.0f, -1.0f, 0.0f}}, {{1.0f, -1.0f, 0.0f}},
    {{1.0f, 1.0f, 2.0f}}, {{-1.0f, 1.0f, 2.0f}}, {{1.0f, 1.0f, 0.0f}}}), cloud);
  std::vector<int> indices{0, 1, 2, 3, 4};
  const auto reference = object_detector::bbox::makeObb(cloud, indices);
  EXPECT_NEAR(0.0, boxYaw(reference), 1e-12);
  std::mt19937 random(42);
  for (int iteration = 0; iteration < 20; ++iteration) {
    std::shuffle(indices.begin(), indices.end(), random);
    EXPECT_EQ(reference, object_detector::bbox::makeObb(cloud, indices));
  }
  const auto vertical = fitPoints({{{1.0f, 1.0f, 0.25f}}, {{1.0f, 1.0f, 1.0f}},
                                   {{1.0f, 1.0f, 1.75f}}});
  ASSERT_EQ(1u, vertical.objects.size());
  expectSemantics(vertical.objects.front());
  EXPECT_DOUBLE_EQ(0.0, vertical.objects.front().bbox_size.x);
  EXPECT_DOUBLE_EQ(0.0, vertical.objects.front().bbox_size.y);
  EXPECT_DOUBLE_EQ(1.5, vertical.objects.front().bbox_size.z);
}

TEST(DbscanObb, HalfOpenYawBoundaryAndLongAxisSwap) {
  for (double degrees : {90.0, -90.0, 105.0}) {
    const auto output = fitPoints(vehicleSurface(degrees*kPi/180.0));
    ASSERT_EQ(1u, output.objects.size());
    expectVehicle(output.objects.front(), degrees*kPi/180.0);
    EXPECT_LT(boxYaw(output.objects.front()), 0.0);
  }
}

TEST(DbscanObb, UsesPostVoxelCentroidsForBox) {
  std::vector<Center> points;
  for (float x : {1.1f, 1.3f, 3.1f, 3.3f})
    for (float y : {0.1f, 0.3f, 1.1f, 1.3f}) points.push_back({{x, y, 0.5f}});
  auto p = testParameters();
  p.voxel_leaf_size = 1.0;
  p.dbscan_eps = 3.0;
  p.dbscan_min_points = 1;
  p.min_cluster_size = 1;
  const auto output = object_detector::Dbscan(p).process(makePoints(points));
  ASSERT_EQ(1u, output.objects.size());
  const auto& box = output.objects.front();
  expectSemantics(box);
  EXPECT_NEAR(2.2, box.position.x, 1e-6);
  EXPECT_NEAR(0.7, box.position.y, 1e-6);
  EXPECT_NEAR(2.0, box.bbox_size.x, 1e-6);  // Raw points span 2.2 m.
  EXPECT_NEAR(1.0, box.bbox_size.y, 1e-6);  // Raw points span 1.2 m.
}

TEST(DbscanObb, RotatedSquareTieChoosesSignedCanonicalYaw) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(makePoints({{{-1.0f, 0.0f, 0.0f}}, {{0.0f, -1.0f, 0.0f}},
                             {{1.0f, 0.0f, 0.0f}}, {{0.0f, 1.0f, 0.0f}}}), cloud);
  const auto box = object_detector::bbox::makeObb(cloud, {0, 1, 2, 3});
  expectSemantics(box);
  EXPECT_NEAR(-kPi/4.0, boxYaw(box), 1e-12);
  EXPECT_NEAR(2.0, box.bbox_size.x * box.bbox_size.y, 1e-12);
  EXPECT_EQ(box, object_detector::bbox::makeObb(cloud, {3, 1, 0, 2}));
}

TEST(DbscanObb, GeneralHullsContainPointsAndBeatSampledAngles) {
  std::mt19937 random(123);
  std::uniform_real_distribution<float> distribution(-2.0f, 2.0f);
  for (int fixture = 0; fixture < 10; ++fixture) {
    SCOPED_TRACE(fixture);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < 64; ++i) {
      pcl::PointXYZ point;
      point.x = distribution(random) + 5.0f;
      point.y = distribution(random);
      point.z = distribution(random);
      cloud.push_back(point);
    }
    std::vector<int> indices(cloud.size());
    std::iota(indices.begin(), indices.end(), 0);
    const auto box = object_detector::bbox::makeObb(cloud, indices);
    expectSemantics(box);
    const double yaw = boxYaw(box), c = std::cos(yaw), s = std::sin(yaw);
    for (const auto& point : cloud) {
      const double dx = point.x-box.position.x, dy = point.y-box.position.y;
      EXPECT_LE(std::abs(c*dx+s*dy), box.bbox_size.x/2.0 + 1e-10);
      EXPECT_LE(std::abs(-s*dx+c*dy), box.bbox_size.y/2.0 + 1e-10);
    }
    // Independent dense angle sweep uses every input point, without the hull helper.
    for (int angle = 0; angle < 720; ++angle) {
      const double a = angle*kPi/720.0, ca = std::cos(a), sa = std::sin(a);
      double min_u = 100.0, max_u = -100.0, min_v = 100.0, max_v = -100.0;
      for (const auto& point : cloud) {
        const double u = ca*point.x+sa*point.y, v = -sa*point.x+ca*point.y;
        min_u = std::min(min_u, u); max_u = std::max(max_u, u);
        min_v = std::min(min_v, v); max_v = std::max(max_v, v);
      }
      EXPECT_LE(box.bbox_size.x*box.bbox_size.y, (max_u-min_u)*(max_v-min_v) + 1e-10);
    }
    std::shuffle(indices.begin(), indices.end(), random);
    EXPECT_EQ(box, object_detector::bbox::makeObb(cloud, indices));
  }
}
