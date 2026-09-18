#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "morai_path_manager/route/route_path.hpp"

namespace morai_path_manager {
namespace {

std::string fixture(const std::string& name) {
  return std::string(MORAI_PATH_MANAGER_TEST_DATA_DIR) + "/" + name;
}

double polylineLength(const std::vector<RoutePoint>& points) {
  double length = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length += std::hypot(points[index].x - points[index - 1U].x,
                         points[index].y - points[index - 1U].y);
  }
  return length;
}

TEST(RoutePath, LoadsDeduplicatesAndComputesGeometry) {
  const RoutePath route =
      RoutePath::loadFromFile(fixture("valid_closed_path.txt"), {});
  EXPECT_EQ(route.rawPointCount(), 6U);
  ASSERT_EQ(route.points().size(), 4U);
  EXPECT_TRUE(route.closedLoop());
  EXPECT_DOUBLE_EQ(route.totalLength(), 4.0);
  EXPECT_DOUBLE_EQ(route.maxSegmentLength(), 1.0);
  EXPECT_DOUBLE_EQ(route.points().front().z, 0.0);
  EXPECT_NEAR(route.points()[0].yaw, 0.0, 1e-12);
  EXPECT_NEAR(route.points()[1].yaw, std::acos(-1.0) * 0.5, 1e-12);
}

TEST(RoutePath, RejectsInvalidFiles) {
  EXPECT_THROW(RoutePath::loadFromFile(fixture("missing.txt"), {}),
               std::runtime_error);
  EXPECT_THROW(
      RoutePath::loadFromFile(fixture("malformed_path.txt"), {}),
      std::runtime_error);
  EXPECT_THROW(
      RoutePath::loadFromFile(fixture("nonfinite_path.txt"), {}),
      std::runtime_error);
  EXPECT_THROW(RoutePath::loadFromFile(fixture("open_path.txt"), {}),
               std::runtime_error);
  EXPECT_THROW(RoutePath::loadFromFile(fixture("gapped_path.txt"), {}),
               std::runtime_error);
}

TEST(RoutePath, RejectsFewerThanThreeNonEmptyRawRecords) {
  EXPECT_THROW(
      RoutePath::loadFromFile(fixture("two_point_path.txt"), RouteLoadOptions{}),
      std::runtime_error);
}

TEST(RoutePath, RejectsClosedPathWithFewerThanThreeDistinctRingPoints) {
  EXPECT_THROW(RoutePath::loadFromFile(fixture("two_distinct_ring_points.txt"),
                                       RouteLoadOptions{}),
               std::runtime_error);
}

TEST(RoutePath, BuildsClosedGlobalAndWrappedLocalPaths) {
  const RoutePath route =
      RoutePath::loadFromFile(fixture("valid_closed_path.txt"), {});
  const std::vector<RoutePoint> global = route.globalPoints();
  ASSERT_EQ(global.size(), 5U);
  EXPECT_DOUBLE_EQ(global.front().x, global.back().x);
  EXPECT_DOUBLE_EQ(global.front().y, global.back().y);
  for (std::size_t index = 1U; index < global.size(); ++index) {
    EXPECT_GT(std::hypot(global[index].x - global[index - 1U].x,
                         global[index].y - global[index - 1U].y),
              0.0);
  }

  const std::vector<RoutePoint> wrapped = route.extractForward(3U, 2.0);
  ASSERT_EQ(wrapped.size(), 3U);
  EXPECT_DOUBLE_EQ(wrapped[0].x, 0.0);
  EXPECT_DOUBLE_EQ(wrapped[0].y, 1.0);
  EXPECT_DOUBLE_EQ(wrapped[1].x, 0.0);
  EXPECT_DOUBLE_EQ(wrapped[1].y, 0.0);
  EXPECT_DOUBLE_EQ(wrapped[2].x, 1.0);
  EXPECT_DOUBLE_EQ(wrapped[2].y, 0.0);

  const std::vector<RoutePoint> wrapped_points =
      route.extractForwardPoints(3U, 3U);
  ASSERT_EQ(wrapped_points.size(), 3U);
  EXPECT_DOUBLE_EQ(wrapped_points[0].x, 0.0);
  EXPECT_DOUBLE_EQ(wrapped_points[0].y, 1.0);
  EXPECT_DOUBLE_EQ(wrapped_points[1].x, 0.0);
  EXPECT_DOUBLE_EQ(wrapped_points[1].y, 0.0);
  EXPECT_DOUBLE_EQ(wrapped_points[2].x, 1.0);
  EXPECT_DOUBLE_EQ(wrapped_points[2].y, 0.0);
  EXPECT_THROW(route.extractForwardPoints(0U, 0U),
               std::invalid_argument);
  EXPECT_THROW(route.extractForwardPoints(0U, 5U),
               std::invalid_argument);
}

TEST(RoutePath, AppliesContinuityAndGlobalReacquisition) {
  const RoutePath route =
      RoutePath::loadFromFile(fixture("valid_closed_path.txt"), {});
  EXPECT_EQ(route.nearest(0.9, 0.1).index, 1U);
  EXPECT_EQ(
      route.nearestInWindow(0.9, 0.9, 0U, 0.1, 1.1).index,
      1U);
  EXPECT_EQ(
      route.nearestWithContinuity(
          0.1, 0.9, 0U, 0.1, 1.1, 2.0).index,
      0U);
  EXPECT_EQ(
      route.nearestWithContinuity(
          0.1, 0.9, 0U, 0.1, 1.1, 0.5).index,
      3U);
}

TEST(RoutePath, AllowsOpenPathOnlyWhenExplicitlyConfigured) {
  RouteLoadOptions options;
  options.require_closed_loop = false;
  const RoutePath route =
      RoutePath::loadFromFile(fixture("open_path.txt"), options);
  EXPECT_FALSE(route.closedLoop());
  EXPECT_EQ(route.points().size(), 3U);
  EXPECT_EQ(route.globalPoints().size(), 3U);
  EXPECT_DOUBLE_EQ(route.totalLength(), 2.0);
}

TEST(RoutePath, MatchesOfficialCompetitionPathIntegrity) {
  const RoutePath official =
      RoutePath::loadFromFile(MORAI_PATH_MANAGER_OFFICIAL_PATH, {});
  EXPECT_EQ(official.rawPointCount(), 4430U);
  EXPECT_EQ(official.points().size(), 4391U);
  EXPECT_TRUE(official.closedLoop());
  EXPECT_NEAR(official.totalLength(), 2184.611723336067, 1e-6);
  EXPECT_NEAR(official.maxSegmentLength(), 0.5, 1e-6);

  const std::vector<RoutePoint> local =
      official.extractForward(official.points().size() - 20U, 100.0);
  EXPECT_GE(polylineLength(local), 100.0);
  EXPECT_LE(polylineLength(local),
            100.0 + official.maxSegmentLength() + 1e-9);

  const std::vector<RoutePoint> local_points =
      official.extractForwardPoints(
          official.points().size() - 10U, 20U);
  ASSERT_EQ(local_points.size(), 20U);
  EXPECT_NEAR(polylineLength(local_points), 9.5, 0.71);
}

}  // namespace
}  // namespace morai_path_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
