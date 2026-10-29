#include <path_planner/cores/rddf_core.hpp>
#include <path_planner/parameters/rddf_core.hpp>
#include <path_planner/validators/rddf_ego.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>

namespace pp = path_planner;
namespace {
geometry_msgs::Point point(double x, double y, double z) {
  geometry_msgs::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

nav_msgs::Odometry egoAt(double x, double y) {
  nav_msgs::Odometry ego;
  ego.pose.pose.position.x = x;
  ego.pose.pose.position.y = y;
  ego.pose.pose.orientation.w = 1.0;
  return ego;
}

void expectPosition(const interfaces::PathPoint& actual, double x, double y, double z) {
  EXPECT_NEAR(actual.pose.pose.position.x, x, 1e-9);
  EXPECT_NEAR(actual.pose.pose.position.y, y, 1e-9);
  EXPECT_NEAR(actual.pose.pose.position.z, z, 1e-9);
}
}

TEST(RddfCore, DefaultsToFiftyMetersWithUninjectedPayload) {
  pp::RddfCoreParameters parameters;
  parameters.global_path = {point(0, 0, 0), point(40, 0, 4), point(100, 0, 10)};
  pp::RddfCore core(parameters);
  auto ego = egoAt(5, 3);
  ego.header.seq = 17;
  ego.header.stamp = ros::Time(123, 456);
  ego.header.frame_id = "map";
  const auto path = core.process(ego);

  ASSERT_EQ(path.points.size(), 3u);
  expectPosition(path.points[0], 5, 0, 0.5);
  expectPosition(path.points[1], 40, 0, 4);
  expectPosition(path.points[2], 55, 0, 5.5);
  EXPECT_EQ(path.header.seq, 0u);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
  for (const auto& sample : path.points) {
    EXPECT_TRUE(std::isnan(sample.velocity));
    EXPECT_EQ(sample.pose.header.seq, 0u);
    EXPECT_TRUE(sample.pose.header.stamp.isZero());
    EXPECT_TRUE(sample.pose.header.frame_id.empty());
    EXPECT_DOUBLE_EQ(sample.pose.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(sample.pose.pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(sample.pose.pose.orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(sample.pose.pose.orientation.w, 1.0);
  }
}

TEST(RddfCore, CropsConfiguredXyArcLengthAndInterpolatesHeight) {
  pp::RddfCoreParameters parameters;
  parameters.local_path_length_m = 12.0;
  parameters.global_path = {point(0, 0, 0), point(10, 0, 10),
                            point(10, 10, 20), point(20, 10, 30)};
  pp::RddfCore core(parameters);
  const auto path = core.process(egoAt(3, -2));

  ASSERT_EQ(path.points.size(), 3u);
  expectPosition(path.points[0], 3, 0, 3);
  expectPosition(path.points[1], 10, 0, 10);
  expectPosition(path.points[2], 10, 5, 15);
  EXPECT_DOUBLE_EQ(path.points[0].pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(path.points[0].pose.pose.orientation.w, 1.0);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(path.points[i].pose.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(path.points[i].pose.pose.orientation.y, 0.0);
    EXPECT_NEAR(path.points[i].pose.pose.orientation.z, 0.7071067811865476, 1e-9);
    EXPECT_NEAR(path.points[i].pose.pose.orientation.w, 0.7071067811865476, 1e-9);
  }
}

TEST(RddfCore, ReprojectsMovingEgoOntoLaterSegmentsWithoutChangingInputs) {
  pp::RddfCoreParameters parameters;
  parameters.local_path_length_m = 4.0;
  parameters.global_path = {point(0, 0, 0), point(10, 0, 10),
                            point(10, 10, 20), point(0, 10, 30)};
  const auto original_points = parameters.global_path;
  pp::RddfCore core(parameters);
  auto ego = egoAt(9.5, 6);
  const auto original_ego = ego;
  const auto first = core.process(ego);

  ASSERT_EQ(first.points.size(), 2u);
  expectPosition(first.points[0], 10, 6, 16);
  expectPosition(first.points[1], 10, 10, 20);
  EXPECT_EQ(ego, original_ego);
  ego.pose.pose.position = point(6, 11, 0);
  const auto second = core.process(ego);
  ASSERT_EQ(second.points.size(), 2u);
  expectPosition(second.points[0], 6, 10, 24);
  expectPosition(second.points[1], 2, 10, 28);
  for (const auto& sample : second.points) {
    EXPECT_NEAR(sample.pose.pose.orientation.z, 1.0, 1e-9);
    EXPECT_NEAR(sample.pose.pose.orientation.w, 0.0, 1e-9);
  }
  EXPECT_EQ(parameters.global_path, original_points);
  EXPECT_DOUBLE_EQ(ego.pose.pose.position.x, 6.0);
  EXPECT_DOUBLE_EQ(ego.pose.pose.position.y, 11.0);
}

TEST(RddfCore, StopsAtOpenPathEndWithoutWrapping) {
  pp::RddfCoreParameters parameters;
  parameters.global_path = {point(0, 0, 0), point(10, 0, 1), point(10, 10, 2)};
  pp::RddfCore core(parameters);
  const auto near_end = core.process(egoAt(10, 8));

  ASSERT_EQ(near_end.points.size(), 2u);
  expectPosition(near_end.points[0], 10, 8, 1.8);
  expectPosition(near_end.points[1], 10, 10, 2);
  EXPECT_NEAR(near_end.points[1].pose.pose.orientation.z, 0.7071067811865476, 1e-9);
  EXPECT_NEAR(near_end.points[1].pose.pose.orientation.w, 0.7071067811865476, 1e-9);
  const auto beyond_end = core.process(egoAt(10, 15));
  ASSERT_EQ(beyond_end.points.size(), 1u);
  expectPosition(beyond_end.points[0], 10, 10, 2);
}

TEST(RddfCore, SkipsDuplicateXyVertices) {
  pp::RddfCoreParameters parameters;
  parameters.local_path_length_m = 12.0;
  parameters.global_path = {point(0, 0, 0), point(0, 0, 0), point(10, 0, 10),
                            point(10, 0, 10), point(10, 10, 20), point(10, 10, 20)};
  pp::RddfCore core(parameters);
  const auto path = core.process(egoAt(2, 1));

  ASSERT_EQ(path.points.size(), 3u);
  expectPosition(path.points[0], 2, 0, 2);
  expectPosition(path.points[1], 10, 0, 10);
  expectPosition(path.points[2], 10, 4, 14);
}

TEST(RddfEgo, RequiresFiniteXy) {
  const pp::RddfEgo validator;
  std::vector<node_pipeline::Kind> issues;
  validator(egoAt(3, -2), node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  struct Case { double x, y; node_pipeline::Kind issue; };
  const Case cases[] = {{nan, 0, node_pipeline::Kind::NaN},
                        {0, nan, node_pipeline::Kind::NaN},
                        {inf, 0, node_pipeline::Kind::Inf},
                        {0, -inf, node_pipeline::Kind::Inf}};
  for (const auto& sample : cases) {
    issues.clear();
    validator(egoAt(sample.x, sample.y), node_pipeline::HealthLimits{}, issues);
    EXPECT_EQ((std::vector<node_pipeline::Kind>{sample.issue}), issues);
  }
}

TEST(RddfCoreParameters, LoadsProvidedMapPathWithDefaultAndConfiguredLength) {
  const auto defaults = pp::loadRddfCoreParameters(node_pipeline::ParamReader::fromString(
      "global_path_file: ../map/2026_molit_comp_global_path.txt", NP_PACKAGE_DIR));
  ASSERT_EQ(defaults.global_path.size(), 4430u);
  EXPECT_NEAR(defaults.global_path.front().x, -131.68979755061446, 1e-9);
  EXPECT_NEAR(defaults.global_path.front().y, -428.3310229377821, 1e-9);
  EXPECT_NEAR(defaults.global_path.front().z, 28.543960281954277, 1e-9);
  EXPECT_EQ(defaults.global_path.front(), defaults.global_path.back());
  pp::RddfCore core(defaults);
  const auto path = core.process(egoAt(-131.68979755061446, -428.3310229377821));
  ASSERT_GT(path.points.size(), 2u);
  double length_m = 0.0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto& from = path.points[i - 1].pose.pose.position;
    const auto& to = path.points[i].pose.pose.position;
    length_m += std::hypot(to.x - from.x, to.y - from.y);
  }
  EXPECT_NEAR(length_m, 50.0, 1e-9);
  const auto configured = pp::loadRddfCoreParameters(node_pipeline::ParamReader::fromString(
      "local_path_length_m: 12.5\nglobal_path_file: ../map/2026_molit_comp_global_path.txt",
      NP_PACKAGE_DIR));
  EXPECT_DOUBLE_EQ(configured.local_path_length_m, 12.5);
}

TEST(RddfCoreParameters, RejectsInvalidLengthAndDegenerateAsset) {
  for (const auto& yaml : {"local_path_length_m: 0", "local_path_length_m: -1",
                           "local_path_length_m: .nan", "local_path_length_m: .inf"}) {
    const auto params = node_pipeline::ParamReader::fromString(yaml);
    EXPECT_THROW(pp::loadRddfCoreParameters(params), std::invalid_argument);
  }
  const auto empty = node_pipeline::ParamReader::fromString("global_path_file: /dev/null");
  EXPECT_THROW(pp::loadRddfCoreParameters(empty), std::invalid_argument);
}
