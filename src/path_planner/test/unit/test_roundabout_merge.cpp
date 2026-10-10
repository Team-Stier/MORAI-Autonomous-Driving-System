#include <path_planner/cores/roundabout_merge.hpp>
#include <path_planner/cores/rddf_core.hpp>
#include <path_planner/parameters/roundabout_merge.hpp>
#include <path_planner/validators/roundabout_merge_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace pp = path_planner;
namespace {
std::string parameterYaml(const std::string& time_step_sec,
                          const std::string& global_path_file = "../map/2026_molit_comp_global_path.txt") {
  return "global_path_file: " + global_path_file + "\nlocal_path_length_m: 40.0\ndesired_speed_mps: 8.0\n"
      "max_acceleration_mps2: 2.0\nmax_deceleration_mps2: 4.0\n"
      "max_jerk_mps3: 4.0\nmax_lateral_acceleration_mps2: 2.0\n"
      "time_step_sec: " + time_step_sec + "\ncommand_step_sec: 0.2\nbeam_width: 64\n"
      "collision_margin_m: 0.5\nprediction_uncertainty_mps: 0.2\n"
      "vehicle_front_m: 3.845008\nvehicle_rear_m: 0.790000\nvehicle_half_width_m: 1.080418\n";
}

geometry_msgs::Point point(double x, double y, double z) {
  geometry_msgs::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

std::vector<geometry_msgs::Point> straightPath(int step_m = 5) {
  std::vector<geometry_msgs::Point> points;
  for (int x_m = 0; x_m <= 60; x_m += step_m)
    points.push_back(point(x_m, 0.0, 10.0 + 0.02*x_m));
  return points;
}

class RoundaboutMergeTest : public testing::Test {
 protected:
  RoundaboutMergeTest() {
    ego.header.stamp.fromSec(10.0);
    ego.header.frame_id = "map";
    ego.pose.pose.position.x = 2.0;
    ego.pose.pose.orientation.w = 1.0;
    competition.header.frame_id = "base_link";
    competition.velocity_x_mps = 0.0;
    objects.header.stamp = ego.header.stamp;
    objects.header.frame_id = "map";
    stamp = ego.header.stamp;
    parameters.global_path = straightPath();
    predictions.fill(objects);
  }

  void observe(double x_m, double y_m, double vx_mps = 0.0, double vy_mps = 0.0) {
    interfaces::Object object;
    object.id = 42;
    object.label = "vehicle";
    object.position.x = x_m;
    object.position.y = y_m;
    object.bbox_pose.position = object.position;
    object.bbox_pose.orientation.w = 1.0;
    object.bbox_size.x = 4.0;
    object.bbox_size.y = 2.0;
    object.bbox_size.z = 1.5;
    objects.objects = {object};
    predictions.fill(objects);
    for (std::size_t i = 0; i < predictions.size(); ++i) {
      auto& predicted = predictions[i].objects.front();
      const double time_sec = 0.5 * (i + 1);
      predicted.position.x += vx_mps * time_sec;
      predicted.position.y += vy_mps * time_sec;
      predicted.bbox_pose.position = predicted.position;
    }
  }

  interfaces::Path plan(pp::RoundaboutMerge& core) {
    competition.header.stamp = stamp;
    return core.process(ego, competition, objects, predictions, stamp);
  }

  pp::RoundaboutMergeParameters parameters;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus competition;
  interfaces::Objects objects;
  pp::PredictedObjects predictions;
  ros::Time stamp;
};
}  // namespace

TEST_F(RoundaboutMergeTest, StartsFromRestAndKeepsRddfGeometryAndPayloadHeaders) {
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  pp::RddfCore centerline({parameters.local_path_length_m, parameters.global_path});
  const auto reference = centerline.process(ego);
  ASSERT_GE(path.points.size(), 2u);
  ASSERT_EQ(path.points.size(), reference.points.size());
  EXPECT_GT(path.points.front().velocity, 0.0);
  EXPECT_LE(path.points.front().velocity,
      parameters.max_acceleration_mps2 * parameters.command_step_sec);
  EXPECT_TRUE(path.header.stamp.isZero());
  EXPECT_TRUE(path.header.frame_id.empty());
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    const auto& point = path.points[i];
    EXPECT_EQ(point.pose.pose, reference.points[i].pose.pose);
    EXPECT_TRUE(point.pose.header.stamp.isZero());
    EXPECT_TRUE(point.pose.header.frame_id.empty());
    EXPECT_TRUE(std::isfinite(point.velocity));
    EXPECT_GE(point.velocity, 0.0);
    EXPECT_LE(point.velocity, parameters.desired_speed_mps);
    EXPECT_TRUE(std::isfinite(point.acceleration));
    EXPECT_GE(point.acceleration, -parameters.max_deceleration_mps2);
    EXPECT_LE(point.acceleration, parameters.max_acceleration_mps2);
  }
}

TEST_F(RoundaboutMergeTest, CropsNonuniformCurvedRddfAndReprojectsTheMovingEgo) {
  parameters.local_path_length_m = 12.0;
  parameters.global_path = {point(0, 0, 0), point(10, 0, 10),
                            point(10, 10, 20), point(30, 10, 40)};
  ego.pose.pose.position = point(3, -2, 0);
  pp::RoundaboutMerge core(parameters);
  pp::RddfCore reference({parameters.local_path_length_m, parameters.global_path});
  const auto path = plan(core);
  const auto expected = reference.process(ego);
  ASSERT_EQ(path.points.size(), 3u);
  ASSERT_EQ(path.points.size(), expected.points.size());
  double length_m = 0.0;
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_EQ(path.points[i].pose.pose, expected.points[i].pose.pose);
    EXPECT_GE(path.points[i].pose.pose.position.x, 3.0);
    if (i > 0) {
      const auto& from = path.points[i - 1].pose.pose.position;
      const auto& to = path.points[i].pose.pose.position;
      length_m += std::hypot(to.x - from.x, to.y - from.y);
    }
  }
  EXPECT_NEAR(length_m, 12.0, 1e-9);
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 3.0, 1e-9);
  EXPECT_NEAR(path.points.front().pose.pose.position.y, 0.0, 1e-9);
  EXPECT_NEAR(path.points.front().pose.pose.position.z, 3.0, 1e-9);
  EXPECT_NEAR(path.points.back().pose.pose.position.x, 10.0, 1e-9);
  EXPECT_NEAR(path.points.back().pose.pose.position.y, 5.0, 1e-9);
  EXPECT_NEAR(path.points.back().pose.pose.position.z, 15.0, 1e-9);
  EXPECT_NEAR(path.points.back().pose.pose.orientation.z, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(path.points.back().pose.pose.orientation.w, std::sqrt(0.5), 1e-9);

  ego.pose.pose.position = point(14, 11, 0);
  stamp.fromSec(10.1);
  ego.header.stamp = stamp;
  const auto moved = plan(core);
  const auto moved_expected = reference.process(ego);
  ASSERT_EQ(moved.points.size(), 2u);
  ASSERT_EQ(moved.points.size(), moved_expected.points.size());
  for (std::size_t i = 0; i < moved.points.size(); ++i)
    EXPECT_EQ(moved.points[i].pose.pose, moved_expected.points[i].pose.pose);
  EXPECT_NEAR(moved.points.front().pose.pose.position.x, 14.0, 1e-9);
  EXPECT_NEAR(moved.points.front().pose.pose.position.y, 10.0, 1e-9);
  EXPECT_NEAR(moved.points.front().pose.pose.position.z, 24.0, 1e-9);
  EXPECT_NEAR(moved.points.back().pose.pose.position.x, 26.0, 1e-9);
  EXPECT_NEAR(moved.points.back().pose.pose.position.z, 36.0, 1e-9);
}

TEST_F(RoundaboutMergeTest, UsesCompetitionSpeedForTheInitialState) {
  ego.twist.twist.linear.x = 100.0;
  competition.velocity_x_mps = 4.0;
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GE(path.points.front().velocity,
      4.0 - parameters.max_deceleration_mps2 * parameters.command_step_sec);
  EXPECT_LE(path.points.front().velocity,
      4.0 + parameters.max_acceleration_mps2 * parameters.command_step_sec);
}

TEST_F(RoundaboutMergeTest, RequestsStopForMapObstacleRegardlessOfEgoHeading) {
  for (double yaw_rad : {0.0, 0.5 * std::acos(-1.0)}) {
    SCOPED_TRACE(yaw_rad);
    pp::RoundaboutMerge core(parameters);
    ego.pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
    ego.pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
    observe(6.0, 0.0);
    const auto path = plan(core);
    ASSERT_FALSE(path.points.empty());
    for (const auto& point : path.points) {
      EXPECT_DOUBLE_EQ(point.velocity, 0.0);
      EXPECT_DOUBLE_EQ(point.acceleration, -parameters.max_deceleration_mps2);
    }
  }
}

TEST_F(RoundaboutMergeTest, RequestsStopForCrossingBetweenSamplesAndReleasesOnNewObservation) {
  observe(4.0, -10.0, 0.0, 80.0);
  pp::RoundaboutMerge core(parameters);
  const auto waiting = plan(core);
  ASSERT_FALSE(waiting.points.empty());
  for (const auto& point : waiting.points) EXPECT_DOUBLE_EQ(point.velocity, 0.0);
  objects.header.stamp.fromSec(10.5);
  stamp = objects.header.stamp;
  ego.header.stamp = stamp;
  observe(4.0, 30.0, 0.0, 80.0);
  const auto released = plan(core);
  ASSERT_FALSE(released.points.empty());
  EXPECT_GT(released.points.front().velocity, 0.0);
}

TEST_F(RoundaboutMergeTest, IncludesFastApproachingVehiclesBehindTheEgo) {
  observe(-6.0, 0.0, 12.0, 0.0);
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  for (const auto& point : path.points) EXPECT_DOUBLE_EQ(point.velocity, 0.0);
}

TEST_F(RoundaboutMergeTest, RetainsAFeasibleWaitingGapAndReplansWhenItClears) {
  observe(9.3, 0.0);
  for (std::size_t i = 5; i < predictions.size(); ++i) {
    auto& object = predictions[i].objects.front();
    object.position.y = 10.0;
    object.bbox_pose.position = object.position;
  }
  pp::RoundaboutMerge core(parameters);
  const auto waiting = plan(core);
  ASSERT_FALSE(waiting.points.empty());
  EXPECT_TRUE(std::isfinite(waiting.points.front().velocity));
  EXPECT_GE(waiting.points.front().velocity, 0.0);
  objects.header.stamp.fromSec(10.5);
  stamp = objects.header.stamp;
  ego.header.stamp = stamp;
  observe(9.3, 10.0);
  const auto released = plan(core);
  ASSERT_FALSE(released.points.empty());
  EXPECT_GT(released.points.front().velocity, 0.0);
  EXPECT_LE(waiting.points.front().velocity, released.points.front().velocity);
}

TEST_F(RoundaboutMergeTest, AlignsPredictionHorizonToThePlanningStamp) {
  observe(6.0, 0.0, 20.0, 0.0);
  pp::RoundaboutMerge core(parameters);
  const auto occupied = plan(core);
  ASSERT_FALSE(occupied.points.empty());
  for (const auto& point : occupied.points) EXPECT_DOUBLE_EQ(point.velocity, 0.0);
  stamp.fromSec(12.0);
  ego.header.stamp = stamp;
  const auto moved = plan(core);
  ASSERT_FALSE(moved.points.empty());
  EXPECT_GT(moved.points.front().velocity, 0.0);
}

TEST_F(RoundaboutMergeTest, HandlesIndependentlyValidObservationAndPredictionObjectSets) {
  observe(6.0, 0.0);
  for (auto& sample : predictions) {
    sample.header.seq = 17;
    auto& object = sample.objects.front();
    object.id = 43;
    object.position.x = 52.0;
    object.bbox_pose.position = object.position;
  }
  objects.header.seq = 18;
  objects.header.stamp.fromSec(10.2);
  stamp.fromSec(10.25);
  ego.header.stamp = stamp;
  const pp::RoundaboutMergeInputs validator;
  std::vector<node_pipeline::Kind> issues;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  validator(predictions, node_pipeline::HealthLimits{}, issues);
  ASSERT_TRUE(issues.empty());
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  for (const auto& point : path.points) {
    EXPECT_DOUBLE_EQ(point.velocity, 0.0);
    EXPECT_TRUE(std::isfinite(point.acceleration));
  }
}

TEST_F(RoundaboutMergeTest, AdvancesTheStartingPoseToThePlanningStamp) {
  competition.velocity_x_mps = 4.0;
  stamp.fromSec(10.5);
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(path.points.front().pose.pose.position.x, 4.0, 1e-9);
  EXPECT_NEAR(path.points.front().pose.pose.position.y, 0.0, 1e-9);
}

TEST_F(RoundaboutMergeTest, AcceptsValidEmptyObservationsAndPredictions) {
  const pp::RoundaboutMergeInputs validator;
  std::vector<node_pipeline::Kind> issues;
  validator(objects, node_pipeline::HealthLimits{}, issues);
  validator(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_TRUE(issues.empty());
}

TEST_F(RoundaboutMergeTest, RejectsBaseLinkPredictions) {
  for (auto& sample : predictions) sample.header.frame_id = "base_link";
  std::vector<node_pipeline::Kind> issues;
  pp::RoundaboutMergeInputs{}(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::FrameMismatch}), issues);
}

TEST_F(RoundaboutMergeTest, UsesMapObjectsWithTranslatedAndRotatedEgo) {
  ego.pose.pose.position.x = 100.0;
  ego.pose.pose.position.y = 202.0;
  ego.pose.pose.orientation.z = ego.pose.pose.orientation.w = std::sqrt(0.5);
  parameters.global_path = {point(100, 200, 0), point(100, 260, 0)};
  pp::RoundaboutMerge core(parameters);
  const auto free = plan(core);
  ASSERT_FALSE(free.points.empty());
  ASSERT_GT(free.points.front().velocity, 0.0);
  observe(100.0, 206.0);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  for (const auto& point : path.points) EXPECT_DOUBLE_EQ(point.velocity, 0.0);
}

TEST_F(RoundaboutMergeTest, RejectsReverseCompetitionSpeed) {
  competition.velocity_x_mps = -1.0;
  std::vector<node_pipeline::Kind> issues;
  pp::RoundaboutMergeInputs{}(competition, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::OutRangeNegative}), issues);
}

TEST_F(RoundaboutMergeTest, RejectsNonfiniteCompetitionSpeed) {
  for (const auto& invalid : {
      std::pair<float, node_pipeline::Kind>{std::numeric_limits<float>::quiet_NaN(), node_pipeline::Kind::NaN},
      std::pair<float, node_pipeline::Kind>{std::numeric_limits<float>::infinity(), node_pipeline::Kind::Inf}}) {
    competition.velocity_x_mps = invalid.first;
    std::vector<node_pipeline::Kind> issues;
    pp::RoundaboutMergeInputs{}(competition, node_pipeline::HealthLimits{}, issues);
    EXPECT_EQ((std::vector<node_pipeline::Kind>{invalid.second}), issues);
  }
}

TEST_F(RoundaboutMergeTest, RejectsAZeroBoundingBoxDimension) {
  observe(12.0, 0.0);
  objects.objects.front().bbox_size.x = 0.0;
  std::vector<node_pipeline::Kind> issues;
  pp::RoundaboutMergeInputs{}(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::OutRangeNegative}), issues);
}

TEST_F(RoundaboutMergeTest, RejectsDifferentIdsWithinThePredictionArray) {
  observe(12.0, 0.0);
  predictions[1].objects.front().id = 43;
  std::vector<node_pipeline::Kind> issues;
  pp::RoundaboutMergeInputs{}(predictions, node_pipeline::HealthLimits{}, issues);
  EXPECT_EQ((std::vector<node_pipeline::Kind>{node_pipeline::Kind::OutRangePositive}), issues);
}

TEST(RoundaboutMergeParameters, AcceptsABoundedConfiguration) {
  const auto parameters = pp::loadRoundaboutMergeParameters(
      node_pipeline::ParamReader::fromString(parameterYaml("0.25"), NP_PACKAGE_DIR));
  EXPECT_DOUBLE_EQ(parameters.time_step_sec, 0.25);
  EXPECT_DOUBLE_EQ(parameters.command_step_sec, 0.2);
  EXPECT_EQ(parameters.beam_width, 64);
  ASSERT_GE(parameters.global_path.size(), 2u);
  EXPECT_NEAR(parameters.global_path.front().x, -131.68979755061446, 1e-9);
  EXPECT_NEAR(parameters.global_path.front().y, -428.3310229377821, 1e-9);
  EXPECT_NEAR(parameters.global_path.front().z, 28.543960281954277, 1e-9);
}

TEST(RoundaboutMergeParameters, RejectsZeroAndNonfiniteTimeSteps) {
  for (const std::string value : {"0", ".nan", ".inf"}) {
    SCOPED_TRACE(value);
    EXPECT_THROW(pp::loadRoundaboutMergeParameters(
        node_pipeline::ParamReader::fromString(parameterYaml(value), NP_PACKAGE_DIR)), std::invalid_argument);
  }
}

TEST(RoundaboutMergeParameters, RejectsEmptyAndNonRddfAssets) {
  for (const std::string file : {"/dev/null", "models/npc_trajectory_1s.txt"}) {
    SCOPED_TRACE(file);
    EXPECT_THROW(pp::loadRoundaboutMergeParameters(
        node_pipeline::ParamReader::fromString(parameterYaml("0.25", file), NP_PACKAGE_DIR)),
        std::invalid_argument);
  }
}

TEST_F(RoundaboutMergeTest, KeepsMovingBetweenPredictedFrontAndRearVehicles) {
  competition.velocity_x_mps = 5.0;
  observe(16.0, 0.0, 5.0, 0.0);
  auto rear = objects.objects.front();
  rear.id = 43;
  rear.position.x = -12.0;
  rear.bbox_pose.position = rear.position;
  objects.objects.push_back(rear);
  for (std::size_t i = 0; i < predictions.size(); ++i) {
    auto predicted_rear = rear;
    predicted_rear.bbox_pose.position.x += 5.0*0.5*(i + 1);
    predicted_rear.position = predicted_rear.bbox_pose.position;
    predictions[i].objects.push_back(predicted_rear);
  }
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  ASSERT_FALSE(path.points.empty());
  EXPECT_GT(path.points.front().velocity, 4.0);
  EXPECT_LE(path.points.front().velocity, 5.0 + parameters.max_acceleration_mps2*parameters.command_step_sec);
}

TEST_F(RoundaboutMergeTest, BrakesSmoothlyAlongThePathBeyondPredictionCoverage) {
  parameters.global_path = straightPath(1);
  stamp.fromSec(13.5);
  ego.header.stamp = stamp;
  competition.header.stamp = stamp;
  competition.velocity_x_mps = 5.0;
  pp::RoundaboutMerge core(parameters);
  const auto path = plan(core);
  ASSERT_GT(path.points.front().velocity, 0.0);
  for (std::size_t i = 2; i < path.points.size(); ++i) {
    const auto& previous = path.points[i - 1];
    const auto& point = path.points[i];
    const double distance = std::hypot(point.pose.pose.position.x - previous.pose.pose.position.x,
                                      point.pose.pose.position.y - previous.pose.pose.position.y);
    EXPECT_LE(previous.velocity*previous.velocity - point.velocity*point.velocity,
              2*parameters.max_deceleration_mps2*distance + 1e-6);
  }
}

TEST_F(RoundaboutMergeTest, KeepsMeasuredAccelerationWhenPeriodicInputIsRepeated) {
  competition.velocity_x_mps = 4.0;
  pp::RoundaboutMerge core(parameters);
  plan(core);
  stamp.fromSec(10.1);
  ego.header.stamp = stamp;
  competition.header.stamp = stamp;
  competition.velocity_x_mps = 4.2;
  ego.pose.pose.position.x += 0.4;
  objects.header.stamp = stamp;
  predictions.fill(objects);
  const auto first = plan(core);
  const auto repeated = plan(core);
  ASSERT_FALSE(first.points.empty());
  ASSERT_EQ(first.points.size(), repeated.points.size());
  EXPECT_DOUBLE_EQ(first.points.front().velocity, repeated.points.front().velocity);
  EXPECT_DOUBLE_EQ(first.points.front().acceleration, repeated.points.front().acceleration);
  EXPECT_GT(first.points.front().velocity, competition.velocity_x_mps);
}

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
