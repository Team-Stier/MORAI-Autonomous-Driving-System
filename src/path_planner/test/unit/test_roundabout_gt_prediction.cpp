#include <path_planner/cores/roundabout_gt_prediction.hpp>
#include <path_planner/validators/prediction_motion_inputs.hpp>
#include <gtest/gtest.h>
#include <cmath>

namespace pp = path_planner;
namespace {
class RoundaboutGtPredictionTest : public testing::Test {
 protected:
  RoundaboutGtPredictionTest() {
    objects.header.stamp.fromSec(100);
    objects.header.seq = 21;
    objects.header.frame_id = "map";
    interfaces::Object object;
    object.id = 77;
    object.label = "vehicle";
    object.velocity_available = object.acceleration_available = true;
    object.bbox_pose.orientation.w = 1;
    object.bbox_size.x = 4;
    object.bbox_size.y = 2;
    object.bbox_size.z = 1.5;
    objects.objects.push_back(object);
  }
  pp::PredictedObjects predict() {
    pp::RoundaboutGtPrediction core({});
    return std::get<6>(core.process(nav_msgs::Odometry{}, interfaces::CompetitionVehicleStatus{},
        objects, tf2_msgs::TFMessage{}, sensor_msgs::Imu{}, pp::Route{}));
  }
  interfaces::Objects objects;
};
}
TEST_F(RoundaboutGtPredictionTest, UsesMapSpeedOnTheFirstObservationAndPreservesMetadata) {
  objects.objects[0].velocity.y = -11;
  const auto result = predict();
  ASSERT_EQ(result.size(), 10u);
  for (std::size_t i = 0; i < result.size(); ++i) {
    EXPECT_EQ(result[i].header, objects.header);
    EXPECT_EQ(result[i].objects[0].id, 77u);
    EXPECT_NEAR(result[i].objects[0].position.x, 0, 1e-10);
    EXPECT_NEAR(result[i].objects[0].position.y, -11*0.5*(i+1), 1e-10);
    EXPECT_EQ(result[i].objects[0].position, result[i].objects[0].bbox_pose.position);
  }
}
TEST_F(RoundaboutGtPredictionTest, FollowsObservedCircularMotionAndRotatesBoxes) {
  objects.objects[0].velocity.x = 5;
  objects.objects[0].acceleration.y = 2;
  const auto result = predict();
  for (std::size_t i = 0; i < result.size(); ++i) {
    const double turn = .4*.5*(i+1);
    const auto& o = result[i].objects[0];
    EXPECT_NEAR(o.position.x, 12.5*std::sin(turn), 1e-10);
    EXPECT_NEAR(o.position.y, 12.5*(1-std::cos(turn)), 1e-10);
    EXPECT_NEAR(o.bbox_pose.orientation.z, std::sin(turn/2), 1e-10);
    EXPECT_NEAR(o.velocity.y, 5*std::sin(turn), 1e-10);
  }
}
TEST_F(RoundaboutGtPredictionTest, BrakesToRestWithoutReversingAcrossTheHorizon) {
  objects.objects[0].velocity.x = 4;
  objects.objects[0].acceleration.x = -2;
  const auto result = predict();
  EXPECT_NEAR(result[0].objects[0].position.x, 1.75, 1e-10);
  EXPECT_NEAR(result[0].objects[0].acceleration.x, -2, 1e-10);
  for (std::size_t i = 3; i < result.size(); ++i) {
    EXPECT_NEAR(result[i].objects[0].position.x, 4, 1e-10);
    EXPECT_DOUBLE_EQ(result[i].objects[0].velocity.x, 0);
  }
}
TEST_F(RoundaboutGtPredictionTest, LimitsAccelerationPersistenceAndHandlesStartupFromRest) {
  objects.objects[0].acceleration.x = 2;
  const auto result = predict();
  EXPECT_NEAR(result[0].objects[0].position.x, .25, 1e-10);
  EXPECT_NEAR(result[0].objects[0].acceleration.x, 2, 1e-10);
  EXPECT_NEAR(result.back().objects[0].position.x, 9, 1e-10);
  EXPECT_NEAR(result.back().objects[0].velocity.x, 2, 1e-10);
  EXPECT_DOUBLE_EQ(result.back().objects[0].acceleration.x, 0);
}
TEST_F(RoundaboutGtPredictionTest, KeepsStationaryObjectsAndEmptyObservations) {
  const auto result = predict();
  for (const auto& frame : result) EXPECT_EQ(frame.objects[0], objects.objects[0]);
  objects.objects.clear();
  const auto empty = predict();
  for (const auto& frame : empty) {
    EXPECT_TRUE(frame.objects.empty());
    EXPECT_EQ(frame.header, objects.header);
  }
}
TEST_F(RoundaboutGtPredictionTest, RequiresAvailableMotionVectorsInTheValidator) {
  objects.objects[0].velocity_available = false;
  std::vector<node_pipeline::Kind> issues;
  pp::PredictionMotionInputs{}(objects, node_pipeline::HealthLimits{}, issues);
  EXPECT_NE(std::find(issues.begin(), issues.end(), node_pipeline::Kind::Missing), issues.end());
}
