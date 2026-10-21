#include <path_planner/cores/core_prediction.hpp>
#include <gtest/gtest.h>
#include <cmath>
namespace {
interfaces::Objects observation(double vx,double vy,double ax,double ay) {
  interfaces::Objects objects;objects.header.frame_id="map";objects.header.stamp=ros::Time(10);
  interfaces::Object o;o.id=909;o.label="vehicle";o.position.x=66.65;o.position.y=174.4;
  o.bbox_pose.position=o.position;o.bbox_pose.orientation.w=1;
  o.bbox_size.x=4.635;o.bbox_size.y=2.160835;o.bbox_size.z=1.644;
  o.velocity.x=vx;o.velocity.y=vy;o.acceleration.x=ax;o.acceleration.y=ay;
  o.velocity_available=true;o.acceleration_available=true;objects.objects.push_back(o);return objects;
}
TEST(CorePredictionStandstill, MeasuredStationaryNoiseDoesNotSpinTheVehicleBox) {
  // R47의 정지 앞차: 수 µm/s 잔차로 나눈 회전율이 OBB를 회전시키면 안 된다.
  const auto input=observation(-4e-7,2e-7,.0001928,.0002039);
  const auto output=std::get<6>(path_planner::CorePrediction{}.process({}, {},input,{}, {},{}));
  for(std::size_t i=0;i<output.size();++i) {
    const auto& o=output[i].objects.front();const double dt=.5*(i+1);
    EXPECT_DOUBLE_EQ(o.bbox_pose.orientation.z,0);EXPECT_DOUBLE_EQ(o.bbox_pose.orientation.w,1);
    EXPECT_NEAR(o.position.x,66.65-4e-7*dt,1e-12);
    EXPECT_NEAR(o.position.y,174.4+2e-7*dt,1e-12);
    EXPECT_EQ(o.velocity,input.objects[0].velocity);EXPECT_EQ(o.bbox_size,input.objects[0].bbox_size);
    EXPECT_EQ(output[i].header,input.header);
  }
}
TEST(CorePredictionStandstill, TinyTranslationIsPreservedRatherThanForcedToZero) {
  const auto input=observation(.0005,0,0,.01);
  const auto output=std::get<6>(path_planner::CorePrediction{}.process({}, {},input,{}, {},{}));
  EXPECT_NEAR(output.back().objects[0].position.x,66.6525,1e-12);
  EXPECT_DOUBLE_EQ(output.back().objects[0].position.y,174.4);
  EXPECT_DOUBLE_EQ(output.back().objects[0].velocity.x,.0005);
  EXPECT_DOUBLE_EQ(output.back().objects[0].bbox_pose.orientation.z,0);
}
TEST(CorePredictionStandstill, ZeroVelocityRemainsFiniteWithAnAccelerationObservation) {
  const auto input=observation(0,0,2,1);
  const auto output=std::get<6>(path_planner::CorePrediction{}.process({}, {},input,{}, {},{}));
  for(const auto& frame:output) {
    EXPECT_EQ(frame.objects[0].position,input.objects[0].position);
    EXPECT_EQ(frame.objects[0].bbox_pose,input.objects[0].bbox_pose);
  }
}
TEST(CorePredictionStandstill, MovingVehicleStillFollowsBothTurnDirections) {
  for(double omega:{-.2,.2}) {
    const auto input=observation(2,0,0,2*omega);
    const auto output=std::get<6>(path_planner::CorePrediction{}.process({}, {},input,{}, {},{}));
    const auto& o=output.back().objects.front();
    EXPECT_NEAR(o.position.x,66.65+2*std::sin(5*omega)/omega,1e-12);
    EXPECT_NEAR(o.position.y,174.4+2*(1-std::cos(5*omega))/omega,1e-12);
    EXPECT_NEAR(o.bbox_pose.orientation.z,std::sin(2.5*omega),1e-12);
  }
}
}
