#include <control/cores/autoware_universe.hpp>
#include <control/validators/path_contract.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <gtest/gtest.h>

namespace {
interfaces::Path path(double target_velocity, double lateral = 0.0) {
  interfaces::Path result;
  result.header.frame_id = "base_link";
  result.header.stamp = ros::Time(1, 0);
  for (int i = -10; i <= 150; ++i) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = i;
    point.pose.pose.position.y = lateral;
    point.pose.pose.orientation.w = 1.0;
    point.velocity = target_velocity;
    point.acceleration = 0.0;
    result.points.push_back(point);
  }
  return result;
}
nav_msgs::Odometry ego() {
  nav_msgs::Odometry result;
  result.header.frame_id = "map";
  result.child_frame_id = "base_link";
  result.pose.pose.orientation.w = 1.0;
  return result;
}
interfaces::CompetitionVehicleStatus status(double speed = 5.0) {
  interfaces::CompetitionVehicleStatus result;
  result.header.frame_id = "base_link";
  result.velocity_x_mps = speed;
  return result;
}
morai_msgs::CtrlCmd drive(control::AutowareUniverse& core, interfaces::Path input,
                          const sensor_msgs::Imu& imu) {
  morai_msgs::CtrlCmd command;
  // The original PID starts in STOPPED; retain its transition and command-rate limit.
  for (int cycle = 0; cycle < 4; ++cycle) {
    input.header.stamp.fromSec(1.0 + cycle * 0.05);
    command = core.process(input, ego(), status(), imu);
  }
  return command;
}
TEST(AutowareUniverseCore, ZeroFeedforwardStillAcceleratesAndBrakesForSpeedError) {
  sensor_msgs::Imu imu;
  control::AutowareUniverse accelerating;
  const auto accel = drive(accelerating, path(10.0), imu);
  EXPECT_EQ(accel.longlCmdType, 1);
  EXPECT_GT(accel.accel, 0.0);
  EXPECT_DOUBLE_EQ(accel.brake, 0.0);
  control::AutowareUniverse braking;
  const auto brake = drive(braking, path(2.0), imu);
  EXPECT_GT(brake.brake, 0.0);
  EXPECT_DOUBLE_EQ(brake.accel, 0.0);
}
TEST(AutowareUniverseCore, TracksStraightAndOppositeLateralOffsets) {
  sensor_msgs::Imu imu;
  control::AutowareUniverse straight, left, right;
  EXPECT_NEAR(straight.process(path(5.0), ego(), status(), imu).steering, 0.0, 1e-6);
  const auto left_command = left.process(path(5.0, 0.5), ego(), status(), imu);
  const auto right_command = right.process(path(5.0, -0.5), ego(), status(), imu);
  EXPECT_GT(left_command.steering, 0.0);
  EXPECT_LT(right_command.steering, 0.0);
  EXPECT_LE(std::abs(left_command.steering), 1.0);
  EXPECT_NEAR(left_command.steering, -right_command.steering, 1e-5);
}
TEST(AutowareUniverseCore, StoppedHoldPreservesNativeMeasuredPhysicalSteering) {
  sensor_msgs::Imu imu;
  for (const double physical_angle : {-0.2, 0.2}) {
    auto competition = status(0.0);
    // Native Competition reports a physical left wheel angle as negative.
    competition.steering_angle_rad = -physical_angle;
    control::AutowareUniverse core;
    const auto command = core.process(path(0.0), ego(), competition, imu);
    EXPECT_NEAR(command.steering * control::universe::defaultParameters().maximum_steering_angle_rad,
                physical_angle, 1e-6);
  }
}
TEST(AutowareUniverseCore, ImuUsesSignedForwardAcceleration) {
  auto parameters = control::universe::defaultParameters();
  parameters.longitudinal.acc_feedback_gain = 1.0;
  parameters.longitudinal.lpf_acc_error_gain = 0.0;
  control::AutowareUniverse baseline(parameters), lateral(parameters), forward(parameters);
  sensor_msgs::Imu zero, sideways, accelerating;
  sideways.linear_acceleration.y = 20.0;
  sideways.linear_acceleration.z = 9.81;
  accelerating.linear_acceleration.x = 2.0;
  const auto input = path(10.0);
  const auto a = drive(baseline, input, zero);
  const auto b = drive(lateral, input, sideways);
  const auto c = drive(forward, input, accelerating);
  EXPECT_DOUBLE_EQ(a.accel, b.accel);
  EXPECT_DOUBLE_EQ(a.brake, b.brake);
  EXPECT_TRUE(c.accel < a.accel || c.brake > a.brake);
}
TEST(AutowareUniverseCore, HoldsStoppedThenStartsWithConvergedSteering) {
  control::AutowareUniverse core;
  sensor_msgs::Imu imu;
  auto stopped = path(0.0);
  const auto held = core.process(stopped, ego(), status(0.0), imu);
  EXPECT_DOUBLE_EQ(held.accel, 0.0);
  EXPECT_GT(held.brake, 0.0);
  auto moving = path(5.0);
  moving.header.stamp = ros::Time(1, 50000000);
  const auto waiting = core.process(moving, ego(), status(0.0), imu);
  EXPECT_DOUBLE_EQ(waiting.accel, 0.0);
  EXPECT_GT(waiting.brake, 0.0);
  morai_msgs::CtrlCmd started;
  for (int cycle = 2; cycle <= 30; ++cycle) {
    moving.header.stamp.fromSec(1.0 + cycle * 0.05);
    started = core.process(moving, ego(), status(0.0), imu);
  }
  EXPECT_GT(started.accel, 0.0);
  EXPECT_DOUBLE_EQ(started.brake, 0.0);
}
TEST(AutowareUniversePipeline, RequiresFreshImuAndRunsAllSections) {
  namespace np=node_pipeline;namespace nd=np::detail;
  np::Clocks clocks{1000000000,1000000000};nd::Engine engine(8,clocks.steady_ns);
  nd::InputSpec path_spec("path","/local_path/velocity_injected");path_spec.limits.frame="base_link";
  nd::InputSpec ego_spec("ego","/localization/ego");ego_spec.limits.frame="map";
  nd::InputSpec status_spec("competition","/vehicle/competition_status");status_spec.limits.frame="base_link";
  nd::InputSpec imu_spec("imu","/sensors/imu/data");imu_spec.limits.frame="imu_link";
  imu_spec.limits.stale_ns=imu_spec.limits.receive_timeout_ns=60000000;
  engine.input<interfaces::Path>(path_spec);engine.input<nav_msgs::Odometry>(ego_spec);
  engine.input<interfaces::CompetitionVehicleStatus>(status_spec);engine.input<sensor_msgs::Imu>(imu_spec);
  nd::CoreSpec declaration;declaration.name="autoware_universe.hpp";declaration.stage="stage1";
  declaration.inputs={"path","ego","competition","imu"};declaration.outputs={{"command",""}};
  engine.core<control::AutowareUniverse,np::NoParameters,control::validators::PathContract>(declaration,{});
  nd::StageSpec stage;stage.name="stage1";for(int i=0;i<=16;++i)stage.section_core_map[i]=declaration.name;engine.stage(stage);
  int outputs=0;auto input=path(100.0/3.6);
  auto odometry=ego();odometry.header.stamp=input.header.stamp;
  auto competition=status();competition.header.stamp=input.header.stamp;
  const auto publish_path=[&] {
    const auto id=engine.push<interfaces::Path>("path",std::make_shared<const interfaces::Path>(input),clocks,input.header.stamp.toNSec());
    const auto report=engine.process(clocks,false,"path",id);
    if(report.status==np::ProcessStatus::Published)++outputs;
    return report.status;
  };
  const auto push_ego=[&] {engine.push<nav_msgs::Odometry>("ego",std::make_shared<const nav_msgs::Odometry>(odometry),clocks,odometry.header.stamp.toNSec());};
  const auto push_status=[&] {engine.push<interfaces::CompetitionVehicleStatus>("competition",std::make_shared<const interfaces::CompetitionVehicleStatus>(competition),clocks,competition.header.stamp.toNSec());};
  push_ego();push_status();
  EXPECT_EQ(np::ProcessStatus::Rejected,publish_path());EXPECT_EQ(0,outputs);
  sensor_msgs::Imu imu;imu.header.frame_id="imu_link";imu.header.stamp=input.header.stamp;
  const auto push_imu=[&] {engine.push<sensor_msgs::Imu>("imu",std::make_shared<const sensor_msgs::Imu>(imu),clocks,imu.header.stamp.toNSec());};
  push_imu();
  for(int i=0;i<=16;++i) {
    engine.section(np::Section{true,static_cast<std::uint8_t>(i),clocks.ros_ns,"map"});
    EXPECT_EQ(np::ProcessStatus::Published,publish_path());
  }
  EXPECT_EQ(17,outputs);
  clocks.ros_ns+=100000000;input.header.stamp.fromNSec(clocks.ros_ns);
  odometry.header.stamp=competition.header.stamp=input.header.stamp;push_ego();push_status();
  EXPECT_EQ(np::ProcessStatus::Rejected,publish_path());
  imu.header.stamp=input.header.stamp;push_imu();
  EXPECT_EQ(np::ProcessStatus::Published,publish_path());
  clocks.steady_ns+=100000000;
  EXPECT_EQ(np::ProcessStatus::Rejected,publish_path());EXPECT_EQ(18,outputs);
}

}  // namespace

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
