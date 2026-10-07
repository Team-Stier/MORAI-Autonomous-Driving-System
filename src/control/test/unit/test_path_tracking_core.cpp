#include <control/cores/path_tracking.hpp>
#include <control/validators/path_contract.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <gtest/gtest.h>
#include <limits>
namespace {
control::tracking::Parameters parameters() {
  control::tracking::Parameters p;
  p.lateral_controller = "pure_pursuit";
  p.longitudinal_controller = "pid";
  p.mpc.solver = "projected_gradient";
  p.longitudinal_mpc.solver = "projected_gradient";
  p.pid.kp = 1;
  p.pid.ki = 0;
  p.pid.kd = 0;
  return p;
}
interfaces::Path path(double speed, double lateral = 0) {
  interfaces::Path result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  for (double x : {-10.0, 0.0, 10.0, 20.0}) {
    interfaces::PathPoint point;
    point.pose.header = result.header;
    point.pose.pose.position.x = x;
    point.pose.pose.position.y = lateral;
    point.pose.pose.orientation.w = 1;
    point.velocity = speed;
    result.points.push_back(point);
  }
  return result;
}
nav_msgs::Odometry ego() {
  nav_msgs::Odometry result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "map";
  result.child_frame_id = "base_link";
  result.pose.pose.orientation.w = 1;
  return result;
}
interfaces::CompetitionVehicleStatus status() {
  interfaces::CompetitionVehicleStatus result;
  result.header.stamp = ros::Time(1, 0);
  result.header.frame_id = "base_link";
  result.velocity_x_mps = 5;
  return result;
}
TEST(PathTrackingCore, UsesInjectedNearestSpeedAndNormalizedSteering) {
  control::PathTracking core(parameters());
  auto input = path(10, 2);
  input.points[1].velocity = 0;
  const auto command = core.process(input, ego(), status());
  EXPECT_EQ(command.longlCmdType, 1);
  EXPECT_DOUBLE_EQ(command.accel, 0);
  EXPECT_GT(command.brake, 0);
  EXPECT_GT(command.steering, 0);
  EXPECT_LE(command.steering, 1);
}
TEST(PathTrackingCore, RetainsPidStateAcrossCalls) {
  auto p = parameters();
  p.pid.kp = 0;
  p.pid.ki = 0.1;
  control::PathTracking core(p);
  auto input = path(6);
  const auto first = core.process(input, ego(), status());
  input.header.stamp = ros::Time(1, 50000000);
  const auto second = core.process(input, ego(), status());
  EXPECT_GT(second.accel, first.accel);
}
TEST(PathTrackingCore, RunsBothMpcSolversWithoutRosIo) {
  auto p = parameters();
  p.lateral_controller = "mpc";
  p.longitudinal_controller = "mpc";
  control::PathTracking core(p);
  const auto command = core.process(path(6), ego(), status());
  EXPECT_TRUE(std::isfinite(command.steering));
  EXPECT_LE(std::abs(command.steering), 1);
  EXPECT_GE(command.accel, 0);
  EXPECT_GE(command.brake, 0);
  EXPECT_FALSE(command.accel > 0 && command.brake > 0);
}
struct TestParameters {
  control::tracking::Parameters operator()(const node_pipeline::Parameters&) const { return parameters(); }
};
TEST(PathTrackingPipeline, AllSectionsSelectCoreAndInvalidInputsAreRemoved) {
  namespace np=node_pipeline; namespace nd=np::detail;
  nd::Engine engine(8,1000000000);
  nd::InputSpec path_spec("path","/local_path/velocity_injected");path_spec.limits.frame="base_link";
  nd::InputSpec ego_spec("ego","/localization/ego");ego_spec.limits.frame="map";
  nd::InputSpec status_spec("competition","/vehicle/competition_status");status_spec.limits.frame="base_link";
  engine.input<interfaces::Path>(path_spec);engine.input<nav_msgs::Odometry>(ego_spec);
  engine.input<interfaces::CompetitionVehicleStatus>(status_spec);
  nd::CoreSpec declaration;declaration.name="path_tracking.hpp";declaration.stage="stage1";
  declaration.inputs={"path","ego","competition"};declaration.outputs={{"command",""}};
  engine.core<control::PathTracking,TestParameters,control::validators::PathContract>(declaration,{});
  nd::StageSpec stage;stage.name="stage1";
  for(int i=0;i<=16;++i)stage.section_core_map[i]=declaration.name;
  engine.stage(stage);
  int outputs=0;const np::Clocks clocks{1000000000,1000000000};
  auto input=path(6);
  const auto publish_path=[&] {
    const auto id=engine.push<interfaces::Path>("path",std::make_shared<const interfaces::Path>(input),clocks,input.header.stamp.toNSec());
    const auto report=engine.process(clocks,false,"path",id);
    if(report.status==np::ProcessStatus::Published)++outputs;
    return report.status;
  };
  EXPECT_EQ(np::ProcessStatus::Rejected,publish_path());
  engine.push<nav_msgs::Odometry>("ego",std::make_shared<const nav_msgs::Odometry>(ego()),clocks,1000000000);
  engine.push<interfaces::CompetitionVehicleStatus>("competition",std::make_shared<const interfaces::CompetitionVehicleStatus>(status()),clocks,1000000000);
  for(int i=0;i<=16;++i) {
    engine.section(np::Section{true,static_cast<std::uint8_t>(i),1000000000,"map"});
    EXPECT_EQ(np::ProcessStatus::Published,publish_path());
  }
  EXPECT_EQ(17,outputs);
  auto invalid=status();invalid.velocity_x_mps=std::numeric_limits<float>::quiet_NaN();
  engine.push<interfaces::CompetitionVehicleStatus>("competition",std::make_shared<const interfaces::CompetitionVehicleStatus>(invalid),clocks,1000000000);
  EXPECT_EQ(np::ProcessStatus::Rejected,publish_path());
  EXPECT_EQ(17,outputs);
}

}
int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
