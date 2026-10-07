#include <control/cores/path_tracking.hpp>
#include <control/validators/path_contract.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <gtest/gtest.h>
#include <limits>
namespace {
struct Parameters {
  control::tracking::Parameters operator()(const node_pipeline::Parameters&) const {
    control::tracking::Parameters p;p.lateral_controller="pure_pursuit";p.longitudinal_controller="pid";
    p.mpc.solver=p.longitudinal_mpc.solver="projected_gradient";return p;
  }
};
interfaces::Path path(double velocity) {
  interfaces::Path result;result.header.frame_id="base_link";result.header.stamp.fromNSec(1000000000);
  for(double x:{0.0,10.0}) {
    interfaces::PathPoint point;point.pose.header=result.header;point.pose.pose.position.x=x;
    point.pose.pose.orientation.w=1;point.velocity=velocity;result.points.push_back(point);
  }
  return result;
}
TEST(ControlInputs, AllowsStoppedAndUnboundedForwardTargets) {
  namespace np=node_pipeline;namespace nd=np::detail;
  nd::Engine engine(8,1000000000);
  nd::InputSpec path_spec("path","/path");path_spec.limits.frame="base_link";
  nd::InputSpec ego_spec("ego","/ego");ego_spec.limits.frame="map";
  nd::InputSpec status_spec("competition","/status");status_spec.limits.frame="base_link";
  engine.input<interfaces::Path>(path_spec);engine.input<nav_msgs::Odometry>(ego_spec);
  engine.input<interfaces::CompetitionVehicleStatus>(status_spec);
  nd::CoreSpec core;core.name="path_tracking.hpp";core.stage="stage1";
  core.inputs={"path","ego","competition"};core.outputs={{"command",""}};
  engine.core<control::PathTracking,Parameters,control::validators::PathContract>(core,{});
  nd::StageSpec stage;stage.name="stage1";stage.section_core_map[0]=core.name;engine.stage(stage);
  const np::Clocks clocks{1000000000,1000000000};
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.child_frame_id="base_link";ego.pose.pose.orientation.w=1;
  engine.push<nav_msgs::Odometry>("ego",std::make_shared<const nav_msgs::Odometry>(ego),clocks,1000000000);
  interfaces::CompetitionVehicleStatus status;status.header.frame_id="base_link";
  engine.push<interfaces::CompetitionVehicleStatus>("competition",std::make_shared<const interfaces::CompetitionVehicleStatus>(status),clocks,1000000000);
  for(double speed:{0.0,100.0/3.6,200.0/3.6,-.1}) {
    const auto message=path(speed);
    const auto id=engine.push<interfaces::Path>("path",std::make_shared<const interfaces::Path>(message),clocks,1000000000);
    EXPECT_EQ(speed>=0?np::ProcessStatus::Published:np::ProcessStatus::Rejected,engine.process(clocks,false,"path",id).status)<<speed;
  }
}
TEST(ControlInputs, RejectsNonfiniteMeasuredSteering) {
  node_pipeline::detail::InputSpec declaration;declaration.limits.frame="base_link";
  interfaces::CompetitionVehicleStatus status;status.header.frame_id="base_link";
  for(float angle:{std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {
    status.steering_angle_rad=angle;std::vector<node_pipeline::Kind> issues;
    node_pipeline::detail::MessageTraits<interfaces::CompetitionVehicleStatus>::inspect(status,declaration.limits,issues);
    EXPECT_FALSE(issues.empty());
  }
}
}
int main(int argc,char** argv) {
  ros::Time::init();testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
