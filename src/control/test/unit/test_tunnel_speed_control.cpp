#include <control/parameters/integrated_mpc.hpp>
#include <control/parameters/morai_mpc.hpp>
#include <gtest/gtest.h>

namespace {
struct Fixture {
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus vehicle;
  interfaces::LocalizationStatus fused,tunnel;
  Fixture() {
    ros::Time::init();ego.header.stamp=ros::Time(100);ego.header.frame_id="map";
    ego.child_frame_id="base_link";ego.pose.pose.orientation.w=1;ego.twist.twist.linear.x=10;
    vehicle.header=ego.header;vehicle.velocity_x_mps=vehicle.signed_velocity_mps=10;
    fused.header=tunnel.header=ego.header;fused.mode=interfaces::LocalizationStatus::GPS_FUSED;
    tunnel.mode=interfaces::LocalizationStatus::IMU_LIDAR;
  }
  node_pipeline::ParamReader params(const std::string& name) {
    return node_pipeline::ParamReader::fromFile(std::string(NP_PACKAGE_DIR)+"/config/control/parameters/"+name+".yaml",NP_PACKAGE_DIR);
  }
  interfaces::Path path(double velocity,bool timed=false,double acceleration=0) {
    interfaces::Path p;p.header.stamp=ego.header.stamp;p.header.frame_id="base_link";
    for(int i=0;i<=60;++i) {
      interfaces::PathPoint q;q.pose.header=p.header;q.pose.pose.orientation.w=1;
      q.pose.pose.position.x=i*.5;q.velocity=velocity;q.acceleration=acceleration;
      if(timed)q.time_from_start=ros::Duration(.1*i);
      p.points.push_back(q);
    }
    return p;
  }
  morai_msgs::CtrlCmd call(control::IntegratedMpc& core,const interfaces::Path& p,const interfaces::LocalizationStatus& mode) {
    return core.process(p,ego,vehicle,mode,ego.header.stamp);
  }
  morai_msgs::CtrlCmd call(control::MoraiMpc& core,const interfaces::Path& p,const interfaces::LocalizationStatus& mode) {
    return core.process(p,ego,vehicle,mode);
  }
  template<class Core,class Parameters>
  void equivalent(Parameters parameters,double velocity,bool timed=false,double acceleration=0) {
    Core capped(parameters),reference(parameters);
    const auto input=path(velocity,timed,acceleration);
    auto limited=input;
    for(auto& p:limited.points)if(p.velocity>=25./3.6){p.velocity=25./3.6;p.acceleration=std::min(0.,p.acceleration);}
    for(int i=0;i<5;++i) {
      ego.header.stamp=ros::Time(100+.03*i);
      auto a=input,b=limited;a.header.stamp=b.header.stamp=ego.header.stamp;
      for(auto& p:a.points)p.pose.header=a.header;
      for(auto& p:b.points)p.pose.header=b.header;
      const auto actual=call(capped,a,tunnel),expected=call(reference,b,fused);
      EXPECT_NEAR(actual.accel,expected.accel,1e-6);
      EXPECT_NEAR(actual.brake,expected.brake,1e-6);
      EXPECT_NEAR(actual.steering,expected.steering,1e-6);
      EXPECT_EQ(actual.longlCmdType,expected.longlCmdType);
    }
  }
};
}
TEST(TunnelSpeedControl, IntegratedSpatialReferenceMatchesThe25KphCeiling) {
  Fixture f;auto p=control::loadIntegratedMpcParameters(f.params("integrated_mpc"));
  EXPECT_DOUBLE_EQ(p.tunnel_speed_mps,25./3.6);
  f.equivalent<control::IntegratedMpc>(p,59./3.6);
}
TEST(TunnelSpeedControl, IntegratedTimedReferenceSuppressesAccelerationAboveTheCeiling) {
  Fixture f;f.equivalent<control::IntegratedMpc>(control::loadIntegratedMpcParameters(f.params("integrated_mpc")),59./3.6,true,1.5);
}
TEST(TunnelSpeedControl, MoraiSpatialReferenceMatchesThe25KphCeiling) {
  Fixture f;auto p=control::loadMoraiMpcParameters(f.params("morai_mpc"));
  EXPECT_DOUBLE_EQ(p.tunnel_speed_mps,25./3.6);
  f.equivalent<control::MoraiMpc>(p,59./3.6);
}
TEST(TunnelSpeedControl, LowerTargetsAndStopsArePreservedInBothControllers) {
  for(double speed:{5.,0.}) {
    Fixture f;
    if(speed>0)f.equivalent<control::IntegratedMpc>(control::loadIntegratedMpcParameters(f.params("integrated_mpc")),speed);
    f.equivalent<control::MoraiMpc>(control::loadMoraiMpcParameters(f.params("morai_mpc")),speed);
  }
}
TEST(TunnelSpeedControl, ModeTransitionsChangeLongitudinalDemandAndRestoreTheNormalTarget) {
  Fixture f;control::IntegratedMpc core(control::loadIntegratedMpcParameters(f.params("integrated_mpc")));
  morai_msgs::CtrlCmd command;
  for(int phase=0;phase<3;++phase) {
    for(int i=0;i<45;++i) {
      f.ego.header.stamp=ros::Time(100+.03*(phase*45+i));
      auto path=f.path(59./3.6);
      command=f.call(core,path,phase==1?f.tunnel:f.fused);
    }
    if(phase==1){EXPECT_EQ(command.accel,0);EXPECT_GT(command.brake,0);}
    else{EXPECT_GT(command.accel,0);EXPECT_EQ(command.brake,0);}
  }
}

TEST(TunnelSpeedControl, GpsModeRetainsExistingDynamicZoneAccelerationReference) {
  Fixture f;f.ego.pose.pose.position.x=-60;f.ego.pose.pose.position.y=0;
  auto parameters=control::loadIntegratedMpcParameters(f.params("integrated_mpc"));
  parameters.dynamic_zone_enabled=true;
  control::IntegratedMpc actual(parameters);
  parameters.dynamic_zone_enabled=false;
  control::IntegratedMpc reference(parameters);
  auto input=f.path(59./3.6,true,1.5);
  auto limited=f.path(25./3.6,true,1.5);
  const auto a=f.call(actual,input,f.fused),b=f.call(reference,limited,f.fused);
  EXPECT_NEAR(a.accel,b.accel,1e-6);
  EXPECT_NEAR(a.brake,b.brake,1e-6);
  EXPECT_NEAR(a.steering,b.steering,1e-6);
}

TEST(TunnelSpeedControl, TimedReferenceAtTheCeilingDoesNotRequestPositiveAcceleration) {
  Fixture f;f.equivalent<control::IntegratedMpc>(control::loadIntegratedMpcParameters(f.params("integrated_mpc")),25./3.6,true,1.5);
}

TEST(TunnelSpeedControl, TunnelZeroSpeedPathCommandsFullBrakeUntilBypassIsAvailable) {
  Fixture f;
  control::IntegratedMpc core(control::loadIntegratedMpcParameters(f.params("integrated_mpc")));
  for(double speed:{7.,1.,0.}) {
    f.vehicle.velocity_x_mps=speed;f.ego.header.stamp+=ros::Duration(.03);
    const auto command=f.call(core,f.path(0,true),f.tunnel);
    EXPECT_DOUBLE_EQ(command.accel,0);
    EXPECT_DOUBLE_EQ(command.brake,1);
  }
  morai_msgs::CtrlCmd command;
  for(int i=0;i<30;++i) {
    f.ego.header.stamp+=ros::Duration(.03);
    command=f.call(core,f.path(8./3.6),f.tunnel);
  }
  EXPECT_GT(command.accel,0);
  EXPECT_DOUBLE_EQ(command.brake,0);
}
TEST(TunnelSpeedControl, TunnelFutureStopWithPositiveApproachSpeedDoesNotForceFullBrake) {
  Fixture f;
  control::IntegratedMpc core(control::loadIntegratedMpcParameters(f.params("integrated_mpc")));
  auto path=f.path(7.);path.points.back().velocity=0;
  const auto command=f.call(core,path,f.tunnel);
  EXPECT_LT(command.brake,1);
}
