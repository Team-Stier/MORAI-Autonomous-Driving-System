#include <control/cores/morai_mpc.hpp>
#include <control/parameters/morai_mpc.hpp>
#include <gtest/gtest.h>
namespace {
using Mpc=control::autoware_ai_mpc::LongitudinalMpc;
using Ref=control::autoware_ai_mpc::LongitudinalReference;
auto testConfig() {
  ros::Time::init();const std::string pkg="/home/stier/morai-path-planner-recovery-20261018/workspace/src/control";
  return control::loadMoraiMpcParameters(node_pipeline::ParamReader::fromFile(
      pkg+"/config/control/parameters/morai_mpc.yaml",pkg)).longitudinal;
}
TEST(MoraiMpcStandstill, ZeroReferenceAndNearZeroSpeedHoldWithoutAccelerating) {
  const auto cfg=testConfig();
  for(double speed:{0.0,.01,.03,.05}) {
    SCOPED_TRACE(speed);Mpc mpc(cfg);
    const auto out=mpc.updateReference(std::vector<Ref>(cfg.horizon_steps,{0,0}),speed,.05);
    EXPECT_DOUBLE_EQ(out.accel,0);EXPECT_GE(out.brake,.05);
  }
}
TEST(MoraiMpcStandstill, RealDepartureReferenceReleasesHoldAndAccelerates) {
  const auto cfg=testConfig();Mpc mpc(cfg);
  for(int k=0;k<5;++k) mpc.updateReference(std::vector<Ref>(cfg.horizon_steps,{0,0}),0,.05);
  const auto out=mpc.updateReference(std::vector<Ref>(cfg.horizon_steps,{1,0}),0,.05);
  EXPECT_GT(out.accel,.01);EXPECT_DOUBLE_EQ(out.brake,0);
}
TEST(MoraiMpcStandstill, HoldRespectsConfiguredBrakeAndCommandRateLimits) {
  auto cfg=testConfig();cfg.maximum_brake_command=.03;cfg.command_rate_limit_per_sec=.2;
  Mpc mpc(cfg);double previous=0;
  for(int k=0;k<10;++k) {
    const auto out=mpc.updateReference(std::vector<Ref>(cfg.horizon_steps,{0,0}),0,.05);
    EXPECT_LE(std::abs(out.signed_command-previous),.010000001);
    EXPECT_LE(out.brake,.03);EXPECT_DOUBLE_EQ(out.accel,0);
    previous=out.signed_command;
  }
  EXPECT_NEAR(previous,-.03,1e-9);
}
}

#include <control/parameters/morai_mpc.hpp>
#include <gtest/gtest.h>
namespace {
auto measuredDeceleration(double speed,double acceleration) {
 ros::Time::init();const std::string root=NP_PACKAGE_DIR;
 const auto cfg=control::loadMoraiMpcParameters(node_pipeline::ParamReader::fromFile(root+"/config/control/parameters/morai_mpc.yaml",root)).longitudinal;
 control::autoware_ai_mpc::LongitudinalMpc core(cfg);control::autoware_ai_mpc::LongitudinalMpcResult out;
 for(int i=0;i<=30;++i) {
  const double v=speed-acceleration*.05*(30-i);
  std::vector<control::autoware_ai_mpc::LongitudinalReference> refs;
  for(int k=1;k<=cfg.horizon_steps;++k) refs.push_back({std::max(0.,v+acceleration*k*cfg.prediction_dt_sec),acceleration});
  out=core.updateReference(refs,v,.05);
 }
 return out;
}
TEST(MoraiMpcBrakeCalibration, HighSpeedResponseMatchesTheMeasuredTenthBrake) {
 const auto out=measuredDeceleration(7.35,-1.215);
 EXPECT_NEAR(out.brake,.1,.02);EXPECT_DOUBLE_EQ(out.accel,0);EXPECT_TRUE(std::isfinite(out.cost));
}
TEST(MoraiMpcBrakeCalibration, LowSpeedResponseKeepsTheExistingTenthBrake) {
 const auto out=measuredDeceleration(4.65,-2.151);
 EXPECT_NEAR(out.brake,.1,.02);EXPECT_DOUBLE_EQ(out.accel,0);EXPECT_TRUE(std::isfinite(out.cost));
}
}

namespace {
TEST(MoraiMpcBrakeCalibration, HighSpeedStrongBrakeMatchesSeparateMeasuredResponse) {
 const auto out=measuredDeceleration(7.0,-4.55);
 EXPECT_NEAR(out.brake,.3,.035);EXPECT_DOUBLE_EQ(out.accel,0);EXPECT_TRUE(std::isfinite(out.cost));
}
TEST(MoraiMpcBrakeCalibration, HighSpeedFifthBrakeMatchesSeparateMeasuredResponse) {
 const auto out=measuredDeceleration(6.5,-2.10);
 EXPECT_NEAR(out.brake,.2,.03);EXPECT_DOUBLE_EQ(out.accel,0);
}
}

namespace {
auto speedBrakingParameters() {
 ros::Time::init();const std::string root=NP_PACKAGE_DIR;
 return control::loadMoraiMpcParameters(node_pipeline::ParamReader::fromFile(root+"/config/control/parameters/morai_mpc.yaml",root));
}
TEST(MoraiMpcBrakeCalibration, ActivatedModelPreservesStandstillAndDeparture) {
 auto cfg=speedBrakingParameters().longitudinal;
 for(double v:{0.,.01,.03,.05}){
  Mpc enabled(cfg);auto old=cfg;old.high_speed_braking_gain_mps2=0;Mpc disabled(old);
  std::vector<Ref> zero(cfg.horizon_steps,{0,0});const auto a=enabled.updateReference(zero,v,.05),b=disabled.updateReference(zero,v,.05);
  EXPECT_DOUBLE_EQ(a.accel,0);EXPECT_GE(a.brake,.05);EXPECT_DOUBLE_EQ(a.signed_command,b.signed_command);
 }
 Mpc enabled(cfg);auto old=cfg;old.high_speed_braking_gain_mps2=0;Mpc disabled(old);
 for(int i=0;i<5;++i){std::vector<Ref> zero(cfg.horizon_steps,{0,0});enabled.updateReference(zero,0,.05);disabled.updateReference(zero,0,.05);}
 std::vector<Ref> start(cfg.horizon_steps,{1,0});const auto a=enabled.updateReference(start,0,.05),b=disabled.updateReference(start,0,.05);
 EXPECT_GT(a.accel,.01);EXPECT_DOUBLE_EQ(a.brake,0);EXPECT_DOUBLE_EQ(a.signed_command,b.signed_command);
}
TEST(MoraiMpcBrakeCalibration, ActivatedModelKeepsPedalAndRateBoundsForEverySolver) {
 for(const auto& solver:{"osqp","coordinate_search","projected_gradient"}){
  auto cfg=speedBrakingParameters().longitudinal;cfg.solver=solver;cfg.command_rate_limit_per_sec=.5;Mpc core(cfg);double prev=0;
  for(int i=0;i<12;++i){std::vector<Ref> refs(cfg.horizon_steps,i<6?Ref{20,2}:Ref{0,-5});const auto q=core.updateReference(refs,7.35,.05);
   EXPECT_TRUE(std::isfinite(q.cost));EXPECT_LE(q.accel,.3);EXPECT_LE(q.brake,1);EXPECT_DOUBLE_EQ(q.accel*q.brake,0);EXPECT_LE(std::abs(q.signed_command-prev),.025+1e-12);prev=q.signed_command;
  }
 }
}
TEST(MoraiMpcBrakeCalibration, RejectsNonfiniteGainAndZeroWidthTransition) {
 auto p=speedBrakingParameters();p.longitudinal.high_speed_braking_gain_mps2=std::numeric_limits<double>::quiet_NaN();EXPECT_THROW(control::autoware_ai_mpc::validateParameters(p),std::invalid_argument);
 p=speedBrakingParameters();p.longitudinal.braking_gain_transition_high_mps=p.longitudinal.braking_gain_transition_low_mps;EXPECT_THROW(control::autoware_ai_mpc::validateParameters(p),std::invalid_argument);
}
}

namespace {
TEST(MoraiMpcBrakeCalibration, RejectsInvalidStrongBrakeInterpolation) {
 auto p=speedBrakingParameters();p.longitudinal.strong_brake_transition_high=p.longitudinal.strong_brake_transition_low;EXPECT_THROW(control::autoware_ai_mpc::validateParameters(p),std::invalid_argument);
 p=speedBrakingParameters();p.longitudinal.strong_brake_transition_high=1.01;EXPECT_THROW(control::autoware_ai_mpc::validateParameters(p),std::invalid_argument);
}
}

namespace {
TEST(MoraiMpcBrakeCalibration, BelowCalibratedRangeMatchesOriginalCommandsExactly) {
 for(double speed:{3.,4.,5.,5.5}) {
  auto p=speedBrakingParameters().longitudinal;Mpc enabled(p);p.high_speed_braking_gain_mps2=0;Mpc disabled(p);
  for(int k=0;k<10;++k){std::vector<Ref> refs(p.horizon_steps,{speed-1,-1});const auto a=enabled.updateReference(refs,speed,.05),b=disabled.updateReference(refs,speed,.05);EXPECT_DOUBLE_EQ(a.signed_command,b.signed_command);EXPECT_DOUBLE_EQ(a.cost,b.cost);}
 }
}
}
