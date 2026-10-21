#include <control/cores/morai_mpc.hpp>
#include <gtest/gtest.h>
#include <control/validators/morai_mpc_inputs.hpp>
#include <ros/serialization.h>
#include <vector>
#include <array>

namespace {
using Mpc = control::autoware_ai_mpc::LongitudinalMpc;
using Config = control::autoware_ai_mpc::LongitudinalMpcConfig;
using Reference = control::autoware_ai_mpc::LongitudinalReference;

Config productionConfig() {
  Config c;
  c.horizon_steps = 6;
  c.prediction_dt_sec = 0.05;
  c.maximum_accel_command = 0.15;
  c.maximum_brake_command = 1;
  c.command_rate_limit_per_sec = 20;
  c.acceleration_gain_mps2 = 5.74;
  c.braking_gain_mps2 = 18.76;
  c.actuator_time_constant_sec = 0.30;
  c.physical_rollout = true;
  c.constant_drag_mps2 = 0.0408;
  c.quadratic_drag_per_m = 0.007326;
  c.maximum_acceleration_mps2 = 6;
  c.maximum_deceleration_mps2 = 15;
  c.speed_weight = 80;
  c.acceleration_weight = 0.20;
  c.jerk_weight = 0.08;
  c.command_rate_weight = 0.3;
  c.terminal_speed_weight = 240;
  return c;
}

auto follow(Mpc& mpc, const std::vector<Reference>& references, double speed) {
  return mpc.updateReference(references, speed, 0.05);
}

TEST(MoraiMpcReference, FutureDepartureRampRaisesCommandWithoutSpeedOffset) {
  Mpc horizon(productionConfig()), constant(productionConfig());
  // Jerk 4 m/s^3: v=2*t^2 and a=4*t at the predicted state times.
  const std::vector<Reference> ramp{{0.005,0.2},{0.02,0.4},{0.045,0.6},
                                   {0.08,0.8},{0.125,1.0},{0.18,1.2}};
  const auto result = follow(horizon, ramp, 0);
  const auto zero = constant.update(0.005,0,0.05);
  EXPECT_GT(result.accel, zero.accel + 0.01);
  EXPECT_DOUBLE_EQ(result.brake, 0);
}

TEST(MoraiMpcReference, FutureStopBrakesEarlierThanRepeatingCurrentSpeed) {
  Mpc horizon(productionConfig()), constant(productionConfig());
  const std::vector<Reference> stop{{3,-2},{2.9,-2},{2.8,-2},
                                   {2.7,-2},{2.6,-2},{2.5,-2}};
  const auto result = follow(horizon, stop, 3);
  EXPECT_LT(result.signed_command, constant.update(3,3,0.05).signed_command);
  EXPECT_GT(result.brake, 0);
}

TEST(MoraiMpcReference, TerminalReferenceIsNotTheFirstReference) {
  Mpc rising(productionConfig()), flat(productionConfig());
  const std::vector<Reference> terminal{{0,0},{0,0},{0,0},{0,0},{0,0},{0.1,0}};
  EXPECT_GT(follow(rising,terminal,0).signed_command,
            follow(flat,std::vector<Reference>(6,{0,0}),0).signed_command);
}

TEST(MoraiMpcReference, AccelerationReferenceContributesToTrackingCost) {
  Mpc positive(productionConfig()), negative(productionConfig());
  const auto up=follow(positive,std::vector<Reference>(6,{1,2}),1);
  const auto down=follow(negative,std::vector<Reference>(6,{1,-2}),1);
  EXPECT_GT(up.signed_command, down.signed_command);
}

TEST(MoraiMpcReference, ZeroWaitAndScalarTargetsRetainExistingBehavior) {
  for (double target : {0.0, 1.0, 5.0}) {
    Mpc horizon(productionConfig()), scalar(productionConfig());
    for (double measured : {0.0,0.001,0.5,1.0,2.0}) {
      const auto result=follow(horizon,std::vector<Reference>(6,{target,0}),measured);
      const auto expected=scalar.update(target,measured,0.05);
      EXPECT_DOUBLE_EQ(result.accel, expected.accel);
      EXPECT_DOUBLE_EQ(result.brake, expected.brake);
      EXPECT_DOUBLE_EQ(result.cost, expected.cost);
    }
  }
}

TEST(MoraiMpcReference, HorizonKeepsPedalAndRateLimitsForEverySolver) {
  for (const auto& solver : {"osqp","coordinate_search","projected_gradient"}) {
    auto config=productionConfig();
    config.solver=solver;
    config.command_rate_limit_per_sec=0.5;
    Mpc mpc(config);
    double previous=0;
    for (int i=0;i<12;++i) {
      const auto reference=i<6 ? Reference{25,5} : Reference{0,-5};
      const auto result=follow(mpc,std::vector<Reference>(6,reference),2);
      EXPECT_LE(result.accel,0.15);
      EXPECT_LE(result.brake,1);
      EXPECT_GE(result.accel,0);
      EXPECT_GE(result.brake,0);
      EXPECT_LE(std::abs(result.signed_command-previous),0.025+1e-12);
      EXPECT_DOUBLE_EQ(result.accel*result.brake,0);
      EXPECT_TRUE(std::isfinite(result.cost));
      previous=result.signed_command;
    }
  }
}
}  // namespace

namespace {
void setPathFrame(interfaces::Path& path,const geometry_msgs::Pose& pose,const ros::Time& stamp) {
  path.map_from_path_frame.header.frame_id="map";
  path.map_from_path_frame.header.stamp=stamp;
  path.map_from_path_frame.child_frame_id="base_link";
  path.map_from_path_frame.transform.translation.x=pose.position.x;
  path.map_from_path_frame.transform.translation.y=pose.position.y;
  path.map_from_path_frame.transform.translation.z=pose.position.z;
  path.map_from_path_frame.transform.rotation=pose.orientation;
}
void identityPathFrame(interfaces::Path& path) {
  geometry_msgs::Pose pose;pose.orientation.w=1;setPathFrame(path,pose,path.header.stamp);
}
interfaces::Path timedPath(std::initializer_list<std::array<double,3>> samples) {
  interfaces::Path path;
  path.header.frame_id="base_link";
  path.header.stamp=ros::Time(100,0);
  identityPathFrame(path);
  double x=0;
  for (const auto& sample:samples) {
    interfaces::PathPoint point;
    point.pose.header=path.header;
    point.pose.pose.orientation.w=1;
    point.pose.pose.position.x=x;
    x+=5;
    point.time_from_start=ros::Duration(sample[0]);
    point.velocity=sample[1];
    point.acceleration=sample[2];
    path.points.push_back(point);
  }
  return path;
}
control::autoware_ai_mpc::Parameters coreParameters() {
  control::autoware_ai_mpc::Parameters p;
  p.longitudinal=productionConfig();
  p.lateral.ctrl_period=0.05;
  return p;
}
auto runCore(control::MoraiMpc& core,const interfaces::Path& path,double elapsed,double speed=0) {
  ros::Time::init();
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w=1;
  ego.twist.twist.linear.x=speed;
  interfaces::CompetitionVehicleStatus status;
  status.velocity_x_mps=speed;
  const ros::Time stamp=path.header.stamp+ros::Duration(elapsed);
  ego.header.stamp=stamp;
  return core.process(path,ego,status,stamp);
}
std::vector<node_pipeline::Kind> pathIssues(const interfaces::Path& path) {
  node_pipeline::HealthLimits limits;
  limits.frame="base_link"; limits.lower=0; limits.upper=111.111111111;
  std::vector<node_pipeline::Kind> issues;
  control::MoraiMpcInputs{}(path,limits,issues);
  return issues;
}

TEST(MoraiMpcConnection, DepartureUsesFutureProfileWhileCurrentSpeedRemainsZero) {
  auto path=timedPath({{0,0,0},{0.1,0.02,0.4},{0.2,0.08,0.8},
                      {0.3,0.18,1.2},{2,2,0}});
  auto zero=path;
  for(auto& p:zero.points) { p.velocity=0; p.acceleration=0; }
  control::MoraiMpc moving(coreParameters()), stopped(coreParameters());
  EXPECT_GT(runCore(moving,path,0).accel,runCore(stopped,zero,0).accel+0.01);
}

TEST(MoraiMpcConnection, PlannedWaitCannotPreviewDepartureBeforeReleaseTime) {
  const auto path=timedPath({{0,0,0},{1,0,0},{1.1,0.02,0.4},
                            {1.2,0.08,0.8},{1.3,0.18,1.2},{3,3,0}});
  auto zero=path;
  for(auto& p:zero.points) { p.velocity=0; p.acceleration=0; }
  control::MoraiMpc waiting(coreParameters()), stopped(coreParameters()), released(coreParameters());
  const auto wait=runCore(waiting,path,0.99);
  const auto hold=runCore(stopped,zero,0.99);
  EXPECT_DOUBLE_EQ(wait.accel,hold.accel);
  EXPECT_DOUBLE_EQ(wait.brake,hold.brake);
  EXPECT_GT(runCore(released,path,1.01).accel,hold.accel+0.01);
}

TEST(MoraiMpcConnection, SamePathStampUsesAdvancingControlTime) {
  auto path=timedPath({{0,0,0},{0.1,0.02,0.4},{0.2,0.08,0.8},
                       {0.3,0.18,1.2},{2,2,0}});
  // Keep both commands below saturation so a frozen reference time is observable.
  for (auto& point:path.points) { point.velocity*=0.1; point.acceleration*=0.1; }
  control::MoraiMpc early(coreParameters()), later(coreParameters());
  EXPECT_GT(runCore(later,path,0.20).accel,runCore(early,path,0).accel);
}

TEST(MoraiMpcConnection, TimeValidatorRejectsUntimedAndReversedPaths) {
  const auto valid=timedPath({{0,0,0},{0.1,1,0},{0.4,2,0}});
  EXPECT_TRUE(pathIssues(valid).empty());
  auto legacy=valid;
  for(auto& point:legacy.points) point.time_from_start=ros::Duration(0);
  EXPECT_FALSE(pathIssues(legacy).empty());
  auto backwards=valid;
  backwards.points.back().time_from_start=ros::Duration(0.05);
  EXPECT_FALSE(pathIssues(backwards).empty());
  auto nonzero_start=valid;
  nonzero_start.points.front().time_from_start=ros::Duration(0.01);
  EXPECT_FALSE(pathIssues(nonzero_start).empty());
}

TEST(MoraiMpcConnection, MessageSerializationPreservesIrregularTimes) {
  const auto path=timedPath({{0,0,0},{0.137,1,0},{2.731,2,0}});
  std::vector<uint8_t> buffer(ros::serialization::serializationLength(path));
  ros::serialization::OStream output(buffer.data(),buffer.size());
  ros::serialization::serialize(output,path);
  interfaces::Path restored;
  ros::serialization::IStream input(buffer.data(),buffer.size());
  ros::serialization::deserialize(input,restored);
  EXPECT_EQ(restored,path);
  EXPECT_EQ(restored.points[1].time_from_start.toNSec(),137000000);
  EXPECT_EQ(restored.points[2].time_from_start.toNSec(),2731000000LL);
}

}  // namespace

namespace {
TEST(MoraiMpcConnection, IrregularTimesAreSampledAtPredictionTimes) {
  const auto path=timedPath({{0,0,0.4},{0.137,0.0548,0.4},
                            {0.373,0.1492,0.4},{0.9,0.36,0.4}});
  const auto refs=control::MoraiMpc::referenceHorizon(path,0,productionConfig());
  ASSERT_EQ(refs.size(),6u);
  for(size_t i=0;i<refs.size();++i) {
    EXPECT_NEAR(refs[i].speed_mps,0.02*(i+1),1e-12);
    EXPECT_NEAR(refs[i].acceleration_mps2,0.4,1e-12);
  }
}
TEST(MoraiMpcConnection, FutureDepartureCannotCancelAnUpcomingStop) {
  const auto path=timedPath({{0,0.2,-2},{0.1,0,0},{0.2,0,0},
                            {0.3,0.02,0.4},{0.4,0.08,0.8},{1,2,0}});
  const auto refs=control::MoraiMpc::referenceHorizon(path,0,productionConfig());
  EXPECT_GT(refs.front().speed_mps,0);
  for(size_t i=1;i<refs.size();++i) {
    EXPECT_DOUBLE_EQ(refs[i].speed_mps,0);
    EXPECT_DOUBLE_EQ(refs[i].acceleration_mps2,0);
  }
  const auto released=control::MoraiMpc::referenceHorizon(path,0.2,productionConfig());
  EXPECT_GT(released.front().speed_mps,0);
}
TEST(MoraiMpcConnection, ReferenceDoesNotExtrapolateMotionPastPlanEnd) {
  const auto path=timedPath({{0,1,0},{0.2,1,0}});
  const auto refs=control::MoraiMpc::referenceHorizon(path,0,productionConfig());
  for(size_t i=0;i<3;++i) EXPECT_DOUBLE_EQ(refs[i].speed_mps,1);
  for(size_t i=3;i<6;++i) EXPECT_DOUBLE_EQ(refs[i].speed_mps,0);
}
}  // namespace

namespace {
TEST(MoraiMpcConnection, HistoricalBodyPathMatchesCurrentBodyPathAfterMeasuredMotion) {
  ros::Time::init();
  interfaces::Path path;path.header.frame_id="base_link";path.header.stamp=ros::Time(100);identityPathFrame(path);
  for(int i=0;i<=80;++i) {
    interfaces::PathPoint point;point.pose.header=path.header;point.pose.pose.orientation.w=1;
    point.pose.pose.position.x=i*.25;point.velocity=1;point.time_from_start=ros::Duration(i*.25);
    path.points.push_back(point);
  }
  nav_msgs::Odometry ego;ego.header=path.header;ego.header.frame_id="map";
  ego.child_frame_id="base_link";ego.pose.pose.orientation.w=1;ego.twist.twist.linear.x=1;
  interfaces::CompetitionVehicleStatus status;status.velocity_x_mps=1;
  control::MoraiMpc historical(coreParameters()), current(coreParameters());
  historical.process(path,ego,status,ego.header.stamp);current.process(path,ego,status,ego.header.stamp);
  ego.header.stamp+=ros::Duration(.25);ego.pose.pose.position.x=.25;ego.pose.pose.position.y=.2;
  ego.twist.twist.linear.y=.8;
  auto rebased=path;rebased.header.stamp=ego.header.stamp;
  setPathFrame(rebased,ego.pose.pose,ego.header.stamp);
  for(auto& point:rebased.points) {
    point.pose.header=rebased.header;point.pose.pose.position.x-=.25;point.pose.pose.position.y-=.2;
  }
  const auto saved=path;
  const auto actual=historical.process(path,ego,status,ego.header.stamp);
  const auto expected=current.process(rebased,ego,status,ego.header.stamp);
  EXPECT_LT(actual.steering,-.01);
  EXPECT_NEAR(actual.steering,expected.steering,1e-10);
  EXPECT_DOUBLE_EQ(actual.accel,expected.accel);EXPECT_DOUBLE_EQ(actual.brake,expected.brake);
  EXPECT_EQ(path,saved);
}
}

namespace {
TEST(MoraiMpcConnection, HistoricalPathRotationMatchesWorldReferenceWithoutChangingProfile) {
  ros::Time::init();
  for(double yaw:{-.1,.1}) {
    interfaces::Path path;path.header.frame_id="base_link";path.header.stamp=ros::Time(100);identityPathFrame(path);
    for(int i=0;i<=80;++i) {
      interfaces::PathPoint point;point.pose.header=path.header;point.pose.pose.orientation.w=1;
      point.pose.pose.position.x=i*.25;point.velocity=1;point.time_from_start=ros::Duration(i*.25);
      path.points.push_back(point);
    }
    nav_msgs::Odometry ego;ego.header=path.header;ego.header.frame_id="map";
    ego.child_frame_id="base_link";ego.pose.pose.orientation.w=1;ego.twist.twist.linear.x=1;
    interfaces::CompetitionVehicleStatus status;status.velocity_x_mps=1;
    control::MoraiMpc historical(coreParameters()), current(coreParameters());
    historical.process(path,ego,status,ego.header.stamp);current.process(path,ego,status,ego.header.stamp);
    ego.header.stamp+=ros::Duration(.25);ego.pose.pose.position.x=.25;
    ego.pose.pose.orientation.z=std::sin(yaw/2);ego.pose.pose.orientation.w=std::cos(yaw/2);
    ego.twist.twist.angular.z=yaw/.25;
    auto rebased=path;rebased.header.stamp=ego.header.stamp;
  setPathFrame(rebased,ego.pose.pose,ego.header.stamp);
    for(auto& point:rebased.points) {
      const double x=point.pose.pose.position.x-.25;
      point.pose.header=rebased.header;
      point.pose.pose.position.x=std::cos(yaw)*x;point.pose.pose.position.y=-std::sin(yaw)*x;
      point.pose.pose.orientation.z=std::sin(-yaw/2);point.pose.pose.orientation.w=std::cos(yaw/2);
    }
    const auto actual=historical.process(path,ego,status,ego.header.stamp);
    const auto expected=current.process(rebased,ego,status,ego.header.stamp);
    EXPECT_NEAR(actual.steering,expected.steering,1e-10);
    EXPECT_DOUBLE_EQ(actual.accel,expected.accel);EXPECT_DOUBLE_EQ(actual.brake,expected.brake);
  }
}
}

namespace {
TEST(MoraiMpcConnection, LongInputGapDoesNotInterpolateAcrossAnUnobservedJourney) {
  ros::Time::init();
  interfaces::Path path;path.header.frame_id="base_link";path.header.stamp=ros::Time(100);identityPathFrame(path);
  for(int i=0;i<=80;++i) {
    interfaces::PathPoint point;point.pose.header=path.header;point.pose.pose.orientation.w=1;
    point.pose.pose.position.x=i*.25;point.velocity=1;point.time_from_start=ros::Duration(i*.25);
    path.points.push_back(point);
  }
  nav_msgs::Odometry ego;ego.header=path.header;ego.header.frame_id="map";
  ego.child_frame_id="base_link";ego.pose.pose.orientation.w=1;
  ego.pose.pose.position.y=5;ego.twist.twist.linear.x=1;
  setPathFrame(path,ego.pose.pose,ego.header.stamp);
  interfaces::CompetitionVehicleStatus status;status.velocity_x_mps=1;
  control::MoraiMpc historical(coreParameters()), current(coreParameters());
  historical.process(path,ego,status,ego.header.stamp);current.process(path,ego,status,ego.header.stamp);
  ego.header.stamp=ros::Time(105);ego.pose.pose.position.x=10;ego.pose.pose.position.y=0;
  path.header.stamp=ros::Time(104,750000000);
  auto source_pose=ego.pose.pose;source_pose.position.x-=.25;
  setPathFrame(path,source_pose,path.header.stamp);
  for(auto& point:path.points)point.pose.header=path.header;
  auto rebased=path;rebased.header.stamp=ego.header.stamp;
  setPathFrame(rebased,ego.pose.pose,ego.header.stamp);
  for(auto& point:rebased.points) {
    point.pose.header=rebased.header;point.pose.pose.position.x-=.25;
  }
  const auto actual=historical.process(path,ego,status,ego.header.stamp);
  const auto expected=current.process(rebased,ego,status,ego.header.stamp);
  EXPECT_NEAR(actual.steering,0,1e-10);
  EXPECT_NEAR(actual.steering,expected.steering,1e-10);
  EXPECT_DOUBLE_EQ(actual.accel,expected.accel);EXPECT_DOUBLE_EQ(actual.brake,expected.brake);
}
}

namespace {
TEST(MoraiMpcConnection, ActualSourceFrameDoesNotRetimestampTheSpeedProfile) {
  ros::Time::init();
  auto world=timedPath({{0,1,0},{1,1,0},{2,1,0},{3,1,0},{4,1,0}});
  nav_msgs::Odometry ego;ego.header.frame_id="map";ego.header.stamp=ros::Time(100,250000000);
  ego.child_frame_id="base_link";ego.pose.pose.position.x=.25;ego.pose.pose.position.y=.02;
  ego.pose.pose.orientation.z=std::sin(.005);ego.pose.pose.orientation.w=std::cos(.005);
  ego.twist.twist.linear.x=1;
  geometry_msgs::Pose source;source.position.x=.1;source.position.y=.008;
  source.orientation.z=std::sin(.002);source.orientation.w=std::cos(.002);
  auto actual_path=world,expected_path=world;
  setPathFrame(actual_path,source,ros::Time(100,100000000));
  setPathFrame(expected_path,ego.pose.pose,ego.header.stamp);
  tf2::Transform map_from_source,map_from_current;
  tf2::fromMsg(source,map_from_source);tf2::fromMsg(ego.pose.pose,map_from_current);
  for(std::size_t i=0;i<world.points.size();++i) {
    tf2::Transform pose;tf2::fromMsg(world.points[i].pose.pose,pose);
    tf2::toMsg(map_from_source.inverse()*pose,actual_path.points[i].pose.pose);
    tf2::toMsg(map_from_current.inverse()*pose,expected_path.points[i].pose.pose);
  }
  interfaces::CompetitionVehicleStatus status;status.velocity_x_mps=1;
  control::MoraiMpc actual(coreParameters()),expected(coreParameters());
  const auto saved=actual_path;
  const auto a=actual.process(actual_path,ego,status,ego.header.stamp);
  const auto b=expected.process(expected_path,ego,status,ego.header.stamp);
  EXPECT_NEAR(a.steering,b.steering,1e-10);
  EXPECT_DOUBLE_EQ(a.accel,b.accel);EXPECT_DOUBLE_EQ(a.brake,b.brake);
  EXPECT_EQ(actual_path,saved);
}
TEST(MoraiMpcConnection, MissingOrInvalidSourceFrameIsRejected) {
  auto path=timedPath({{0,1,0},{1,1,0}});
  EXPECT_TRUE(pathIssues(path).empty());
  path.map_from_path_frame=geometry_msgs::TransformStamped{};
  EXPECT_FALSE(pathIssues(path).empty());
}
}

TEST(MoraiMpcReference, CoupledFutureCommandsReachQuadraticMinimum) {
  using namespace control::autoware_ai_mpc;
  // 두 미래 명령의 교차항이 큰 양의 정부호 비용이다. 내부 최솟값은
  // (0.1, 0.2)이며, 대각 근사 3회는 비용 0.2273을 남겼다.
  OsqpSqpConfig config;
  config.sqp_iterations = 3;
  config.finite_difference_epsilon = 0.001;
  const auto objective = [](const std::vector<double>& command) {
    const double x = command[0] - 0.1;
    const double y = command[1] - 0.2;
    return 100.0 * x * x + 18.0 * x * y + y * y;
  };
  const auto result = solveWithOsqpSqp(objective, {0.8, -0.7},
      {1.0, 0.0, 0.0, 1.0}, 2, {-1.0, -1.0}, {1.0, 1.0}, config);
  EXPECT_LT(result.cost, 1.0e-8);
  EXPECT_NEAR(result.solution[0], 0.1, 1.0e-4);
  EXPECT_NEAR(result.solution[1], 0.2, 1.0e-4);
}
