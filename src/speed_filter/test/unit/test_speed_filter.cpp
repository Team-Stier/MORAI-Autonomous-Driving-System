#include <speed_filter/cores/no_speed_limit.hpp>
#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/parameters/speed_limit60.hpp>
#include <gtest/gtest.h>

namespace {
void expectCommand(const morai_msgs::CtrlCmd& expected, const morai_msgs::CtrlCmd& actual) {
  EXPECT_EQ(expected.longlCmdType, actual.longlCmdType);
  EXPECT_DOUBLE_EQ(expected.accel, actual.accel);
  EXPECT_DOUBLE_EQ(expected.brake, actual.brake);
  EXPECT_DOUBLE_EQ(expected.steering, actual.steering);
  EXPECT_DOUBLE_EQ(expected.velocity, actual.velocity);
  EXPECT_DOUBLE_EQ(expected.acceleration, actual.acceleration);
}

template<class Core> void checkFilter() {
  speed_filter::SpeedFilterParameters parameters{72.0, 2.0, 1.0, 0.3, 0.1};
  Core core(parameters);
  morai_msgs::CtrlCmd command;
  command.longlCmdType = 7;
  command.accel = 0.4;
  command.brake = 0.5;
  command.steering = -0.25;
  command.velocity = 50;
  command.acceleration = 1.5;
  interfaces::CompetitionVehicleStatus status;
  auto process = [&](double stamp_sec, float speed_mps) {
    status.header.stamp.fromSec(stamp_sec);
    status.signed_velocity_mps = speed_mps;
    return core.process(command, status);
  };
  expectCommand(command, process(10, 2));
  expectCommand(command, process(11, 20));
  auto expected = command;
  expected.accel = 0;
  expected.brake = 1;
  expectCommand(expected, process(12, 20.1));
  expectCommand(expected, process(13, -20.1));

  command.accel = 0;
  command.brake = 0;
  expectCommand(command, process(20, 0));
  expectCommand(command, process(21.9, -0.05));
  expected = command;
  expected.accel = 0.3;
  expected.brake = 0;
  expected.steering = 0;
  expectCommand(expected, process(22, 0));
  expectCommand(expected, process(22.2, 1));
  expectCommand(command, process(23, 0));
  expectCommand(command, process(24.9, 0));
  expectCommand(expected, process(25, 0));

  command.accel = 0.2;
  expectCommand(command, process(25.1, 0));
  command.accel = 0;
  expectCommand(command, process(25.2, 0));
  expectCommand(command, process(27.1, 0));
  expectCommand(expected, process(27.3, 0));
  auto braking = command;
  braking.brake = 1;
  expectCommand(braking, process(27.4, 21));
  expectCommand(command, process(28, 0));
  expectCommand(command, process(29, 0.2));
  expectCommand(command, process(30, 0));
  expectCommand(command, process(31.9, 0));
  expectCommand(expected, process(32, 0));
  expectCommand(command, process(1, 0));
  expectCommand(command, process(2.9, 0));
  expectCommand(expected, process(3, 0));
}
}  // namespace

TEST(SpeedFilter, BothCoresLimitAndRecoverWithIndependentParameters) {
  checkFilter<speed_filter::SpeedLimit60>();
  checkFilter<speed_filter::NoSpeedLimit>();
  speed_filter::NoSpeedLimit unlimited;
  speed_filter::SpeedLimit60 limited;
  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromSec(10);
  status.signed_velocity_mps = 30;
  morai_msgs::CtrlCmd command;
  command.accel = 0.4;
  command.steering = 0.25;
  expectCommand(command, unlimited.process(command, status));
  command.accel = 0;
  command.brake = 1;
  expectCommand(command, limited.process(command, status));
}

TEST(SpeedFilter, LoaderReadsAllParametersAndRejectsInvalidAccel) {
  const auto parameters = speed_filter::loadSpeedLimit60Parameters(node_pipeline::ParamReader::fromString(
      "speed_limit_kph: 60.0\nstop_tolerance_sec: 30.0\nstart_duration_sec: 1.0\nstart_accel: 0.2\nstop_speed_mps: 0.1\n"));
  EXPECT_DOUBLE_EQ(60.0, parameters.speed_limit_kph);
  EXPECT_DOUBLE_EQ(30.0, parameters.stop_tolerance_sec);
  EXPECT_THROW(speed_filter::loadSpeedLimit60Parameters(node_pipeline::ParamReader::fromString(
      "speed_limit_kph: 60.0\nstop_tolerance_sec: 30.0\nstart_duration_sec: 1.0\nstart_accel: 1.5\nstop_speed_mps: 0.1\n")),
      std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SpeedFilter, ExplicitBrakeCannotBeReleasedByTimedDepartureAssist) {
  for (double brake : {0.1, 1.0}) {
    speed_filter::SpeedLimit60 core;
    interfaces::CompetitionVehicleStatus status;
    morai_msgs::CtrlCmd command;
    command.brake = brake;
    for (double t : {100.0, 131.0, 161.0, 200.0}) {
      status.header.stamp.fromSec(t);
      const auto output = core.process(command, status);
      EXPECT_DOUBLE_EQ(output.accel, 0);
      EXPECT_DOUBLE_EQ(output.brake, brake);
    }
  }
}

TEST(SpeedFilter, BrakeInterruptsActiveAssistAndRequiresNewStopInterval) {
  speed_filter::SpeedFilterParameters parameters;parameters.stop_tolerance_sec=30;
  speed_filter::SpeedLimit60 core(parameters);
  interfaces::CompetitionVehicleStatus status;morai_msgs::CtrlCmd command;
  const auto sample=[&](double t){status.header.stamp.fromSec(t);return core.process(command,status);};
  EXPECT_DOUBLE_EQ(sample(100).accel,0);
  EXPECT_GT(sample(131).accel,0);
  command.brake=1;
  const auto stop=sample(131.1);
  EXPECT_DOUBLE_EQ(stop.accel,0);EXPECT_DOUBLE_EQ(stop.brake,1);
  command.brake=0;
  EXPECT_DOUBLE_EQ(sample(132).accel,0);
  EXPECT_DOUBLE_EQ(sample(161.9).accel,0);
  EXPECT_GT(sample(162).accel,0);
}

TEST(SpeedFilter, OfficialHighwayExceptionPreservesCommandAtTwentyMetresPerSecond) {
 speed_filter::SpeedLimit60Parameters parameters;
 parameters.speed_exemption_link_ids={"A2256W000411","A2256W000153"};
 speed_filter::SpeedLimit60 core(parameters);interfaces::CompetitionVehicleStatus status;
 status.header.stamp=ros::Time(10);status.signed_velocity_mps=20.0136;status.link_id="A2256W000411";
 morai_msgs::CtrlCmd command;command.accel=.1;command.steering=.05;
 expectCommand(command,core.process(command,status));
}

TEST(SpeedFilter, OfficialExceptionKeepsExplicitBrakeAndEndsAtTheNextLink) {
 speed_filter::SpeedLimit60Parameters parameters;
 parameters.speed_exemption_link_ids={"A2256W000411","A2256W000153"};
 speed_filter::SpeedLimit60 core(parameters);interfaces::CompetitionVehicleStatus status;
 status.header.stamp=ros::Time(10);status.signed_velocity_mps=20;
 morai_msgs::CtrlCmd command;command.brake=.4;command.steering=-.1;
 for(const std::string id:{"A2256W000411","A2256W000153"}) {
  status.link_id=id;expectCommand(command,core.process(command,status));
 }
 status.link_id="A2256W000451";auto expected=command;expected.brake=1;
 expectCommand(expected,core.process(command,status));
 status.link_id="";expectCommand(expected,core.process(command,status));
 status.signed_velocity_mps=16;expectCommand(command,core.process(command,status));
}
TEST(SpeedFilter, LoadsOfficialExceptionLinksAndDoesNotExemptEntryApproachOrExitRoad) {
 const std::string package=NP_PACKAGE_DIR;
 const auto p=speed_filter::loadSpeedLimit60Parameters(node_pipeline::ParamReader::fromFile(
     package+"/config/speed_filter/parameters/speed_limit60.yaml",package));
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000411"),1u);
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000153"),1u);
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000846"),0u);
 EXPECT_EQ(p.speed_exemption_link_ids.count("A2256W000451"),0u);
}
