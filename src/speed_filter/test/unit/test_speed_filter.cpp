#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/cores/no_speed_limit.hpp>
#include <node_pipeline/detail/message_traits.hpp>
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

TEST(SpeedFilter, QueueOneConnectsLatestStatusToBothRegisteredCores) {
  namespace np = node_pipeline;
  namespace sf = speed_filter;
  namespace nd=np::detail;
  nd::Engine engine(1,1);
  nd::InputSpec command_spec("command","/control_cmd"); command_spec.limits.check_stamp=false;
  command_spec.limits.check_skew=false;
  nd::InputSpec status_spec("competition","/vehicle/competition_status");status_spec.limits.frame="base_link";
  engine.input<morai_msgs::CtrlCmd>(command_spec); engine.input<interfaces::CompetitionVehicleStatus>(status_spec);
  nd::CoreSpec limited;limited.name="speed_limit60.hpp";limited.stage="stage1";
  limited.inputs={"command","competition"};limited.outputs={{"command","command"}};
  auto unlimited=limited;unlimited.name="no_speed_limit.hpp";
  engine.core<sf::SpeedLimit60>(limited,{});engine.core<sf::NoSpeedLimit>(unlimited,{});
  nd::StageSpec stage;stage.name="stage1";stage.section_core_map[0]=unlimited.name;
  stage.section_core_map[1]=limited.name;engine.stage(stage);
  EXPECT_EQ(np::ProcessStatus::Rejected,engine.process({10000000000,1}).status);
  EXPECT_TRUE(engine.output().empty());
  interfaces::CompetitionVehicleStatus status;
  status.header.frame_id="base_link";status.header.stamp.fromSec(10);
  status.signed_velocity_mps=50.0/3.6;
  engine.push<interfaces::CompetitionVehicleStatus>("competition",std::make_shared<const interfaces::CompetitionVehicleStatus>(status),{10000000000,1},10000000000);
  status.header.stamp.fromSec(10.04);status.signed_velocity_mps=65.0/3.6;
  const auto latest=engine.push<interfaces::CompetitionVehicleStatus>("competition",std::make_shared<const interfaces::CompetitionVehicleStatus>(status),{10040000000,2},10040000000);
  morai_msgs::CtrlCmd command;command.accel=.4;command.steering=.25;
  const auto id=engine.push<morai_msgs::CtrlCmd>("command",std::make_shared<const morai_msgs::CtrlCmd>(command),{10040000000,3},0,false);
  ASSERT_EQ(np::ProcessStatus::Published,engine.process({10040000000,3},false,"command",id).status);
  EXPECT_EQ(0,engine.assembled().at("command").stamp_ns);
  EXPECT_FALSE(engine.assembled().at("command").has_stamp);
  EXPECT_EQ(latest,engine.assembled().at("competition").id);
  expectCommand(command,engine.result<morai_msgs::CtrlCmd>("command"));
  EXPECT_EQ(id,engine.output().at("command").id);
  EXPECT_EQ(3,engine.output().at("command").receipt_ns);
  engine.section(np::Section{true,1,10040000000,"map"});
  const auto next=engine.push<morai_msgs::CtrlCmd>("command",std::make_shared<const morai_msgs::CtrlCmd>(command),{10040000000,4},0,false);
  ASSERT_EQ(np::ProcessStatus::Published,engine.process({10040000000,4},false,"command",next).status);
  auto expected=command;expected.accel=0;expected.brake=1;
  expectCommand(expected,engine.result<morai_msgs::CtrlCmd>("command"));

}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
