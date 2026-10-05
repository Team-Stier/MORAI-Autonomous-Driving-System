#include <speed_filter/components.hpp>
#include <gtest/gtest.h>

namespace {
struct Access {
  template<class Message> static std::int64_t stamp(const Message& message) { return message.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& message) { return message.header.frame_id; }
  static std::int64_t stamp(const morai_msgs::CtrlCmd&) { return 0; }
  static std::string frame(const morai_msgs::CtrlCmd&) { return ""; }
};
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
  using Components = sf::Components<Access>;
  Components::Buffer buffer(1);
  Components::Assembler assembler;
  sf::CoreSet cores{sf::SpeedLimit60{}, sf::NoSpeedLimit{}};
  np::StageData missing;
  missing.update(sf::Data{});
  np::CoreResult result;
  int attempts = 0;
  const auto before = [&] { ++attempts; };
  for (const auto id : {sf::CoreId::SpeedLimit60, sf::CoreId::NoSpeedLimit})
    EXPECT_FALSE(sf::processCore(cores, id, missing, result, before));
  EXPECT_EQ(0, attempts);

  interfaces::CompetitionVehicleStatus status;
  status.header.stamp.fromSec(10);
  status.signed_velocity_mps = 50.0 / 3.6;
  buffer.pushInput(sf::InputTag<sf::Input::Competition>{}, status, np::Clocks{10000000000, 1});
  status.header.stamp.fromSec(10.04);
  status.signed_velocity_mps = 65.0 / 3.6;
  const auto latest = buffer.pushInput(sf::InputTag<sf::Input::Competition>{}, status, np::Clocks{10040000000, 2});
  morai_msgs::CtrlCmd command;
  command.accel = 0.4;
  command.steering = 0.25;
  const auto input = buffer.pushInput(sf::InputTag<sf::Input::Command>{}, command, np::Clocks{10040000000, 3});
  const auto snapshot = assembler.assemble(buffer,
    np::Trigger<sf::Input>{np::ExecutionMode::OnInput, sf::Input::Command, input.id, input.stamp_ns});
  EXPECT_EQ(0, snapshot.data.get<0>().sample.stamp_ns);
  EXPECT_EQ(latest.id, snapshot.data.get<1>().sample.id);
  EXPECT_EQ(1, snapshot.context.evictions[1]);
  np::StageData data;
  data.update(snapshot.data);
  ASSERT_TRUE(sf::processCore(cores, sf::CoreId::NoSpeedLimit, data, result, before));
  const auto unlimited = result(data);
  expectCommand(command, unlimited.message<morai_msgs::CtrlCmd>(0));
  EXPECT_EQ(input.id, unlimited.inputs[0].id);
  EXPECT_EQ(input.receipt_steady_ns, unlimited.inputs[0].receipt_steady_ns);
  auto expected = command;
  expected.accel = 0;
  expected.brake = 1;
  ASSERT_TRUE(sf::processCore(cores, sf::CoreId::SpeedLimit60, data, result, before));
  const auto limited = result(data);
  expectCommand(expected, limited.message<morai_msgs::CtrlCmd>(0));
  EXPECT_EQ(2, attempts);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
