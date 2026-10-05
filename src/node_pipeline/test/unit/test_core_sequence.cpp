#include <node_pipeline/pipeline.hpp>
#include <gtest/gtest.h>
#include <limits>
namespace np = node_pipeline;
namespace {
struct Message { double value = 0; std::int64_t stamp_ns = 1000000000; std::string frame = "map"; };
enum class Input { Primary, Auxiliary };
enum class Id { Main, Alternate };
struct Access {
  static std::int64_t stamp(const Message& m) { return m.stamp_ns; }
  static const std::string& frame(const Message& m) { return m.frame; }
  static double& value(Message& m) { return m.value; }
};
struct Validator {
  void operator()(const Message& m, const np::HealthLimits& limits, std::vector<np::Kind>& issues) const { np::checkNumber(m.value, limits, issues); }
  void operator()(int, const np::HealthLimits&, std::vector<np::Kind>&) const {}
  void operator()(double, const np::HealthLimits&, std::vector<np::Kind>&) const {}
};
using Data = np::Data<Message, Message>;
using Buffer = np::InputBuffer<Input, Access, Message, Message>;
using Context = Buffer::ContextType;
using Policy = np::InputPolicy<Input, np::RemoveInput, np::RemoveInput>;
using Selector = np::CoreSelector<Input, Id>;
struct State {
  std::vector<int> calls;
  std::vector<Message> published;
  std::vector<np::StageHealth> health;
  std::size_t headers = 0, clocks = 0;
};
struct IO {
  using MessageType = Message;
  std::shared_ptr<State> state;
  np::Clocks readClocks() const { ++state->clocks; return {1000000000, 1000000000}; }
  void publish(const Message& message) { state->published.push_back(message); }
  void reportHealth(const np::StageHealth& health, std::size_t) const { state->health.push_back(health); }
  void report(const np::ProcessReport&) const noexcept {}
};
struct Header {
  std::shared_ptr<State> state;
  void injectHeader(Message& message, const Context& context) const {
    ++state->headers; message.stamp_ns = context.stamp_ns; message.frame = context.frame;
  }
};
struct First {
  std::shared_ptr<State> state;
  std::pair<Message,int> process(const Message& a, const Message& b) {
    state->calls.push_back(1); return {{a.value+b.value,0,""},2};
  }
};
struct Middle {
  std::shared_ptr<State> state;
  std::tuple<Message,int,double> process(const Message& m,int scale) {
    state->calls.push_back(2); return {m,scale,0.5};
  }
};
struct Last {
  std::shared_ptr<State> state;
  Message process(const Message& m,int scale,double gain) {
    state->calls.push_back(3); return {m.value*scale*gain,0,""};
  }
};
struct Alternate {
  std::unique_ptr<int> updates{new int(0)};
  std::tuple<Message,int,std::string> process(const Message& message) {
    return {Message{message.value + ++*updates,0,""},1,"additional output"};
  }
};
struct Cores {
  First first;
  Middle middle;
  Last last;
  Alternate alternate;
  explicit Cores(std::shared_ptr<State> state) : first{state},middle{state},last{state} {}
};
using Node = np::PipelineNode<Buffer, np::SnapshotAssembler<Buffer, Access>, Cores, Header, IO>;
bool first(Cores& cores, Id id, const np::StageData& data, np::CoreResult& result, const np::BeforeCore& before) {
  switch (id) {
    case Id::Main:
      if (!data.hasInputs({0, 1})) return false;
      before(); result = np::wrapResult(cores.first.process(data.message<Message>(0), data.message<Message>(1))); return true;
    case Id::Alternate:
      if (!data.hasInputs({0})) return false;
      before(); result = np::wrapResult(cores.alternate.process(data.message<Message>(0))); return true;
  }
  throw std::invalid_argument("unknown core");
}
bool middle(Cores& cores, Id, const np::StageData& data, np::CoreResult& result, const np::BeforeCore& before) {
  if (!data.hasInputs({0, 1})) return false;
  before(); result = np::wrapResult(cores.middle.process(data.message<Message>(0), data.message<int>(1)), {0, 1, -1}); return true;
}
bool last(Cores& cores, Id, const np::StageData& data, np::CoreResult& result, const np::BeforeCore& before) {
  if (!data.hasInputs({0, 1, 2})) return false;
  before(); result = np::wrapResult(cores.last.process(data.message<Message>(0), data.message<int>(1), data.message<double>(2))); return true;
}
template<class D, std::size_t N, class Dispatch>
np::Stage<Cores, Context> stage(Dispatch dispatch, Selector selector, np::HealthLimits limits = {}) {
  return np::makeStage<Cores, Context, D>(np::HealthChecker<Input, N, Validator>(limits, 1000000000),
    Policy{{limits.upper}, {limits.lower}}, selector, dispatch);
}
Selector selected() { Selector selector; selector.section_core_map[0] = Id::Main; return selector; }
std::vector<np::Stage<Cores, Context>> stages(Selector first_selector, Selector second_selector, Selector last_selector, np::HealthLimits limits) {
  return {stage<Data, 2>(first, first_selector, limits),
    stage<np::Data<Message, int>, 2>(middle, second_selector, limits),
    stage<np::Data<Message, int, double>, 3>(last, last_selector, limits)};
}
TEST(CoreSequence, ThreeStagesCheckTheirOwnInputsAndPublishOnce) {
  auto state = std::make_shared<State>(); np::HealthLimits limits; limits.upper = 100;
  Node node({np::ExecutionMode::OnInput, Input::Primary, 10}, Buffer(8), {},
    stages(selected(), selected(), selected(), limits), Cores{state}, Header{state}, IO{state});
  EXPECT_EQ(np::ProcessStatus::NoCore, node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{4}).status);
  EXPECT_TRUE(state->calls.empty());
  node.onInput(std::integral_constant<Input, Input::Auxiliary>{}, Message{3});
  state->health.clear(); const auto clocks = state->clocks;
  EXPECT_EQ(np::ProcessStatus::Published, node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{4}).status);
  EXPECT_EQ(clocks + 2, state->clocks); // one receive clock, one shared health clock
  EXPECT_EQ((std::vector<int>{1,2,3}), state->calls);
  ASSERT_EQ(3u, state->health.size());
  EXPECT_EQ(2u, state->health[0].inputs.size()); EXPECT_EQ(2u, state->health[1].inputs.size()); EXPECT_EQ(3u, state->health[2].inputs.size());
  for (const auto& health : state->health) for (const auto& input : health.inputs) EXPECT_TRUE(input.green());
  ASSERT_EQ(1u, state->published.size()); EXPECT_DOUBLE_EQ(7, state->published[0].value);
  EXPECT_EQ(1000000000, state->published[0].stamp_ns); EXPECT_EQ("map", state->published[0].frame); EXPECT_EQ(1u, state->headers);
}
TEST(CoreSequence, FailureStopsOnlyRemainingStagesAndFinalPublication) {
  auto state = std::make_shared<State>(); np::HealthLimits limits; limits.upper = 100;
  Node node({np::ExecutionMode::OnInput, Input::Primary, 10}, Buffer(8), {},
    stages(selected(), {}, selected(), limits), Cores{state}, Header{state}, IO{state});
  node.onInput(std::integral_constant<Input, Input::Auxiliary>{}, Message{3});
  EXPECT_EQ(np::ProcessStatus::NoCore, node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{4}).status);
  EXPECT_EQ((std::vector<int>{1}), state->calls); EXPECT_TRUE(state->published.empty()); EXPECT_EQ(0u, state->headers);
}
TEST(CoreSequence, LaterPolicyRemovalPreventsItsCoreCall) {
  auto state = std::make_shared<State>(); np::HealthLimits first_limits; first_limits.upper = 100;
  np::HealthLimits later_limits; later_limits.upper = 5;
  Node node({np::ExecutionMode::OnInput, Input::Primary, 10}, Buffer(8), {},
    {stage<Data, 2>(first, selected(), first_limits), stage<np::Data<Message,int>, 2>(middle, selected(), later_limits)},
    Cores{state}, Header{state}, IO{state});
  node.onInput(std::integral_constant<Input, Input::Auxiliary>{}, Message{3});
  const auto report = node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{4});
  EXPECT_EQ(np::ProcessStatus::NoCore, report.status); EXPECT_EQ("required core input missing", report.reason);
  EXPECT_EQ((std::vector<int>{1}), state->calls); EXPECT_TRUE(state->published.empty());
}
TEST(CoreSequence, CoreStateSurvivesSelectionAndMissingPolicy) {
  auto state = std::make_shared<State>(); np::HealthLimits limits; limits.upper = 100;
  auto selector = selected(); selector.health_core_map = {{{{Input::Auxiliary,np::Kind::Missing}},100,Id::Alternate}};
  Node node({np::ExecutionMode::OnInput, Input::Primary, 10}, Buffer(8), {},
    stages(selector, selected(), selected(), limits), Cores{state}, Header{state}, IO{state});
  node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{10});
  node.onInput(std::integral_constant<Input, Input::Auxiliary>{}, Message{5});
  node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{10});
  Message invalid{std::numeric_limits<double>::infinity()};
  node.onInput(std::integral_constant<Input, Input::Auxiliary>{}, invalid);
  node.onInput(std::integral_constant<Input, Input::Primary>{}, Message{10});
  ASSERT_EQ(3u,state->published.size());
  EXPECT_DOUBLE_EQ(5.5,state->published[0].value); EXPECT_DOUBLE_EQ(15,state->published[1].value); EXPECT_DOUBLE_EQ(6,state->published[2].value);
  EXPECT_TRUE(std::isinf(invalid.value));
}
TEST(CoreSequence, ModifiedAndBorrowedResultsInheritOnlyDeclaredOrigins) {
  Data typed; typed.get<0>().sample.message = std::make_shared<Message>(Message{4});
  typed.get<0>().sample.stamp_ns = 123; typed.get<0>().sample.receipt_steady_ns = 456;
  np::StageData input; input.update(typed);
  const auto& borrowed = input.message<Message>(0); Message modified = borrowed; modified.value = 5;
  auto output = np::wrapResult(std::tuple<const Message&,Message,int>{borrowed,modified,7}, {0,0,-1})(input);
  ASSERT_EQ(3u,output.inputs.size()); EXPECT_EQ(input.inputs[0].message,output.inputs[0].message);
  for (std::size_t i : {0u,1u}) { EXPECT_TRUE(output.inputs[i].has_stamp); EXPECT_EQ(123,output.inputs[i].stamp_ns); EXPECT_EQ(456,output.inputs[i].receipt_steady_ns); }
  EXPECT_FALSE(output.inputs[2].has_stamp); EXPECT_FALSE(output.inputs[2].has_receipt); EXPECT_TRUE(output.inputs[2].present());
  EXPECT_DOUBLE_EQ(5,output.message<Message>(1).value); EXPECT_DOUBLE_EQ(4,input.message<Message>(0).value);
}
TEST(CoreSequence, NewValuesWithEmptyTimeMetadataStillReceivePayloadValidation) {
  np::StageData input;
  auto output = np::wrapResult(Message{std::numeric_limits<double>::infinity(),0,""})(input);
  auto typed = output.typed<np::Data<Message>>(); np::Context<1> context;
  const auto health = np::HealthChecker<Input,1,Validator>(np::HealthLimits{},0).healthCheck(typed,context,{1000000000,1000000000});
  EXPECT_EQ((std::vector<np::Kind>{np::Kind::Inf}),health.inputs[0].issues);
}
TEST(CoreSequence, RejectsIncompatibleStageInputContract) {
  np::StageData input;
  auto output = np::wrapResult(std::make_pair(Message{4}, std::string("wrong type")))(input);
  EXPECT_THROW((output.typed<np::Data<Message,int>>()),std::invalid_argument);
  auto short_output = np::wrapResult(Message{4})(input);
  EXPECT_THROW((short_output.typed<Data>()),std::invalid_argument);
}
TEST(CoreSequence, AttemptTokenIsRecordedOnceAndContextSurvivesStages) {
  auto state = std::make_shared<State>(); np::HealthLimits limits; limits.upper = 100;
  Node node({np::ExecutionMode::Periodic,Input::Primary,10,true}, Buffer(8), {},
    stages(selected(),selected(),selected(),limits), Cores{state},Header{state},IO{state});
  node.onInput(std::integral_constant<Input,Input::Primary>{},Message{4});
  EXPECT_EQ(np::ProcessStatus::NoCore,node.onPeriodic().status);
  EXPECT_EQ(np::ProcessStatus::NoCore,node.onPeriodic().status); // no actual call => not attempted
  node.onInput(std::integral_constant<Input,Input::Auxiliary>{},Message{3});
  EXPECT_EQ(np::ProcessStatus::Published,node.onPeriodic().status);
  EXPECT_EQ(np::ProcessStatus::Skipped,node.onPeriodic().status);
  EXPECT_EQ((std::vector<int>{1,2,3}),state->calls); EXPECT_EQ(1u,state->headers);
}
}  // namespace
