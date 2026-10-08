// 고정 실행 계약을 ROS 없이 검증한다.
#include <node_pipeline/detail/pipeline.hpp>
#include <gtest/gtest.h>

namespace np = node_pipeline;
namespace npd = node_pipeline::detail;

namespace {
struct Stamp {
  std::uint64_t ns = 0;
  std::uint64_t toNSec() const { return ns; }
  void fromNSec(std::uint64_t value) { ns = value; }
};
struct Header {
  Stamp stamp;
  std::string frame_id;
};
struct Reading {
  Header header;
  double value = 0;
};

struct Offset {
  double value = 0.5;
};
struct Sum {
  explicit Sum(Offset& offset) : offset_(offset) {}
  Reading process(const Reading& a, const Reading& b) {
    Reading result;
    result.value = a.value + b.value + offset_.value;
    return result;
  }
  Offset& offset_;
};
struct Negate {
  Reading process(const Reading& a) {
    Reading result;
    result.value = -a.value;
    return result;
  }
};
struct Constant {
  Reading process() {
    Reading result;
    result.value = 7;
    return result;
  }
};
struct Forward {
  std::tuple<Reading, Reading> process(const Reading& a, const Reading& b) { return std::make_tuple(a, b); }
};
struct RejectNegative {
  void operator()(const Reading& message, const np::HealthLimits&, std::vector<np::Kind>& issues) const {
    if (message.value < 0) issues.push_back(np::Kind::OutRangeNegative);
  }
};
struct RemoveAll {
  int priority(np::Kind) const { return 100; }
  void correct(np::EditableInput& input, np::Kind) const { input.remove(); }
};
struct ValueNumbers {
  static std::vector<double> collect(const Reading& message) { return {message.value}; }
};

enum class TestCoreId : std::size_t { Sum, Negate, Constant, Forward };
struct CoreSet {
  npd::BankSet<Offset> banks;
  Sum sum;
  Negate negate;
  Constant constant;
  Forward forward;
};
CoreSet makeCores() {
  const auto offset = std::make_shared<Offset>();
  return CoreSet{npd::BankSet<Offset>{offset}, Sum(*offset), Negate{}, Constant{}, Forward{}};
}

struct StageOne {
  using CoreId = TestCoreId;
  using DataType = npd::Data<Reading, Reading>;
  using Numbers = npd::Types<ValueNumbers, npd::NoNumbers>;
  using Outputs = npd::Types<Reading>;
  static std::vector<npd::CoreProfile<StageOne>> coreProfiles() {
    auto sum = npd::makeProfile<StageOne, RejectNegative, RemoveAll>(CoreId::Sum, "sum.hpp", {0, 1}, "", "");
    sum.allowed_issues[1].push_back(np::Kind::Stale);
    return {sum, npd::makeProfile<StageOne, npd::NoValidator, npd::NoPolicy>(CoreId::Negate, "negate.hpp", {0}, "", "")};
  }
  static bool processCore(CoreSet& cores, CoreId id, const npd::StageData& data, npd::CoreResult& result) {
    const std::vector<int> origins{-1};
    switch (id) {
      case CoreId::Sum: {
        const std::vector<std::size_t> required_sum{0, 1};
        if (!data.hasInputs(required_sum)) return false;
        auto value = cores.sum.process(data.message<Reading>(0), data.message<Reading>(1));
        result = npd::wrapResult(std::move(value), origins);
        break;
      }
      case CoreId::Negate: {
        const std::vector<std::size_t> required_negate{0};
        if (!data.hasInputs(required_negate)) return false;
        auto value = cores.negate.process(data.message<Reading>(0));
        result = npd::wrapResult(std::move(value), origins);
        break;
      }
      default:
        throw std::invalid_argument("unregistered core id");
    }
    return true;
  }
};
struct ForwardStage {
  using CoreId = TestCoreId;
  using DataType = npd::Data<Reading, Reading>;
  using Numbers = npd::Types<npd::NoNumbers, npd::NoNumbers>;
  using Outputs = npd::Types<Reading, Reading>;
  static std::vector<npd::CoreProfile<ForwardStage>> coreProfiles() {
    return {npd::makeProfile<ForwardStage, npd::NoValidator, npd::NoPolicy>(CoreId::Forward, "forward.hpp", {0, 1}, "", "")};
  }
  static bool processCore(CoreSet& cores, CoreId, const npd::StageData& data, npd::CoreResult& result) {
    auto value = cores.forward.process(data.message<Reading>(0), data.message<Reading>(1));
    result = npd::wrapResult(std::move(value), {0, 1});
    return true;
  }
};
struct SumStage {
  using CoreId = TestCoreId;
  using DataType = npd::Data<Reading, Reading>;
  using Numbers = npd::Types<npd::NoNumbers, npd::NoNumbers>;
  using Outputs = npd::Types<Reading>;
  static std::vector<npd::CoreProfile<SumStage>> coreProfiles() {
    auto sum = npd::makeProfile<SumStage, npd::NoValidator, RemoveAll>(CoreId::Sum, "sum.hpp", {0, 1}, "", "");
    sum.allowed_issues[1].push_back(np::Kind::Stale);
    return {sum};
  }
  static bool processCore(CoreSet& cores, CoreId, const npd::StageData& data, npd::CoreResult& result) {
    auto value = cores.sum.process(data.message<Reading>(0), data.message<Reading>(1));
    result = npd::wrapResult(std::move(value), {-1});
    return true;
  }
};
struct ConstantStage {
  using CoreId = TestCoreId;
  using DataType = npd::Data<>;
  using Numbers = npd::Types<>;
  using Outputs = npd::Types<Reading>;
  static std::vector<npd::CoreProfile<ConstantStage>> coreProfiles() {
    return {npd::makeProfile<ConstantStage, npd::NoValidator, npd::NoPolicy>(CoreId::Constant, "constant.hpp", {}, "", "")};
  }
  static bool processCore(CoreSet& cores, CoreId, const npd::StageData&, npd::CoreResult& result) {
    result = npd::wrapResult(cores.constant.process(), {-1});
    return true;
  }
};

struct FakeIO {
  using MessageType = Reading;
  std::shared_ptr<npd::Clocks> clocks = std::make_shared<npd::Clocks>();
  std::shared_ptr<std::vector<Reading>> published = std::make_shared<std::vector<Reading>>();
  std::shared_ptr<std::vector<npd::ProcessReport>> reports = std::make_shared<std::vector<npd::ProcessReport>>();
  std::shared_ptr<std::vector<std::vector<std::vector<np::Kind>>>> healths =
      std::make_shared<std::vector<std::vector<std::vector<np::Kind>>>>();
  npd::Clocks readClocks() const { return *clocks; }
  void publish(const Reading& message) const { published->push_back(message); }
  template<class HealthType> void reportHealth(const HealthType& health, std::size_t) const {
    healths->emplace_back();
    for (const auto& input : health.inputs) healths->back().push_back(input.issues);
  }
  void report(const npd::ProcessReport& report) const { reports->push_back(report); }
};
struct FakeInjector {
  std::string frame_id = "map";
  void injectHeader(Reading& message, const npd::Context& context) const { npd::Access::inject(message, context.stamp_ns, frame_id); }
};

np::HealthLimits limits(std::int64_t stale_ns = 1000000000) {
  np::HealthLimits value;
  value.stale_ns = stale_ns;
  value.receive_timeout_ns = 1000000000;
  value.lower = -100;
  value.upper = 100;
  return value;
}

template<class Schema>
npd::Stage<Schema, CoreSet> stageOf(npd::CoreSelector<TestCoreId> selector, std::vector<np::HealthLimits> input_limits,
                                    bool subscribed, std::int64_t max_skew_ns = 10000000000) {
  auto core_profiles = Schema::coreProfiles();
  for (auto& profile : core_profiles) {
    const auto checker = std::make_shared<const npd::HealthChecker<Schema>>(
        "stage", std::vector<std::string>{"a", "b"}, input_limits, std::vector<bool>(input_limits.size(), subscribed),
        max_skew_ns, 0, profile.validator);
    profile.ContractCheck = npd::makeContractCheck<Schema>(checker, profile.required_inputs);
  }
  return npd::makeStage(core_profiles, selector, &Schema::processCore);
}
npd::CoreSelector<TestCoreId> selector(std::map<std::uint8_t, TestCoreId> sections,
                                       std::vector<npd::HealthCoreRule<TestCoreId>> rules = {}) {
  npd::CoreSelector<TestCoreId> result;
  result.section_core_map = std::move(sections);
  result.health_core_map = std::move(rules);
  return result;
}

using Buffer = npd::InputBuffer<Reading, Reading>;
using Assembler = npd::SnapshotAssembler<Buffer>;
template<class... Schemas>
using NodeOf = npd::PipelineNode<Buffer, Assembler, std::tuple<npd::Stage<Schemas, CoreSet>...>, CoreSet, FakeInjector, FakeIO>;
using SingleNode = NodeOf<StageOne>;
using A = std::integral_constant<std::size_t, 0>;
using B = std::integral_constant<std::size_t, 1>;

std::shared_ptr<const Reading> reading(double value, std::int64_t stamp_ns) {
  auto message = std::make_shared<Reading>();
  message->value = value;
  message->header.stamp.ns = static_cast<std::uint64_t>(stamp_ns);
  return message;
}
npd::ExecutionConfig onInput() {
  npd::ExecutionConfig execution;
  execution.mode = npd::ExecutionMode::OnInput;
  execution.trigger_input = 0;
  execution.max_health_checks = 4;
  return execution;
}

struct SingleFixture {
  FakeIO io;
  std::unique_ptr<SingleNode> node;
  explicit SingleFixture(npd::CoreSelector<TestCoreId> stage_selector = selector({{0, TestCoreId::Sum}}),
                         std::int64_t max_skew_ns = 10000000000) {
    io.clocks->ros_ns = 10000000000;
    io.clocks->steady_ns = 1000;
    node.reset(new SingleNode(onInput(), Buffer(8), Assembler(-1),
                              std::make_tuple(stageOf<StageOne>(stage_selector, {limits(), limits()}, true, max_skew_ns)),
                              makeCores(), FakeInjector{}, io));
  }
};
}  // namespace

TEST(Pipeline, PublishesSelectedCoreWithInjectedHeaderAndBank) {
  SingleFixture fixture;
  fixture.node->onInput(B{}, reading(2, 9500000000));
  const auto report = fixture.node->onInput(A{}, reading(3, 9900000000));
  EXPECT_EQ(npd::ProcessStatus::Published, report.status);
  ASSERT_EQ(1u, fixture.io.published->size());
  EXPECT_DOUBLE_EQ(5.5, fixture.io.published->back().value);
  EXPECT_EQ(9900000000u, fixture.io.published->back().header.stamp.ns);
  EXPECT_EQ("map", fixture.io.published->back().header.frame_id);
}

TEST(Pipeline, SelectsSectionCoreOnlyForValidSection) {
  SingleFixture fixture(selector({{0, TestCoreId::Sum}, {3, TestCoreId::Negate}}));
  fixture.node->onInput(B{}, reading(2, 9500000000));
  fixture.node->onSection({true, 3, 9000000000, "map"});
  fixture.node->onInput(A{}, reading(3, 9900000000));
  EXPECT_DOUBLE_EQ(-3, fixture.io.published->back().value);
  fixture.node->onSection({true, 3, 9000000000, "odom"});
  fixture.node->onInput(A{}, reading(4, 9900000000));
  EXPECT_DOUBLE_EQ(6.5, fixture.io.published->back().value);
}

TEST(Pipeline, MissingSectionMappingIsNoCore) {
  SingleFixture fixture;
  fixture.node->onSection({true, 5, 9000000000, "map"});
  const auto report = fixture.node->onInput(A{}, reading(3, 9900000000));
  EXPECT_EQ(npd::ProcessStatus::NoCore, report.status);
  EXPECT_EQ("no compatible core", report.reason);
}

TEST(Pipeline, HealthRuleSwitchesCoreWhenDeclaredInputIsMissing) {
  npd::HealthCoreRule<TestCoreId> rule;
  rule.when = {{1, np::Kind::Missing}};
  rule.priority = 10;
  rule.core = TestCoreId::Negate;
  SingleFixture fixture(selector({{0, TestCoreId::Sum}}, {rule}));
  const auto report = fixture.node->onInput(A{}, reading(3, 9900000000));
  EXPECT_EQ(npd::ProcessStatus::Published, report.status);
  EXPECT_EQ(2u, report.health_checks);
  EXPECT_DOUBLE_EQ(-3, fixture.io.published->back().value);
}

TEST(Pipeline, PolicyRemovesInvalidInputAndRejectsAtCheckLimit) {
  SingleFixture fixture;
  fixture.node->onInput(B{}, reading(2, 9500000000));
  const auto report = fixture.node->onInput(A{}, reading(-3, 9900000000));
  EXPECT_EQ(npd::ProcessStatus::Rejected, report.status);
  EXPECT_EQ(4u, report.health_checks);
  EXPECT_EQ(std::vector<np::Kind>{np::Kind::OutRangeNegative}, fixture.io.healths->at(0).at(0));
  EXPECT_EQ(std::vector<np::Kind>{np::Kind::Missing}, fixture.io.healths->at(1).at(0));
  EXPECT_TRUE(fixture.io.published->empty());
}

TEST(Pipeline, CommonChecksRejectOutOfRangeNumbers) {
  SingleFixture fixture;
  fixture.node->onInput(B{}, reading(2, 9500000000));
  const auto report = fixture.node->onInput(A{}, reading(500, 9900000000));
  EXPECT_EQ(npd::ProcessStatus::Rejected, report.status);
  EXPECT_EQ(std::vector<np::Kind>{np::Kind::OutRangePositive}, fixture.io.healths->at(0).at(0));
}

TEST(Pipeline, SkewIsRecordedOnBothStampedInputs) {
  SingleFixture fixture(selector({{0, TestCoreId::Sum}}), 100000000);
  fixture.node->onInput(B{}, reading(2, 9500000000));
  EXPECT_EQ(npd::ProcessStatus::Rejected, fixture.node->onInput(A{}, reading(3, 9900000000)).status);
  EXPECT_EQ(std::vector<np::Kind>{np::Kind::Skew}, fixture.io.healths->at(0).at(0));
  EXPECT_EQ(std::vector<np::Kind>{np::Kind::Skew}, fixture.io.healths->at(0).at(1));
}

TEST(Pipeline, AllowedIssuesAreAcceptedOnlyForTheirInput) {
  SingleFixture fixture;
  fixture.node->onInput(B{}, reading(2, 5000000000));
  EXPECT_EQ(npd::ProcessStatus::Published, fixture.node->onInput(A{}, reading(3, 9900000000)).status);
  EXPECT_EQ(npd::ProcessStatus::Rejected, fixture.node->onInput(A{}, reading(3, 5000000000)).status);
}

TEST(Pipeline, IssuesOfUndeclaredInputsDoNotBlockTheSelectedCore) {
  SingleFixture fixture(selector({{0, TestCoreId::Negate}}));
  fixture.node->onInput(B{}, reading(-2, 1));
  EXPECT_EQ(npd::ProcessStatus::Published, fixture.node->onInput(A{}, reading(3, 9900000000)).status);
}

TEST(Pipeline, AmbiguousHealthRulesAreConfigurationErrors) {
  npd::HealthCoreRule<TestCoreId> first;
  first.when = {{1, np::Kind::Missing}};
  first.priority = 10;
  first.core = TestCoreId::Negate;
  auto second = first;
  second.core = TestCoreId::Sum;
  SingleFixture fixture(selector({{0, TestCoreId::Sum}}, {first, second}));
  EXPECT_THROW(fixture.node->onInput(A{}, reading(3, 9900000000)), std::invalid_argument);
}

TEST(Pipeline, PeriodicRequireNewSnapshotSkipsRepeatedBundle) {
  FakeIO io;
  io.clocks->ros_ns = 10000000000;
  npd::ExecutionConfig execution;
  execution.mode = npd::ExecutionMode::Periodic;
  execution.require_new_snapshot = true;
  SingleNode node(execution, Buffer(8), Assembler(0),
                  std::make_tuple(stageOf<StageOne>(selector({{0, TestCoreId::Sum}}), {limits(), limits()}, true)),
                  makeCores(), FakeInjector{}, io);
  node.onInput(A{}, reading(3, 9900000000));
  node.onInput(B{}, reading(2, 9800000000));
  EXPECT_EQ(npd::ProcessStatus::Published, node.onPeriodic().status);
  EXPECT_EQ(npd::ProcessStatus::Skipped, node.onPeriodic().status);
  EXPECT_EQ(9900000000u, io.published->back().header.stamp.ns);
}

TEST(Pipeline, PassThroughOutputsInheritTimeMetadataAcrossStages) {
  FakeIO io;
  io.clocks->ros_ns = 10000000000;
  NodeOf<ForwardStage, SumStage> node(
      onInput(), Buffer(8), Assembler(-1),
      std::make_tuple(stageOf<ForwardStage>(selector({{0, TestCoreId::Forward}}), {limits(), limits()}, true),
                      stageOf<SumStage>(selector({{0, TestCoreId::Sum}}), {limits(50000000), limits()}, false)),
      makeCores(), FakeInjector{}, io);
  node.onInput(B{}, reading(2, 9500000000));
  EXPECT_EQ(npd::ProcessStatus::Rejected, node.onInput(A{}, reading(3, 9900000000)).status);
  EXPECT_EQ(npd::ProcessStatus::Published, node.onInput(A{}, reading(3, 9970000000)).status);
  EXPECT_DOUBLE_EQ(5.5, io.published->back().value);
}

TEST(Pipeline, LatchedNodePublishesOnceAtStartupWithZeroContextStamp) {
  FakeIO io;
  npd::ExecutionConfig execution;
  execution.mode = npd::ExecutionMode::Latched;
  npd::PipelineNode<npd::InputBuffer<>, npd::SnapshotAssembler<npd::InputBuffer<>>,
                    std::tuple<npd::Stage<ConstantStage, CoreSet>>, CoreSet, FakeInjector, FakeIO>
      node(execution, npd::InputBuffer<>(8), npd::SnapshotAssembler<npd::InputBuffer<>>(-1),
           std::make_tuple(stageOf<ConstantStage>(selector({{0, TestCoreId::Constant}}), {}, false)), makeCores(),
           FakeInjector{"base_link"}, io);
  EXPECT_EQ(npd::ProcessStatus::Published, node.onStartup().status);
  EXPECT_EQ(0u, io.published->back().header.stamp.ns);
  EXPECT_EQ("base_link", io.published->back().header.frame_id);
  EXPECT_DOUBLE_EQ(7, io.published->back().value);
}

TEST(Pipeline, RejectsInvalidExecutionConfiguration) {
  FakeIO io;
  auto execution = onInput();
  execution.trigger_input = 5;
  EXPECT_THROW(SingleNode(execution, Buffer(8), Assembler(-1),
                          std::make_tuple(stageOf<StageOne>(selector({{0, TestCoreId::Sum}}), {limits(), limits()}, true)),
                          makeCores(), FakeInjector{}, io),
               std::invalid_argument);
}

TEST(Pipeline, ContractCheckRejectsPayloadTypeMismatch) {
  npd::StageData data;
  data.inputs.resize(2);
  data.inputs[0].message = std::make_shared<int>(1);
  data.inputs[0].type = std::type_index(typeid(int));
  const auto profile = stageOf<StageOne>(selector({{0, TestCoreId::Sum}}), {limits(), limits()}, true).coreProfile(TestCoreId::Sum);
  EXPECT_THROW(profile.ContractCheck(data, npd::Context{}, npd::Clocks{}), std::invalid_argument);
}
