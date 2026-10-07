#include <node_pipeline/detail/runtime.hpp>
#include <cassert>
#include <limits>

using namespace node_pipeline;
using namespace node_pipeline::detail;
struct DoubleCore { double process(const double& x) { return x + 1; } };
struct NaNCore { double process(const double&) { return std::numeric_limits<double>::quiet_NaN(); } };
struct Fallback { double process(const int& x) { return x * 2; } };
struct PairCore {
  int calls = 0;
  std::tuple<double,int> process(const double& x) { return {x, ++calls}; }
};
struct SumCore { double process(const double& x, const int& count) { return x + count; } };
struct Bank { int count=0; };
struct BankCore {
  explicit BankCore(std::shared_ptr<Bank> bank):bank_(std::move(bank)){}
  int process(const int&) { return ++bank_->count; }
  std::shared_ptr<Bank> bank_;
};
struct SwitchPolicy {
  static int calls;
  bool apply(InputView&, const ProfileHealth&) const { ++calls; return false; }
};
int SwitchPolicy::calls = 0;
struct MutatingValidator {
  void operator()(const InputView& inputs, ProfileHealth&)const { auto copy=inputs;copy.remove("x"); }
};
struct WaivingValidator {
  void operator()(const InputView&, ProfileHealth& health)const { health.inputs.clear(); }
};
int main() {
  Engine engine;
  engine.input<double>({"x"});
  engine.input<int>({"backup"});
  CoreSpec first; first.name = "first"; first.inputs = {"x"}; first.outputs = {{"out",{}}};
  CoreSpec next; next.name = "fallback"; next.inputs = {"backup"}; next.outputs = {{"out",{}}};
  engine.core<DoubleCore>(first, Parameters{});
  engine.core<Fallback, NoParameters, NoValidator, SwitchPolicy>(next, Parameters{});
  StageSpec stage; stage.name = "estimate"; stage.section_core_map[0] = "first";
  stage.health_core_map.push_back({{{"x", Kind::Missing}}, 10, "fallback"});
  engine.stage(stage);
  engine.push("backup", std::make_shared<const int>(3), {100,100}, 100);
  auto report = engine.process({100,100});
  assert(report.status == ProcessStatus::Published);
  assert(engine.result<double>("out") == 6);
  assert(SwitchPolicy::calls == 1);
  assert(engine.process({100,100}, true).status == ProcessStatus::Skipped);
  Engine missing;
  missing.input<double>({"x"});
  first.allowed_issues["x"] = {Kind::Missing};
  missing.core<DoubleCore>(first, Parameters{});
  StageSpec only; only.name="estimate"; only.section_core_map[0]="first"; missing.stage(only);
  assert(missing.process({100,100}).status == ProcessStatus::Rejected);
  Engine invalid;
  invalid.input<double>({"x"}); invalid.core<DoubleCore>(first, Parameters{}); invalid.stage(only);
  invalid.push("x",std::make_shared<const double>(std::numeric_limits<double>::quiet_NaN()),{100,100},100);
  assert(invalid.process({100,100}).status == ProcessStatus::Rejected);
  InputSpec watched("x"); watched.limits.receive_timeout_ns=100;
  Engine awaiting(10,100);
  awaiting.input<double>(watched);awaiting.core<DoubleCore>(first,{});awaiting.stage(only);
  assert(awaiting.process({100,300}).status==ProcessStatus::Rejected);
  const auto& waiting_issues=awaiting.health().issues("x");
  assert(std::find(waiting_issues.begin(),waiting_issues.end(),Kind::ReceiveTimeout)!=waiting_issues.end());
  Engine removed(10,100);
  removed.input<double>(watched);removed.core<DoubleCore>(first,{});removed.stage(only);
  removed.push("x",std::make_shared<const double>(std::numeric_limits<double>::quiet_NaN()),{100,290},100);
  assert(removed.process({100,300}).status==ProcessStatus::Rejected);
  const auto& removed_issues=removed.health().issues("x");
  assert(std::find(removed_issues.begin(),removed_issues.end(),Kind::ReceiveTimeout)!=removed_issues.end());
  Engine calculated(10,100);
  calculated.input<double>(watched);calculated.limits("derived",watched.limits);
  CoreSpec producer;producer.name="producer";producer.inputs={"x"};producer.outputs={{"derived",{}}};
  CoreSpec consumer;consumer.name="consumer";consumer.inputs={"derived"};consumer.outputs={{"out",{}}};
  calculated.core<NaNCore>(producer,{});calculated.core<DoubleCore>(consumer,{});
  StageSpec produce;produce.name="produce";produce.section_core_map[0]="producer";
  StageSpec consume;consume.name="consume";consume.section_core_map[0]="consumer";
  calculated.stage(produce);calculated.stage(consume);
  calculated.push("x",std::make_shared<const double>(0.5),{100,290},100);
  assert(calculated.process({100,300}).status==ProcessStatus::Rejected);
  const auto& calculated_issues=calculated.health().issues("derived");
  assert(std::find(calculated_issues.begin(),calculated_issues.end(),Kind::ReceiveTimeout)==calculated_issues.end());
  Engine inherited(10,100);
  inherited.input<double>(watched);inherited.limits("derived",watched.limits);
  producer.outputs[0].time_from="x";
  inherited.core<NaNCore>(producer,{});inherited.core<DoubleCore>(consumer,{});
  inherited.stage(produce);inherited.stage(consume);
  inherited.push("x",std::make_shared<const double>(0.5),{100,290},100);
  assert(inherited.process({100,300}).status==ProcessStatus::Rejected);
  assert(inherited.health().issues("derived")==std::vector<Kind>{Kind::Missing});
  Engine guarded;
  guarded.input<double>({"x"});guarded.core<DoubleCore,NoParameters,MutatingValidator>(first,{});guarded.stage(only);
  guarded.push("x",std::make_shared<const double>(4),{100,100},100);
  bool denied=false;
  try {guarded.process({100,100});}catch(const std::logic_error&){denied=true;}
  assert(denied);
  Engine waiving;
  waiving.input<double>({"x"});waiving.core<DoubleCore,NoParameters,WaivingValidator>(first,{});waiving.stage(only);
  waiving.push("x",std::make_shared<const double>(std::numeric_limits<double>::quiet_NaN()),{100,100},100);
  assert(waiving.process({100,100}).status==ProcessStatus::Rejected);
  Engine sequence;
  sequence.input<double>({"x"});
  CoreSpec pair; pair.name="pair"; pair.inputs={"x"}; pair.outputs={{"value",{}},{"count",{}}};
  CoreSpec sum; sum.name="sum"; sum.inputs={"value","count"}; sum.outputs={{"out",{}}};
  sequence.core<PairCore>(pair,{});sequence.core<SumCore>(sum,{});
  StageSpec a;a.name="first";a.section_core_map[0]="pair";
  StageSpec b;b.name="second";b.section_core_map[0]="sum";
  sequence.stage(a);sequence.stage(b);
  sequence.push("x",std::make_shared<const double>(4),{100,100},100);
  assert(sequence.process({100,100}).status==ProcessStatus::Published);
  assert(sequence.result<double>("out")==5);
  assert(sequence.process({100,100}).status==ProcessStatus::Published);
  assert(sequence.result<double>("out")==6);
  assert(!sequence.output().at("out").has_stamp);
  assert(sequence.output().count("x")==0);
  auto bank=std::make_shared<Bank>();Engine banking;
  banking.input<int>({"backup"});
  CoreSpec one;one.name="one";one.inputs={"backup"};one.outputs={{"out",{}}};
  CoreSpec two=one;two.name="two";
  banking.core<BankCore,NoParameters,NoValidator,RemoveInvalid,Bank>(one,{},bank);
  banking.core<BankCore,NoParameters,NoValidator,RemoveInvalid,Bank>(two,{},bank);
  StageSpec s;s.name="banked";s.section_core_map={{0,"one"},{1,"two"}};banking.stage(s);
  banking.push("backup",std::make_shared<const int>(3),{100,100},100);
  assert(banking.process({100,100}).status==ProcessStatus::Published);
  assert(banking.result<int>("out")==1);
  banking.section({true,1,100,"map"});
  assert(banking.process({100,100}).status==ProcessStatus::Published);
  assert(banking.result<int>("out")==2);
}
