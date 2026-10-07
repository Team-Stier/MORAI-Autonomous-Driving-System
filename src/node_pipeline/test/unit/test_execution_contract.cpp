#include <node_pipeline/detail/runtime.hpp>
#include <cassert>
using namespace node_pipeline;
using namespace node_pipeline::detail;
struct Sum { double process(double a,double b) { return a+b; } };
struct Pass { double process(double a) { return a; } };
struct Shift {
  bool apply(InputView& data,const ProfileHealth&) const {
    data.modify<double>("z",[](double& x){x=0;});return false;
  }
};
static std::vector<std::string> calls;
struct FirstValidator {void operator()(const InputView&,ProfileHealth&)const {calls.push_back("first validator");}};
struct SecondValidator {void operator()(const InputView&,ProfileHealth&)const {calls.push_back("second validator");}};
struct SecondPolicy {bool apply(InputView& data,const ProfileHealth&)const {calls.push_back("second policy");data.modify<double>("backup",[](double& x){x=7;});return false;}};
struct TracedCore {double process(double x){calls.push_back("core");return x;}};
int main() {
  Engine e(2,10);
  InputSpec z("z"),a("a");z.limits.max_skew_ns=200;a.limits.max_skew_ns=1;
  e.input<double>(z);e.input<double>(a);
  CoreSpec c;c.name="sum";c.inputs={"z","a"};c.outputs={{"sum",{}}};
  e.core<Sum>(c,{});
  StageSpec s;s.name="s";s.section_core_map[0]="sum";e.stage(s);
  auto one=std::make_shared<const double>(1);
  e.push("z",one,{100,10},100);
  e.push("a",std::make_shared<const double>(2),{110,10},110);
  assert(e.prepare(false,"z",1));
  assert(e.assembled().at("z").value.get()==one.get());
  assert(e.execute({110,10}).status==ProcessStatus::Published);
  assert(e.result<double>("sum")==3);
  assert(!e.prepare(true,"z",1));
  // Equal distances choose the newer ID; periodic reference is largest stamp.
  e.push("z",std::make_shared<const double>(5),{120,10},120);
  assert(e.prepare(false,"a",2));
  assert(e.assembled().at("z").id==3);
  assert(e.execute({120,10}).status==ProcessStatus::Published);
  e.push("z",std::make_shared<const double>(7),{120,10},120);
  assert(e.evictions().at("z")==1);
  assert(e.prepare(false));assert(e.assembled().at("z").id==4);
  assert(e.execute({120,10}).status==ProcessStatus::Published);
  // Exact timeout/stale boundary is accepted. No duplicate skew issues.
  Engine bad;
  z.limits.max_skew_ns=5;a.limits.max_skew_ns=200;
  bad.input<double>(z);bad.input<double>(a);bad.core<Sum>(c,{});bad.stage(s);
  bad.push("z",one,{100,10},100);bad.push("a",one,{110,10},110);
  assert(bad.process({110,10}).status==ProcessStatus::Rejected);
  assert(bad.health().issues("z")==std::vector<Kind>{Kind::Skew});
  // Policy false never settles; successful last permitted check does settle.
  Engine last;
  z.limits.upper=1;last.input<double>(z);
  CoreSpec p;p.name="pass";p.inputs={"z"};p.outputs={{"z","z"}};
  last.core<Pass,NoParameters,NoValidator,Shift>(p,{});
  StageSpec t;t.name="t";t.section_core_map[0]="pass";t.max_health_checks=2;last.stage(t);
  last.push("z",std::make_shared<const double>(9),{100,10},100);
  assert(last.process({100,10}).status==ProcessStatus::Published);
  assert(last.output().at("z").has_stamp&&last.output().at("z").stamp_ns==100);
  Engine empty;
  bool invalid=false;try{empty.validateExecution();}catch(const std::invalid_argument&){invalid=true;}assert(invalid);

  Engine switcher;
  switcher.input<double>({"missing"});switcher.input<double>({"backup"});
  CoreSpec primary;primary.name="primary";primary.inputs={"missing"};primary.outputs={{"result",{}}};
  CoreSpec replacement;replacement.name="replacement";replacement.inputs={"backup"};replacement.outputs={{"result",{}}};
  switcher.core<TracedCore,NoParameters,FirstValidator>(primary,{});
  switcher.core<TracedCore,NoParameters,SecondValidator,SecondPolicy>(replacement,{});
  StageSpec mapped;mapped.name="mapped";mapped.section_core_map[0]="primary";
  mapped.health_core_map={{{{"missing",Kind::Missing}},100,"replacement"}};switcher.stage(mapped);
  switcher.push("backup",one,{100,10},100);
  assert(switcher.process({100,10}).status==ProcessStatus::Published);
  assert(calls==std::vector<std::string>({"first validator","second policy","second validator","core"}));
  assert(switcher.result<double>("result")==7);
  Engine absent;absent.input<double>(z);absent.core<Pass>(p,{});t.section_core_map.clear();absent.stage(t);
  assert(absent.process({100,10}).status==ProcessStatus::NoCore);
  assert(absent.prepare(true));
}
