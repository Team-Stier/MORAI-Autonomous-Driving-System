#pragma once
#include <node_pipeline/sdk.hpp>
#include <node_pipeline/components.hpp>
#include <node_pipeline/detail/native_traits.hpp>

namespace node_pipeline { namespace detail {
struct InputSpec {
  std::string name, topic;
  HealthLimits limits;
  InputSpec(std::string n={},std::string t={}):name(std::move(n)),topic(std::move(t)) { limits.lower=-std::numeric_limits<double>::max(); limits.upper=std::numeric_limits<double>::max(); limits.frame.clear(); }
};
struct OutputSlotSpec { std::string name, time_from; };
struct CoreSpec {
  std::string name, stage;
  std::vector<std::string> inputs, assets;
  std::vector<OutputSlotSpec> outputs;
  std::map<std::string,std::vector<Kind>> allowed_issues;
};
struct HealthRule {
  std::vector<Issue<std::string,Kind>> when;
  int priority=0;
  std::string core;
};
struct StageSpec {
  std::string name;
  std::map<std::uint8_t,std::string> section_core_map;
  std::vector<HealthRule> health_core_map;
  std::size_t max_health_checks=10;
};
struct OutputSpec {
  std::string source, topic, stamp_from, frame_id;
};
struct ExecutionSpec {
  std::string mode="periodic", trigger_input;
  double frequency_hz=50;
  std::size_t queue_capacity=10;
  bool require_new_snapshot=false;
};
inline void validateLimits(const HealthLimits& limits) {
  if(!std::isfinite(limits.lower) || !std::isfinite(limits.upper) || limits.lower>limits.upper ||
     limits.stale_ns<=0 || limits.receive_timeout_ns<=0 || limits.max_skew_ns<=0)
    throw std::invalid_argument("invalid common health limits");
}
template<class T> Slot nativeSlot(std::shared_ptr<const T> value) {
  Slot slot; slot.value=std::move(value); slot.type=typeid(T);
  slot.inspect=[](const void* p,const HealthLimits& limits,std::vector<Kind>& issues) {
    MessageTraits<T>::inspect(*static_cast<const T*>(p),limits,issues);
  };
  return slot;
}
template<class T> struct Native { using type=typename std::remove_cv<typename std::remove_reference<T>::type>::type; };
template<class> struct Signature;
template<class C,class R,class... Args> struct Signature<R(C::*)(Args...)> { using ArgsTuple=std::tuple<typename Native<Args>::type...>; using Result=R; };
template<class C,class R,class... Args> struct Signature<R(C::*)(Args...) const> : Signature<R(C::*)(Args...)> {};
template<class T> struct ResultTuple { using type=std::tuple<typename Native<T>::type>; static type pack(T value){return std::make_tuple(std::move(value));} };
template<class... T> struct ResultTuple<std::tuple<T...>> { using type=std::tuple<typename Native<T>::type...>; static type pack(std::tuple<T...> value){return value;} };
template<class A,class B> struct ResultTuple<std::pair<A,B>> { using type=std::tuple<typename Native<A>::type,typename Native<B>::type>; static type pack(std::pair<A,B> value){return type(std::move(value.first),std::move(value.second));} };
template<class Core,class Args,class Results> void checkCoreSignature() {
  using S=Signature<decltype(&Core::process)>;
  static_assert(std::is_same<typename S::ArgsTuple,Args>::value,"registration input types/order differ from Core::process");
  static_assert(std::is_same<typename ResultTuple<typename S::Result>::type,Results>::value,"registration output types/order differ from Core::process");
}
struct NoBank {};
class CoreBase {
 public:
  explicit CoreBase(CoreSpec spec):spec_(std::move(spec)){}
  virtual ~CoreBase()=default;
  const CoreSpec& spec()const{return spec_;}
  virtual void contract(const Frame&)const=0;
  virtual void validate(Frame&,ProfileHealth&)const=0;
  virtual bool apply(Frame&,const ProfileHealth&)const=0;
  virtual Frame compute(Frame&)=0;
 private:
  CoreSpec spec_;
};
template<class C,class... Args>
typename std::enable_if<std::is_constructible<C,Args...,const Visualizer&>::value,std::unique_ptr<C>>::type
createCore(const Visualizer* visualizer,Args... args) {
  if(!visualizer)throw std::invalid_argument("Visualizer unavailable");
  return std::unique_ptr<C>(new C(std::move(args)...,*visualizer));
}
template<class C,class... Args>
typename std::enable_if<!std::is_constructible<C,Args...,const Visualizer&>::value,std::unique_ptr<C>>::type
createCore(const Visualizer*,Args... args) {return std::unique_ptr<C>(new C(std::move(args)...));}
template<class C,class Params,class Bank>
typename std::enable_if<!std::is_same<Bank,NoBank>::value,std::unique_ptr<C>>::type
construct(const Params& p,std::shared_ptr<Bank> bank,const Visualizer* v) {return createCore<C>(v,p,std::move(bank));}
template<class C,class Params>
typename std::enable_if<!std::is_same<Params,NoParameters>::value,std::unique_ptr<C>>::type
construct(const Params& p,std::shared_ptr<NoBank>,const Visualizer* v) {return createCore<C>(v,p);}
template<class C> std::unique_ptr<C> construct(const NoParameters&,std::shared_ptr<NoBank>,const Visualizer* v) {return createCore<C>(v);}
template<class C,class Bank>
typename std::enable_if<!std::is_same<Bank,NoBank>::value,std::unique_ptr<C>>::type
construct(const NoParameters&,std::shared_ptr<Bank> bank,const Visualizer* v) {return createCore<C>(v,std::move(bank));}
template<class C,class L=NoParameters,class V=NoValidator,class P=RemoveInvalid,class B=NoBank>
class CoreModel final:public CoreBase {
  using S=Signature<decltype(&C::process)>;
  using Args=typename S::ArgsTuple;
  using Results=typename ResultTuple<typename S::Result>::type;
 public:
  CoreModel(CoreSpec spec,const Parameters& parameters,std::shared_ptr<B> bank={},const Visualizer* visualizer=nullptr)
    :CoreBase(std::move(spec)),core_(construct<C>(L{}(parameters),std::move(bank),visualizer)) {
    if(this->spec().inputs.size()!=std::tuple_size<Args>::value || this->spec().outputs.size()!=std::tuple_size<Results>::value)
      throw std::invalid_argument("Core declaration arity mismatch");
    for(const auto& out:this->spec().outputs) if(!out.time_from.empty() && std::find(this->spec().inputs.begin(),this->spec().inputs.end(),out.time_from)==this->spec().inputs.end())
      throw std::invalid_argument("time_from is not a Core input");
  }
  void contract(const Frame& frame)const override { check(frame,std::make_index_sequence<std::tuple_size<Args>::value>{}); }
  void validate(Frame& frame,ProfileHealth& health)const override {
    ProfileHealth domain;
    for(const auto& name:this->spec().inputs)domain.inputs.push_back({name,{}});
    validator_(InputView(frame,this->spec().inputs),domain);
    for(const auto& input:domain.inputs) {
      auto& issues=health.issues(input.input);
      for(auto issue:input.issues)if(std::find(issues.begin(),issues.end(),issue)==issues.end())issues.push_back(issue);
    }
  }
  bool apply(Frame& frame,const ProfileHealth& health)const override { InputView view(frame,this->spec().inputs,true); return policy_.apply(view,health); }
  Frame compute(Frame& frame)override {
    InputView view(frame,this->spec().inputs);
    auto result=ResultTuple<typename S::Result>::pack(invoke(view,std::make_index_sequence<std::tuple_size<Args>::value>{}));
    Frame output;
    each(result,[&](auto tag,auto& value) {
      using T=typename Native<decltype(value)>::type;
      auto slot=nativeSlot<T>(std::make_shared<const T>(std::move(value)));
      const auto& declaration=this->spec().outputs[tag];
      if(!declaration.time_from.empty()) {
        const auto& origin=frame.at(declaration.time_from);
        slot.id=origin.id; slot.stamp_ns=origin.stamp_ns; slot.receipt_ns=origin.receipt_ns;
        slot.has_stamp=origin.has_stamp; slot.has_receipt=origin.has_receipt;
      }
      output.emplace(declaration.name,std::move(slot));
    });
    return output;
  }
 private:
  template<std::size_t... I> void check(const Frame& frame,std::index_sequence<I...>)const {
    const int checks[]={0,(checkType<typename std::tuple_element<I,Args>::type>(frame,this->spec().inputs[I]),0)...};(void)checks;
  }
  template<class T> static void checkType(const Frame& frame,const std::string& name) {
    const auto it=frame.find(name);
    if(it==frame.end() || it->second.type!=std::type_index(typeid(T))) throw std::invalid_argument("Core input type/count mismatch: "+name);
  }
  template<std::size_t... I> typename S::Result invoke(InputView& view,std::index_sequence<I...>) {
    return core_->process(view.template get<typename std::tuple_element<I,Args>::type>(this->spec().inputs[I])...);
  }
  std::unique_ptr<C> core_;
  V validator_;
  P policy_;
};
class Engine final {
 public:
  explicit Engine(std::size_t capacity=10,std::int64_t started=0):capacity_(capacity),started_(started) {
    if(!capacity_)throw std::invalid_argument("empty assembly queue");
  }
  template<class T> void input(InputSpec spec) {
    if(spec.name.empty() || queues_.count(spec.name))throw std::invalid_argument("duplicate input");
    validateLimits(spec.limits);
    auto empty=nativeSlot<T>({}); empty.receipt_ns=started_;empty.has_receipt=true;
    prototypes_[spec.name]=empty; limits_[spec.name]=spec.limits;
    queues_[spec.name]={}; input_order_.push_back(spec.name);evictions_[spec.name]=0;
  }
  void limits(const std::string& name,const HealthLimits& limits) { validateLimits(limits);limits_[name]=limits; }
  template<class C,class L=NoParameters,class V=NoValidator,class P=RemoveInvalid,class B=NoBank>
  void core(CoreSpec spec,const Parameters& parameters,std::shared_ptr<B> bank={},const Visualizer* visualizer=nullptr) {
    const auto name=spec.name;
    if(cores_.count(name))throw std::invalid_argument("duplicate Core");
    cores_[name]=std::unique_ptr<CoreBase>(new CoreModel<C,L,V,P,B>(std::move(spec),parameters,std::move(bank),visualizer));
  }
  void stage(StageSpec spec) {
    if(!spec.max_health_checks)throw std::invalid_argument("zero health check limit");
    for(const auto& entry:spec.section_core_map) if(!cores_.count(entry.second))throw std::invalid_argument("unknown section Core");
    for(const auto& rule:spec.health_core_map) if(rule.when.empty() || !cores_.count(rule.core))throw std::invalid_argument("invalid health mapping");
    stages_.push_back(std::move(spec));
  }
  template<class T> std::uint64_t push(const std::string& name,std::shared_ptr<const T> value,const Clocks& clocks,std::int64_t stamp,bool has_stamp=true) {
    if(!value || prototypes_.at(name).type!=std::type_index(typeid(T)))throw std::invalid_argument("input payload mismatch");
    auto slot=nativeSlot<T>(std::move(value)); slot.id=++next_id_;slot.stamp_ns=stamp;slot.receipt_ns=clocks.steady_ns;
    slot.has_stamp=has_stamp;slot.has_receipt=true; auto& queue=queues_.at(name);queue.push_back(std::move(slot));
    if(queue.size()>capacity_){queue.pop_front();++evictions_[name];}
    return next_id_;
  }
  void section(const Section& section){section_=section;}
  const Frame& assembled()const{return assembled_;}
  const std::map<std::string,std::uint64_t>& evictions()const{return evictions_;}
  void validateExecution()const {if(stages_.empty())throw std::invalid_argument("no registered stage");}
  const Frame& output()const{return output_;}
  const ProfileHealth& health()const{return health_;}
  template<class T> const T& result(const std::string& name)const {
    const auto& slot=output_.at(name);if(!slot.value || slot.type!=std::type_index(typeid(T)))throw std::invalid_argument("output type mismatch");
    return *static_cast<const T*>(slot.value.get());
  }
  ProcessReport process(const Clocks& clocks,bool require_new=false,const std::string& trigger={},std::uint64_t trigger_id=0) {
    if(!prepare(require_new,trigger,trigger_id))return {ProcessStatus::Skipped,0,"bundle already attempted"};
    return execute(clocks);
  }
  bool prepare(bool require_new=false,const std::string& trigger={},std::uint64_t trigger_id=0) {
    validateExecution();auto snapshot=assemble(trigger,trigger_id);
    assembled_=std::move(snapshot.data);snapshot_token_=std::move(snapshot.token);is_new_=snapshot.is_new;
    return !((require_new || !trigger.empty()) && !is_new_);
  }
  ProcessReport execute(const Clocks& clocks) {
    Frame frame=assembled_;std::size_t checks=0;
    const std::uint8_t section=section_.present && section_.frame=="map" && section_.stamp_ns>0 && section_.stamp_ns<=clocks.ros_ns?section_.state:0;
    for(std::size_t stage_index=0;stage_index<stages_.size();++stage_index) {
      const auto& stage=stages_[stage_index];auto initial=stage.section_core_map.find(section);
      if(initial==stage.section_core_map.end())return {ProcessStatus::NoCore,0,"no section Core"};
      std::string selected=initial->second;bool settled=false;checks=0;
      for(;checks<stage.max_health_checks;) {
        ++checks;auto& profile=*cores_.at(selected);profile.contract(frame);
        health_=commonHealth(profile.spec(),frame,clocks,stage_index==0);profile.validate(frame,health_);reportHealth(health_,checks);
        if(acceptable(profile.spec(),frame,health_)) {settled=true;break;}
        selected=overrideHealth(stage.health_core_map,health_,selected);
        cores_.at(selected)->apply(frame,health_);
      }
      if(!settled)return {ProcessStatus::Rejected,checks,"health check limit"};
      if(stage_index==0)markAttempted(snapshot_token_);
      if(!hasInputs(cores_.at(selected)->spec(),frame))return {ProcessStatus::NoCore,checks,"required core input missing"};
      frame=cores_.at(selected)->compute(frame);
    }
    output_=std::move(frame);return {ProcessStatus::Published,checks,{}};
  }
 private:
  friend class RosRuntime;
  void reportHealth(const ProfileHealth& health,std::size_t checks)const {report_health_(health,checks);}
  void markAttempted(const std::vector<std::uint64_t>& token){attempted_=true;token_=token;}
  static bool hasInputs(const CoreSpec& spec,const Frame& frame) {
    for(const auto& name:spec.inputs)if(!frame.at(name).value)return false;
    return true;
  }
  static std::string overrideHealth(const std::vector<HealthRule>& rules,const ProfileHealth& health,std::string selected) {
    const HealthRule* best=nullptr;bool tied=false;
    for(const auto& rule:rules) {
      if(rule.when.empty())throw std::invalid_argument("empty health when");
      for(const auto& condition:rule.when)health.issues(condition.input);
      const bool matches=std::all_of(rule.when.begin(),rule.when.end(),[&](const auto& condition) {
        const auto& issues=health.issues(condition.input);
        return std::find(issues.begin(),issues.end(),condition.kind)!=issues.end();
      });
      if(!matches)continue;
      if(!best || rule.priority>best->priority){best=&rule;tied=false;}
      else if(rule.priority==best->priority)tied=true;
    }
    if(tied)throw std::invalid_argument("ambiguous health_core_map priority");
    if(best)selected=best->core;
    return selected;
  }
  struct Snapshot {Frame data;std::vector<std::uint64_t> token;bool is_new;};
  Snapshot assemble(const std::string& trigger,std::uint64_t trigger_id) const {
    std::int64_t stamp=0;std::uint64_t id=0;bool found=false;
    for(const auto& name:input_order_)for(const auto& sample:queues_.at(name)) {
      if(assemblyReference(!trigger.empty(),name==trigger && sample.id==trigger_id,found,sample.stamp_ns,sample.id,stamp,id)) {
        stamp=sample.stamp_ns;id=sample.id;found=true;
      }
    }
    Snapshot snapshot{prototypes_,{},true};
    for(const auto& name:input_order_) {
      const auto* chosen=nearestSample(queues_.at(name),stamp,name==trigger,trigger_id);
      if(chosen)snapshot.data.at(name)=*chosen;
      snapshot.token.push_back(snapshot.data.at(name).id);
    }
    snapshot.is_new=!attempted_ || snapshot.token!=token_;
    return snapshot;
  }
  ProfileHealth commonHealth(const CoreSpec& spec,const Frame& frame,const Clocks& clocks,bool first_stage)const {
    ProfileHealth health;
    for(const auto& name:spec.inputs)health.inputs.push_back({name,{}});
    for(const auto& name:spec.inputs) {
      const auto& slot=frame.at(name);const auto found=limits_.find(name);
      auto limits=defaultLimits();if(found!=limits_.end())limits=found->second;
      std::vector<Kind> issues;
      if(!slot.value)issues.push_back(Kind::Missing);
      else {
        slot.inspect(slot.value.get(),limits,issues);
      }
      checkTimes(bool(slot.value),slot.stamp_ns,slot.value ? slot.receipt_ns : started_,slot.has_stamp,slot.has_receipt && (slot.value || first_stage),limits,clocks,issues);
      health.issues(name)=std::move(issues);
    }
    const auto skew_limit=spec.inputs.empty()?defaultLimits().max_skew_ns:limitsFor(spec.inputs.front()).max_skew_ns;
    for(std::size_t i=0;i<spec.inputs.size();++i)for(std::size_t j=i+1;j<spec.inputs.size();++j) {
      const auto& a=frame.at(spec.inputs[i]);const auto& b=frame.at(spec.inputs[j]);
      const auto la=limits_.count(spec.inputs[i])?limits_.at(spec.inputs[i]):defaultLimits();
      const auto lb=limits_.count(spec.inputs[j])?limits_.at(spec.inputs[j]):defaultLimits();
      if(a.value && b.value && a.has_stamp && b.has_stamp && la.check_skew && lb.check_skew && std::fabs(static_cast<long double>(a.stamp_ns)-b.stamp_ns)>skew_limit) {
        addIssue(health.issues(spec.inputs[i]),Kind::Skew);addIssue(health.issues(spec.inputs[j]),Kind::Skew);
      }
    }
    return health;
  }
  static bool acceptable(const CoreSpec& spec,const Frame& frame,const ProfileHealth& health) {
    if(!hasInputs(spec,frame))return false;
    for(const auto& input:health.inputs)for(auto issue:input.issues) {
      if(issue==Kind::Missing || issue==Kind::InvalidStamp || issue==Kind::FutureStamp || issue==Kind::FrameMismatch || issue==Kind::NaN || issue==Kind::Inf)return false;
      const auto allowed=spec.allowed_issues.find(input.input);
      if(allowed==spec.allowed_issues.end() || std::find(allowed->second.begin(),allowed->second.end(),issue)==allowed->second.end())return false;
    }
    return true;
  }
  HealthLimits limitsFor(const std::string& name)const {const auto found=limits_.find(name);return found==limits_.end()?defaultLimits():found->second;}
  static HealthLimits defaultLimits() {
    HealthLimits limits;limits.lower=-std::numeric_limits<double>::max();limits.upper=std::numeric_limits<double>::max();limits.frame.clear();return limits;
  }
  std::size_t capacity_;std::int64_t started_;std::uint64_t next_id_=0;
  std::map<std::string,std::deque<Slot>> queues_;
  Frame prototypes_,assembled_,output_;
  std::vector<std::string> input_order_;
  std::map<std::string,std::uint64_t> evictions_;
  std::function<void(const ProfileHealth&,std::size_t)> report_health_=[](const ProfileHealth&,std::size_t){};
  std::map<std::string,HealthLimits> limits_;
  std::map<std::string,std::unique_ptr<CoreBase>> cores_;
  std::vector<StageSpec> stages_;Section section_;ProfileHealth health_;
  bool attempted_=false,is_new_=true;std::vector<std::uint64_t> token_,snapshot_token_;
};
}}  // namespace node_pipeline::detail
