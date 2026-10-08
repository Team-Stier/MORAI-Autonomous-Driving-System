#pragma once
// Stage 구성과 Core 반환값의 슬롯 연결이다.
#include <node_pipeline/detail/components.hpp>

namespace node_pipeline {
namespace detail {

// Core 반환값을 다음 Stage 입력 슬롯으로 감싼다. from 출력은 원본 슬롯의 시간 메타데이터를 상속한다.
using CoreResult = std::function<StageData(const StageData&)>;

inline void appendResult(StageData& output, std::shared_ptr<const void> message, std::type_index type,
                         const StageData& input, const std::vector<int>& origins) {
  const int origin = origins.at(output.inputs.size());
  if (origin < -1) throw std::invalid_argument("invalid result origin");
  StageSlot slot;
  if (origin >= 0) slot = input.inputs.at(static_cast<std::size_t>(origin));
  slot.message = std::move(message);
  slot.type = type;
  output.inputs.push_back(std::move(slot));
}
template<class Value> void appendValue(StageData& output, Value&& value, const StageData& input, const std::vector<int>& origins) {
  using Message = typename std::decay<Value>::type;
  appendResult(output, std::make_shared<Message>(std::forward<Value>(value)), std::type_index(typeid(Message)), input, origins);
}
template<class Value> void packResult(StageData& output, Value&& value, const StageData& input, const std::vector<int>& origins) {
  appendValue(output, std::forward<Value>(value), input, origins);
}
template<class... Values>
void packResult(StageData& output, std::tuple<Values...>&& values, const StageData& input, const std::vector<int>& origins) {
  forEachIndex<sizeof...(Values)>([&](auto tag) {
    appendValue(output, std::move(std::get<decltype(tag)::value>(values)), input, origins);
  });
}
template<class A, class B>
void packResult(StageData& output, std::pair<A, B>&& values, const StageData& input, const std::vector<int>& origins) {
  appendValue(output, std::move(values.first), input, origins);
  appendValue(output, std::move(values.second), input, origins);
}
template<class Value> CoreResult wrapResult(Value&& value, std::vector<int> origins) {
  using Result = typename std::decay<Value>::type;
  auto owned = std::make_shared<Result>(std::forward<Value>(value));
  return [owned, origins](const StageData& input) {
    StageData output;
    packResult(output, std::move(*owned), input, origins);
    if (origins.size() != output.inputs.size()) throw std::invalid_argument("invalid result origin count");
    return output;
  };
}

// 설정 단계에서 읽은 Core·Bank 파라미터 값이다. 생성된 로더 진입점이 실제 타입으로 만든다.
using ParameterLoader = std::shared_ptr<const void> (*)(const ParamReader&);
template<class Entry> std::shared_ptr<const void> loadErased(const ParamReader& reader) {
  using Parameters = decltype(Entry::loadParameters(reader));
  return std::make_shared<const Parameters>(Entry::loadParameters(reader));
}

// CoreSet이 소유하는 Bank다. 노드마다 하나 생성되며 Core 선택이 바뀌어도 유지된다.
template<class... Banks> using BankSet = std::tuple<std::shared_ptr<Banks>...>;

struct Snapshot {
  StageData data;
  Context context;
  std::vector<std::uint64_t> token;
  bool is_new = true;
};
// 반환값으로 data.inputs만 교체하고 context·token·is_new는 이어받는다.
inline void replaceInputs(Snapshot& snapshot, const CoreResult& result) { snapshot.data.inputs = result(snapshot.data).inputs; }

template<class Schema, class CoreSet> struct Stage {
  using DataType = typename Schema::DataType;
  using CoreId = typename Schema::CoreId;
  using Profile = CoreProfile<Schema>;
  using Dispatch = bool (*)(CoreSet&, CoreId, const StageData&, CoreResult&);
  std::vector<Profile> core_profiles;
  CoreSelector<CoreId> selector;
  Dispatch process = nullptr;
  const Profile& coreProfile(CoreId id) const {
    const auto found = std::find_if(core_profiles.begin(), core_profiles.end(), [&](const Profile& profile) { return profile.id == id; });
    if (found == core_profiles.end()) throw std::invalid_argument("unregistered core id");
    return *found;
  }
  // 선택된 Core의 선언 입력 issue가 모두 허용 issue인지 판정한다. 선언 입력의 Missing은 면제하지 않는다.
  template<class HealthType> bool acceptsIssues(CoreId id, const HealthType& health) const {
    const auto& profile = coreProfile(id);
    for (const auto input : profile.required_inputs)
      for (const auto issue : health.inputs.at(input).issues) {
        if (issue == Kind::Missing) return false;
        const auto& allowed = profile.allowed_issues.at(input);
        if (std::find(allowed.begin(), allowed.end(), issue) == allowed.end()) return false;
      }
    return true;
  }
  bool complete() const {
    return process && std::all_of(core_profiles.begin(), core_profiles.end(),
                                  [](const Profile& profile) { return static_cast<bool>(profile.ContractCheck); });
  }
};
template<class Schema, class CoreSet>
Stage<Schema, CoreSet> makeStage(std::vector<CoreProfile<Schema>> core_profiles, CoreSelector<typename Schema::CoreId> selector,
                                 bool (*dispatch)(CoreSet&, typename Schema::CoreId, const StageData&, CoreResult&)) {
  Stage<Schema, CoreSet> stage;
  stage.core_profiles = std::move(core_profiles);
  stage.selector = std::move(selector);
  stage.process = dispatch;
  return stage;
}

template<class Schema, class Validator, class Policy>
CoreProfile<Schema> makeProfile(typename Schema::CoreId id, std::string name, std::vector<std::size_t> required_inputs,
                                std::string validator_name, std::string policy_name) {
  CoreProfile<Schema> profile;
  profile.id = id;
  profile.name = std::move(name);
  profile.required_inputs = std::move(required_inputs);
  profile.allowed_issues.assign(Schema::DataType::count, {});
  profile.validator_name = std::move(validator_name);
  profile.policy_name = std::move(policy_name);
  profile.validator = ProfileValidator<typename Schema::DataType>::template of<Validator>();
  profile.policy = InputPolicy::of<Policy>(profile.required_inputs);
  return profile;
}
// 입력 계약을 확인한 뒤 typed data로 health를 생성한다.
template<class Schema>
std::function<typename CoreProfile<Schema>::HealthType(const StageData&, const Context&, const Clocks&)>
makeContractCheck(std::shared_ptr<const HealthChecker<Schema>> checker, std::vector<std::size_t> required_inputs) {
  using Messages = typename Schema::DataType::MessageTypes;
  return [checker, required_inputs](const StageData& data, const Context& context, const Clocks& health_clocks) {
    for (const auto input : required_inputs)
      if (input >= data.inputs.size()) throw std::invalid_argument("core input slot missing");
    forEachIndex<Messages::size>([&](auto tag) {
      if (tag < data.inputs.size() && data.inputs[tag].present() &&
          data.inputs[tag].type != std::type_index(typeid(AtT<decltype(tag)::value, Messages>)))
        throw std::invalid_argument("stage input type mismatch");
    });
    const auto typed_data = data.template typed<typename Schema::DataType>();
    return checker->healthCheck(typed_data, context, health_clocks);
  };
}

// process()가 선언 입력의 const&를 순서대로 받고 Stage 출력 계약대로 반환하는지 컴파일 시 확인한다.
template<class F> struct MemberFunction;
template<class C, class R, class... A> struct MemberFunction<R (C::*)(A...)> {
  using Result = R;
  using Arguments = Types<A...>;
};
template<class C, class R, class... A> struct MemberFunction<R (C::*)(A...) const> : MemberFunction<R (C::*)(A...)> {};
template<class Inputs, class Indices> struct ExpectedArguments;
template<class Inputs, std::size_t... I> struct ExpectedArguments<Inputs, std::index_sequence<I...>> {
  using type = Types<const AtT<I, Inputs>&...>;
};
template<class Outputs> struct ExpectedResult;
template<class T> struct ExpectedResult<Types<T>> { using type = T; };
template<class A, class B, class... Rest> struct ExpectedResult<Types<A, B, Rest...>> { using type = std::tuple<A, B, Rest...>; };
template<class Result, class Outputs> struct ResultMatches : std::is_same<Result, typename ExpectedResult<Outputs>::type> {};
template<class A, class B> struct ResultMatches<std::pair<A, B>, Types<A, B>> : std::true_type {};
template<class Core, class Schema, class Indices> constexpr bool requireProcessContract() {
  using Function = MemberFunction<decltype(&Core::process)>;
  static_assert(std::is_same<typename Function::Arguments,
                             typename ExpectedArguments<typename Schema::DataType::MessageTypes, Indices>::type>::value,
                "process() must take const references to the registered inputs in order");
  static_assert(ResultMatches<typename Function::Result, typename Schema::Outputs>::value,
                "process() must return the Stage outputs declared in node.yaml");
  return true;
}
}  // namespace detail
}  // namespace node_pipeline
