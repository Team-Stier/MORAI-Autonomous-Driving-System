#pragma once
// 입력 큐·조립·입력 계약·검사·선택·보정 구성 요소다. ROS에 의존하지 않는다.
#include <node_pipeline/detail/types.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <typeindex>

namespace node_pipeline {
namespace detail {

// 원본 메시지를 원래 타입 그대로 공유 소유하고 sample ID·시간 메타데이터를 함께 둔다.
template<class Message> struct Sample {
  std::shared_ptr<const Message> message;
  std::uint64_t id = 0;
  std::int64_t stamp_ns = 0;
  std::int64_t receipt_steady_ns = 0;
  bool has_stamp = false;
  bool has_receipt = false;
};
template<class Message> struct Slot {
  Sample<Message> sample;
  bool present() const { return static_cast<bool>(sample.message); }
  const Message& message() const { return *sample.message; }
};

// payload의 실제 타입과 소유권, 원본 시간 메타데이터를 보존하는 Stage 입력 슬롯이다.
struct StageSlot {
  std::shared_ptr<const void> message;
  std::type_index type{typeid(void)};
  std::uint64_t id = 0;
  std::int64_t stamp_ns = 0;
  std::int64_t receipt_steady_ns = 0;
  bool has_stamp = false;
  bool has_receipt = false;
  bool present() const { return static_cast<bool>(message); }
};

// Stage 입력 계약의 타입으로 StageSlot을 읽는다.
template<class Message> struct TypedSlot {
  const StageSlot& sample;
  bool present() const { return sample.present(); }
  const Message& message() const { return *static_cast<const Message*>(sample.message.get()); }
};
// StageData를 Stage 입력 계약의 타입으로 보는 view다. 입력 계약은 Core profile의 ContractCheck가 확인한다.
// 스냅샷에 없는 슬롯은 Missing으로 읽는다.
template<class Owner, class... Messages> class TypedData {
 public:
  explicit TypedData(Owner& data) : data_(data) {}
  template<std::size_t I> TypedSlot<AtT<I, Types<Messages...>>> slot() const { return {sample(I)}; }
  const StageSlot& sample(std::size_t index) const {
    static const StageSlot missing;
    return index < data_.inputs.size() ? data_.inputs[index] : missing;
  }
  auto& at(std::size_t index) const { return data_.inputs.at(index); }

 private:
  Owner& data_;
};

// 구독 메시지 타입들의 슬롯 튜플이다.
template<class... Messages> struct Data {
  static constexpr std::size_t count = sizeof...(Messages);
  using MessageTypes = Types<Messages...>;
  template<class Owner> using View = TypedData<Owner, Messages...>;
  std::tuple<Slot<Messages>...> inputs;
};

struct StageData {
  std::vector<StageSlot> inputs;
  template<class... Messages> void update(const Data<Messages...>& data) {
    inputs.resize(sizeof...(Messages));
    forEachIndex<sizeof...(Messages)>([&](auto tag) {
      constexpr std::size_t I = decltype(tag)::value;
      const auto& sample = std::get<I>(data.inputs).sample;
      inputs[I] = {sample.message, std::type_index(typeid(AtT<I, Types<Messages...>>)), sample.id, sample.stamp_ns,
                   sample.receipt_steady_ns, sample.has_stamp, sample.has_receipt};
    });
  }
  template<class DataType> typename DataType::template View<StageData> typed() {
    return typename DataType::template View<StageData>(*this);
  }
  template<class DataType> typename DataType::template View<const StageData> typed() const {
    return typename DataType::template View<const StageData>(*this);
  }
  template<class Indices> bool hasInputs(const Indices& indices) const {
    return std::all_of(indices.begin(), indices.end(), [&](std::size_t i) { return i < inputs.size() && inputs[i].present(); });
  }
  template<class Message> const Message& message(std::size_t index) const {
    if (index >= inputs.size() || !inputs[index].present()) throw std::invalid_argument("stage input missing");
    if (inputs[index].type != std::type_index(typeid(Message))) throw std::invalid_argument("incompatible stage input type");
    return *static_cast<const Message*>(inputs[index].message.get());
  }
};

// 구독 입력별 큐를 소유한다.
template<class... Messages> class InputBuffer {
 public:
  using DataType = Data<Messages...>;
  using Queues = std::tuple<std::deque<Sample<Messages>>...>;
  static constexpr std::size_t count = sizeof...(Messages);
  explicit InputBuffer(std::size_t capacity) : capacity_(capacity) {}
  bool containsInput(std::size_t input) const { return input < count; }
  template<std::size_t Input, class Message>
  Sample<Message> pushInput(std::integral_constant<std::size_t, Input> input_tag,
                            const std::shared_ptr<const Message>& retained_message, const Clocks& receive_clocks) {
    static_assert(std::is_same<Message, AtT<decltype(input_tag)::value, Types<Messages...>>>::value,
                  "input message type must match node.yaml");
    Sample<Message> sample;
    sample.message = retained_message;
    sample.id = ++next_id_;
    sample.has_stamp = Access::stamp(*retained_message, sample.stamp_ns);
    sample.receipt_steady_ns = receive_clocks.steady_ns;
    sample.has_receipt = true;
    auto& queue = std::get<Input>(queues_);
    queue.push_back(sample);
    if (queue.size() > capacity_) queue.pop_front();
    return sample;
  }
  void storeRawSection(const Section& section) { section_ = section; }
  const Queues& queues() const { return queues_; }
  const Section& section() const { return section_; }

 private:
  std::size_t capacity_;
  std::uint64_t next_id_ = 0;
  Queues queues_;
  Section section_;
};

// 누락·오래됨·invalid 여부와 무관하게 항상 스냅샷을 반환한다.
template<class Buffer> class SnapshotAssembler {
 public:
  using Token = std::vector<std::uint64_t>;
  struct Assembled {
    typename Buffer::DataType data;
    Context context;
    Token token;
    bool is_new = true;
  };
  explicit SnapshotAssembler(int reference_input) : reference_input_(reference_input) {}
  Assembled assemble(const Buffer& buffer, const Trigger& trigger) const {
    Assembled assembled;
    assembled.context.section = buffer.section();
    assembled.token.assign(Buffer::count, 0);
    const auto& queues = buffer.queues();
    bool found = false;
    bool fixed = false;
    std::size_t reference_input = 0;
    std::uint64_t reference_id = 0;
    std::int64_t& stamp_ns = assembled.context.stamp_ns;
    const bool use_reference = trigger.mode == ExecutionMode::Periodic && reference_input_ >= 0 &&
                               queueSize(queues, static_cast<std::size_t>(reference_input_)) > 0;
    forEachIndex<Buffer::count>([&](auto tag) {
      constexpr std::size_t I = decltype(tag)::value;
      for (const auto& sample : std::get<I>(queues)) {
        const bool later = !found || sample.stamp_ns > stamp_ns || (sample.stamp_ns == stamp_ns && sample.id > reference_id);
        const bool candidate =
            trigger.mode == ExecutionMode::OnInput ? I == trigger.input && sample.id == trigger.sample_id
            : trigger.mode == ExecutionMode::Periodic ? (!use_reference || I == static_cast<std::size_t>(reference_input_)) && later
            : false;
        if (!candidate) continue;
        found = true;
        fixed = trigger.mode == ExecutionMode::OnInput || use_reference;
        reference_input = I;
        reference_id = sample.id;
        stamp_ns = sample.stamp_ns;
      }
    });
    forEachIndex<Buffer::count>([&](auto tag) {
      constexpr std::size_t I = decltype(tag)::value;
      auto& slot = std::get<I>(assembled.data.inputs);
      long double best = std::numeric_limits<long double>::infinity();
      for (const auto& sample : std::get<I>(queues)) {
        if (fixed && I == reference_input) {
          if (sample.id == reference_id) slot.sample = sample;
          continue;
        }
        const auto distance = found ? std::fabs(static_cast<long double>(sample.stamp_ns) - stamp_ns) : 0.0L;
        if (distance < best || (distance == best && (!slot.present() || sample.id > slot.sample.id))) {
          best = distance;
          slot.sample = sample;
        }
      }
      assembled.token[I] = slot.present() ? slot.sample.id : 0;
    });
    assembled.is_new = !attempted_ || assembled.token != token_;
    return assembled;
  }
  void markAttempted(const Token& token) {
    attempted_ = true;
    token_ = token;
  }

 private:
  template<class Queues> static std::size_t queueSize(const Queues& queues, std::size_t input) {
    std::size_t size = 0;
    forEachIndex<Buffer::count>([&](auto tag) {
      if (decltype(tag)::value == input) size = std::get<decltype(tag)::value>(queues).size();
    });
    return size;
  }
  int reference_input_;
  bool attempted_ = false;
  Token token_;
};

inline bool exceeds(std::int64_t now, std::int64_t then, std::int64_t limit) {
  return static_cast<long double>(now) - then > limit;
}
// 공통 수치 검사다.
inline void checkNumber(double value, const HealthLimits& limits, std::vector<Kind>& issues) {
  if (std::isnan(value)) issues.push_back(Kind::NaN);
  else if (std::isinf(value)) issues.push_back(Kind::Inf);
  else if (value > limits.upper) issues.push_back(Kind::OutRangePositive);
  else if (value < limits.lower) issues.push_back(Kind::OutRangeNegative);
}

template<class V, class T, class = void> struct CanValidate : std::false_type {};
template<class V, class T> struct CanValidate<V, T, decltype(void(std::declval<const V&>()(
    std::declval<const T&>(), std::declval<const HealthLimits&>(), std::declval<std::vector<Kind>&>())))> : std::true_type {};
template<class Validator, class Message>
void validateMessage(const Message& message, const HealthLimits& limits, std::vector<Kind>& issues, std::true_type) {
  const Validator validator{};
  validator(message, limits, issues);
}
template<class Validator, class Message>
void validateMessage(const Message&, const HealthLimits&, std::vector<Kind>&, std::false_type) {}
template<class Validator, class Message>
void validateWith(const Message& message, const HealthLimits& limits, std::vector<Kind>& issues) {
  validateMessage<Validator>(message, limits, issues, CanValidate<Validator, Message>{});
}
template<class T, class... Ts> struct IndexOf;
template<class T, class... Ts> struct IndexOf<T, T, Ts...> : std::integral_constant<std::size_t, 0> {};
template<class T, class U, class... Ts> struct IndexOf<T, U, Ts...>
    : std::integral_constant<std::size_t, 1 + IndexOf<T, Ts...>::value> {};

// Core에 등록한 Validator를 Stage 입력 타입별로 호출한다. 오버로드가 없는 타입은 검사하지 않는다.
template<class DataType> class ProfileValidator;
template<class... Messages> class ProfileValidator<Data<Messages...>> {
 public:
  template<class Validator> static ProfileValidator of() {
    static_assert(std::is_empty<Validator>::value, "Validator must be a stateless type");
    ProfileValidator validator;
    validator.checks_ = std::make_tuple(&validateWith<Validator, Messages>...);
    return validator;
  }
  template<class Message> void operator()(const Message& message, const HealthLimits& limits, std::vector<Kind>& issues) const {
    std::get<IndexOf<Message, Messages...>::value>(checks_)(message, limits, issues);
  }

 private:
  std::tuple<void (*)(const Messages&, const HealthLimits&, std::vector<Kind>&)...> checks_{};
};

// 공통 검사 후 선택된 Core의 Validator를 호출한다.
template<class Schema> class HealthChecker {
 public:
  using DataType = typename Schema::DataType;
  using Numbers = typename Schema::Numbers;
  using HealthType = Health<DataType::count>;
  HealthChecker(std::string stage, std::vector<std::string> names, std::vector<HealthLimits> input_limits,
                std::vector<bool> subscribed, std::int64_t max_skew_ns, std::int64_t started,
                ProfileValidator<DataType> validator)
      : stage_(std::move(stage)), names_(std::move(names)), input_limits_(std::move(input_limits)),
        subscribed_(std::move(subscribed)), max_skew_ns_(max_skew_ns), started_(started), validator_(validator) {}
  template<class Typed>
  HealthType healthCheck(const Typed& typed_data, const Context& context, const Clocks& health_clocks) const {
    static_cast<void>(context);
    HealthType health;
    health.stage = stage_;
    forEachIndex<DataType::count>([&](auto tag) {
      constexpr std::size_t I = decltype(tag)::value;
      const auto slot = typed_data.template slot<I>();
      health.inputs[tag].input = tag;
      health.inputs[tag].name = names_[tag];
      auto& issues = health.inputs[tag].issues;
      if (!slot.present()) {
        issues.push_back(Kind::Missing);
        if (subscribed_[tag] && input_limits_[tag].check_receive_timeout &&
            exceeds(health_clocks.steady_ns, started_, input_limits_[tag].receive_timeout_ns))
          issues.push_back(Kind::ReceiveTimeout);
      } else {
        if (!input_limits_[tag].frame.empty() && Access::frame(slot.message()) != input_limits_[tag].frame)
          issues.push_back(Kind::FrameMismatch);
        for (const double value : AtT<I, Numbers>::collect(slot.message())) checkNumber(value, input_limits_[tag], issues);
        if (input_limits_[tag].check_stamp && slot.sample.has_stamp) {
          if (slot.sample.stamp_ns <= 0) issues.push_back(Kind::InvalidStamp);
          if (slot.sample.stamp_ns > health_clocks.ros_ns) issues.push_back(Kind::FutureStamp);
          if (slot.sample.stamp_ns > 0 && exceeds(health_clocks.ros_ns, slot.sample.stamp_ns, input_limits_[tag].stale_ns))
            issues.push_back(Kind::Stale);
        }
        if (input_limits_[tag].check_receive_timeout && slot.sample.has_receipt &&
            exceeds(health_clocks.steady_ns, slot.sample.receipt_steady_ns, input_limits_[tag].receive_timeout_ns))
          issues.push_back(Kind::ReceiveTimeout);
        if (skewed(typed_data, tag)) issues.push_back(Kind::Skew);
        validator_(slot.message(), input_limits_[tag], issues);
      }
    });
    return health;
  }

 private:
  template<class Typed> bool skewed(const Typed& typed_data, std::size_t tag) const {
    const auto& slot = typed_data.sample(tag);
    if (!input_limits_[tag].check_skew || !slot.has_stamp) return false;
    for (std::size_t other = 0; other < input_limits_.size(); ++other) {
      const auto& pair = typed_data.sample(other);
      if (other != tag && input_limits_[other].check_skew && pair.present() && pair.has_stamp &&
          std::fabs(static_cast<long double>(slot.stamp_ns) - pair.stamp_ns) > max_skew_ns_)
        return true;
    }
    return false;
  }
  std::string stage_;
  std::vector<std::string> names_;
  std::vector<HealthLimits> input_limits_;
  std::vector<bool> subscribed_;
  std::int64_t max_skew_ns_;
  std::int64_t started_;
  ProfileValidator<DataType> validator_;
};

// section 매핑으로 Core를 선택하고 health 매핑으로 덮어쓴다.
constexpr std::uint8_t kUnknownSection = 0;
struct HealthCondition {
  std::size_t input = 0;
  Kind kind = Kind::Missing;
};
template<class CoreId> struct HealthCoreRule {
  std::vector<HealthCondition> when;
  int priority = 0;
  CoreId core{};
};
template<class CoreId> struct CoreSelector {
  std::map<std::uint8_t, CoreId> section_core_map;
  std::vector<HealthCoreRule<CoreId>> health_core_map;
  Selection<CoreId> selectSection(const Section& context_section, const Clocks& health_clocks) const {
    std::uint8_t section = kUnknownSection;
    if (!context_section.present || context_section.frame != "map" || context_section.stamp_ns <= 0 ||
        context_section.stamp_ns > health_clocks.ros_ns)
      section = kUnknownSection;
    else
      section = context_section.state;
    Selection<CoreId> selection{};
    const auto base = section_core_map.find(section);
    if (base != section_core_map.end()) selection = {true, base->second};
    return selection;
  }
  template<class HealthType>
  Selection<CoreId> overrideHealth(const Selection<CoreId>& selection, const HealthType& health) const {
    const HealthCoreRule<CoreId>* best = nullptr;
    bool tied = false;
    for (const auto& rule : health_core_map) {
      if (rule.when.empty()) throw std::invalid_argument("empty health when");
      const bool matches = std::all_of(rule.when.begin(), rule.when.end(), [&](const HealthCondition& condition) {
        const auto& issues = health.inputs.at(condition.input).issues;
        return std::find(issues.begin(), issues.end(), condition.kind) != issues.end();
      });
      if (!matches) continue;
      if (!best || rule.priority > best->priority) {
        best = &rule;
        tied = false;
      } else if (rule.priority == best->priority) {
        tied = true;
      }
    }
    if (tied) throw std::invalid_argument("ambiguous health_core_map priority");
    Selection<CoreId> next_selection = selection;
    if (best) next_selection = {true, best->core};
    return next_selection;
  }
};

// 선택된 Core에 등록한 Policy로 선언 입력의 최고 priority issue 하나를 보정한다.
template<class Policy> int policyPriority(Kind kind) { return Policy{}.priority(kind); }
template<class Policy> void policyCorrect(EditableInput& input, Kind kind) { Policy{}.correct(input, kind); }
class InputPolicy {
 public:
  template<class Policy> static InputPolicy of(std::vector<std::size_t> inputs) {
    return make(std::move(inputs), static_cast<Policy*>(nullptr));
  }
  template<class Typed, class HealthType> bool apply(Typed&& typed_data, const HealthType& health) const {
    if (!priority_) return false;
    int best = -1;
    std::size_t target = 0;
    Kind kind = Kind::Missing;
    for (const auto input : inputs_) {
      if (!typed_data.sample(input).present()) continue;
      for (const auto issue : health.inputs.at(input).issues) {
        const int rank = priority_(issue);
        if (rank > best) {
          best = rank;
          target = input;
          kind = issue;
        }
      }
    }
    if (best < 0) return false;
    auto& slot = typed_data.at(target);
    EditableInput input(slot.message, slot.type);
    correct_(input, kind);
    return true;
  }

 private:
  template<class Policy> static InputPolicy make(std::vector<std::size_t> inputs, Policy*) {
    static_assert(std::is_empty<Policy>::value, "Policy must be a stateless type");
    InputPolicy policy;
    policy.inputs_ = std::move(inputs);
    std::sort(policy.inputs_.begin(), policy.inputs_.end());
    policy.priority_ = &policyPriority<Policy>;
    policy.correct_ = &policyCorrect<Policy>;
    return policy;
  }
  static InputPolicy make(std::vector<std::size_t>, NoPolicy*) { return {}; }
  std::vector<std::size_t> inputs_;
  int (*priority_)(Kind) = nullptr;
  void (*correct_)(EditableInput&, Kind) = nullptr;
};

// 등록 YAML에서 생성한 Core profile이다.
template<class Schema> struct CoreProfile {
  using DataType = typename Schema::DataType;
  using CoreId = typename Schema::CoreId;
  using HealthType = Health<DataType::count>;
  CoreId id{};
  std::string name;
  std::vector<std::size_t> argument_inputs;
  std::vector<std::size_t> required_inputs;
  std::vector<std::vector<Kind>> allowed_issues;
  std::string validator_name;
  std::string policy_name;
  ProfileValidator<DataType> validator;
  std::function<HealthType(const StageData&, const Context&, const Clocks&)> ContractCheck;
  InputPolicy policy;
};
}  // namespace detail
}  // namespace node_pipeline
