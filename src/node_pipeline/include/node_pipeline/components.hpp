#pragma once
// 메시지 타입을 보존하는 큐·조립·검사·정책·선택 공통 구현이다.
#include "pipeline.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <type_traits>

namespace node_pipeline {
// 공통 오류다. 노드 고유 오류는 Validator에서 추가할 수 있다.
enum class Kind { Missing, InvalidStamp, FutureStamp, FrameMismatch, NaN, Inf,
                  OutRangePositive, OutRangeNegative, Stale, ReceiveTimeout, Skew, Count };
// tuple의 모든 서로 다른 타입을 컴파일 시점에 순회한다.
template<class Tuple, class Fn, std::size_t... I>
void eachImpl(Tuple& tuple, Fn fn, std::index_sequence<I...>) {
  using Expand = int[];
  (void)Expand{0, (fn(std::integral_constant<std::size_t, I>{}, std::get<I>(tuple)), 0)...};
}
// const tuple도 같은 API로 읽을 수 있다.
template<class Tuple, class Fn>
void each(Tuple& tuple, Fn fn) {
  eachImpl(tuple, fn, std::make_index_sequence<std::tuple_size<typename std::remove_cv<typename std::remove_reference<Tuple>::type>::type>::value>{});
}
// boost/std shared_ptr의 원본 메시지와 소유권을 유지하는 adapter다.
template<class Owner> auto retainMessage(Owner owner) {
  using Message = typename std::remove_const<typename Owner::element_type>::type;
  return std::shared_ptr<const Message>(owner.get(), [owner](const Message*) {});
}
// 원본 타입 그대로 공유 소유하고, policy 수정 시에만 복사한다.
template<class Message> struct Sample {
  std::shared_ptr<const Message> message;
  std::uint64_t id = 0;
  std::int64_t stamp_ns = 0;
  std::int64_t receipt_steady_ns = 0;
};
// 누락은 nullptr로 표현한다. core 호출 연결은 메시지만 꺼내 전달한다.
template<class Message> struct Slot {
  Sample<Message> sample;
  bool present() const { return static_cast<bool>(sample.message); }
  const Message& message() const {
    if (!present()) throw std::invalid_argument("core input missing");
    return *sample.message;
  }
  // 큐 원본을 보존하며 선택 데이터만 copy-on-write로 수정한다.
  template<class Fn> void modify(Fn fn) {
    if (!present()) return;
    auto corrected = std::make_shared<Message>(*sample.message);
    fn(*corrected);
    sample.message = std::move(corrected);
  }
};
// 노드별 입력 타입들을 tuple에 보관한다.
template<class... Messages> struct Data {
  std::tuple<Slot<Messages>...> inputs;
  // Core가 선언한 필수 입력만 검사한다. 추가 입력은 제한하지 않는다.
  template<class Indices> bool hasInputs(const Indices& required) const {
    std::array<bool, sizeof...(Messages)> present{};
    each(inputs, [&](auto tag, const auto& slot) { present[decltype(tag)::value] = slot.present(); });
    return std::all_of(required.begin(), required.end(), [&](std::size_t i) {
      return i < present.size() && present[i];
    });
  }
  template<std::size_t I> auto& get() { return std::get<I>(inputs); }
  template<std::size_t I> const auto& get() const { return std::get<I>(inputs); }
};
// 입력 큐와 분리된 구역 정보다. ROS section callback에서 그대로 옮긴다.
struct Section { bool present = false; std::uint8_t state = 0; std::int64_t stamp_ns = 0; std::string frame = "map"; };
// 조립 기준 메타데이터는 core에 전달하지 않는다.
template<std::size_t N> struct Context {
  std::int64_t stamp_ns = 0;
  std::string frame;
  std::array<std::uint64_t, N> evictions{};
  Section section;
};
// 큐를 소유하는 객체다. Access는 메시지별 stamp/frame 필드 접근만 정의한다.
template<class Id, class Access, class... Messages> class InputBuffer {
 public:
  using InputId = Id;
  using DataType = Data<Messages...>;
  static constexpr std::size_t count = sizeof...(Messages);
  using ContextType = Context<count>;
  explicit InputBuffer(std::size_t capacity) : capacity_(capacity) {
    if (!capacity) throw std::invalid_argument("queue capacity must be positive");
  }
  bool containsInput(Id input) const { return static_cast<std::size_t>(input) < count; }
  // 입력 ID를 컴파일 시점에 지정하여 같은 메시지 타입도 서로 다른 입력으로 구별한다.
  template<Id Input, class Message> auto pushInput(std::integral_constant<Id, Input>, const Message& message, const Clocks& clocks) {
    return push<Input>(std::make_shared<Message>(message), clocks);
  }
  template<Id Input, class Message> auto pushInput(std::integral_constant<Id, Input>, const std::shared_ptr<const Message>& message, const Clocks& clocks) {
    return push<Input>(message, clocks);
  }
  void storeRawSection(const Section& section) { context_.section = section; }
  const auto& queues() const { return queues_; }
  ContextType context() const { return context_; }
 private:
  template<Id Input, class Message> auto push(std::shared_ptr<Message> message, const Clocks& clocks) {
    constexpr std::size_t i = static_cast<std::size_t>(Input);
    static_assert(i < count, "input index out of range");
    if (!message) throw std::invalid_argument("null input message");
    auto& queue = std::get<i>(queues_);
    using Item = typename std::decay<decltype(queue)>::type::value_type;
    Item sample{std::move(message), ++next_id_, 0, clocks.steady_ns};
    sample.stamp_ns = Access::stamp(*sample.message);
    queue.push_back(sample);
    if (queue.size() > capacity_) { queue.pop_front(); ++context_.evictions[i]; }
    return sample;
  }
  std::size_t capacity_;
  std::uint64_t next_id_ = 0;
  std::tuple<std::deque<Sample<Messages>>...> queues_;
  ContextType context_;
};
// 큐에서 선택만 수행하며 값이나 시각을 이유로 조립을 거부하지 않는다.
template<class Buffer, class Access> class SnapshotAssembler {
 public:
  using Token = std::array<std::uint64_t, Buffer::count>;
  struct Snapshot { typename Buffer::DataType data; typename Buffer::ContextType context; Token token{}; bool is_new = true; };
  Snapshot assemble(const Buffer& buffer, const Trigger<typename Buffer::InputId>& trigger) const {
    Snapshot snapshot;
    snapshot.context = buffer.context();
    bool found = false;
    std::uint64_t reference_id = 0;
    // ponytail: bounded linear scan; indexed queues only if measured assembly cost matters.
    each(buffer.queues(), [&](auto tag, const auto& queue) {
      for (const auto& sample : queue) {
        const bool candidate = trigger.mode == ExecutionMode::OnInput
          ? static_cast<std::size_t>(trigger.input) == tag && sample.id == trigger.sample_id
          : !found || sample.stamp_ns > snapshot.context.stamp_ns ||
            (sample.stamp_ns == snapshot.context.stamp_ns && sample.id > reference_id);
        if (candidate) {
          found = true; reference_id = sample.id;
          snapshot.context.stamp_ns = sample.stamp_ns;
          snapshot.context.frame = Access::frame(*sample.message);
        }
      }
    });
    each(buffer.queues(), [&](auto tag, const auto& queue) {
      auto& slot = snapshot.data.template get<decltype(tag)::value>();
      long double best = std::numeric_limits<long double>::infinity();
      for (const auto& sample : queue) {
        if (trigger.mode == ExecutionMode::OnInput && static_cast<std::size_t>(trigger.input) == tag && sample.id != reference_id) continue;
        const auto distance = std::fabs(static_cast<long double>(sample.stamp_ns) - snapshot.context.stamp_ns);
        if (distance < best || (distance == best && (!slot.present() || sample.id > slot.sample.id))) {
          best = distance; slot.sample = sample;
        }
      }
      snapshot.token[tag] = slot.present() ? slot.sample.id : 0;
    });
    snapshot.is_new = !attempted_ || snapshot.token != token_;
    return snapshot;
  }
  void markAttempted(const Token& token) { attempted_ = true; token_ = token; }
 private:
  bool attempted_ = false;
  Token token_{};
};
// 시간과 값의 한도는 config에서 주입하고 Validator는 필요한 필드만 검사한다.
struct HealthLimits {
  double lower = -1, upper = 1;
  std::int64_t stale_ns = 300000000, receive_timeout_ns = 300000000, max_skew_ns = 100000000;
  std::string frame = "map";
  bool check_stamp = true;
  bool check_receive_timeout = true;
  bool check_skew = true;
  void validate() const {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper || frame.empty() || stale_ns <= 0 || receive_timeout_ns <= 0 || max_skew_ns <= 0)
      throw std::invalid_argument("invalid health limits");
  }
};
template<class Id, std::size_t N> struct Health {
  std::array<InputHealth<Id, Kind>, N> inputs{};
  std::array<std::uint64_t, N> evictions{};
  std::uint8_t section = 0;
  bool invalid_section = false;
};
// stamp·timeout·skew는 공통 검사하고 값의 의미는 Validator에 맡긴다.
template<class Id, std::size_t N, class Validator> class HealthChecker {
 public:
  using HealthType = Health<Id, N>;
  HealthChecker(HealthLimits limits, std::int64_t started, Validator validator = {})
      : limits_(limits), started_(started), validator_(std::move(validator)) {
    limits_.validate(); input_limits_.fill(limits);
  }
  HealthChecker(std::array<HealthLimits, N> limits, std::int64_t started, Validator validator = {})
      : limits_(limits[0]), input_limits_(limits), started_(started), validator_(std::move(validator)) {
    for (const auto& limit : input_limits_) limit.validate();
  }
  template<class DataType, class ContextType>
  HealthType healthCheck(const DataType& data, const ContextType& context, const Clocks& clocks) const {
    HealthType health; health.evictions = context.evictions;
    std::array<bool, N> present{}; std::array<std::int64_t, N> stamps{};
    each(data.inputs, [&](auto tag, const auto& slot) {
      auto& result = health.inputs[tag]; result.input = static_cast<Id>(decltype(tag)::value);
      present[tag] = slot.present(); stamps[tag] = slot.sample.stamp_ns;
      if (!slot.present()) result.issues.push_back(Kind::Missing);
      else {
        validator_(*slot.sample.message, input_limits_[tag], result.issues);
        if (input_limits_[tag].check_stamp && slot.sample.stamp_ns <= 0) result.issues.push_back(Kind::InvalidStamp);
        if (input_limits_[tag].check_stamp && slot.sample.stamp_ns > clocks.ros_ns) result.issues.push_back(Kind::FutureStamp);
        if (input_limits_[tag].check_stamp && slot.sample.stamp_ns > 0 && exceeds(clocks.ros_ns, slot.sample.stamp_ns, input_limits_[tag].stale_ns)) result.issues.push_back(Kind::Stale);
      }
      if (input_limits_[tag].check_receive_timeout && exceeds(clocks.steady_ns, slot.present() ? slot.sample.receipt_steady_ns : started_, input_limits_[tag].receive_timeout_ns)) result.issues.push_back(Kind::ReceiveTimeout);
    });
    for (std::size_t i = 0; i < N; ++i) for (std::size_t j = i + 1; j < N; ++j) {
      if (!input_limits_[i].check_skew || !input_limits_[j].check_skew || !present[i] || !present[j] || std::fabs(static_cast<long double>(stamps[i]) - stamps[j]) <= limits_.max_skew_ns) continue;
      for (auto input : {i, j}) {
        auto& issues = health.inputs[input].issues;
        if (std::find(issues.begin(), issues.end(), Kind::Skew) == issues.end()) issues.push_back(Kind::Skew);
      }
    }
    if (context.section.present) {
      health.invalid_section = context.section.frame != "map" || context.section.stamp_ns <= 0 || context.section.stamp_ns > clocks.ros_ns;
      if (!health.invalid_section) health.section = context.section.state;
    }
    return health;
  }
 private:
  static bool exceeds(std::int64_t now, std::int64_t then, std::int64_t limit) { return static_cast<long double>(now) - then > limit; }
  HealthLimits limits_;
  std::array<HealthLimits, N> input_limits_; std::int64_t started_; Validator validator_;
};
// 메시지의 보정할 수치 필드 접근을 Access로 주입한다.
template<class Access> struct ClampUpper {
  double upper;
  template<class DataType, class IssueType> void operator()(DataType& data, const IssueType& issue) const {
    each(data.inputs, [&](auto tag, auto& slot) { if (tag == static_cast<std::size_t>(issue.input)) slot.modify([&](auto& message) { Access::value(message) = upper; }); });
  }
};
template<class Access> struct ClampLower {
  double lower;
  template<class DataType, class IssueType> void operator()(DataType& data, const IssueType& issue) const {
    each(data.inputs, [&](auto tag, auto& slot) { if (tag == static_cast<std::size_t>(issue.input)) slot.modify([&](auto& message) { Access::value(message) = lower; }); });
  }
};
// 데이터 타입과 무관하게 지정 입력을 Missing으로 바꾼다.
struct RemoveInput {
  // 공통 config 로더의 strategy 생성 인자를 받지만 수치 한도는 사용하지 않는다.
  RemoveInput(double = 0) {}
  template<class DataType, class IssueType> void operator()(DataType& data, const IssueType& issue) const {
    each(data.inputs, [&](auto tag, auto& slot) { if (tag == static_cast<std::size_t>(issue.input)) slot.sample.message.reset(); });
  }
};
// core 선택과 분리된 보정 우선순위와 strategy 라우팅이다.
template<class Id, class Upper, class Lower, class Remove = RemoveInput> class InputPolicy {
 public:
  InputPolicy(Upper upper, Lower lower, Remove remove = {}) : upper_(std::move(upper)), lower_(std::move(lower)), remove_(std::move(remove)) {}
  std::array<int, static_cast<std::size_t>(Kind::Count)> priority{{-1,200,200,200,200,200,100,100,-1,-1,-1}};
  template<class DataType, class HealthType> bool apply(DataType& data, const HealthType& health) const {
    Issue<Id, Kind> issue{};
    if (!highestPriorityIssue(health, [&](Id input, Kind kind) {
      // 이미 제거한 입력의 Missing·timeout은 다시 보정하지 않는다.
      const std::array<std::size_t, 1> required{{static_cast<std::size_t>(input)}};
      return data.hasInputs(required) ? priority[static_cast<std::size_t>(kind)] : -1;
    }, issue)) return false;
    switch (issue.kind) {
      case Kind::OutRangePositive: upper_(data, issue); break;
      case Kind::OutRangeNegative: lower_(data, issue); break;
      default: remove_(data, issue); break;
    }
    return true;
  }
 private:
  Upper upper_; Lower lower_; Remove remove_;
};
// 공통 수치 검사다. GPS NO_FIX 같은 특화 검사는 노드 Validator에 둔다.
inline void checkNumber(double value, const HealthLimits& limits, std::vector<Kind>& issues) {
  if (std::isnan(value)) issues.push_back(Kind::NaN);
  else if (std::isinf(value)) issues.push_back(Kind::Inf);
  else if (value > limits.upper) issues.push_back(Kind::OutRangePositive);
  else if (value < limits.lower) issues.push_back(Kind::OutRangeNegative);
}
// when의 input/issue 조건은 모두 일치해야 한다.
template<class Input, class CoreId> struct HealthCoreRule {
  // AND로 결합할 조건 목록이다. 비어 있는 규칙은 설정에서 거부한다.
  std::vector<Issue<Input, Kind>> when;
  // 동시에 일치한 규칙 중 가장 큰 priority를 선택한다.
  int priority = 0;
  // 이 규칙이 지정하는 계산 core다.
  CoreId core{};
};
// section 선택 후 일치하는 health 규칙으로 덮어쓴다.
template<class Input, class CoreId> struct CoreSelector {
  // 미수신 section은 UNKNOWN(0) 매핑을 사용한다.
  std::map<std::uint8_t, CoreId> section_core_map;
  std::vector<HealthCoreRule<Input, CoreId>> health_core_map;
  // 최종 health로 계산 core를 결정한다. input을 자동으로 보정하지 않는다.
  template<class HealthType> Selection<CoreId> select(std::uint8_t section, const HealthType& health) const {
    // section 매핑이 없으면 아직 선택된 core가 없는 상태로 시작한다.
    Selection<CoreId> selection{};
    // section의 기본 core를 조회한다.
    const auto base = section_core_map.find(section);
    // 지정된 section 매핑을 먼저 적용한다.
    if (base != section_core_map.end()) selection = {true, base->second};
    // 일치한 health 규칙을 모아 최고 priority를 찾는다.
    const HealthCoreRule<Input, CoreId>* best = nullptr;
    // 동률은 마지막까지 검사한다. 이후 더 높은 규칙이 있으면 그것이 우선한다.
    bool tied = false;
    // 각 규칙의 모든 조건을 검사한다.
    for (const auto& rule : health_core_map) {
      // 조건 목록이 비면 항상 일치하는 규칙이 되므로 거부한다.
      if (rule.when.empty()) throw std::invalid_argument("empty health when");
      // 지정하지 않은 input/issue는 제한하지 않는다.
      const bool matches = std::all_of(rule.when.begin(), rule.when.end(), [&](const auto& condition) {
        // 해당 input의 최종 오류 목록에서 지정한 issue를 찾는다.
        const auto& issues = health.inputs.at(static_cast<std::size_t>(condition.input)).issues;
        return std::find(issues.begin(), issues.end(), condition.kind) != issues.end();
      });
      // AND 조건 중 하나라도 실패하면 이 규칙은 적용하지 않는다.
      if (!matches) continue;
      // 더 높은 priority의 규칙으로 후보를 교체한다.
      if (!best || rule.priority > best->priority) { best = &rule; tied = false; }
      // 최고 priority가 같은 두 규칙은 임의 순서로 선택하지 않는다.
      else if (rule.priority == best->priority) tied = true;
    }
    // 동시에 일치한 최고 priority 동률은 명시적인 설정 오류다.
    if (tied) throw std::invalid_argument("ambiguous health_core_map priority");
    // 일치한 규칙이 있으면 section 선택을 덮어쓴다. 없으면 그대로 둔다.
    if (best) selection = {true, best->core};
    // 추가 Green 검사나 자동 fallback 없이 선택 결과를 반환한다.
    return selection;
  }
};


}  // namespace node_pipeline
