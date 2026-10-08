#pragma once
// 공통 main이 사용하는 설정 로드와 ROS 연결이다.
#include <node_pipeline/detail/pipeline.hpp>
#include <interfaces/SectionAnnotation.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <chrono>

namespace node_pipeline {
namespace detail {

// YAML 설정 파일의 읽기 전용 노드다. 키가 없거나 타입이 다르면 std::invalid_argument로 종료한다.
class ConfigNode {
 public:
  struct Impl;
  ConfigNode();
  static ConfigNode fromFile(const std::string& path);
  bool defined() const;
  bool isNull() const;
  bool isMap() const;
  bool isList() const;
  ConfigNode operator[](const std::string& key) const;
  ConfigNode at(std::size_t index) const;
  std::size_t size() const;
  std::vector<std::string> keys() const;
  std::string text() const;
  double number() const;
  std::int64_t integer() const;
  bool flag() const;
  std::string where() const;

 private:
  explicit ConfigNode(std::shared_ptr<const Impl> impl);
  std::shared_ptr<const Impl> impl_;
};

// 설정 파일 위치와 생성된 파라미터 로더다. 파일은 loadRosConfig가 읽는다.
struct NodeParams {
  std::string package;
  std::string node;
  std::string config_dir;
  std::vector<std::string> cores;
  std::map<std::string, ParameterLoader> loaders;
};

struct StageParams {
  std::string name;
  std::vector<std::string> slots;
  std::vector<HealthLimits> limits;
  std::vector<bool> subscribed;
  std::vector<int> origins;
  std::int64_t max_skew_ns = 0;
  std::int64_t started_steady_ns = 0;
  bool last = false;
  std::string output_frame;
  ConfigNode selection;
  std::map<std::string, ConfigNode> registrations;
};
struct RosSettings {
  ExecutionConfig execution;
  std::size_t queue_capacity = 8;
  double frequency_hz = 0;
  int reference_input = -1;
  std::string section_topic;
  std::string output_topic;
  std::string output_frame;
  std::vector<StageParams> stages;
  std::map<std::string, std::shared_ptr<const void>> parameters;
};
// 빌드 시 복사한 node.yaml·등록 YAML과 selection.yaml·파라미터 파일을 읽고 입력 토픽을 해석한다.
RosSettings loadRosConfig(ros::NodeHandle& nh, const NodeParams& params, std::vector<std::string>& topics,
                          const std::map<std::string, std::size_t>& inputs);
// 등록 YAML을 생성된 Core profile과 대조하고 허용 issue를 읽는다.
void loadRegistration(const ConfigNode& registration, const StageParams& stage_params,
                      const std::map<std::string, std::size_t>& input_registry, const std::vector<std::size_t>& required_inputs,
                      const std::string& validator_name, const std::string& policy_name,
                      std::vector<std::vector<Kind>>& allowed_issues);
std::uint8_t sectionKey(const std::string& key, const std::string& stage);
std::string coreStem(const std::string& core);

template<class Messages> std::vector<bool> framesAvailable() {
  std::vector<bool> frames(Messages::size);
  forEachIndex<Messages::size>([&](auto tag) { frames[tag] = Access::hasFrame<AtT<decltype(tag)::value, Messages>>(); });
  return frames;
}

// Stage 입력 계약·등록·selection.yaml을 읽어 Core profile과 선택기를 완성한다.
template<class Schema>
void loadStageConfig(const StageParams& stage_params, const std::map<std::string, std::size_t>& input_registry,
                     std::vector<CoreProfile<Schema>>& core_profiles, CoreSelector<typename Schema::CoreId>& selector) {
  using DataType = typename Schema::DataType;
  using CoreId = typename Schema::CoreId;
  const auto& name = stage_params.name;
  if (stage_params.slots.size() != DataType::count || input_registry.size() != DataType::count)
    throw std::invalid_argument("stage inputs do not match the build: " + name);
  for (std::size_t i = 0; i < stage_params.slots.size(); ++i) {
    const auto found = input_registry.find(stage_params.slots[i]);
    if (found == input_registry.end() || found->second != i)
      throw std::invalid_argument("stage inputs do not match the build: " + name);
  }
  if (stage_params.origins != Schema::origins()) throw std::invalid_argument("stage outputs do not match the build: " + name);
  const auto frames = framesAvailable<typename DataType::MessageTypes>();
  for (std::size_t i = 0; i < frames.size(); ++i)
    if (!stage_params.limits[i].frame.empty() && !frames[i])
      throw std::invalid_argument("frame declared for a message without a frame: " + stage_params.slots[i]);
  if (stage_params.last &&
      Access::hasFrame<AtT<0, typename Schema::Outputs>>() == stage_params.output_frame.empty())
    throw std::invalid_argument("output frame_id must be set exactly for messages with a frame");

  std::map<std::string, CoreId> core_ids;
  for (auto& profile : core_profiles) {
    const auto registration = stage_params.registrations.find(profile.name);
    if (registration == stage_params.registrations.end())
      throw std::invalid_argument("registration does not match the build: " + profile.name);
    loadRegistration(registration->second, stage_params, input_registry, profile.required_inputs, profile.validator_name,
                     profile.policy_name, profile.allowed_issues);
    core_ids[profile.name] = profile.id;
  }
  if (stage_params.registrations.size() != core_profiles.size())
    throw std::invalid_argument("registrations do not match the build: " + name);
  const auto coreId = [&](const std::string& core) {
    const auto found = core_ids.find(core);
    if (found == core_ids.end()) throw std::invalid_argument("core is not registered in stage " + name + ": " + core);
    return found->second;
  };

  const auto& selection = stage_params.selection;
  if (selection.defined() && !selection.isNull()) {
    if (!selection.isMap()) throw std::invalid_argument("stage selection must be a map: " + name);
    for (const auto& key : selection.keys())
      if (key != "section_core_map" && key != "health_core_map")
        throw std::invalid_argument("unknown key '" + key + "' in selection of stage " + name);
    const auto sections = selection["section_core_map"];
    if (sections.defined() && !sections.isNull()) {
      if (!sections.isMap()) throw std::invalid_argument("section_core_map must be a map: " + name);
      for (const auto& key : sections.keys()) {
        const auto core = sections[key].text();
        if (!core.empty()) selector.section_core_map[sectionKey(key, name)] = coreId(core);
      }
    }
    const auto rules = selection["health_core_map"];
    if (rules.defined() && !rules.isNull()) {
      if (!rules.isList()) throw std::invalid_argument("health_core_map must be a list: " + name);
      for (std::size_t r = 0; r < rules.size(); ++r) {
        const auto row = rules.at(r);
        const auto when = row["when"];
        if (!row.isMap() || !when.isList() || when.size() == 0)
          throw std::invalid_argument("invalid health mapping rule in stage " + name);
        HealthCoreRule<CoreId> rule;
        rule.priority = static_cast<int>(row["priority"].integer());
        rule.core = coreId(row["core"].text());
        for (std::size_t c = 0; c < when.size(); ++c) {
          const auto input = input_registry.find(when.at(c)["input"].text());
          if (input == input_registry.end()) throw std::invalid_argument("unknown input in health rule of stage " + name);
          rule.when.push_back({input->second, kindFromName(when.at(c)["issue"].text())});
        }
        selector.health_core_map.push_back(std::move(rule));
      }
    }
  }
  if (!core_profiles.empty() && !selector.section_core_map.count(kUnknownSection))
    throw std::invalid_argument("UNKNOWN mapping required in stage " + name);

  for (auto& profile : core_profiles) {
    const auto checker = std::make_shared<const HealthChecker<Schema>>(
        name, stage_params.slots, stage_params.limits, stage_params.subscribed, stage_params.max_skew_ns,
        stage_params.started_steady_ns, profile.validator);
    profile.ContractCheck = makeContractCheck<Schema>(checker, profile.required_inputs);
  }
}

// Stage 순서대로 Stage 객체를 만든다.
template<class CoreSet, class... Schemas, class Fn>
std::tuple<Stage<Schemas, CoreSet>...> makeStages(Types<Schemas...>, Fn&& fn) {
  return std::tuple<Stage<Schemas, CoreSet>...>{fn(TypeTag<Schemas>{})...};
}

// 설정 단계에서 읽은 파라미터 값으로 Bank와 Core 객체를 만든다. Core는 파라미터 값, Bank 참조 순서로 받는다.
template<class Entry> const auto& parametersOf(const std::map<std::string, std::shared_ptr<const void>>& parameters) {
  using Parameters = decltype(Entry::loadParameters(std::declval<const ParamReader&>()));
  return *static_cast<const Parameters*>(parameters.at(Entry::name()).get());
}
template<class Entry> std::shared_ptr<typename Entry::Type> makeBank(const std::map<std::string, std::shared_ptr<const void>>& parameters,
                                                                     std::true_type) {
  return std::make_shared<typename Entry::Type>(parametersOf<Entry>(parameters));
}
template<class Entry> std::shared_ptr<typename Entry::Type> makeBank(const std::map<std::string, std::shared_ptr<const void>>&,
                                                                     std::false_type) {
  return std::make_shared<typename Entry::Type>();
}
template<class Entry, class Banks, class... Bank>
typename Entry::Type makeCore(const Banks& banks, const std::map<std::string, std::shared_ptr<const void>>& parameters,
                              std::true_type, Types<Bank...>) {
  return typename Entry::Type(parametersOf<Entry>(parameters), *std::get<std::shared_ptr<Bank>>(banks)...);
}
template<class Entry, class Banks, class... Bank>
typename Entry::Type makeCore(const Banks& banks, const std::map<std::string, std::shared_ptr<const void>>&, std::false_type,
                              Types<Bank...>) {
  static_cast<void>(banks);
  return typename Entry::Type(*std::get<std::shared_ptr<Bank>>(banks)...);
}
// Bank와 Core 객체를 만들어 fn에 넘긴다. fn이 CoreSet{core objects}를 구성한다.
template<class... BankEntries, class... CoreEntries, class Fn>
auto createCoreObjects(Types<BankEntries...>, Types<CoreEntries...>,
                       const std::map<std::string, std::shared_ptr<const void>>& parameters, Fn&& fn) {
  const BankSet<typename BankEntries::Type...> banks{
      makeBank<BankEntries>(parameters, std::integral_constant<bool, BankEntries::has_parameters>{})...};
  return fn(banks, makeCore<CoreEntries>(banks, parameters, std::integral_constant<bool, CoreEntries::has_parameters>{},
                                         typename CoreEntries::Banks{})...);
}

template<class Message> struct RosIO {
  using MessageType = Message;
  ros::Publisher publisher;
  Clocks readClocks() const {
    return {static_cast<std::int64_t>(ros::Time::now().toNSec()),
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()};
  }
  void publish(const Message& message) const { publisher.publish(message); }
  template<class HealthType> void reportHealth(const HealthType& health, std::size_t checks) const {
    ROS_DEBUG_STREAM_NAMED("health", "stage " << health.stage << " check " << checks << ":" << describe(health));
  }
  template<class HealthType> static std::string describe(const HealthType& health) {
    std::string text;
    for (const auto& input : health.inputs) {
      if (input.issues.empty()) continue;
      text += " " + input.name + "[";
      for (std::size_t j = 0; j < input.issues.size(); ++j)
        text += (j ? "," : "") + std::string(kindNames()[static_cast<std::size_t>(input.issues[j])]);
      text += "]";
    }
    return text.empty() ? " ok" : text;
  }
  void report(const ProcessReport& report) const {
    if (report.status == ProcessStatus::Failed)
      ROS_ERROR_STREAM_THROTTLE(1.0, "pipeline Failed: " << report.reason);
    if (report.status == ProcessStatus::NoCore || report.status == ProcessStatus::Rejected)
      ROS_WARN_STREAM_THROTTLE(1.0, "pipeline " << (report.status == ProcessStatus::NoCore ? "NoCore" : "Rejected")
                                                << ": " << report.reason);
  }
};

// 최종 메시지에 context stamp와 output.frame_id로 헤더만 주입한다.
template<class Message> struct RosHeaderInjector {
  std::string frame_id;
  void injectHeader(Message& message, const Context& context) const { Access::inject(message, context.stamp_ns, frame_id); }
};

// 구독 메시지를 원래 타입 그대로 PipelineNode에 전달한다. callback 밖으로 예외를 내보내지 않는다.
template<class Node> struct RosCallbacks {
  Node& node;
  template<class Fn> static void guarded(Fn&& fn) {
    try {
      fn();
    } catch (const std::exception& error) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "pipeline callback failed: " << error.what());
    } catch (...) {
      ROS_ERROR_STREAM_THROTTLE(1.0, "pipeline callback failed: unknown exception");
    }
  }
  template<class Message> std::shared_ptr<const Message> retainMessage(const boost::shared_ptr<const Message>& message) const {
    return std::shared_ptr<const Message>(message.get(), [message](const Message*) {});
  }
  void onSection(const interfaces::SectionAnnotation::ConstPtr& message) {
    guarded([&] {
      node.onSection(Section{true, message->state, static_cast<std::int64_t>(message->header.stamp.toNSec()),
                             message->header.frame_id});
    });
  }
  template<std::size_t Input, class Message> void onInput(const boost::shared_ptr<const Message>& message) {
    guarded([&] {
      const auto retained_message = retainMessage(message);
      const std::integral_constant<std::size_t, Input> input_tag{};
      node.onInput(input_tag, retained_message);
    });
  }
  void onPeriodic(const ros::SteadyTimerEvent& event) {
    static_cast<void>(event);
    guarded([&] { node.onPeriodic(); });
  }
};

template<class Inputs, class Callbacks>
std::vector<ros::Subscriber> subscribeInputs(ros::NodeHandle& nh, const std::vector<std::string>& topics, Callbacks& callbacks) {
  std::vector<ros::Subscriber> subscribers;
  forEachIndex<Inputs::size>([&](auto tag) {
    constexpr std::size_t I = decltype(tag)::value;
    using Message = AtT<I, Inputs>;
    subscribers.push_back(nh.subscribe<Message>(topics.at(I), 1, &Callbacks::template onInput<I, Message>, &callbacks));
  });
  return subscribers;
}
}  // namespace detail
}  // namespace node_pipeline
