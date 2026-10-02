#pragma once
// ROS 실행 패키지만 include한다. 공통 계산 헤더는 ROS에 의존하지 않는다.
#include "components.hpp"
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <chrono>

namespace node_pipeline {
// 오류 이름과 공통 enum의 순서를 연결한다.
inline const std::array<std::string, static_cast<std::size_t>(Kind::Count)>& issueNames() {
  static const std::array<std::string, static_cast<std::size_t>(Kind::Count)> names{{
    "Missing", "InvalidStamp", "FutureStamp", "FrameMismatch", "NaN", "Inf",
    "OutRangePositive", "OutRangeNegative", "Stale", "ReceiveTimeout", "Skew"}};
  return names;
}
// 파라미터의 양수 초를 ns로 변환한다. 변환 범위와 1ns 미만도 검사한다.
inline std::int64_t readDurationNs(ros::NodeHandle& node, const std::string& key, double fallback) {
  double seconds;
  node.param(key, seconds, fallback);
  if (!std::isfinite(seconds) || seconds <= 0 || seconds >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9)
    throw std::invalid_argument("invalid duration: " + key);
  const auto ns = static_cast<std::int64_t>(seconds * 1e9);
  if (!ns) throw std::invalid_argument("duration below 1 ns: " + key);
  return ns;
}
// 등록된 이름만 config에서 사용할 수 있다. 문자열을 새로운 객체 타입으로 만들지 않는다.
template<class Enum> Enum named(const std::map<std::string, Enum>& registry, const std::string& name) {
  const auto entry = registry.find(name);
  if (entry == registry.end()) throw std::invalid_argument("unknown configured name: " + name);
  return entry->second;
}
// 메시지 타입과 무관한 공통 실행 설정이다.
template<class Id> struct RosSettings {
  ExecutionConfig<Id> execution;
  std::size_t queue_capacity = 8;
  double frequency_hz = 50;
  HealthLimits health;
};
// config 해석과 검증을 공통화한다. 노드는 이름→실제 ID 표만 제공한다.
template<class Id, class CoreId, class Policy>
RosSettings<Id> loadRosConfig(ros::NodeHandle& node, ros::NodeHandle& params,
                            const std::map<std::string, Id>& topics,
                            const std::map<std::string, Id>& inputs,
                            const std::map<std::string, CoreId>& cores,
                            CoreSelector<Id, CoreId>& selector, Policy& policy) {
  RosSettings<Id> result;
  std::string mode;
  params.param<std::string>("execution_mode", mode, "periodic");
  if (mode != "periodic" && mode != "on_input") throw std::invalid_argument("invalid execution_mode");
  result.execution.mode = mode == "periodic" ? ExecutionMode::Periodic : ExecutionMode::OnInput;
  // timer 모드에서는 trigger_input을 읽거나 검사하지 않는다.
  if (mode == "on_input") {
    if (topics.empty()) throw std::invalid_argument("no registered input topics");
    std::string trigger;
    // 기본 trigger는 기본 input ID에 등록된 토픽이다. 이름 정렬 순서로 선택하지 않는다.
    const auto default_topic = std::find_if(topics.begin(), topics.end(), [&](const auto& entry) {
      return entry.second == result.execution.trigger_input;
    });
    if (default_topic == topics.end()) throw std::invalid_argument("default trigger input not registered");
    params.param<std::string>("trigger_input", trigger, default_topic->first);
    if (trigger.empty() || trigger.front() != '/') throw std::invalid_argument("trigger_input must be an absolute topic name");
    result.execution.trigger_input = named(topics, node.resolveName(trigger));
  }
  params.param("require_new_snapshot", result.execution.require_new_snapshot, false);
  // 정수 설정은 음수 상태에서 검사한 후 unsigned로 변환한다.
  int capacity, checks;
  params.param("queue_capacity", capacity, 8);
  params.param("max_health_checks", checks, 10);
  if (capacity <= 0 || checks <= 0) throw std::invalid_argument("invalid count setting");
  result.queue_capacity = static_cast<std::size_t>(capacity);
  result.execution.max_health_checks = static_cast<std::size_t>(checks);
  params.param("lower", result.health.lower, -1.0);
  params.param("upper", result.health.upper, 1.0);
  params.param<std::string>("frame", result.health.frame, "map");
  result.health.stale_ns = readDurationNs(params, "stale_after_sec", 0.3);
  result.health.receive_timeout_ns = readDurationNs(params, "receive_timeout_sec", 0.3);
  result.health.max_skew_ns = readDurationNs(params, "max_sensor_skew_sec", 0.1);
  result.health.validate();
  // policy의 기본 strategy 한도도 config 값으로 구성한다.
  policy = Policy{{result.health.upper}, {result.health.lower}};
  for (std::size_t i = 0; i < issueNames().size(); ++i)
    params.param("input_policy/priority/" + issueNames()[i], policy.priority[i], policy.priority[i]);
  // section→core 표를 검증하며 읽는다.
  XmlRpc::XmlRpcValue mapping;
  if (params.getParam("section_core_map", mapping)) {
    if (mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct) throw std::invalid_argument("invalid section_core_map");
    selector.section_core_map.clear();
    for (auto entry = mapping.begin(); entry != mapping.end(); ++entry) {
      std::size_t parsed = 0;
      const int section = std::stoi(entry->first, &parsed);
      if (parsed != entry->first.size() || section < 0 || section > 255 || entry->second.getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::invalid_argument("invalid section mapping entry");
      const auto core_name = static_cast<std::string>(entry->second);
      if (core_name.empty()) continue;  // 명시적인 Core 미등록 항목이다.
      selector.section_core_map[static_cast<std::uint8_t>(section)] = named(cores, core_name);
    }
  }
  if (!cores.empty() && !selector.section_core_map.count(0)) throw std::invalid_argument("UNKNOWN mapping required");
  // health 규칙은 when의 AND 조건과 core 선택 priority를 갖는다.
  if (params.getParam("health_core_map", mapping)) {
    if (mapping.getType() != XmlRpc::XmlRpcValue::TypeArray) throw std::invalid_argument("health_core_map must be an array");
    selector.health_core_map.clear();
    for (int i = 0; i < mapping.size(); ++i) {
      auto& row = mapping[i];
      if (row.getType() != XmlRpc::XmlRpcValue::TypeStruct || !row.hasMember("when") || !row.hasMember("priority") || !row.hasMember("core") ||
          row["when"].getType() != XmlRpc::XmlRpcValue::TypeArray || row["when"].size() == 0 ||
          row["priority"].getType() != XmlRpc::XmlRpcValue::TypeInt || row["core"].getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::invalid_argument("invalid health mapping rule");
      HealthCoreRule<Id, CoreId> rule;
      rule.priority = static_cast<int>(row["priority"]);
      rule.core = named(cores, static_cast<std::string>(row["core"]));
      for (int j = 0; j < row["when"].size(); ++j) {
        auto& condition = row["when"][j];
        if (condition.getType() != XmlRpc::XmlRpcValue::TypeStruct || !condition.hasMember("input") || !condition.hasMember("issue") ||
            condition["input"].getType() != XmlRpc::XmlRpcValue::TypeString || condition["issue"].getType() != XmlRpc::XmlRpcValue::TypeString)
          throw std::invalid_argument("invalid health condition");
        const std::string name = static_cast<std::string>(condition["issue"]);
        const auto kind = std::find(issueNames().begin(), issueNames().end(), name);
        if (kind == issueNames().end()) throw std::invalid_argument("unknown health issue: " + name);
        rule.when.push_back({named(inputs, static_cast<std::string>(condition["input"])), static_cast<Kind>(kind - issueNames().begin())});
      }
      selector.health_core_map.push_back(std::move(rule));
    }
  }
  params.param("frequency_hz", result.frequency_hz, 50.0);
  if (mode == "periodic" && (!std::isfinite(result.frequency_hz) || result.frequency_hz <= 0 ||
      1.0 / result.frequency_hz < 1e-9 || 1.0 / result.frequency_hz > 2147483647.0)) throw std::invalid_argument("invalid frequency_hz");
  return result;
}
// 시계·진단·완성된 메시지 발행만 제공한다. 메시지 빌드는 하지 않는다.
template<class Message> struct RosIO {
  ros::Publisher publisher;
  Clocks readClocks() const {
    return {static_cast<std::int64_t>(ros::Time::now().toNSec()),
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()};
  }
  void publish(const Message& message) const { publisher.publish(message); }
  template<class HealthType> void reportHealth(const HealthType&, std::size_t) const {}
  void report(const ProcessReport& report) const noexcept {
    if (report.status == ProcessStatus::NoCore && report.reason == "no compatible core")
      ROS_WARN_STREAM("pipeline status=" << static_cast<int>(report.status) << " reason=" << report.reason);
  }
};
}
