#pragma once
#include <node_pipeline/detail/message_traits.hpp>
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <chrono>

namespace node_pipeline { namespace detail {
template<class Message> VisualizerPublisher<Message> bindVisualizer(const std::shared_ptr<void>& owner,const std::string& topic) {
  const auto node=std::static_pointer_cast<ros::NodeHandle>(owner);
  const auto resolved=node->resolveName(topic);
  if(resolved.compare(0,12,"/visualizer/")!=0)throw std::invalid_argument("Visualizer topic must resolve under /visualizer/");
  const auto publisher=node->advertise<Message>(resolved,1);
  return VisualizerPublisher<Message>([publisher](const Message& message){publisher.publish(message);});
}
struct AssetSpec { std::string parameter, fallback_parameter, sibling; };
class RosRuntime final {
 public:
  RosRuntime(int argc, char** argv, const std::string& name) {
    ros::init(argc, argv, name);
    node_.reset(new ros::NodeHandle());
    private_.reset(new ros::NodeHandle("~"));
    XmlRpc::XmlRpcValue values;
    if (private_->getParam("", values)) values_ = parameterValue(values);
    else values_.type = ParameterValue::Type::Object;
    validateConfig(values_);
  }
  void execution(ExecutionSpec spec) {
    if (!subscriptions_.empty()) throw std::invalid_argument("execution must precede inputs");
    if (spec.mode != "on_input" && spec.mode != "periodic" && spec.mode != "latched")
      throw std::invalid_argument("invalid execution_mode");
    if(!spec.queue_capacity)throw std::invalid_argument("invalid queue_capacity");
    if (spec.mode == "periodic" && (!std::isfinite(spec.frequency_hz) || spec.frequency_hz <= 0 ||
        1/spec.frequency_hz < 1e-9 || 1/spec.frequency_hz > 2147483647.0))
      throw std::invalid_argument("invalid frequency_hz");
    execution_ = std::move(spec);
    engine_.reset(new Engine(execution_.queue_capacity, clocks().steady_ns));
    engine_->report_health_=[](const ProfileHealth& health,std::size_t checks) {
      std::ostringstream report;report << "health check " << checks;
      for(const auto& input:health.inputs){report << " " << input.input << ":";for(auto issue:input.issues)report << static_cast<int>(issue) << ",";}
      ROS_DEBUG_STREAM(report.str());
    };
  }
  void asset(const std::string& name, const AssetSpec& spec) {
    const Parameters parameters(values_);
    std::string path;
    if (parameters.contains(spec.parameter)) path = parameters.require<std::string>(spec.parameter);
    else {
      if (spec.fallback_parameter.empty()) throw std::invalid_argument("missing declared asset: " + name);
      path = parameters.require<std::string>(spec.fallback_parameter);
      if (!spec.sibling.empty()) path = path.substr(0, path.find_last_of('/') + 1) + spec.sibling;
    }
    if (path.empty() || assets_.count(name)) throw std::invalid_argument("invalid declared asset: " + name);
    assets_.emplace(name, std::move(path));
  }
  void sectionTopic(std::string topic) { section_topic_ = std::move(topic); }
  template<class Message> void input(InputSpec spec) {
    validateLimits(spec.limits);
    spec.topic = node_->resolveName(spec.topic);
    engine_->template input<Message>(spec);
    topics_.emplace(spec.name, spec.topic);
    const auto name = spec.name;
    boost::function<void(const typename Message::ConstPtr&)> callback =
      [this, name](const typename Message::ConstPtr& message) {
        auto retained=retainMessage(message);
        if(!retained)throw std::invalid_argument("null input payload");
        const auto receive = clocks();
        const auto id = engine_->template push<Message>(name, std::move(retained), receive,
          RosMessageAccess<Message>::stamp(*message), RosMessageAccess<Message>::hasStamp());
        if (execution_.mode == "on_input" && name == trigger_)
          process(name, id);
      };
    subscriptions_.push_back(node_->template subscribe<Message>(spec.topic, 1, callback));
  }
  template<class Bank> void bank(const std::string& name) {
    if (name.empty() || banks_.count(name)) throw std::invalid_argument("invalid Bank declaration");
    banks_.emplace(name, std::make_shared<Bank>()); bank_types_.emplace(name, typeid(Bank));
  }
  template<class Core, class Loader = NoParameters, class Validator = NoValidator,
           class Policy = RemoveInvalid, class Bank = NoBank>
  void core(CoreSpec spec, const std::string& parameter_namespace = {}, const std::string& bank_name = {}) {
    std::shared_ptr<Bank> bank;
    if (!bank_name.empty()) {
      if (bank_types_.at(bank_name) != std::type_index(typeid(Bank))) throw std::invalid_argument("Bank type mismatch");
      bank = std::static_pointer_cast<Bank>(banks_.at(bank_name));
    }
    std::map<std::string,std::string> permitted;
    for(const auto& name:spec.assets)permitted.emplace(name,assets_.at(name));
    Parameters parameters;
    if(!parameter_namespace.empty())parameters=Parameters(values_,std::move(permitted)).child(parameter_namespace);
    else parameters=Parameters(ParameterValue{},std::move(permitted));
    const Visualizer visualizer(std::shared_ptr<void>(new ros::NodeHandle(*node_)));
    core_stages_.emplace(spec.name,spec.stage);
    engine_->template core<Core,Loader,Validator,Policy,Bank>(std::move(spec),parameters,std::move(bank),&visualizer);
  }
  void stage(StageSpec spec) {
    for(const auto& entry:spec.section_core_map)checkStageCore(spec.name,entry.second);
    for(const auto& rule:spec.health_core_map)checkStageCore(spec.name,rule.core);
    engine_->stage(std::move(spec));
  }
  template<class Message> void output(OutputSpec spec) {
    const auto publisher = node_->template advertise<Message>(spec.topic, 1, execution_.mode == "latched");
    output_topics_.insert(node_->resolveName(spec.topic));
    outputs_.push_back([this, publisher, spec] {
      Message message = engine_->template result<Message>(spec.source);
      injectHeader(message, spec);
      publisher.publish(message);
    });
  }
  int run() {
    if (execution_.mode == "on_input") {
      trigger_ = execution_.trigger_input;
      if (!topics_.count(trigger_)) throw std::invalid_argument("unregistered trigger input");
    }
    engine_->validateExecution();
    const auto section=section_topic_.empty()?std::string{}:node_->resolveName(section_topic_);
    const auto reserved_section=node_->resolveName("/section_annotation");
    for(const auto& input:topics_)if(input.second==reserved_section || (!section.empty() && input.second==section))
      throw std::invalid_argument("selection topic cannot be a general input");
    if (!section_topic_.empty()) {
      section_subscription_ = node_->subscribe<interfaces::SectionAnnotation>(section_topic_, 1,
        [this](const interfaces::SectionAnnotation::ConstPtr& message) {
          engine_->section(Section{true, message->state,
            static_cast<std::int64_t>(message->header.stamp.toNSec()), message->header.frame_id});
        });
    }
    if (execution_.mode == "latched") process({}, 0);
    if (execution_.mode == "periodic")
      timer_ = node_->createSteadyTimer(ros::WallDuration(1/execution_.frequency_hz),
        [this](const ros::SteadyTimerEvent&) { process({}, 0); });
    ros::spin();
    return 0;
  }
 private:
  template<class Message>
  typename std::enable_if<ros::message_traits::HasHeader<Message>::value>::type
  injectHeader(Message& message, const OutputSpec& spec) const {
    if (!spec.stamp_from.empty()) {
      const auto stamp=engine_->assembled().at(spec.stamp_from).stamp_ns;
      message.header.stamp.fromNSec(stamp);
    }
    if (!spec.frame_id.empty()) message.header.frame_id = spec.frame_id;
  }
  template<class Message>
  typename std::enable_if<!ros::message_traits::HasHeader<Message>::value>::type
  injectHeader(Message&, const OutputSpec&) const {}
  static ParameterValue parameterValue(XmlRpc::XmlRpcValue& value) {
    ParameterValue result;
    switch (value.getType()) {
      case XmlRpc::XmlRpcValue::TypeInt:
        result.type = ParameterValue::Type::Integer; result.number = static_cast<int>(value); break;
      case XmlRpc::XmlRpcValue::TypeDouble:
        result.type = ParameterValue::Type::Number; result.number = static_cast<double>(value); break;
      case XmlRpc::XmlRpcValue::TypeBoolean:
        result.type = ParameterValue::Type::Boolean; result.boolean = static_cast<bool>(value); break;
      case XmlRpc::XmlRpcValue::TypeString:
        result.type = ParameterValue::Type::String; result.string = static_cast<std::string>(value); break;
      case XmlRpc::XmlRpcValue::TypeStruct:
        result.type = ParameterValue::Type::Object;
        for (auto entry = value.begin(); entry != value.end(); ++entry)
          result.object.emplace(entry->first, parameterValue(entry->second));
        break;
      case XmlRpc::XmlRpcValue::TypeArray:
        result.type = ParameterValue::Type::Array;
        for (int i = 0; i < value.size(); ++i) result.array.push_back(parameterValue(value[i]));
        break;
      default: throw std::invalid_argument("unsupported ROS parameter type");
    }
    return result;
  }
  static void validateConfig(const ParameterValue& values,const std::string& path={}) {
    static const std::set<std::string> reserved={"execution","execution_mode","trigger_input","frequency_hz",
      "queue_capacity","max_health_checks","require_new_snapshot","inputs","section_core_map","health_core_map",
      "section_topic","output","stages","banks","assets","enable_static_transforms","stale_after_sec","receive_timeout_sec","max_sensor_skew_sec",
      "check_stamp","check_receive_timeout","check_skew"};
    for(const auto& item:values.object) {
      if(reserved.count(item.first))throw std::invalid_argument("Pipeline config belongs in node.yaml: "+path+item.first);
      validateConfig(item.second,path+item.first+"/");
    }
  }
  void checkStageCore(const std::string& stage, const std::string& core) const {
    const auto found = core_stages_.find(core);
    if (found == core_stages_.end() || found->second != stage) throw std::invalid_argument("Core outside stage: " + core);
  }
  static Clocks clocks() {
    return {static_cast<std::int64_t>(ros::Time::now().toNSec()),
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()};
  }
  void finish(const ProcessReport& report) const {
    if(report.status==ProcessStatus::NoCore)ROS_WARN_STREAM("pipeline: " << report.reason);
    ROS_DEBUG_STREAM("pipeline status=" << static_cast<int>(report.status) << " health_checks=" << report.health_checks << " " << report.reason);
  }
  void process(const std::string& trigger,std::uint64_t id) {
    if(!engine_->prepare(execution_.require_new_snapshot,trigger,id)) {finish({ProcessStatus::Skipped,0,"bundle already attempted"});return;}
    const auto report=engine_->execute(clocks());
    if(report.status==ProcessStatus::Published)for(const auto& publish:outputs_)publish();
    finish(report);
  }
  std::unique_ptr<ros::NodeHandle> node_, private_;
  ParameterValue values_;
  std::map<std::string, std::string> assets_, topics_, core_stages_;
  std::map<std::string, std::shared_ptr<void>> banks_;
  std::map<std::string, std::type_index> bank_types_;
  ExecutionSpec execution_;
  std::unique_ptr<Engine> engine_;
  std::vector<ros::Subscriber> subscriptions_;
  std::vector<std::function<void()>> outputs_;
  std::set<std::string> output_topics_;
  ros::Subscriber section_subscription_;
  ros::SteadyTimer timer_;
  std::string trigger_;
  std::string section_topic_;
};
}}  // namespace node_pipeline::detail
