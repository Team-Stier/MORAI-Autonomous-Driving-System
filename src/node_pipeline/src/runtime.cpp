// ParamReader와 공통 실행기의 YAML 설정 로더다. YAML 의존성은 이 파일에만 둔다.
#include <node_pipeline/detail/ros.hpp>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <cmath>

namespace node_pipeline {

struct ParamReader::Impl {
  YAML::Node root;
  std::string base_dir;
  std::string source;
};

namespace {
YAML::Node find(const ParamReader::Impl* impl, const std::string& key) {
  if (!impl) throw std::invalid_argument("parameters are not declared for this core: " + key);
  YAML::Node node = impl->root;
  std::size_t begin = 0;
  while (!key.empty()) {
    const auto end = key.find('/', begin);
    const auto part = key.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    if (part.empty() || !node.IsMap()) return YAML::Node(YAML::NodeType::Undefined);
    const YAML::Node child = static_cast<const YAML::Node&>(node)[part];
    if (!child) return YAML::Node(YAML::NodeType::Undefined);
    node.reset(child);
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return node;
}
YAML::Node require(const ParamReader::Impl* impl, const std::string& key) {
  auto node = find(impl, key);
  if (!node.IsDefined() || node.IsNull()) throw std::invalid_argument("missing parameter '" + key + "' in " + impl->source);
  return node;
}
template<class T> T scalar(const ParamReader::Impl* impl, const std::string& key, const char* type) {
  const auto node = require(impl, key);
  if (!node.IsScalar()) throw std::invalid_argument("parameter '" + key + "' must be " + type);
  try {
    return node.as<T>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument("parameter '" + key + "' must be " + type);
  }
}
}  // namespace

ParamReader::ParamReader() = default;
ParamReader::ParamReader(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

ParamReader ParamReader::fromFile(const std::string& path, const std::string& base_dir) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open parameters " + path);
  const std::string text{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  auto reader = fromString(text, base_dir);
  auto impl = std::make_shared<Impl>(*reader.impl_);
  impl->source = path;
  return ParamReader(std::move(impl));
}

ParamReader ParamReader::fromString(const std::string& yaml, const std::string& base_dir) {
  auto impl = std::make_shared<Impl>();
  impl->root = YAML::Load(yaml);
  if (!impl->root.IsMap()) throw std::invalid_argument("parameters must be a YAML map");
  impl->base_dir = base_dir;
  impl->source = "parameters";
  return ParamReader(std::move(impl));
}

bool ParamReader::has(const std::string& key) const {
  if (!impl_) return false;
  const auto node = find(impl_.get(), key);
  return node.IsDefined() && !node.IsNull();
}
double ParamReader::number(const std::string& key) const { return scalar<double>(impl_.get(), key, "a number"); }
std::int64_t ParamReader::integer(const std::string& key) const {
  const auto text = scalar<std::string>(impl_.get(), key, "an integer");
  std::size_t parsed = 0;
  long long value = 0;
  try {
    value = std::stoll(text, &parsed);
  } catch (const std::exception&) {
    parsed = 0;
  }
  if (parsed != text.size() || text.empty()) throw std::invalid_argument("parameter '" + key + "' must be an integer");
  return value;
}
std::string ParamReader::text(const std::string& key) const { return scalar<std::string>(impl_.get(), key, "a string"); }
bool ParamReader::flag(const std::string& key) const { return scalar<bool>(impl_.get(), key, "a boolean"); }
std::vector<double> ParamReader::numbers(const std::string& key) const {
  const auto node = require(impl_.get(), key);
  if (!node.IsSequence()) throw std::invalid_argument("parameter '" + key + "' must be a number list");
  std::vector<double> values;
  for (const auto& item : node) {
    try {
      values.push_back(item.as<double>());
    } catch (const YAML::Exception&) {
      throw std::invalid_argument("parameter '" + key + "' must be a number list");
    }
  }
  return values;
}
std::vector<std::string> ParamReader::keys(const std::string& key) const {
  if (!impl_) throw std::invalid_argument("parameters are not declared for this core");
  const YAML::Node node = key.empty() ? impl_->root : require(impl_.get(), key);
  if (!node.IsMap()) throw std::invalid_argument("parameter '" + key + "' must be a map");
  std::vector<std::string> names;
  for (const auto& entry : node) names.push_back(entry.first.as<std::string>());
  return names;
}
std::string ParamReader::asset(const std::string& key) const {
  auto path = text(key);
  if (path.empty()) throw std::invalid_argument("parameter '" + key + "' must name an asset file");
  if (path.front() != '/') path = impl_->base_dir + "/" + path;
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open asset " + path);
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

namespace detail {

struct ConfigNode::Impl {
  YAML::Node node;
  bool defined = false;
  std::string where;
};

namespace {
template<class T> T scalarOf(const ConfigNode::Impl* impl, const std::string& where, const char* type) {
  if (!impl || !impl->defined || impl->node.IsNull()) throw std::invalid_argument(where + ": missing " + type);
  if (!impl->node.IsScalar()) throw std::invalid_argument(where + ": must be " + type);
  try {
    return impl->node.as<T>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument(where + ": must be " + type);
  }
}
std::shared_ptr<ConfigNode::Impl> childImpl(const std::string& where) {
  auto impl = std::make_shared<ConfigNode::Impl>();
  impl->where = where;
  return impl;
}
}  // namespace

ConfigNode::ConfigNode() = default;
ConfigNode::ConfigNode(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

ConfigNode ConfigNode::fromFile(const std::string& path) {
  auto impl = std::make_shared<Impl>();
  try {
    impl->node = YAML::LoadFile(path);
  } catch (const YAML::Exception& error) {
    throw std::invalid_argument("cannot read " + path + ": " + error.what());
  }
  impl->defined = true;
  impl->where = path;
  return ConfigNode(std::move(impl));
}
bool ConfigNode::defined() const { return impl_ && impl_->defined; }
bool ConfigNode::isNull() const { return defined() && impl_->node.IsNull(); }
bool ConfigNode::isMap() const { return defined() && impl_->node.IsMap(); }
bool ConfigNode::isList() const { return defined() && impl_->node.IsSequence(); }
ConfigNode ConfigNode::operator[](const std::string& key) const {
  auto impl = childImpl(where() + "/" + key);
  if (isMap()) {
    const YAML::Node& parent = impl_->node;
    const YAML::Node child = parent[key];
    if (child.IsDefined()) {
      impl->node.reset(child);
      impl->defined = true;
    }
  }
  return ConfigNode(std::move(impl));
}
ConfigNode ConfigNode::at(std::size_t index) const {
  auto impl = childImpl(where() + "[" + std::to_string(index) + "]");
  if (isList() && index < impl_->node.size()) {
    const YAML::Node& parent = impl_->node;
    impl->node.reset(parent[index]);
    impl->defined = true;
  }
  return ConfigNode(std::move(impl));
}
std::size_t ConfigNode::size() const { return isMap() || isList() ? impl_->node.size() : 0; }
std::vector<std::string> ConfigNode::keys() const {
  if (!isMap()) throw std::invalid_argument(where() + ": must be a map");
  std::vector<std::string> names;
  for (const auto& entry : impl_->node) names.push_back(entry.first.as<std::string>());
  return names;
}
std::string ConfigNode::text() const { return scalarOf<std::string>(impl_.get(), where(), "a string"); }
double ConfigNode::number() const { return scalarOf<double>(impl_.get(), where(), "a number"); }
std::int64_t ConfigNode::integer() const {
  const auto value = scalarOf<std::string>(impl_.get(), where(), "an integer");
  std::size_t parsed = 0;
  long long result = 0;
  try {
    result = std::stoll(value, &parsed);
  } catch (const std::exception&) {
    parsed = 0;
  }
  if (value.empty() || parsed != value.size()) throw std::invalid_argument(where() + ": must be an integer");
  return result;
}
bool ConfigNode::flag() const { return scalarOf<bool>(impl_.get(), where(), "true or false"); }
std::string ConfigNode::where() const { return impl_ ? impl_->where : std::string("config"); }

std::string coreStem(const std::string& core) {
  const auto suffix = std::string(".hpp");
  if (core.size() <= suffix.size() || core.compare(core.size() - suffix.size(), suffix.size(), suffix) != 0)
    throw std::invalid_argument("invalid core name: " + core);
  return core.substr(0, core.size() - suffix.size());
}

namespace {
std::int64_t positiveNs(const ConfigNode& value) {
  const double seconds = value.number();
  if (!std::isfinite(seconds) || seconds <= 0) throw std::invalid_argument(value.where() + ": must be a positive number of seconds");
  const auto ns = static_cast<std::int64_t>(std::llround(seconds * 1e9));
  if (ns < 1) throw std::invalid_argument(value.where() + ": below 1 ns");
  return ns;
}
std::size_t positiveCount(const ConfigNode& execution, const std::string& key, std::size_t fallback) {
  const auto value = execution[key];
  if (!value.defined()) return fallback;
  const auto count = value.integer();
  if (count <= 0) throw std::invalid_argument(value.where() + ": must be a positive integer");
  return static_cast<std::size_t>(count);
}
bool flagOr(const ConfigNode& node, const std::string& key, bool fallback) {
  const auto value = node[key];
  return value.defined() ? value.flag() : fallback;
}
std::string textOr(const ConfigNode& node, const std::string& key) {
  const auto value = node[key];
  return value.defined() ? value.text() : std::string();
}
void applyChecks(HealthLimits& limits, const ConfigNode& decl) {
  if (decl["frame"].defined()) limits.frame = decl["frame"].text();
  const auto range = decl["range"];
  if (!range.defined()) return;
  if (!range.isList() || range.size() != 2) throw std::invalid_argument(range.where() + ": must be [lower, upper]");
  limits.lower = range.at(0).number();
  limits.upper = range.at(1).number();
  if (!std::isfinite(limits.lower) || !std::isfinite(limits.upper) || limits.lower > limits.upper)
    throw std::invalid_argument(range.where() + ": must be [lower, upper]");
}
std::size_t inputIndex(const std::map<std::string, std::size_t>& inputs, const ConfigNode& name) {
  const auto found = inputs.find(name.text());
  if (found == inputs.end()) throw std::invalid_argument(name.where() + ": must be an input name");
  return found->second;
}
}  // namespace

RosSettings loadRosConfig(ros::NodeHandle& nh, const NodeParams& params, std::vector<std::string>& topics,
                          const std::map<std::string, std::size_t>& inputs) {
  const auto package_dir = ros::package::getPath(params.package);
  if (package_dir.empty()) throw std::invalid_argument("package not found: " + params.package);
  const auto node_dir = package_dir + "/config/" + params.node;
  const auto node = ConfigNode::fromFile(params.config_dir + "/node.yaml");
  std::map<std::string, ConfigNode> registrations;
  for (const auto& core : params.cores)
    registrations[core] = ConfigNode::fromFile(params.config_dir + "/registrations/" + coreStem(core) + ".yaml");
  const auto selection = ConfigNode::fromFile(node_dir + "/selection.yaml");
  RosSettings settings;
  const auto execution = node["execution"];
  const auto mode = execution["mode"].text();
  if (mode == "on_input") settings.execution.mode = ExecutionMode::OnInput;
  else if (mode == "periodic") settings.execution.mode = ExecutionMode::Periodic;
  else if (mode == "latched") settings.execution.mode = ExecutionMode::Latched;
  else throw std::invalid_argument(execution["mode"].where() + ": must be on_input, periodic or latched");
  settings.queue_capacity = positiveCount(execution, "queue_capacity", 8);
  settings.execution.max_health_checks = positiveCount(execution, "max_health_checks", 10);
  settings.execution.require_new_snapshot = flagOr(execution, "require_new_snapshot", false);

  const auto declared = node["inputs"];
  const auto names = declared.defined() && !declared.isNull() ? declared.keys() : std::vector<std::string>();
  if (names.size() != inputs.size()) throw std::invalid_argument(declared.where() + ": inputs do not match the build");
  StageParams current;
  topics.assign(names.size(), std::string());
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto found = inputs.find(names[i]);
    if (found == inputs.end() || found->second != i) throw std::invalid_argument(declared.where() + ": inputs do not match the build");
    const auto decl = declared[names[i]];
    topics[i] = nh.resolveName(decl["topic"].text());
    HealthLimits limits;
    limits.check_stamp = flagOr(decl, "check_stamp", true);
    limits.check_receive_timeout = flagOr(decl, "check_receive_timeout", true);
    limits.check_skew = flagOr(decl, "check_skew", true);
    if (limits.check_stamp) limits.stale_ns = positiveNs(decl["stale_after_sec"]);
    if (limits.check_receive_timeout) limits.receive_timeout_ns = positiveNs(decl["receive_timeout_sec"]);
    applyChecks(limits, decl);
    current.slots.push_back(names[i]);
    current.limits.push_back(limits);
    current.subscribed.push_back(true);
  }
  if (settings.execution.mode == ExecutionMode::Periodic) {
    settings.frequency_hz = execution["frequency_hz"].number();
    if (!std::isfinite(settings.frequency_hz) || settings.frequency_hz <= 0)
      throw std::invalid_argument(execution["frequency_hz"].where() + ": must be positive");
    if (execution["reference_input"].defined())
      settings.reference_input = static_cast<int>(inputIndex(inputs, execution["reference_input"]));
  }
  if (settings.execution.mode == ExecutionMode::OnInput)
    settings.execution.trigger_input = inputIndex(inputs, execution["trigger_input"]);
  if (node["section_topic"].defined()) settings.section_topic = nh.resolveName(node["section_topic"].text());
  const std::int64_t max_skew_ns = node["max_skew_sec"].defined() ? positiveNs(node["max_skew_sec"]) : 0;
  const std::int64_t started_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

  const auto stages = node["stages"];
  if (!stages.isList() || stages.size() == 0) throw std::invalid_argument(stages.where() + ": must be a non-empty list");
  for (std::size_t s = 0; s < stages.size(); ++s) {
    const auto decl = stages.at(s);
    current.name = decl["name"].text();
    current.max_skew_ns = max_skew_ns;
    current.started_steady_ns = started_steady_ns;
    current.selection = selection[current.name];
    StageParams next;
    const auto outputs = decl["outputs"];
    if (!outputs.isList() || outputs.size() == 0) throw std::invalid_argument(outputs.where() + ": must be a non-empty list");
    for (std::size_t o = 0; o < outputs.size(); ++o) {
      const auto output = outputs.at(o);
      HealthLimits limits;
      int origin = -1;
      if (output["from"].defined()) {
        const auto source = std::find(current.slots.begin(), current.slots.end(), output["from"].text());
        if (source == current.slots.end()) throw std::invalid_argument(output.where() + ": from must name an input of this stage");
        origin = static_cast<int>(source - current.slots.begin());
        limits = current.limits[static_cast<std::size_t>(origin)];
      } else {
        limits.check_stamp = false;
        limits.check_receive_timeout = false;
        limits.check_skew = false;
      }
      applyChecks(limits, output);
      current.origins.push_back(origin);
      next.slots.push_back(output["name"].text());
      next.limits.push_back(limits);
      next.subscribed.push_back(false);
    }
    settings.stages.push_back(current);
    current = next;
  }

  for (const auto& entry : registrations) {
    const auto stage = entry.second["stage"];
    const auto found = std::find_if(settings.stages.begin(), settings.stages.end(),
                                    [&](const StageParams& candidate) { return candidate.name == stage.text(); });
    if (found == settings.stages.end()) throw std::invalid_argument(stage.where() + ": unknown stage");
    found->registrations[entry.first] = entry.second;
  }
  if (selection.defined() && !selection.isNull()) {
    for (const auto& key : selection.keys())
      if (std::none_of(settings.stages.begin(), settings.stages.end(), [&](const StageParams& stage) { return stage.name == key; }))
        throw std::invalid_argument(selection.where() + ": unknown stage '" + key + "'");
  }
  const auto output = node["output"];
  settings.output_topic = nh.resolveName(output["topic"].text());
  settings.output_frame = textOr(output, "frame_id");
  settings.stages.back().last = true;
  settings.stages.back().output_frame = settings.output_frame;
  for (const auto& entry : params.loaders) {
    const auto path = node_dir + "/parameters/" + coreStem(entry.first) + ".yaml";
    try {
      settings.parameters[entry.first] = entry.second(ParamReader::fromFile(path, package_dir));
    } catch (const std::exception& error) {
      throw std::invalid_argument(path + ": " + error.what());
    }
  }
  return settings;
}

void loadRegistration(const ConfigNode& registration, const StageParams& stage_params,
                      const std::map<std::string, std::size_t>& input_registry, const std::vector<std::size_t>& required_inputs,
                      const std::string& validator_name, const std::string& policy_name,
                      std::vector<std::vector<Kind>>& allowed_issues) {
  const auto where = registration.where();
  if (registration["stage"].text() != stage_params.name) throw std::invalid_argument(where + ": stage does not match the build");
  const auto inputs = registration["inputs"];
  if (!inputs.isList() || inputs.size() != required_inputs.size())
    throw std::invalid_argument(where + ": inputs do not match the build");
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    const auto found = input_registry.find(inputs.at(i).text());
    if (found == input_registry.end() || found->second != required_inputs[i])
      throw std::invalid_argument(where + ": inputs do not match the build");
  }
  if (textOr(registration, "validator") != validator_name || textOr(registration, "policy") != policy_name)
    throw std::invalid_argument(where + ": validator or policy does not match the build");
  const auto allowed = registration["allowed_issues"];
  if (!allowed.defined() || allowed.isNull()) return;
  for (const auto& input : allowed.keys()) {
    const auto found = input_registry.find(input);
    if (found == input_registry.end() ||
        std::find(required_inputs.begin(), required_inputs.end(), found->second) == required_inputs.end())
      throw std::invalid_argument(where + ": allowed_issues input '" + input + "' is not a core input");
    const auto issues = allowed[input];
    if (!issues.isList()) throw std::invalid_argument(issues.where() + ": must be a list of issues");
    for (std::size_t k = 0; k < issues.size(); ++k) {
      const auto kind = kindFromName(issues.at(k).text());
      if (kind == Kind::Missing) throw std::invalid_argument(issues.where() + ": Missing cannot be allowed");
      allowed_issues.at(found->second).push_back(kind);
    }
  }
}

std::uint8_t sectionKey(const std::string& key, const std::string& stage) {
  std::size_t parsed = 0;
  int section = -1;
  try {
    section = std::stoi(key, &parsed);
  } catch (const std::exception&) {
    parsed = 0;
  }
  if (key.empty() || parsed != key.size() || section < 0 || section > 255)
    throw std::invalid_argument("invalid section mapping entry in stage " + stage + ": " + key);
  return static_cast<std::uint8_t>(section);
}
}  // namespace detail
}  // namespace node_pipeline
