#!/usr/bin/env python3
"""node.yaml과 등록 YAML을 검증하고 노드별 타입 연결 헤더를 생성한다."""
import argparse
import math
import os
import re
import sys

import yaml

KINDS = ["Missing", "InvalidStamp", "FutureStamp", "FrameMismatch", "NaN", "Inf",
         "OutRangePositive", "OutRangeNegative", "Stale", "ReceiveTimeout", "Skew"]
IDENT = re.compile(r"^[a-z][a-z0-9_]*$")
FIELD = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*$")
ROS_TYPE = re.compile(r"^([a-z][a-z0-9_]*)/([A-Z][A-Za-z0-9]*)$")
CPP_TYPE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(::[A-Za-z_][A-Za-z0-9_]*)+$")
TOPIC = re.compile(r"^(/[A-Za-z0-9_]+)+$")
FRAME = re.compile(r"^[A-Za-z0-9_/]+$")
EXTENSION_FILE = re.compile(r"^([a-z][a-z0-9_]*)\.hpp$")


class ConfigError(Exception):
    pass


def pascal(stem):
    return "".join(part[:1].upper() + part[1:] for part in stem.split("_"))


def load_yaml(path):
    try:
        with open(path, encoding="utf-8") as stream:
            return yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ConfigError(f"{path}: {error}")


def check_keys(mapping, allowed, required, where):
    if not isinstance(mapping, dict):
        raise ConfigError(f"{where}: must be a map")
    for key in mapping:
        if key not in allowed:
            raise ConfigError(f"{where}: unknown key '{key}'")
    for key in required:
        if key not in mapping:
            raise ConfigError(f"{where}: missing key '{key}'")


def positive_ns(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ConfigError(f"{where}: must be a positive number of seconds")
    ns = int(round(value * 1e9))
    if ns < 1:
        raise ConfigError(f"{where}: below 1 ns")
    return ns


def flag(mapping, key, default, where):
    value = mapping.get(key, default)
    if not isinstance(value, bool):
        raise ConfigError(f"{where}.{key}: must be true or false")
    return value


class TypeRef:
    def __init__(self, cpp, include, is_ros):
        self.cpp = cpp
        self.include = include
        self.is_ros = is_ros


def ros_type(text, where):
    match = ROS_TYPE.match(text) if isinstance(text, str) else None
    if not match:
        raise ConfigError(f"{where}: '{text}' is not a ROS message type such as sensor_msgs/Imu")
    return TypeRef(f"::{match.group(1)}::{match.group(2)}", f"{match.group(1)}/{match.group(2)}.h", True)


def any_type(decl, package, where):
    text = decl.get("type")
    if isinstance(text, str) and CPP_TYPE.match(text):
        header = decl.get("header")
        if not isinstance(header, str) or not header.startswith(f"{package}/libraries/") or not header.endswith(".hpp"):
            raise ConfigError(f"{where}: C++ type '{text}' needs header {package}/libraries/<name>.hpp")
        return TypeRef("::" + text, header, False)
    if "header" in decl:
        raise ConfigError(f"{where}: header is only allowed for C++ types")
    return ros_type(text, where)


class Limits:
    def __init__(self):
        self.frame = ""
        self.numbers = []
        self.lower = -math.inf
        self.upper = math.inf
        self.stale_ns = 0
        self.receive_timeout_ns = 0
        self.check_stamp = False
        self.check_receive_timeout = False
        self.check_skew = False

    def copy(self):
        other = Limits()
        other.__dict__.update(self.__dict__)
        other.numbers = list(self.numbers)
        return other


def apply_checks(limits, decl, where):
    if "frame" in decl:
        if not isinstance(decl["frame"], str) or not FRAME.match(decl["frame"]):
            raise ConfigError(f"{where}.frame: invalid frame id")
        limits.frame = decl["frame"]
    if "numbers" in decl:
        fields = decl["numbers"]
        if not isinstance(fields, list) or not fields or not all(isinstance(f, str) and FIELD.match(f) for f in fields):
            raise ConfigError(f"{where}.numbers: must be a list of field paths such as pose.position.x")
        limits.numbers = list(fields)
    if "range" in decl:
        bounds = decl["range"]
        if (not isinstance(bounds, list) or len(bounds) != 2 or
                not all(isinstance(b, (int, float)) and not isinstance(b, bool) and math.isfinite(b) for b in bounds) or
                bounds[0] > bounds[1]):
            raise ConfigError(f"{where}.range: must be [lower, upper]")
        limits.lower, limits.upper = float(bounds[0]), float(bounds[1])


class Slot:
    def __init__(self, name, type_ref, limits, subscribed):
        self.name = name
        self.type = type_ref
        self.limits = limits
        self.subscribed = subscribed


class Stage:
    def __init__(self, name, slots):
        self.name = name
        self.slots = slots
        self.outputs = []
        self.origins = []
        self.cores = []


class Core:
    def __init__(self, stem):
        self.stem = stem
        self.name = f"{stem}.hpp"
        self.args = []
        self.validator = None
        self.policy = None
        self.bank = None
        self.allowed = []
        self.parameters = False


class NodeModel:
    def __init__(self, package, node):
        self.package = package
        self.node = node
        self.mode = None
        self.frequency_hz = 0.0
        self.trigger_input = -1
        self.reference_input = -1
        self.queue_capacity = 8
        self.max_health_checks = 10
        self.require_new_snapshot = False
        self.section_topic = ""
        self.max_skew_ns = 0
        self.inputs = []
        self.stages = []
        self.output_type = None
        self.output_topic = ""
        self.output_frame = ""
        self.bank_parameters = {}

    def banks(self):
        return sorted({core.bank for stage in self.stages for core in stage.cores if core.bank})

    def extension_files(self):
        """등록으로 빌드에 연결되는 확장점 파일과 역할이다."""
        files = []
        for stage in self.stages:
            for core in stage.cores:
                files.append(("core", core.stem, core))
                for role, stem in (("validator", core.validator), ("policy", core.policy), ("bank", core.bank)):
                    if stem:
                        files.append((role, stem, core))
                if core.parameters:
                    files.append(("parameters", core.stem, core))
        for bank in self.banks():
            if self.bank_parameters.get(bank):
                files.append(("parameters", bank, BankRef(bank)))
        return files


class BankRef:
    """파라미터 로더를 가진 Bank다. 로더 검사에서 소유자 이름으로 쓴다."""
    def __init__(self, stem):
        self.stem = stem
        self.bank = ""


def integer_setting(mapping, key, default, where):
    value = mapping.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ConfigError(f"{where}.{key}: must be a positive integer")
    return value


def load_node(package_dir, package, node):
    node_dir = os.path.join(package_dir, "config", node)
    path = os.path.join(node_dir, "node.yaml")
    data = load_yaml(path)
    where = f"{path}"
    check_keys(data, {"execution", "section_topic", "max_skew_sec", "inputs", "stages", "output"},
               {"execution", "stages", "output"}, where)
    model = NodeModel(package, node)

    execution = data["execution"]
    check_keys(execution, {"mode", "frequency_hz", "trigger_input", "reference_input", "queue_capacity",
                           "max_health_checks", "require_new_snapshot"}, {"mode"}, f"{where}: execution")
    model.mode = execution["mode"]
    if model.mode not in ("on_input", "periodic", "latched"):
        raise ConfigError(f"{where}: execution.mode must be on_input, periodic or latched")
    model.queue_capacity = integer_setting(execution, "queue_capacity", 8, f"{where}: execution")
    model.max_health_checks = integer_setting(execution, "max_health_checks", 10, f"{where}: execution")
    model.require_new_snapshot = flag(execution, "require_new_snapshot", False, f"{where}: execution")

    inputs = data.get("inputs") or {}
    if not isinstance(inputs, dict):
        raise ConfigError(f"{where}: inputs must be a map")
    first_slots = []
    for name, decl in inputs.items():
        at = f"{where}: inputs.{name}"
        if not isinstance(name, str) or not IDENT.match(name):
            raise ConfigError(f"{at}: input names must be lower snake_case")
        if name == "stamp":
            raise ConfigError(f"{at}: stamp is reserved for the execution stamp")
        check_keys(decl, {"type", "topic", "frame", "numbers", "range", "stale_after_sec", "receive_timeout_sec",
                          "check_stamp", "check_receive_timeout", "check_skew"}, {"type", "topic"}, at)
        if not isinstance(decl["topic"], str) or not TOPIC.match(decl["topic"]):
            raise ConfigError(f"{at}.topic: must be an absolute topic name")
        limits = Limits()
        limits.check_stamp = flag(decl, "check_stamp", True, at)
        limits.check_receive_timeout = flag(decl, "check_receive_timeout", True, at)
        limits.check_skew = flag(decl, "check_skew", True, at)
        if limits.check_stamp:
            limits.stale_ns = positive_ns(decl.get("stale_after_sec"), f"{at}.stale_after_sec")
        elif "stale_after_sec" in decl:
            raise ConfigError(f"{at}: stale_after_sec requires check_stamp")
        if limits.check_receive_timeout:
            limits.receive_timeout_ns = positive_ns(decl.get("receive_timeout_sec"), f"{at}.receive_timeout_sec")
        elif "receive_timeout_sec" in decl:
            raise ConfigError(f"{at}: receive_timeout_sec requires check_receive_timeout")
        apply_checks(limits, decl, at)
        first_slots.append(Slot(name, ros_type(decl["type"], f"{at}.type"), limits, True))
        model.inputs.append((name, decl["topic"]))
    names = [slot.name for slot in first_slots]

    if model.mode == "periodic":
        model.frequency_hz = execution.get("frequency_hz")
        if isinstance(model.frequency_hz, bool) or not isinstance(model.frequency_hz, (int, float)) or \
                not math.isfinite(model.frequency_hz) or model.frequency_hz <= 0:
            raise ConfigError(f"{where}: execution.frequency_hz must be positive for periodic")
        if "reference_input" in execution:
            if execution["reference_input"] not in names:
                raise ConfigError(f"{where}: execution.reference_input must be an input name")
            model.reference_input = names.index(execution["reference_input"])
    elif "frequency_hz" in execution or "reference_input" in execution:
        raise ConfigError(f"{where}: frequency_hz and reference_input are only for periodic")
    if model.mode == "on_input":
        if execution.get("trigger_input") not in names:
            raise ConfigError(f"{where}: execution.trigger_input must be an input name")
        model.trigger_input = names.index(execution["trigger_input"])
    elif "trigger_input" in execution:
        raise ConfigError(f"{where}: trigger_input is only for on_input")
    if model.mode == "latched" and first_slots:
        raise ConfigError(f"{where}: latched nodes have no inputs")

    if "section_topic" in data:
        if not isinstance(data["section_topic"], str) or not TOPIC.match(data["section_topic"]):
            raise ConfigError(f"{where}: section_topic must be an absolute topic name")
        model.section_topic = data["section_topic"]
    skew_inputs = [slot for slot in first_slots if slot.limits.check_skew and slot.limits.check_stamp]
    if "max_skew_sec" in data:
        model.max_skew_ns = positive_ns(data["max_skew_sec"], f"{where}: max_skew_sec")
    elif len(skew_inputs) >= 2:
        raise ConfigError(f"{where}: max_skew_sec is required when two or more inputs check skew")

    stages = data["stages"]
    if not isinstance(stages, list) or not stages:
        raise ConfigError(f"{where}: stages must be a non-empty list")
    slots = first_slots
    stage_names = set()
    for index, decl in enumerate(stages):
        at = f"{where}: stages[{index}]"
        check_keys(decl, {"name", "outputs"}, {"name", "outputs"}, at)
        if not isinstance(decl["name"], str) or not IDENT.match(decl["name"]) or decl["name"] in stage_names:
            raise ConfigError(f"{at}.name: stage names must be unique lower snake_case")
        stage_names.add(decl["name"])
        stage = Stage(decl["name"], slots)
        outputs = decl["outputs"]
        if not isinstance(outputs, list) or not outputs:
            raise ConfigError(f"{at}.outputs: must be a non-empty list")
        next_slots = []
        slot_names = [slot.name for slot in slots]
        for out_index, output in enumerate(outputs):
            out_at = f"{at}.outputs[{out_index}]"
            check_keys(output, {"name", "type", "header", "from", "frame", "numbers", "range"}, {"name"}, out_at)
            if not isinstance(output["name"], str) or not IDENT.match(output["name"]) or \
                    output["name"] in [slot.name for slot in next_slots]:
                raise ConfigError(f"{out_at}.name: output names must be unique lower snake_case")
            if output["name"] == "stamp":
                raise ConfigError(f"{out_at}.name: stamp is reserved for the execution stamp")
            if ("type" in output) == ("from" in output):
                raise ConfigError(f"{out_at}: declare exactly one of type or from")
            if "from" in output:
                if output["from"] not in slot_names or "header" in output:
                    raise ConfigError(f"{out_at}.from: must name an input of this stage")
                source = slots[slot_names.index(output["from"])]
                limits = source.limits.copy()
                apply_checks(limits, output, out_at)
                next_slots.append(Slot(output["name"], source.type, limits, False))
                stage.origins.append(slot_names.index(output["from"]))
            else:
                limits = Limits()
                apply_checks(limits, output, out_at)
                next_slots.append(Slot(output["name"], any_type(output, package, out_at), limits, False))
                stage.origins.append(-1)
        stage.outputs = next_slots
        model.stages.append(stage)
        slots = next_slots
    if len(slots) != 1 or not slots[0].type.is_ros:
        raise ConfigError(f"{where}: the last stage must return exactly one ROS message")
    model.output_type = slots[0].type

    output = data["output"]
    check_keys(output, {"topic", "frame_id"}, {"topic"}, f"{where}: output")
    if not isinstance(output["topic"], str) or not TOPIC.match(output["topic"]):
        raise ConfigError(f"{where}: output.topic must be an absolute topic name")
    model.output_topic = output["topic"]
    if "frame_id" in output:
        if not isinstance(output["frame_id"], str) or not FRAME.match(output["frame_id"]):
            raise ConfigError(f"{where}: output.frame_id is invalid")
        model.output_frame = output["frame_id"]

    load_registrations(package_dir, package, node, model)
    return model


def extension_path(package_dir, package, kind, stem):
    return os.path.join(package_dir, "include", package, kind, f"{stem}.hpp")


def extension_stem(value, kind, package_dir, package, where):
    match = EXTENSION_FILE.match(value) if isinstance(value, str) else None
    if not match:
        raise ConfigError(f"{where}: {kind} must be a file name such as name.hpp")
    stem = match.group(1)
    if not os.path.isfile(extension_path(package_dir, package, kind, stem)):
        raise ConfigError(f"{where}: include/{package}/{kind}/{value} does not exist")
    return stem


def load_registrations(package_dir, package, node, model):
    directory = os.path.join(package_dir, "config", node, "registrations")
    files = sorted(f for f in os.listdir(directory) if f.endswith(".yaml")) if os.path.isdir(directory) else []
    stages = {stage.name: stage for stage in model.stages}
    for file_name in files:
        path = os.path.join(directory, file_name)
        stem = file_name[:-len(".yaml")]
        if not IDENT.match(stem):
            raise ConfigError(f"{path}: registration file names must be lower snake_case")
        if not os.path.isfile(extension_path(package_dir, package, "cores", stem)):
            raise ConfigError(f"{path}: include/{package}/cores/{stem}.hpp does not exist")
        data = load_yaml(path)
        check_keys(data, {"stage", "inputs", "validator", "policy", "bank", "allowed_issues"}, {"stage", "inputs"}, path)
        if data["stage"] not in stages:
            raise ConfigError(f"{path}: unknown stage '{data['stage']}'")
        stage = stages[data["stage"]]
        slot_names = [slot.name for slot in stage.slots]
        inputs = data["inputs"]
        if not isinstance(inputs, list) or len(set(inputs)) != len(inputs) or not all(i in slot_names or i == "stamp" for i in inputs):
            raise ConfigError(f"{path}: inputs must be distinct inputs of stage '{stage.name}' or stamp: {slot_names}")
        core = Core(stem)
        core.args = [-1 if name == "stamp" else slot_names.index(name) for name in inputs]
        for kind, attribute in (("validators", "validator"), ("policies", "policy"), ("banks", "bank")):
            if attribute in data:
                setattr(core, attribute, extension_stem(data[attribute], kind, package_dir, package, f"{path}: {attribute}"))
        allowed = data.get("allowed_issues") or {}
        if not isinstance(allowed, dict):
            raise ConfigError(f"{path}: allowed_issues must be a map of input to issue list")
        for name, issues in allowed.items():
            if name == "stamp":
                raise ConfigError(f"{path}: stamp has no health issues")
            if name not in inputs:
                raise ConfigError(f"{path}: allowed_issues input '{name}' is not a core input")
            if not isinstance(issues, list) or not all(issue in KINDS and issue != "Missing" for issue in issues):
                raise ConfigError(f"{path}: allowed_issues.{name} must list issue names other than Missing")
            core.allowed.extend((slot_names.index(name), issue) for issue in issues)
        loader = os.path.isfile(extension_path(package_dir, package, "parameters", stem))
        values = os.path.isfile(os.path.join(package_dir, "config", node, "parameters", f"{stem}.yaml"))
        if loader != values:
            raise ConfigError(f"{path}: include/{package}/parameters/{stem}.hpp and "
                              f"config/{node}/parameters/{stem}.yaml must exist together")
        core.parameters = loader
        stage.cores.append(core)
    cores = {core.stem for stage in model.stages for core in stage.cores}
    for bank in model.banks():
        if bank in cores:
            raise ConfigError(f"{directory}: bank '{bank}' must not share its name with a core")
        loader = os.path.isfile(extension_path(package_dir, package, "parameters", bank))
        values = os.path.isfile(os.path.join(package_dir, "config", node, "parameters", f"{bank}.yaml"))
        if loader != values:
            raise ConfigError(f"include/{package}/parameters/{bank}.hpp and "
                              f"config/{node}/parameters/{bank}.yaml must exist together")
        model.bank_parameters[bank] = loader


def cpp_string(text):
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def cpp_list(values):
    return "{" + ", ".join(str(value) for value in values) + "}"


def generate(model):
    """노드별 타입 연결 헤더다. include·타입 목록·Core ID별 case·Core profile만 담는다."""
    package = model.package
    includes = {"node_pipeline/detail/stages.hpp", "cstddef", "map", "memory", "stdexcept", "string", "utility", "vector"}
    for stage in model.stages:
        for slot in stage.slots + stage.outputs:
            includes.add(slot.type.include)
        for core in stage.cores:
            includes.add(f"{package}/cores/{core.stem}.hpp")
            for kind, stem in (("validators", core.validator), ("policies", core.policy), ("banks", core.bank)):
                if stem:
                    includes.add(f"{package}/{kind}/{stem}.hpp")
            if core.parameters:
                includes.add(f"{package}/parameters/{core.stem}.hpp")
    for bank in model.banks():
        includes.add(f"{package}/banks/{bank}.hpp")
        if model.bank_parameters.get(bank):
            includes.add(f"{package}/parameters/{bank}.hpp")
    cores = [core for stage in model.stages for core in stage.cores]
    banks = model.banks()

    lines = ["// node_pipeline 생성 파일이다. 수정하지 않는다.", "#pragma once"]
    lines += [f"#include <{include}>" for include in sorted(includes)]
    lines += ["", "namespace np_generated {", "namespace npd = node_pipeline::detail;", ""]
    lines.append(f"enum class CoreId : std::size_t {{{', '.join(pascal(core.stem) for core in cores)}}};")
    lines.append("")

    for bank in banks:
        lines += [f"struct Bank_{bank} {{", f"  using Type = ::{package}::{pascal(bank)};",
                  f"  static constexpr bool has_parameters = {'true' if model.bank_parameters.get(bank) else 'false'};",
                  f"  static const char* name() {{ return {cpp_string(bank + '.hpp')}; }}"]
        if model.bank_parameters.get(bank):
            lines.append("  static auto loadParameters(const node_pipeline::ParamReader& params) "
                         f"{{ return ::{package}::load{pascal(bank)}Parameters(params); }}")
        lines += ["};", ""]
    for core in cores:
        bank_types = f"::{package}::{pascal(core.bank)}" if core.bank else ""
        lines += [f"struct Core_{core.stem} {{", f"  using Type = ::{package}::{pascal(core.stem)};",
                  f"  using Banks = npd::Types<{bank_types}>;",
                  f"  static constexpr bool has_parameters = {'true' if core.parameters else 'false'};",
                  f"  static const char* name() {{ return {cpp_string(core.name)}; }}"]
        if core.parameters:
            lines.append("  static auto loadParameters(const node_pipeline::ParamReader& params) "
                         f"{{ return ::{package}::load{pascal(core.stem)}Parameters(params); }}")
        lines += ["};", ""]
    lines.append("struct CoreSet {")
    lines.append(f"  npd::BankSet<{', '.join(f'::{package}::{pascal(bank)}' for bank in banks)}> banks;")
    lines += [f"  ::{package}::{pascal(core.stem)} {core.stem};" for core in cores]
    lines += ["};", ""]

    for s, stage in enumerate(model.stages):
        for i, slot in enumerate(stage.slots):
            if slot.limits.numbers:
                values = ", ".join(f"static_cast<double>(m.{field})" for field in slot.limits.numbers)
                lines += [f"struct Numbers_{s}_{i} {{",
                          f"  template<class T> static std::vector<double> collect(const T& m) {{ return {{{values}}}; }}",
                          "};"]
        numbers = [f"Numbers_{s}_{i}" if slot.limits.numbers else "npd::NoNumbers" for i, slot in enumerate(stage.slots)]
        registry = ", ".join(f"{{{cpp_string(slot.name)}, {i}}}" for i, slot in enumerate(stage.slots))
        lines += [
            f"struct Stage_{s} {{",
            f"  static constexpr std::size_t index = {s};",
            "  using CoreId = np_generated::CoreId;",
            f"  using DataType = npd::Data<{', '.join(slot.type.cpp for slot in stage.slots)}>;",
            f"  using Numbers = npd::Types<{', '.join(numbers)}>;",
            f"  using Outputs = npd::Types<{', '.join(slot.type.cpp for slot in stage.outputs)}>;",
            f"  static std::map<std::string, std::size_t> inputRegistry() {{ return {{{registry}}}; }}",
            f"  static std::vector<int> origins() {{ return {cpp_list(stage.origins)}; }}",
            f"  static std::vector<npd::CoreProfile<Stage_{s}>> coreProfiles();",
            "  static bool processCore(CoreSet& cores, CoreId id, const npd::StageData& data, "
            "const std::int64_t& stamp_ns, npd::CoreResult& result);",
            "};",
            "",
        ]

    for s, stage in enumerate(model.stages):
        profiles = []
        for core in stage.cores:
            validator = f"::{package}::{pascal(core.validator)}" if core.validator else "npd::NoValidator"
            policy = f"::{package}::{pascal(core.policy)}" if core.policy else "npd::NoPolicy"
            validator_name = f"{core.validator}.hpp" if core.validator else ""
            policy_name = f"{core.policy}.hpp" if core.policy else ""
            profiles.append(f"      npd::makeProfile<Stage_{s}, {validator}, {policy}>(CoreId::{pascal(core.stem)}, "
                            f"{cpp_string(core.name)}, {cpp_list('npd::kStampInput' if arg < 0 else arg for arg in core.args)}, "
                            f"{cpp_string(validator_name)}, "
                            f"{cpp_string(policy_name)}),")
        lines += [f"inline std::vector<npd::CoreProfile<Stage_{s}>> Stage_{s}::coreProfiles() {{", "  return {"]
        lines += profiles
        lines += ["  };", "}", ""]
        if stage.cores:
            stamp_parameter = "const std::int64_t& stamp_ns" if any(-1 in core.args for core in stage.cores) else "const std::int64_t&"
            lines.append(f"inline bool Stage_{s}::processCore(CoreSet& cores, CoreId id, const npd::StageData& data, "
                         f"{stamp_parameter}, npd::CoreResult& result) {{")
            lines.append(f"  const std::vector<int> origins = Stage_{s}::origins();")
        else:
            lines.append(f"inline bool Stage_{s}::processCore(CoreSet&, CoreId id, const npd::StageData&, "
                         "const std::int64_t&, npd::CoreResult&) {")
        lines.append("  switch (id) {")
        for core in stage.cores:
            arguments = ", ".join("stamp" if arg < 0 else f"data.message<{stage.slots[arg].type.cpp}>({arg})" for arg in core.args)
            indices = ", ".join("npd::kStampInput" if arg < 0 else str(arg) for arg in core.args)
            lines += [
                f"    case CoreId::{pascal(core.stem)}: {{",
                f"      static_assert(npd::requireProcessContract<::{package}::{pascal(core.stem)}, Stage_{s}, "
                f"std::index_sequence<{indices}>>(), \"process() contract\");",
                f"      const std::vector<std::size_t> required_{core.stem}{cpp_list(arg for arg in core.args if arg >= 0)};",
                f"      const bool present = data.hasInputs(required_{core.stem});",
                "      if (!present) return false;",
                f"      auto& core = cores.{core.stem};",
            ]
            if -1 in core.args:
                lines += ["      ros::Time stamp;", "      stamp.fromNSec(static_cast<std::uint64_t>(stamp_ns));"]
            lines += [f"      auto value = core.process({arguments});",
                "      result = npd::wrapResult(std::move(value), origins);",
                "      break;",
                "    }",
            ]
        lines += ["    default:", "      throw std::invalid_argument(\"unregistered core id\");", "  }", "  return true;", "}", ""]

    loaders = [f"{{{cpp_string(core.name)}, &npd::loadErased<Core_{core.stem}>}}" for core in cores if core.parameters]
    loaders += [f"{{{cpp_string(bank + '.hpp')}, &npd::loadErased<Bank_{bank}>}}" for bank in banks if model.bank_parameters.get(bank)]
    input_types = ", ".join(slot.type.cpp for slot in model.stages[0].slots)
    registry = ", ".join(f"{{{cpp_string(slot.name)}, {i}}}" for i, slot in enumerate(model.stages[0].slots))
    lines += [
        "struct Spec {",
        f"  static const char* package() {{ return {cpp_string(package)}; }}",
        f"  static const char* node() {{ return {cpp_string(model.node)}; }}",
        f"  using Inputs = npd::Types<{input_types}>;",
        f"  using Buffer = npd::InputBuffer<{input_types}>;",
        f"  using Output = {model.output_type.cpp};",
        "  using CoreId = np_generated::CoreId;",
        "  using CoreSet = np_generated::CoreSet;",
        f"  using Schemas = npd::Types<{', '.join(f'Stage_{s}' for s in range(len(model.stages)))}>;",
        f"  using Banks = npd::Types<{', '.join(f'Bank_{bank}' for bank in banks)}>;",
        f"  using Cores = npd::Types<{', '.join(f'Core_{core.stem}' for core in cores)}>;",
        f"  static std::map<std::string, std::size_t> inputRegistry() {{ return {{{registry}}}; }}",
        f"  static std::vector<std::string> coreNames() {{ return {{{', '.join(cpp_string(core.name) for core in cores)}}}; }}",
        "  static std::map<std::string, npd::ParameterLoader> parameterLoaders() {",
        f"    return {{{', '.join(loaders)}}};",
        "  }",
        "};",
        "}  // namespace np_generated",
        "",
    ]
    return "\n".join(lines)


def write_if_changed(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    previous = None
    if os.path.exists(path):
        with open(path, encoding="utf-8") as stream:
            previous = stream.read()
    if previous != text:
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(text)


def copy_config(package_dir, node, model, config_copy):
    """실행 시 공통 main이 읽는 node.yaml·등록 YAML 사본이다. 재빌드할 때만 갱신된다."""
    node_dir = os.path.join(package_dir, "config", node)
    with open(os.path.join(node_dir, "node.yaml"), encoding="utf-8") as stream:
        write_if_changed(os.path.join(config_copy, "node.yaml"), stream.read())
    registrations = os.path.join(config_copy, "registrations")
    os.makedirs(registrations, exist_ok=True)
    keep = {f"{core.stem}.yaml" for stage in model.stages for core in stage.cores}
    for name in os.listdir(registrations):
        if name not in keep:
            os.remove(os.path.join(registrations, name))
    for name in keep:
        with open(os.path.join(node_dir, "registrations", name), encoding="utf-8") as stream:
            write_if_changed(os.path.join(registrations, name), stream.read())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", required=True)
    parser.add_argument("--package", required=True)
    parser.add_argument("--node", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--config-copy", required=True)
    args = parser.parse_args()
    try:
        model = load_node(args.package_dir, args.package, args.node)
    except ConfigError as error:
        print(f"node_pipeline: {error}", file=sys.stderr)
        return 1
    write_if_changed(args.output, generate(model))
    copy_config(args.package_dir, args.node, model, args.config_copy)
    return 0


if __name__ == "__main__":
    sys.exit(main())
