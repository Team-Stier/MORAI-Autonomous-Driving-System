#!/usr/bin/env python3
"""Validate declarations and emit only typed connections to the fixed runtime."""
import argparse
import json
import math
from pathlib import Path
import re
import sys

import yaml
from generate_numeric_traits import emit_numeric_traits


class DeclarationError(ValueError):
    pass


class UniqueLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, (str, int)) or isinstance(key, bool):
            raise DeclarationError("mapping keys must be names")
        if key in result:
            raise DeclarationError("duplicate key: " + str(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)
ISSUES = {"Missing", "InvalidStamp", "FutureStamp", "FrameMismatch", "NaN", "Inf",
          "OutRangePositive", "OutRangeNegative", "Stale", "ReceiveTimeout", "Skew"}
IDENTIFIER = re.compile(r"[A-Za-z_][A-Za-z_0-9]*\Z")
QUALIFIED = re.compile(r"(?:::)?[A-Za-z_][A-Za-z_0-9]*(?:::[A-Za-z_][A-Za-z_0-9]*)*\Z")
HEADER = re.compile(r"[A-Za-z_0-9/-]+\.(?:hpp|h)\Z")


def fail(message):
    raise DeclarationError(message)


def mapping(value, where, allowed=None, required=()):
    if not isinstance(value, dict):
        fail(where + " must be a mapping")
    if allowed is not None:
        extra = set(value) - set(allowed)
        if extra:
            fail(where + " unknown fields: " + ", ".join(sorted(map(str, extra))))
    missing = set(required) - set(value)
    if missing:
        fail(where + " missing fields: " + ", ".join(sorted(missing)))
    return value


def sequence(value, where):
    if not isinstance(value, list):
        fail(where + " must be an ordered list")
    return value


def string(value, where, empty=False):
    if not isinstance(value, str) or (not value and not empty) or "\0" in value:
        fail(where + " must be a string")
    return value


def name(value, where):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        fail(where + " must be an identifier")
    return value


def header(value, where):
    if not isinstance(value, str) or not HEADER.fullmatch(value) or ".." in value or value.startswith("/"):
        fail(where + " must be a relative include header")
    return value


def native_type(value, where):
    value = string(value, where)
    if re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*/[A-Za-z_][A-Za-z_0-9]*", value):
        return value.replace("/", "::")
    # A small type grammar, never a C++ expression or declarator.
    tokens = re.findall(r"::|[A-Za-z_][A-Za-z_0-9]*|[0-9]+|[<>,]", value)
    if "".join(tokens) != re.sub(r"\s+", "", value):
        fail(where + " invalid native type")
    index = 0

    def parse():
        nonlocal index
        if index < len(tokens) and tokens[index] == "::":
            index += 1
        if index >= len(tokens) or not IDENTIFIER.fullmatch(tokens[index]):
            fail(where + " invalid native type")
        index += 1
        while index < len(tokens) and tokens[index] == "::":
            index += 1
            if index >= len(tokens) or not IDENTIFIER.fullmatch(tokens[index]):
                fail(where + " invalid native type")
            index += 1
        if index < len(tokens) and tokens[index] == "<":
            index += 1
            while True:
                if index < len(tokens) and tokens[index].isdigit():
                    index += 1
                else:
                    parse()
                if index < len(tokens) and tokens[index] == ",":
                    index += 1
                    continue
                if index >= len(tokens) or tokens[index] != ">":
                    fail(where + " invalid native type")
                index += 1
                break
    parse()
    if index != len(tokens):
        fail(where + " invalid native type")
    return "".join(tokens).removeprefix("::") if hasattr(str, "removeprefix") else "".join(tokens).lstrip(":")


def inferred_header(type_name):
    if not type_name.startswith("std::") and "<" not in type_name and type_name.count("::") == 1:
        return type_name.replace("::", "/") + ".h"
    return None


def number(value, where, positive=False, integer=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value < -sys.float_info.max or value > sys.float_info.max or not math.isfinite(value):
        fail(where + " must be finite numeric data")
    if (positive and value <= 0) or (integer and not isinstance(value, int)):
        fail(where + " must be a positive integer" if integer else where + " must be positive")
    return value


def bounded_integer(value, where, minimum, maximum):
    number(value, where, integer=True)
    if not minimum <= value <= maximum:
        fail(where + " outside representable integer range")
    return value


def duration_ns(value, where):
    number(value, where, positive=True)
    if value > 9223372036854775807 / 1e9:
        fail(where + " outside representable duration")
    ns = round(value * 1e9)
    if ns <= 0 or ns > 9223372036854775807:
        fail(where + " outside representable duration")
    return ns


def boolean(value, where):
    if not isinstance(value, bool):
        fail(where + " must be boolean")
    return value


def load(path):
    try:
        return yaml.load(path.read_text(), Loader=UniqueLoader)
    except (yaml.YAMLError, DeclarationError) as error:
        fail(str(path) + ": " + str(error))


def camel(stem):
    return "".join(piece[:1].upper() + piece[1:] for piece in stem.split("_"))


def adapter(value, role, package, stem):
    defaults = {"parameter_loader": "NoParameters", "validator": "NoValidator", "policy": "RemoveInvalid"}
    builtin = defaults[role]
    if value is None or value == builtin or value == "node_pipeline::" + builtin:
        return {"type": "node_pipeline::" + builtin}
    directory = {"parameter_loader": "parameters", "validator": "validators", "policy": "policies"}[role]
    if not isinstance(value, str):
        fail(role + " must be a builtin name or " + directory + " file reference")
    if value.startswith(package + "/"):
        header_value = value
    elif value.startswith(directory + "/"):
        header_value = package + "/" + value
    else:
        fail(role + " must be a builtin name or " + directory + " file reference")
    type_value = package + "::" + directory + "::" + camel(Path(header_value).stem)
    header_value = header(header_value, role + ".header")
    if not header_value.startswith(package + "/" + directory + "/"):
        fail(role + " must use this package's " + directory + " extension")
    if not isinstance(type_value, str) or not QUALIFIED.fullmatch(type_value):
        fail(role + " invalid class type")
    if not type_value.lstrip(":").startswith(package + "::" + directory + "::"):
        fail(role + " must use this package's " + directory + " class")
    return {"header": header_value, "type": type_value.lstrip(":")}


def read_declarations(package_dir, node_path="node.yaml"):
    package_dir = Path(package_dir).resolve()
    package = package_dir.name
    name(package, "package")
    node = mapping(load(package_dir / node_path), "node", {
        "name", "inputs", "banks", "stages", "output", "execution", "assets", "section_topic"},
        {"name", "inputs", "stages", "output"})
    name(node["name"], "node.name")
    node_inputs = mapping(node["inputs"], "inputs")
    inputs = {}
    for key, value in node_inputs.items():
        name(key, "input name")
        if key in {"section", "section_annotation"}:
            fail("section is selection metadata, not a Core input")
        value = mapping(value, "input " + key, {"type", "topic", "limits"}, {"type", "topic"})
        value = dict(value, name=key, type=native_type(value["type"], "input " + key + ".type"))
        string(value["topic"], "input.topic")
        if value["topic"].strip("/") == "section_annotation" or value["topic"] == node.get("section_topic"):
            fail("/section_annotation is selection metadata, not a Core input")
        limits = mapping(value.get("limits", {}), "input.limits", {"lower", "upper", "frame", "stale_after_sec", "receive_timeout_sec", "max_sensor_skew_sec", "check_stamp", "check_receive_timeout", "check_skew"})
        for field, limit in limits.items():
            if field.startswith("check_"):
                boolean(limit, field)
            elif field == "frame":
                string(limit, field)
            else:
                if field.endswith("_sec"):
                    duration_ns(limit, field)
                else:
                    number(limit, field)
        if limits.get("lower", -sys.float_info.max) > limits.get("upper", sys.float_info.max):
            fail("input lower exceeds upper")
        inputs[key] = value
    banks = {}
    for key, value in mapping(node.get("banks", {}), "banks").items():
        name(key, "bank name")
        value = mapping(value, "bank " + key, {"type", "header"}, {"type", "header"})
        banks[key] = {"type": native_type(value["type"], "bank.type"), "header": header(value["header"], "bank.header")}
        if not banks[key]["header"].startswith(package + "/") or not banks[key]["type"].startswith(package + "::"):
            fail("Bank must belong to this package")
    stages = mapping(node["stages"], "stages")
    if not stages:
        fail("at least one Stage is required")
    for key, stage in stages.items():
        name(key, "stage name")
        mapping(stage, "stage " + key, {"cores", "section_core_map", "health_core_map", "max_health_checks"}, {"cores"})
        candidates = sequence(stage["cores"], "stage.cores")
        for candidate in candidates:
            string(candidate, "Stage candidate")
        if len(set(candidates)) != len(candidates):
            fail("duplicate Stage candidate")
        if "max_health_checks" in stage:
            bounded_integer(stage["max_health_checks"], "max_health_checks", 1, 18446744073709551615)
    assets = mapping(node.get("assets", {}), "assets")
    for key, value in assets.items():
        name(key, "asset name")
        mapping(value, "asset", {"parameter", "fallback_parameter", "sibling"}, {"parameter"})
        for field, item in value.items():
            string(item, "asset." + field)
    cores = []
    ids = set()
    for path in sorted((package_dir / "registrations").glob("*.yaml")):
        core = mapping(load(path), str(path), {"name", "type", "stage", "inputs", "outputs", "validator", "policy", "parameter_loader", "parameter_namespace", "bank", "allowed_issues", "assets"}, {"name", "type", "stage", "inputs", "outputs"})
        core = dict(core)
        if not isinstance(core["name"], str) or not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*\.hpp", core["name"]):
            fail("Core name must be its .hpp filename")
        if core["name"] in ids:
            fail("duplicate Core name: " + core["name"])
        ids.add(core["name"])
        core["type"] = native_type(core["type"], "Core.type")
        if not core["type"].startswith(package + "::") or "<" in core["type"]:
            fail("Core type must be a class in this package")
        core["header"] = package + "/cores/" + core["name"]
        if not (package_dir / "include" / core["header"]).is_file():
            fail("missing Core header: " + core["header"])
        name(core["stage"], "Core.stage")
        for role in ("validator", "policy", "parameter_loader"):
            if role in core and not isinstance(core[role], str):
                fail(role + " must be a builtin name or role file reference")
            core[role] = adapter(core.get(role), role, package, Path(core["name"]).stem)
        no_parameters = core["parameter_loader"]["type"] == "node_pipeline::NoParameters"
        if not no_parameters and "parameter_namespace" not in core:
            fail("parameter_namespace is required for a parameter_loader")
        core["parameter_namespace"] = string(core.get("parameter_namespace", ""), "parameter_namespace", no_parameters)
        if core["parameter_namespace"] and not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*(?:/[A-Za-z_][A-Za-z_0-9]*)*", core["parameter_namespace"]):
            fail("parameter_namespace must be a relative namespace")
        if core.get("bank") is not None:
            name(core["bank"], "Core.bank")
        core["inputs"] = sequence(core["inputs"], "Core.inputs")
        for input_name in core["inputs"]:
            name(input_name, "Core input")
        if len(set(core["inputs"])) != len(core["inputs"]):
            fail("duplicate Core input")
        core["assets"] = sequence(core.get("assets", []), "Core.assets")
        for asset_name in core["assets"]:
            name(asset_name, "Core asset")
        if len(set(core["assets"])) != len(core["assets"]):
            fail("duplicate Core asset")
        core["outputs"] = sequence(core["outputs"], "Core.outputs")
        if not core["outputs"]:
            fail("Core must return native output values")
        for output in core["outputs"]:
            mapping(output, "Core.output", {"name", "type", "time_from"}, {"name", "type"})
            name(output["name"], "output name")
            output["type"] = native_type(output["type"], "output.type")
            if "time_from" in output:
                string(output["time_from"], "output.time_from", True)
        if len({out["name"] for out in core["outputs"]}) != len(core["outputs"]):
            fail("duplicate Core output")
        cores.append(core)
    cores = [core for core in cores if any(core["name"] in stage["cores"] for stage in stages.values())]
    by_id = {core["name"]: core for core in cores}
    available = {key: value["type"] for key, value in inputs.items()}
    for stage_name, stage in stages.items():
        stage_cores = []
        for candidate in stage["cores"]:
            if candidate not in by_id or by_id[candidate]["stage"] != stage_name:
                fail("unknown Stage candidate: " + str(candidate))
            core = by_id[candidate]
            if core.get("bank") is not None and core["bank"] not in banks:
                fail("unknown Bank: " + str(core["bank"]))
            if any(asset_name not in assets for asset_name in core["assets"]):
                fail("Core.assets must select declared node assets")
            resolved = []
            for input_name in core["inputs"]:
                if input_name not in available:
                    fail("Core input not returned by the previous Stage: " + input_name)
                resolved.append(dict(name=input_name, type=available[input_name]))
            if len({item["name"] for item in resolved}) != len(resolved):
                fail("duplicate Core input")
            core["inputs"] = resolved
            input_names = {item["name"] for item in resolved}
            for out in core["outputs"]:
                if out.get("time_from", "") not in input_names | {""}:
                    fail("time_from must name a Core input")
            allowed = mapping(core.get("allowed_issues", {}), "allowed_issues")
            for input_name, issues in allowed.items():
                if input_name not in input_names:
                    fail("allowed_issues unknown input: " + input_name)
                nonwaivable = {"Missing", "InvalidStamp", "FutureStamp", "FrameMismatch", "NaN", "Inf"}
                if any(not isinstance(issue, str) or issue not in ISSUES or issue in nonwaivable for issue in sequence(issues, "allowed_issues")):
                    fail("allowed_issues contains unknown or non-waivable issue")
            core["allowed_issues"] = allowed
            stage_cores.append(core)
        section_map = mapping(stage.get("section_core_map", {}), "section_core_map")
        for section, candidate in section_map.items():
            if not re.fullmatch(r"[0-9]+", str(section)) or not 0 <= int(section) <= 255 or (candidate not in stage["cores"] and candidate != ""):
                fail("invalid section_core_map entry")
        normalized = [str(int(key)) for key in section_map]
        if len(set(normalized)) != len(normalized):
            fail("duplicate normalized section_core_map key")
        stage["section_core_map"] = {str(int(key)): value for key, value in section_map.items() if value != ""}
        stage["health_core_map"] = sequence(stage.get("health_core_map", []), "health_core_map")
        for rule in stage["health_core_map"]:
            mapping(rule, "health rule", {"when", "priority", "core"}, {"when", "priority", "core"})
            bounded_integer(rule["priority"], "health priority", -2147483648, 2147483647)
            if rule["core"] not in stage["cores"]:
                fail("health rule must select a Stage candidate")
            when = sequence(rule["when"], "health.when")
            if not when:
                fail("health rule has empty when")
            for condition in when:
                mapping(condition, "health condition", {"input", "issue"}, {"input", "issue"})
                if not isinstance(condition["input"], str) or not isinstance(condition["issue"], str) or condition["input"] not in available or condition["issue"] not in ISSUES:
                    fail("health condition has unknown input or issue")
        # Native outputs replace the whole frame. No original-input forwarding.
        signatures = [{out["name"]: out["type"] for out in core["outputs"]} for core in stage_cores]
        available = signatures[0] if signatures else {}
        for signature in signatures[1:]:
            available = {key: value for key, value in available.items() if signature.get(key) == value}
    output = dict(mapping(node["output"], "output", {"source", "topic", "header", "type"}, {"source", "topic"}))
    source = string(output["source"], "output.source")
    if "." in source:
        stage_name, source = source.split(".", 1)
        if stage_name != list(stages)[-1]:
            fail("output source must be from the final Stage")
    name(source, "output source")
    if available:
        if source not in available:
            fail("output source must be returned by every final Stage candidate")
        inferred = available[source]
    elif cores:
        fail("final Stage has no output")
    elif "type" in output:
        inferred = native_type(output["type"], "output.type")
    else:
        fail("a node with no registered Core needs an explicit output.type")
    if cores and "type" in output:
        fail("output.type is inferred from final registered Core")
    output.update(source=source, type=inferred)
    string(output["topic"], "output.topic")
    metadata = mapping(output.get("header", {}), "output.header", {"stamp_from", "frame_id"})
    for key, value in metadata.items():
        string(value, "output.header." + key, True)
    if metadata.get("stamp_from", "") not in set(inputs) | {""}:
        fail("output stamp_from must name a subscribed input")
    output["header"] = metadata
    execution = mapping(node.get("execution", {}), "execution", {"mode", "trigger_input", "frequency_hz", "queue_capacity", "require_new_snapshot"})
    mode = execution.get("mode", "periodic")
    if mode not in {"periodic", "on_input", "latched"}:
        fail("invalid execution mode")
    if mode == "on_input" and execution.get("trigger_input") not in inputs:
        fail("on_input trigger_input must name a subscribed input")
    if mode != "on_input" and "trigger_input" in execution:
        fail("trigger_input is only valid for on_input")
    if mode != "periodic" and "frequency_hz" in execution:
        fail("frequency_hz is only valid for periodic")
    for key in ("frequency_hz", "queue_capacity"):
        if key in execution:
            if key == "queue_capacity":
                bounded_integer(execution[key], key, 1, 18446744073709551615)
            else:
                number(execution[key], key, True)
    if "require_new_snapshot" in execution:
        boolean(execution["require_new_snapshot"], "require_new_snapshot")
    if "section_topic" in node:
        string(node["section_topic"], "section_topic")
    extensions = []
    for core in cores:
        extensions.append({"header": core["header"], "role": "core", "core": core["type"]})
        for role in ("validator", "policy", "parameter_loader"):
            if "header" in core[role]:
                extensions.append({"header": core[role]["header"], "role": role, "core": core["type"]})
    for value in banks.values():
        extensions.append({"header": value["header"], "role": "bank", "core": ""})
    for extension in extensions:
        if not (package_dir / "include" / extension["header"]).is_file():
            fail("missing extension header: " + extension["header"])
    return dict(package=package, package_dir=str(package_dir), name=node["name"], inputs=inputs, input_order=list(inputs),
                banks=banks, stages=stages, cores=cores, output=output, execution=execution, assets=assets,
                section_topic=node.get("section_topic"),
                extensions=extensions)


def quote(value):
    return json.dumps(value, ensure_ascii=True)


def emit(manifest):
    includes = {"node_pipeline/detail/ros_runtime.hpp"}
    for item in list(manifest["inputs"].values()) + [manifest["output"]] + [port for core in manifest["cores"] for port in core["inputs"] + core["outputs"]]:
        include = item.get("native_header", item.get("header"))
        if not isinstance(include, str):
            include = inferred_header(item["type"])
        if include:
            includes.add(include)
    includes.update(extension["header"] for extension in manifest["extensions"])
    lines = ["// Generated type/declaration connections. Edit YAML, not this file."]
    lines.append(emit_numeric_traits(manifest))
    lines += ['#include <' + item + '>' for item in sorted(includes)]
    lines += ["namespace node_pipeline { namespace detail {", "int runGenerated(int argc, char** argv) {", "  RosRuntime runtime(argc, argv, " + quote(manifest["name"]) + ");", "  ExecutionSpec execution;"]
    for key, value in manifest["execution"].items():
        literal = quote(value) if isinstance(value, str) else str(value).lower()
        if key == "queue_capacity":
            literal += "ULL"
        lines.append("  execution." + key + " = " + literal + ";")
    lines.append("  runtime.execution(execution);")
    if manifest["section_topic"]:
        lines.append("  runtime.sectionTopic(" + quote(manifest["section_topic"]) + ");")
    for asset_name, asset in manifest["assets"].items():
        lines.append("  runtime.asset(" + quote(asset_name) + ", AssetSpec{" + ", ".join(quote(asset.get(key, "")) for key in ("parameter", "fallback_parameter", "sibling")) + "});")
    for index, item in enumerate(manifest["inputs"].values()):
        variable = "input_" + str(index)
        lines.append("  InputSpec " + variable + "{" + quote(item["name"]) + ", " + quote(item["topic"]) + "};")
        for key, value in item.get("limits", {}).items():
            cpp_key = {"stale_after_sec": "stale_ns", "receive_timeout_sec": "receive_timeout_ns", "max_sensor_skew_sec": "max_skew_ns"}.get(key, key)
            if key.endswith("_sec"):
                ns = duration_ns(value, key)
                literal = str(ns) + "LL"
            else:
                literal = quote(value) if isinstance(value, str) else str(value).lower()
            lines.append("  " + variable + ".limits." + cpp_key + " = " + literal + ";")
        lines.append("  runtime.input<" + item["type"] + ">(" + variable + ");")
    for bank_name, bank in manifest["banks"].items():
        lines.append("  runtime.bank<" + bank["type"] + ">(" + quote(bank_name) + ");")
    for index, core in enumerate(manifest["cores"]):
        args = "std::tuple<" + ", ".join(port["type"] for port in core["inputs"]) + ">"
        outs = "std::tuple<" + ", ".join(port["type"] for port in core["outputs"]) + ">"
        lines.append("  checkCoreSignature<" + core["type"] + ", " + args + ", " + outs + ">();")
        variable = "core_" + str(index)
        lines += ["  CoreSpec " + variable + ";", "  " + variable + ".name = " + quote(core["name"]) + ";", "  " + variable + ".stage = " + quote(core["stage"]) + ";"]
        lines.append("  " + variable + ".inputs = {" + ", ".join(quote(port["name"]) for port in core["inputs"]) + "};")
        lines.append("  " + variable + ".assets = {" + ", ".join(quote(asset_name) for asset_name in core["assets"]) + "};")
        lines.append("  " + variable + ".outputs = {" + ", ".join("{" + quote(port["name"]) + ", " + quote(port.get("time_from", "")) + "}" for port in core["outputs"]) + "};")
        for key, issues in core["allowed_issues"].items():
            lines.append("  " + variable + ".allowed_issues[" + quote(key) + "] = {" + ", ".join("Kind::" + issue for issue in issues) + "};")
        types = [core["type"]] + [core[role]["type"] for role in ("parameter_loader", "validator", "policy")]
        if core.get("bank"):
            types.append(manifest["banks"][core["bank"]]["type"])
        lines.append("  runtime.core<" + ", ".join(types) + ">(" + variable + ", " + quote(core["parameter_namespace"]) + ", " + quote(core.get("bank", "")) + ");")
    for index, (stage_name, stage) in enumerate(manifest["stages"].items()):
        variable = "stage_" + str(index)
        lines += ["  StageSpec " + variable + ";", "  " + variable + ".name = " + quote(stage_name) + ";"]
        lines.append("  " + variable + ".max_health_checks = " + str(stage.get("max_health_checks", 10)) + "ULL;")
        lines.append("  " + variable + ".section_core_map = {" + ", ".join("{" + key + ", " + quote(value) + "}" for key, value in stage["section_core_map"].items()) + "};")
        for rule_index, rule in enumerate(stage["health_core_map"]):
            rule_variable = "rule_" + str(index) + "_" + str(rule_index)
            lines += ["  HealthRule " + rule_variable + ";", "  " + rule_variable + ".core = " + quote(rule["core"]) + ";", "  " + rule_variable + ".priority = " + str(rule["priority"]) + ";"]
            lines.append("  " + rule_variable + ".when = {" + ", ".join("{" + quote(item["input"]) + ", Kind::" + item["issue"] + "}" for item in rule["when"]) + "};")
            lines.append("  " + variable + ".health_core_map.push_back(" + rule_variable + ");")
        lines.append("  runtime.stage(" + variable + ");")
    output = manifest["output"]
    metadata = output["header"]
    lines += ["  OutputSpec output;", "  output.source = " + quote(output["source"]) + ";", "  output.topic = " + quote(output["topic"]) + ";"]
    for key in ("stamp_from", "frame_id"):
        lines.append("  output." + key + " = " + quote(metadata.get(key, "")) + ";")
    lines += ["  runtime.output<" + output["type"] + ">(output);", "  return runtime.run();", "}", "}}", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", required=True)
    parser.add_argument("--node", default="node.yaml")
    parser.add_argument("--output", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--traits-header")
    args = parser.parse_args()
    try:
        manifest = read_declarations(args.package_dir, args.node)
        generated = emit(manifest)
        outputs = [(Path(args.output), generated), (Path(args.manifest), json.dumps(manifest, indent=2) + "\n")]
        if args.traits_header:
            outputs.append((Path(args.traits_header), "#pragma once\n" + emit_numeric_traits(manifest)))
        for path, content in outputs:
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists() or path.read_text() != content:
                path.write_text(content)
    except (ValueError, ImportError, OSError) as error:
        print("registration error: " + str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
