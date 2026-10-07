#!/usr/bin/env python3
"""Contract failures at the declaration and compiled-extension boundaries."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import yaml


SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"


class RegistrationToolsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.package = Path(self.directory.name) / "example"
        self.package.mkdir()
        (self.package / "registrations").mkdir()
        self.headers = self.package / "include" / "example" / "cores"
        self.headers.mkdir(parents=True)
        (self.headers / "own.hpp").write_text(
            "#pragma once\nnamespace example { struct Own { int process(int x) { return x; } }; }\n")
        (self.package / "node.yaml").write_text("""name: example_node
inputs:
  sample: {type: int, topic: /sample}
stages:
  first:
    cores: [own.hpp]
    section_core_map: {"0": own.hpp}
    health_core_map: []
output:
  source: first.result
  topic: /result
  header: {stamp_from: sample}
""")
        self.registration = self.package / "registrations" / "own.yaml"
        self.registration.write_text("""name: own.hpp
type: example::Own
stage: first
inputs: [sample]
outputs: [{name: result, type: int, time_from: sample}]
allowed_issues: {}
""")

    def tearDown(self):
        self.directory.cleanup()

    def generate(self, node="node.yaml", traits_header=None):
        arguments = [sys.executable, str(SCRIPTS / "generate_registration.py"),
                     "--package-dir", str(self.package), "--node", node, "--output", str(self.package / "generated.cpp"),
                     "--manifest", str(self.package / "manifest.json")]
        if traits_header:
            arguments += ["--traits-header", str(traits_header)]
        return subprocess.run(arguments,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_canonical_names_infer_native_signature_and_output(self):
        result = self.generate()
        self.assertEqual(result.returncode, 0, result.stderr)
        manifest = json.loads((self.package / "manifest.json").read_text())
        self.assertEqual(manifest["cores"][0]["inputs"], [{"name": "sample", "type": "int"}])
        self.assertEqual(manifest["output"]["type"], "int")
        self.assertEqual(manifest["output"]["source"], "result")
        self.assertEqual(manifest["input_order"], ["sample"])
        self.assertEqual(manifest["cores"][0]["assets"], [])
        self.assertEqual(manifest["cores"][0]["parameter_namespace"], "")

    def change_yaml(self, path, change):
        value = yaml.safe_load(path.read_text())
        change(value)
        path.write_text(yaml.safe_dump(value, sort_keys=False))

    def test_numeric_traits_header_is_a_separate_test_build_artifact(self):
        traits = self.package / "numeric.hpp"
        result = self.generate(traits_header=traits)
        self.assertEqual(result.returncode, 0, result.stderr)
        content = traits.read_text()
        self.assertTrue(content.startswith("#pragma once\n"))
        self.assertIn("NumericInspector", content)
        self.assertNotIn("runGenerated", content)
        self.assertNotIn("ros_runtime.hpp", content)

    def test_native_string_between_stages_does_not_infer_a_ros_header(self):
        node = self.package / "node.yaml"
        node.write_text(node.read_text().replace("output:\n", "  second:\n    cores: [second.hpp]\n    section_core_map: {\"0\": second.hpp}\noutput:\n").replace("first.result", "second.result"))
        self.registration.write_text(self.registration.read_text().replace("type: int", "type: std::string"))
        (self.headers / "own.hpp").write_text("#include <string>\nnamespace example { struct Own { std::string process(int value) { return std::to_string(value); } }; }\n")
        (self.package / "registrations/second.yaml").write_text("name: second.hpp\ntype: example::Second\nstage: second\ninputs: [result]\noutputs: [{name: result, type: int}]\n")
        (self.headers / "second.hpp").write_text("#include <string>\nnamespace example { struct Second { int process(const std::string& value) { return value.size(); } }; }\n")
        result = self.generate()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("#include <std/string.h>", (self.package / "generated.cpp").read_text())

    def test_typed_core_inputs_and_output_header_overrides_are_rejected(self):
        original = self.registration.read_text()
        for change in (lambda value: value.update(inputs=[{"name": "sample", "type": "int"}]),
                       lambda value: value.update(inputs=[{"name": "sample", "header": "injected.hpp"}]),
                       lambda value: value["outputs"][0].update(header="injected.hpp")):
            with self.subTest(change=change):
                self.registration.write_text(original)
                self.change_yaml(self.registration, change)
                self.assertNotEqual(self.generate().returncode, 0)
        self.registration.write_text(original)
        self.change_yaml(self.package / "node.yaml", lambda value: value["inputs"]["sample"].update(header="injected.hpp"))
        self.assertNotEqual(self.generate().returncode, 0)

    def test_execution_fields_belong_only_to_their_modes(self):
        node = self.package / "node.yaml"
        original = node.read_text()
        invalid = ({"mode": "on_input", "trigger_input": "sample", "frequency_hz": 5},
                   {"mode": "periodic", "trigger_input": "sample"},
                   {"mode": "latched", "frequency_hz": 5},
                   {"mode": "periodic", "max_health_checks": 10})
        for execution in invalid:
            with self.subTest(execution=execution):
                node.write_text(original)
                self.change_yaml(node, lambda value: value.update(execution=execution))
                self.assertNotEqual(self.generate().returncode, 0)
        node.write_text(original)
        self.change_yaml(node, lambda value: value.update(execution={"mode": "latched"}))
        self.assertEqual(self.generate().returncode, 0)
        generated = (self.package / "generated.cpp").read_text()
        self.assertNotIn("output.latched", generated)
        self.assertNotIn("staticTransforms", generated)

    def test_health_priority_rejects_int32_overflow_and_underflow(self):
        node = self.package / "node.yaml"
        original = node.read_text()
        for priority in (-2147483648, 2147483647, -2147483649, 2147483648):
            with self.subTest(priority=priority):
                node.write_text(original)
                self.change_yaml(node, lambda value: value["stages"]["first"].update(health_core_map=[
                    {"when": [{"input": "sample", "issue": "Missing"}], "priority": priority, "core": "own.hpp"}]))
                result = self.generate()
                self.assertEqual(result.returncode == 0, -2147483648 <= priority <= 2147483647, result.stderr)

    def test_integer_counts_and_durations_have_cpp_representation_limits(self):
        node = self.package / "node.yaml"
        original = node.read_text()
        changes = [
            lambda value: value.update(execution={"queue_capacity": 18446744073709551616}),
            lambda value: value["stages"]["first"].update(max_health_checks=18446744073709551616),
            lambda value: value["inputs"]["sample"].update(limits={"receive_timeout_sec": 1e300}),
            lambda value: value["inputs"]["sample"].update(limits={"receive_timeout_sec": 1e-20}),
            lambda value: value["inputs"]["sample"].update(limits={"lower": 10 ** 1000}),
        ]
        for change in changes:
            with self.subTest(change=change):
                node.write_text(original)
                self.change_yaml(node, change)
                result = self.generate()
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("Traceback", result.stderr)
        node.write_text(original)
        self.change_yaml(node, lambda value: value.update(execution={"queue_capacity": 18446744073709551615}))
        self.assertEqual(self.generate().returncode, 0)
        self.assertIn("18446744073709551615ULL", (self.package / "generated.cpp").read_text())

    def test_legacy_node_output_controls_are_rejected(self):
        node = self.package / "node.yaml"
        original = node.read_text()
        for change in (lambda value: value["output"].update(type="int"),
                       lambda value: value["output"].update(latched=True),
                       lambda value: value.update(enable_static_transforms=True)):
            node.write_text(original)
            self.change_yaml(node, change)
            self.assertNotEqual(self.generate().returncode, 0)

    def test_relative_section_topic_cannot_be_a_core_input(self):
        for topic in ("section_annotation", "/section_annotation"):
            self.change_yaml(self.package / "node.yaml", lambda value: value["inputs"]["sample"].update(topic=topic))
            result = self.generate()
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("selection metadata", result.stderr)

    def test_declared_assets_are_granted_only_to_the_registered_core(self):
        self.change_yaml(self.package / "node.yaml", lambda value: value.update(assets={"checkpoints": {"parameter": "own/file"}}))
        self.change_yaml(self.registration, lambda value: value.update(assets=["checkpoints"]))
        result = self.generate()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('core_0.assets = {"checkpoints"}', (self.package / "generated.cpp").read_text())
        self.change_yaml(self.registration, lambda value: value.update(assets=["undeclared"]))
        self.assertNotEqual(self.generate().returncode, 0)

    def test_different_manifest_cores_do_not_require_current_stage_or_assets(self):
        (self.headers / "static.hpp").write_text("namespace example { struct Static { int process() { return 1; } }; }\n")
        detached = self.package / "registrations/static.yaml"
        detached.write_text("name: static.hpp\ntype: example::Static\nstage: startup\ninputs: []\noutputs: [{name: result, type: int}]\nassets: [mounts]\n")
        result = self.generate()
        self.assertEqual(result.returncode, 0, result.stderr)
        manifest = json.loads((self.package / "manifest.json").read_text())
        self.assertEqual([core["name"] for core in manifest["cores"]], ["own.hpp"])
        self.assertNotIn("static.hpp", (self.package / "generated.cpp").read_text())
        static_node = self.package / "static_node.yaml"
        static_node.write_text("name: static_node\ninputs: {}\nassets: {mounts: {parameter: static/mounts}}\nstages: {startup: {cores: [static.hpp], section_core_map: {'0': static.hpp}}}\nexecution: {mode: latched}\noutput: {source: startup.result, topic: /static}\n")
        result = self.generate(str(static_node))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("#include <example/cores/static.hpp>", (self.package / "generated.cpp").read_text())
        detached.write_text(detached.read_text().replace("name: static.hpp", "name: own.hpp"))
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("duplicate Core name", result.stderr)

    def test_parameter_loader_requires_its_exact_namespace(self):
        parameters = self.headers.parent / "parameters"
        parameters.mkdir()
        (parameters / "own.hpp").write_text("namespace example { namespace parameters { struct Own {}; }}\n")
        self.change_yaml(self.registration, lambda value: value.update(parameter_loader="parameters/own.hpp"))
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("parameter_namespace", result.stderr)
        self.change_yaml(self.registration, lambda value: value.update(parameter_namespace="stage1/own"))
        result = self.generate()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"stage1/own"', (self.package / "generated.cpp").read_text())

    def test_adapters_accept_only_builtin_names_or_their_role_file_references(self):
        original = self.registration.read_text()
        for role, directory, builtin in (("parameter_loader", "parameters", "NoParameters"),
                                         ("validator", "validators", "NoValidator"),
                                         ("policy", "policies", "RemoveInvalid")):
            for value in ({"header": "example/" + directory + "/own.hpp", "type": "example::" + directory + "::Own"},
                          "example::" + directory + "::Own", "own.hpp", "own", "other/own.hpp",
                          "../" + directory + "/own.hpp", "makeFactory()", [builtin], 1, None):
                with self.subTest(role=role, value=value):
                    self.registration.write_text(original)
                    self.change_yaml(self.registration, lambda declaration: declaration.update({role: value}))
                    result = self.generate()
                    self.assertNotEqual(result.returncode, 0)
            self.registration.write_text(original)
            self.change_yaml(self.registration, lambda declaration: declaration.update({role: builtin}))
            self.assertEqual(self.generate().returncode, 0)

    def test_duplicate_ports_and_missing_issue_waivers_are_rejected(self):
        original = self.registration.read_text()
        for change in (lambda value: value.update(inputs=["sample", "sample"]),
                       lambda value: value["outputs"].append(dict(value["outputs"][0])),
                       lambda value: value.update(allowed_issues={"sample": ["Missing"]}),
                       lambda value: value.update(assets=["checkpoints", "checkpoints"])):
            with self.subTest(change=change):
                self.registration.write_text(original)
                self.change_yaml(self.registration, change)
                self.assertNotEqual(self.generate().returncode, 0)

    def test_every_candidate_must_produce_downstream_inputs(self):
        node = self.package / "node.yaml"
        node.write_text(node.read_text().replace("cores: [own.hpp]", "cores: [own.hpp, alternative.hpp]").replace("output:\n", "  second:\n    cores: [second.hpp]\n    section_core_map: {\"0\": second.hpp}\noutput:\n").replace("first.result", "second.result"))
        original = self.registration.read_text()
        (self.package / "registrations/alternative.yaml").write_text(original.replace("own.hpp", "alternative.hpp").replace("example::Own", "example::Alternative").replace("name: result", "name: other"))
        (self.package / "registrations/second.yaml").write_text(original.replace("own.hpp", "second.hpp").replace("example::Own", "example::Second").replace("stage: first", "stage: second").replace("inputs: [sample]", "inputs: [result]").replace("time_from: sample", "time_from: result"))
        for stem in ("alternative", "second"):
            (self.headers / (stem + ".hpp")).write_text("namespace example { struct Placeholder {}; }\n")
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("result", result.stderr)

    def test_declaration_order_and_standard_ros_include_are_preserved(self):
        self.change_yaml(self.package / "node.yaml", lambda value: value.update(inputs={"z": {"type": "std_msgs/Float64", "topic": "/z"}, "a": {"type": "std_msgs/Float64", "topic": "/a"}}))
        self.change_yaml(self.package / "node.yaml", lambda value: value["output"].update(header={}))
        self.change_yaml(self.registration, lambda value: value.update(inputs=["z", "a"], outputs=[{"name": "result", "type": "std_msgs/Float64"}]))
        result = self.generate()
        self.assertEqual(result.returncode, 0, result.stderr)
        manifest = json.loads((self.package / "manifest.json").read_text())
        self.assertEqual(manifest["input_order"], ["z", "a"])
        generated = (self.package / "generated.cpp").read_text()
        self.assertIn("#include <std_msgs/Float64.h>", generated)
        self.assertLess(generated.index('InputSpec input_0{"z"'), generated.index('InputSpec input_1{"a"'))

    def test_duplicate_key_cannot_silently_replace_core_identity(self):
        self.registration.write_text(self.registration.read_text() + "name: another.hpp\n")
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("duplicate", result.stderr)

    def test_optional_required_inputs_cannot_weaken_native_contract(self):
        self.registration.write_text(self.registration.read_text() + "required_inputs: []\n")
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("required_inputs", result.stderr)

    def test_type_expression_is_rejected_before_codegen(self):
        self.registration.write_text(self.registration.read_text().replace("type: int", "type: 'int); injected(); ('"))
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("type", result.stderr)

    def test_next_stage_cannot_reuse_unreturned_original_input(self):
        node = self.package / "node.yaml"
        node.write_text(node.read_text().replace("output:\n", "  second:\n    cores: [second.hpp]\n    section_core_map: {\"0\": second.hpp}\noutput:\n").replace("first.result", "second.result"))
        (self.package / "registrations" / "second.yaml").write_text(self.registration.read_text().replace("own.hpp", "second.hpp").replace("example::Own", "example::Second").replace("stage: first", "stage: second"))
        (self.headers / "second.hpp").write_text("namespace example { struct Second { int process(int x) { return x; } }; }\n")
        result = self.generate()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("sample", result.stderr)

    def check_extension(self, source):
        self.assertEqual(self.generate().returncode, 0)
        (self.headers / "own.hpp").write_text(source)
        unit = self.package / "check.cpp"
        extensions = json.loads((self.package / "manifest.json").read_text())["extensions"]
        unit.write_text("\n".join('#include "' + item["header"] + '"' for item in extensions) + "\n")
        database = self.package / "compile_commands.json"
        database.write_text(json.dumps([{"directory": str(self.package), "file": str(unit),
                                        "arguments": ["c++", "-std=c++14", "-I" + str(self.package / "include"), "-I" + str(SCRIPTS.parent / "include"), "-c", str(unit)]}]))
        return subprocess.run([sys.executable, str(SCRIPTS / "check_extensions.py"),
                               "--manifest", str(self.package / "manifest.json"), "--compile-commands", str(database),
                               "--translation-unit", str(unit), "--clang", shutil.which("clang++") or "clang++"],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_transitive_alias_to_other_core_is_rejected(self):
        (self.headers / "other.hpp").write_text("#pragma once\nnamespace example { struct Other { int process(int); }; using Alias = Other; }\n")
        result = self.check_extension('#include "other.hpp"\nnamespace example { struct Own { Alias* other; int process(int x) { return other->process(x); } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Other", result.stderr)

    def test_transitive_helper_cannot_hide_pipeline_access(self):
        (self.headers.parent / "helper.hpp").write_text("namespace node_pipeline { namespace detail { struct Engine { void run(); }; }}\nnamespace example { inline void helper(node_pipeline::detail::Engine& e) { e.run(); } }\n")
        result = self.check_extension('#include "../helper.hpp"\nnamespace example { struct Own { int process(int x) { return x; } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Engine", result.stderr)

    def test_native_calculation_and_declared_bank_are_allowed(self):
        node = self.package / "node.yaml"
        node.write_text(node.read_text() + "banks:\n  state: {type: example::State, header: example/cores/state.hpp}\n")
        (self.headers / "state.hpp").write_text("#pragma once\nnamespace example { struct State { int value = 1; }; }\n")
        result = self.check_extension('#include "state.hpp"\nnamespace example { struct Own { State* state; int process(int x) { return x + state->value; } }; }\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_unauthorized_ros_subscription_is_rejected(self):
        result = self.check_extension('namespace ros { struct NodeHandle { void subscribe(const char*); }; }\nnamespace example { struct Own { ros::NodeHandle node; int process(int x) { node.subscribe("/input"); return x; } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("subscribe", result.stderr)

    def test_global_main_outside_package_namespace_is_rejected(self):
        result = self.check_extension('namespace example { struct Own { int process(int x) { return x; } }; }\nint main() { return 0; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("main entry point", result.stderr)

    def test_raw_literal_visualizer_publisher_is_rejected(self):
        result = self.check_extension('namespace ros { struct Publisher { void publish(int); }; struct NodeHandle { Publisher advertise(const char*, int); }; }\nnamespace example { struct Own { ros::NodeHandle node; int process(int x) { auto publisher = node.advertise("/visualizer/result", 1); publisher.publish(x); return x; } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("raw ROS API", result.stderr)

    def test_sdk_visualizer_publisher_is_allowed(self):
        result = self.check_extension('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { explicit Own(node_pipeline::Visualizer visualizer) : publisher(visualizer.advertise<int>("/visualizer/result")) {} node_pipeline::VisualizerPublisher<int> publisher; int process(int x) { publisher.publish(x); return x; } }; }\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_publisher_without_visualizer_binding_is_rejected(self):
        result = self.check_extension('namespace ros { struct Publisher { void publish(int); }; }\nnamespace example { struct Own { ros::Publisher publisher; int process(int x) { publisher.publish(x); return x; } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("publish", result.stderr)

    def test_policy_can_read_public_profile_health_inputs(self):
        policies = self.headers.parent / "policies"
        policies.mkdir()
        (policies / "own_policy.hpp").write_text('#include <node_pipeline/sdk.hpp>\nnamespace example { namespace policies { struct OwnPolicy { bool apply(node_pipeline::InputView&, const node_pipeline::ProfileHealth& health) const { for (const auto& input : health.inputs) { if (!input.issues.empty()) return true; } return false; } }; }}\n')
        self.registration.write_text(self.registration.read_text() + 'policy: policies/own_policy.hpp\n')
        result = self.check_extension('namespace example { struct Own { int process(int x) { return x; } }; }\n')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_other_package_core_cannot_escape_owned_source_scope(self):
        foreign = Path(self.directory.name) / "foreign" / "include" / "foreign" / "cores"
        foreign.mkdir(parents=True)
        other = foreign / "other.hpp"
        other.write_text("namespace foreign { struct Other { int process(int); }; }\n")
        result = self.check_extension('#include "' + str(other) + '"\nnamespace example { struct Own { foreign::Other* other; int process(int x) { return other->process(x); } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("foreign::Other", result.stderr)

    def test_visualizer_literal_inside_dynamic_topic_does_not_authorize_publish(self):
        for expression in ('remap("/visualizer/result")', 'flag ? "/visualizer/result" : other'):
            with self.subTest(expression=expression):
                result = self.check_extension('namespace ros { struct Publisher { void publish(int); }; struct NodeHandle { Publisher advertise(const char*, int); }; }\nconst char* remap(const char*); extern bool flag; extern const char* other;\nnamespace example { struct Own { ros::NodeHandle node; int process(int x) { auto publisher = node.advertise(' + expression + ', 1); publisher.publish(x); return x; } }; }\n')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("raw ROS API", result.stderr)

    def test_raw_std_string_visualizer_publisher_is_rejected(self):
        result = self.check_extension('#include <string>\nnamespace ros { struct Publisher { void publish(int); }; struct NodeHandle { Publisher advertise(const std::string&, int); }; }\nnamespace example { struct Own { ros::NodeHandle node; int process(int x) { auto publisher = node.advertise(std::string("/visualizer/result"), 1); publisher.publish(x); return x; } }; }\n')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("raw ROS API", result.stderr)

    def test_ros_member_pointer_and_using_alias_cannot_bypass_call_checks(self):
        for body in ('auto fn = &ros::NodeHandle::subscribe; (node.*fn)("/input");',
                     'using Subscribe = decltype(&ros::NodeHandle::subscribe); Subscribe fn = &ros::NodeHandle::subscribe; (node.*fn)("/input");'):
            with self.subTest(body=body):
                result = self.check_extension('namespace ros { struct NodeHandle { void subscribe(const char*); }; }\nnamespace example { struct Own { ros::NodeHandle node; int process(int x) { ' + body + ' return x; } }; }\n')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("subscribe", result.stderr)

    def test_numeric_assets_preserve_checks_for_nonliteral_initializers(self):
        rows = "{1.25f, 2.5f, 3.75f},\n" * 4000
        pure = self.check_extension('static const float points[][3] = {' + rows + '};\nnamespace example { struct Own { int process(int x) { return x; } }; }\n')
        self.assertEqual(pure.returncode, 0, pure.stderr)
        unsafe = self.check_extension('namespace ros { float init(); }\nstatic const float points[][3] = {' + rows + '{ros::init(), 0.0f, 0.0f}};\nnamespace example { struct Own { int process(int x) { return x; } }; }\n')
        self.assertNotEqual(unsafe.returncode, 0)
        self.assertIn("ros::init", unsafe.stderr)


    def test_split_numeric_assets_do_not_materialize_literal_cursors(self):
        for name in ("points_0.inc", "points_1.inc"):
            (self.headers / name).write_text("{-1.25f, +2.5f, 3.75f},\n" * 3000)
        result = self.check_extension('static const float points[][3] = {\n#include "points_0.inc"\n#include "points_1.inc"\n};\nnamespace example { struct Own { int process(int x) { return x; } }; }\n')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertLess(int(result.stdout.partition("(")[2].split()[0]), 100)

    def test_split_numeric_assets_preserve_forbidden_calls_and_references(self):
        (self.headers / "points_0.inc").write_text("{1.25f, 2.5f, 3.75f},\n" * 3000)
        for expression in ("ros::init()", "(&ros::init != nullptr)"):
            with self.subTest(expression=expression):
                (self.headers / "points_1.inc").write_text("{1.25f, 2.5f, 3.75f},\n" * 3000 + "{" + expression + ", 0.0f, 0.0f},\n")
                result = self.check_extension('namespace ros { float init(); }\nstatic const float points[][3] = {\n#include "points_0.inc"\n#include "points_1.inc"\n};\nnamespace example { struct Own { int process(int x) { return x; } }; }\n')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("ros::init", result.stderr)


if __name__ == "__main__":
    unittest.main()
