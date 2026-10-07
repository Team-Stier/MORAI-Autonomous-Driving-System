#!/usr/bin/env python3
"""Compiled AST regression checks for extension capabilities."""
import json
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


PIPELINE = Path(__file__).resolve().parents[2]


class ExtensionContractsTest(unittest.TestCase):
    def test_sdk_numeric_and_asset_contract(self):
        spec = importlib.util.spec_from_file_location("generate_numeric_traits", PIPELINE / "scripts/generate_numeric_traits.py")
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        manifest = {"package_dir": str(PIPELINE), "inputs": {
            "text": {"type": "std_msgs::String"}, "grid": {"type": "nav_msgs::OccupancyGrid"}}}
        with tempfile.TemporaryDirectory() as directory:
            metadata = Path(directory) / "numeric.hpp"
            metadata.write_text(helper.emit_numeric_traits(manifest))
            includes = [PIPELINE / "include", Path("/opt/ros/noetic/include")]
            includes += [parent / "devel/include" for parent in PIPELINE.parents if (parent / "devel/include").is_dir()]
            includes += [Path(prefix) / "include" for prefix in os.environ.get("CMAKE_PREFIX_PATH", "").split(":") if prefix]
            binary = Path(directory) / "sdk_contract"
            command = [shutil.which("c++") or "c++", "-std=c++14", "-include", str(metadata)]
            command += ["-I" + str(path) for path in includes]
            command += [str(PIPELINE / "test/unit/test_sdk_contract.cpp"), "-o", str(binary)]
            compiled = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            checked = subprocess.run([str(binary)], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.assertEqual(checked.returncode, 0, checked.stderr)

    def check_source(self, source, role="core", header_name="own.hpp", helpers=None):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "example"
            headers = package / "include" / "example"
            headers.mkdir(parents=True)
            header = headers / header_name
            header.parent.mkdir(parents=True, exist_ok=True)
            header.write_text(source)
            for name, body in (helpers or {}).items():
                helper = headers / name
                helper.parent.mkdir(parents=True, exist_ok=True)
                helper.write_text(body)
            unit = package / "check.cpp"
            unit.write_text('#include <example/' + header_name + '>\n')
            manifest = package / "manifest.json"
            core = "example::Own" if role == "core" else "example::Calculation"
            manifest.write_text(json.dumps({"package_dir": str(package), "cores": [{"type": core}],
                "banks": {"owned": {"type": "example::Own"}} if role == "bank" else {}, "extensions": [{"header": "example/" + header_name, "role": role, "core": core}]}))
            database = package / "compile_commands.json"
            database.write_text(json.dumps([{"directory": str(package), "file": str(unit), "arguments":
                ["c++", "-std=c++14", "-I" + str(package / "include"), "-I" + str(PIPELINE / "include"),
                 "-I/opt/ros/noetic/include", "-c", str(unit)]}]))
            return subprocess.run([sys.executable, str(PIPELINE / "scripts/check_extensions.py"),
                "--manifest", str(manifest), "--compile-commands", str(database), "--translation-unit", str(unit),
                "--clang", shutil.which("clang++") or "clang++"], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def reject(self, source, role="core", text=""):
        result = self.check_source(source, role)
        self.assertNotEqual(result.returncode, 0, result.stdout)
        if text:
            self.assertIn(text, result.stderr)

    def test_validator_cannot_strip_input_constness(self):
        self.reject('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { void operator()(const node_pipeline::InputView& v, node_pipeline::ProfileHealth&) const { const_cast<int&>(v.get<int>("sample")) = 99; } }; }', "validator", "const")

    def test_alias_and_address_cast_cannot_mutate_raw_inputs(self):
        for expression in ('(int*)&value', '*reinterpret_cast<int*>(reinterpret_cast<unsigned long>(&value))'):
            self.reject('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { void operator()(const node_pipeline::InputView& v, node_pipeline::ProfileHealth&) const { const auto& value = v.get<int>("sample"); auto writable = ' + expression + '; (void)writable; } }; }', "validator")

    def test_mutable_global_and_extern_state_are_rejected(self):
        for state in ('int shared = 0;', 'extern int shared;', 'static int shared = 0;'):
            self.reject(state + '\nnamespace example { struct Own { int process(int x) { return x + shared++; } }; }', text="mutable")

    def test_const_global_indirection_does_not_hide_mutable_state(self):
        for state in ('static int* const shared = new int(1);',
                      'struct State { mutable int value; }; static const State shared{1};'):
            self.reject(state + '\nnamespace example { struct Own { int process(int x) { return x; } }; }', text="mutable")

    def test_const_global_smart_pointer_cannot_hide_mutable_pointee(self):
        for pointer in ("shared_ptr", "unique_ptr"):
            source = '#include <memory>\nstatic const std::' + pointer + '<int> shared(new int(1));\nnamespace example { struct Own { int process(int x) { return x + ++*shared; } }; }'
            with self.subTest(pointer=pointer):
                self.reject(source, text="mutable")
        for pointer in ("shared_ptr", "unique_ptr"):
            source = '#include <memory>\nstatic const std::' + pointer + '<const int> shared(new const int(1));\nnamespace example { struct Own { int process(int x) { return x + *shared; } }; }'
            result = self.check_source(source)
            self.assertEqual(result.returncode, 0, result.stderr)
        owned = '#include <memory>\nnamespace example { struct Own { std::shared_ptr<int> owned{new int(1)}; int process(int x) { return x + ++*owned; } }; }'
        for role in ("core", "bank"):
            result = self.check_source(owned, role)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_pointer_union_cannot_mutate_readonly_input(self):
        self.reject('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { void operator()(const node_pipeline::InputView& v, node_pipeline::ProfileHealth&) const { union Alias { const int* input; int* writable; }; Alias a; a.input = &v.get<int>("sample"); *a.writable = 99; } }; }', "validator", "alias")

    def test_pointer_byte_copy_cannot_mutate_readonly_input(self):
        self.reject('#include <node_pipeline/sdk.hpp>\n#include <cstring>\nnamespace example { struct Own { void operator()(const node_pipeline::InputView& v, node_pipeline::ProfileHealth&) const { const int* input = &v.get<int>("sample"); int* writable; std::memcpy(&writable, &input, sizeof(writable)); *writable = 99; } }; }', "validator", "alias")

    def test_loader_cannot_open_arbitrary_file(self):
        self.reject('#include <fstream>\nnamespace example { struct Own { int operator()() const { std::ifstream stream("/tmp/private"); return 0; } }; }', "parameter_loader", "file")

    def test_posix_system_declarations_are_rejected(self):
        cases = (("unistd.h", '::execv("/does/not/run", nullptr);'),
                 ("unistd.h", 'int ends[2]; ::pipe(ends); ::pipe2(ends, 0); ::dup(0);'),
                 ("unistd.h", 'auto launch = &::execv; (void)launch;'),
                 ("fcntl.h", '::fcntl(0, F_GETFD);'), ("dirent.h", '::opendir("/tmp");'),
                 ("sys/socket.h", 'int ends[2]; ::socketpair(AF_UNIX, SOCK_STREAM, 0, ends);'),
                 ("sys/uio.h", '::readv(0, nullptr, 0);'), ("sys/wait.h", '::waitpid(0, nullptr, 0);'),
                 ("sys/mman.h", '::munmap(nullptr, 0);'), ("sys/select.h", '::select(0, nullptr, nullptr, nullptr, nullptr);'),
                 ("sys/shm.h", '::shmget(0, 1, 0);'), ("sys/msg.h", '::msgget(0, 0);'),
                 ("dlfcn.h", '::dlopen("private.so", RTLD_LAZY); ::dlsym(nullptr, "process");'),
                 ("stdlib.h", '::mkstemp(nullptr);'))
        for header, expression in cases:
            with self.subTest(header=header, expression=expression):
                self.reject('#include <' + header + '>\nnamespace example { struct Own { int process(int x) { ' + expression + ' return x; } }; }', text="external file/process/communication/clock API")
        self.reject('extern "C" int popen(const char*, const char*);\nnamespace example { struct Own { int process(int x) { popen("true", "r"); return x; } }; }', text="external file/process/communication/clock API")

    def test_experimental_filesystem_access_is_rejected(self):
        self.reject('#include <experimental/filesystem>\nnamespace example { struct Own { int process(int x) { std::experimental::filesystem::exists("/tmp/private"); return x; } }; }', text="direct file access")

    def test_system_memory_conversion_and_math_still_pass(self):
        result = self.check_source('#include <cstdlib>\n#include <cmath>\nnamespace example { struct Own { int process(int x) { auto memory = std::malloc(20); memory = std::realloc(memory, 30); std::free(memory); char* end; auto integer = std::strtol("12", &end, 10); auto real = std::strtod("3.5", &end); auto quotient = std::div(11, 3); return x + integer + quotient.quot + static_cast<int>(std::sqrt(real) + std::abs(-2)); } }; }')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_managed_reference_families_preserve_pointee_constness(self):
        cases = (("#include <memory>", 'std::weak_ptr<T> shared(*new std::shared_ptr<T>(new T(0)))', '*shared.lock()'),
                 ("#include <functional>", 'std::reference_wrapper<T> shared(*new T(0))', 'shared.get()'),
                 ("#include <boost/shared_ptr.hpp>", 'boost::shared_ptr<T> shared(new T(0))', '*shared'),
                 ("#include <boost/shared_ptr.hpp>\n#include <boost/weak_ptr.hpp>", 'boost::weak_ptr<T> shared(*new boost::shared_ptr<T>(new T(0)))', '*shared.lock()'))
        for include, declaration, access in cases:
            with self.subTest(declaration=declaration):
                source = include + '\nstatic const ' + declaration.replace('T', 'int') + ';\nnamespace example { struct Own { int process(int x) { return x + ++(' + access + '); } }; }'
                self.reject(source, text="mutable")
                source = include + '\nstatic const ' + declaration.replace('T', 'const int') + ';\nnamespace example { struct Own { int process(int x) { return x + ' + access + '; } }; }'
                result = self.check_source(source)
                self.assertEqual(result.returncode, 0, result.stderr)
        nested = '#include <memory>\n#include <vector>\nstatic const std::vector<std::weak_ptr<int>> shared{std::weak_ptr<int>(*new std::shared_ptr<int>(new int(0)))};\nnamespace example { struct Own { int process(int x) { return x + ++*shared.front().lock(); } }; }'
        self.reject(nested, text="mutable")

    def test_type_erased_callable_global_is_rejected_but_owned_calculation_passes(self):
        self.reject('#include <functional>\nstatic const std::function<int()> shared = [value=0]() mutable { return ++value; };\nnamespace example { struct Own { int process(int x) { return x + shared(); } }; }', text="mutable")
        pure = 'static int plus(int x) { return x + 1; } static int (*const calculate)(int) = plus; static const auto literal = [](int x) { return x + 2; };\nnamespace example { struct Own { int process(int x) { return calculate(x) + literal(x); } }; }'
        result = self.check_source(pure)
        self.assertEqual(result.returncode, 0, result.stderr)
        owned = '#include <functional>\nnamespace example { struct Own { std::function<int()> calculate = [value=0]() mutable { return ++value; }; int process(int x) { return x + calculate(); } }; }'
        for role in ("core", "bank"):
            result = self.check_source(owned, role)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_stdio_file_operations_are_rejected(self):
        expressions = ('auto file = std::tmpfile(); std::fwrite("x", 1, 1, file);',
                       'char data; std::fread(&data, 1, 1, nullptr);', 'std::fflush(nullptr);',
                       'std::fgetc(nullptr);', 'std::fseek(nullptr, 0, SEEK_SET);',
                       'std::ferror(nullptr);', 'std::printf("x");', '::getline(nullptr, nullptr, nullptr);',
                       'auto write_file = &std::fwrite; (void)write_file;', 'std::remove("/tmp/private");')
        for expression in expressions:
            with self.subTest(expression=expression):
                self.reject('#include <cstdio>\nnamespace example { struct Own { int process(int x) { ' + expression + ' return x; } }; }', text="external file/process/communication/clock API")
        blocked = self.check_source('#include <cstdio>\nnamespace example { struct Own { int process(int x) { auto file = std::tmpfile(); std::fwrite("x", 1, 1, file); return x; } }; }')
        self.assertIn("tmpfile", blocked.stderr)
        self.assertIn("fwrite", blocked.stderr)

    def test_memory_stdio_formatting_and_remove_algorithm_pass(self):
        result = self.check_source('#include <cstdio>\n#include <algorithm>\nnamespace example { struct Own { int process(int x) { char text[20]; std::snprintf(text, sizeof(text), "%d", x); int value = 0; std::sscanf(text, "%d", &value); int values[] = {1, 2, 3}; auto end = std::remove(values, values + 3, value); auto remover = &std::remove<int*, int>; end = remover(values, end, 9); return value + static_cast<int>(end - values); } }; }')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_raw_visualizer_ros_api_is_rejected(self):
        self.reject('namespace ros { struct Publisher { void publish(int); }; struct NodeHandle { Publisher advertise(const char*, int); }; }\nnamespace example { struct Own { ros::NodeHandle node; int process(int x) { auto p = node.advertise("/visualizer/result", 1); p.publish(x); return x; } }; }', text="ROS")

    def test_process_and_clock_entry_points_are_rejected(self):
        for body in ('std::system("true");', 'auto now = std::chrono::steady_clock::now(); (void)now;'):
            self.reject('#include <cstdlib>\n#include <chrono>\nnamespace example { struct Own { int process(int x) { ' + body + ' return x; } }; }')

    def test_ros_clock_access_is_rejected_and_value_calculation_passes(self):
        for expression in ('ros::Time::setNow(ros::Time(42, 0));', 'ros::Time::now();', 'ros::Duration(1.0).sleep();'):
            with self.subTest(expression=expression):
                self.reject('#include <ros/time.h>\nnamespace example { struct Own { int process(int x) { ' + expression + ' return x; } }; }', text="unauthorized ROS call or API reference")
        result = self.check_source('#include <ros/time.h>\nnamespace example { struct Own { int process(int x) { ros::Time stamp(42, 0); stamp.fromSec(x); ros::Duration delta(1.0); delta.fromNSec(2); ros::WallDuration wall(1.0); wall.fromSec(2.0); auto shifted = stamp + delta; auto elapsed = shifted - stamp; return static_cast<int>(shifted.toSec() + elapsed.toNSec() + wall.toSec()); } }; }')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_joined_native_calculation_thread_passes(self):
        result = self.check_source('#include <thread>\nnamespace example { struct Own { int process(int x) { int result = 0; std::thread worker([&] { result = x + 1; }); worker.join(); return result; } }; }')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_detached_thread_is_rejected(self):
        self.reject('#include <thread>\nnamespace example { struct Own { int process(int x) { std::thread worker([] {}); worker.detach(); return x; } }; }', text="detach")

    def test_sdk_private_member_name_is_rejected_even_if_redeclared(self):
        self.reject('namespace node_pipeline { struct Parameters { int values_; }; }\nnamespace example { struct Own { int operator()(const node_pipeline::Parameters& p) const { return p.values_; } }; }', "parameter_loader", "values_")

    def test_policy_only_can_modify_inputs(self):
        self.reject('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { void operator()(node_pipeline::InputView& v, node_pipeline::ProfileHealth&) const { v.modify<int>("sample", [](int& x) { x = 1; }); } }; }', "validator", "modify")

    def test_fake_parameters_and_assets_cannot_be_constructed(self):
        self.reject('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { int operator()() const { node_pipeline::ParameterValue v; node_pipeline::Parameters p(v, {{"private", "/tmp/private"}}); return 0; } }; }', "parameter_loader")

    def test_uninspectable_native_payload_is_a_compile_error(self):
        result = subprocess.run([shutil.which("c++") or "c++", "-std=c++14", "-fsyntax-only",
            "-I" + str(PIPELINE / "include"), "-x", "c++", "-"], text=True, input=
            '#include <node_pipeline/detail/native_traits.hpp>\nstruct Unknown {};\nint main() { node_pipeline::HealthLimits limits; std::vector<node_pipeline::Kind> issues; node_pipeline::detail::MessageTraits<Unknown>::inspect({}, limits, issues); }\n',
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("native payload needs a numeric inspector", result.stderr)

    def test_native_calculation_and_immutable_model_data_pass(self):
        result = self.check_source('static constexpr double model[] = {1, 2, 3};\nnamespace example { struct Own { int process(int x) { return x + static_cast<int>(model[1]); } }; }')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_validator_read_and_domain_issue_addition_pass(self):
        result = self.check_source('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { void operator()(const node_pipeline::InputView& v, node_pipeline::ProfileHealth& h) const { if (v.get<int>("sample") < 0) h.issues("sample").push_back(static_cast<node_pipeline::Kind>(100)); } }; }', "validator")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_registered_policy_can_include_separate_void_strategy(self):
        strategy = '#include <node_pipeline/sdk.hpp>\nnamespace example { namespace policies { struct Upper { void operator()(node_pipeline::InputView& input) const { input.modify<int>("sample", [](int& value) { value = 10; }); } }; }}'
        policy = '#include <example/policies/upper.hpp>\nnamespace example { struct Own { bool apply(node_pipeline::InputView& input, const node_pipeline::ProfileHealth&) const { policies::Upper{}(input); return true; } }; }'
        helpers = {"policies/upper.hpp": strategy}
        result = self.check_source(policy, "policy", "policies/clamp.hpp", helpers)
        self.assertEqual(result.returncode, 0, result.stderr)
        for role in ("validator", "core", "helper"):
            result = self.check_source(policy, role, "policies/clamp.hpp", helpers)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("modify", result.stderr)
        result = self.check_source(policy.replace("policies/upper.hpp", "helpers/upper.hpp"), "policy", "policies/clamp.hpp", {"helpers/upper.hpp": strategy})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("modify", result.stderr)
        coupled = strategy.replace('struct Upper', 'struct Foreign { int process(int value) { return value; } }; struct Upper').replace('input.modify<int>', 'Foreign foreign; (void)foreign; input.modify<int>')
        result = self.check_source(policy, "policy", "policies/clamp.hpp", {"policies/upper.hpp": coupled})
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("other Core", result.stderr)

    def test_extension_cannot_include_private_sdk_header(self):
        self.reject('#include <node_pipeline/detail/sdk_types.hpp>\nnamespace example { struct Own { int process(int value) { return value; } }; }', text="only node_pipeline/sdk.hpp may be included")

    def test_official_builtin_policy_alias_passes(self):
        result = self.check_source('#include <node_pipeline/sdk.hpp>\nnamespace example { namespace policies { using Own = node_pipeline::RemoveInvalid; }}', "policy")
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_existing_issue_helpers_are_allowed_only_in_policy(self):
        source = '#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { bool apply(node_pipeline::InputView& input, const node_pipeline::ProfileHealth& health) const { node_pipeline::Issue<std::string, node_pipeline::Kind> selected; auto rank = [&input](const std::string& name, node_pipeline::Kind) { return input.present(name) ? 0 : -1; }; if (!node_pipeline::highestPriorityIssue(health, rank, selected)) return false; (void)selected.kind; input.remove(selected.input); return true; } }; }'
        result = self.check_source(source, "policy")
        self.assertEqual(result.returncode, 0, result.stderr)
        blocked = self.check_source(source, "core")
        self.assertNotEqual(blocked.returncode, 0)
        self.assertIn("Issue", blocked.stderr)
        self.assertIn("highestPriorityIssue", blocked.stderr)

    def test_loader_readonly_asset_stream_passes(self):
        result = self.check_source('#include <node_pipeline/sdk.hpp>\nnamespace example { struct Own { int operator()(const node_pipeline::Parameters& p) const { auto stream = p.openAsset("declared"); int value = 0; *stream >> value; std::string line; std::getline(*stream, line); return p.get("coefficient", value); } }; }', "parameter_loader")
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
