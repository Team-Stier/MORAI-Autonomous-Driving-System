"""계약 검사기가 공식 경로 밖 확장과 Core 디커플링 위반을 거부하는지 검증한다."""
import os
import sys
import tempfile
import textwrap
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
import np_check  # noqa: E402

GOOD_CORE = """
#pragma once
#include <vector>
namespace demo {
class Good {
 public:
  int process() { history_.push_back(1); return static_cast<int>(history_.size()); }
 private:
  std::vector<int> history_;
};
}
"""


def write(root, relative, text):
    path = os.path.join(root, relative)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        stream.write(textwrap.dedent(text))


class ContractCheckTest(unittest.TestCase):
    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory()
        self.src = self.workspace.name
        self.package = os.path.join(self.src, "demo")
        write(self.package, "package.xml", "<package format='2'><name>demo</name><depend>node_pipeline</depend></package>")
        write(self.package, "config/demo/node.yaml", """
            execution: {mode: latched}
            stages:
              - name: compute
                outputs: [{name: value, type: std_msgs/Int32}]
            output: {topic: /demo}
            """)
        write(self.package, "config/demo/selection.yaml", "compute: {section_core_map: {'0': good.hpp}}\n")
        write(self.package, "config/demo/registrations/good.yaml", "stage: compute\ninputs: []\n")
        write(self.package, "include/demo/cores/good.hpp", GOOD_CORE)

    def tearDown(self):
        self.workspace.cleanup()

    def errors(self):
        return "\n".join(np_check.check(self.package, "demo", self.src, [os.path.join(self.package, "include")]))

    def test_accepts_official_extension_files(self):
        self.assertEqual("", self.errors())

    def test_excludes_external_library_internals(self):
        write(self.package, "include/demo/libraries/vendor.hpp", """
            #pragma once
            #include <fstream>
            namespace third_party {
            inline int read() {
              static int calls = 0;
              std::ifstream file("/tmp/x");
              return ++calls;
            }
            }
            """)
        write(self.package, "src/libraries/vendor.cpp", """
            #include <demo/libraries/vendor.hpp>
            namespace third_party { int counter = 0; }
            """)
        write(self.package, "include/demo/cores/good.hpp", GOOD_CORE.replace(
            "#include <vector>", "#include <vector>\n#include <demo/libraries/vendor.hpp>").replace(
            "return static_cast<int>(history_.size());", "return third_party::read();"))
        self.assertEqual("", self.errors())

    def test_rejects_sources_and_configuration_outside_official_paths(self):
        write(self.package, "src/ros_node.cpp", "int main() { return 0; }\n")
        write(self.package, "config/demo.yaml", "execution_mode: periodic\n")
        errors = self.errors()
        self.assertIn("src/ros_node.cpp: C++ source outside the official extension paths", errors)
        self.assertIn("config/demo.yaml: unexpected configuration file", errors)

    def test_rejects_side_effects_in_core(self):
        write(self.package, "include/demo/cores/good.hpp", """
            #pragma once
            #include <fstream>
            namespace demo {
            int counter = 0;
            class Good {
             public:
              int process() { static int calls = 0; std::ifstream file("/tmp/x"); return ++calls + counter; }
            };
            }
            """)
        errors = self.errors()
        self.assertIn("#include <fstream> is not allowed", errors)
        self.assertIn("global mutable state 'counter'", errors)
        self.assertIn("static mutable variable 'calls'", errors)
        self.assertIn("'std::ifstream' is not allowed", errors)

    def test_rejects_references_to_other_cores(self):
        write(self.package, "include/demo/cores/other.hpp", """
            #pragma once
            namespace demo { struct Other { int process() { return 1; } }; }
            """)
        write(self.package, "include/demo/cores/good.hpp", """
            #pragma once
            #include <demo/cores/other.hpp>
            namespace demo { class Good { public: int process() { return other_.process(); } Other other_; }; }
            """)
        errors = self.errors()
        self.assertIn("#include <demo/cores/other.hpp> is not allowed for a core file", errors)
        self.assertIn("'demo::Other' belongs to another Core", errors)

    def test_rejects_ros_and_pipeline_internal_includes(self):
        path = os.path.join(self.package, "include/demo/cores/good.hpp")
        write(self.package, "include/demo/cores/good.hpp", """
            #include <ros/ros.h>
            #include <node_pipeline/detail/pipeline.hpp>
            #include <node_pipeline/extension.hpp>
            #include <localization/cores/full_ekf.hpp>
            """)
        errors = "\n".join(np_check.include_errors(path, "core", "good", "", "demo", {"node_pipeline"}, {"localization"}))
        self.assertIn("#include <ros/ros.h> is ROS communication", errors)
        self.assertIn("only node_pipeline/extension.hpp is public", errors)
        self.assertNotIn("<node_pipeline/extension.hpp>", errors)
        self.assertIn("couples execution packages", errors)

    def test_package_xml_declares_external_include_roots(self):
        write(self.package, "package.xml", """
            <package format='2'><name>demo</name><depend>node_pipeline</depend><depend>libpcl-all-dev</depend>
              <export><node_pipeline include_roots="pcl flann"/></export></package>
            """)
        path = os.path.join(self.package, "include/demo/cores/good.hpp")
        write(self.package, "include/demo/cores/good.hpp", """
            #include <pcl/point_types.h>
            #include <flann/flann.hpp>
            #include <onnxruntime_cxx_api.h>
            """)
        roots = np_check.declared_include_roots(self.package)
        self.assertEqual({"pcl", "flann"}, roots)
        errors = "\n".join(np_check.include_errors(
            path, "core", "good", "", "demo", np_check.package_dependencies(self.package), set(), roots))
        self.assertNotIn("<pcl/point_types.h>", errors)
        self.assertNotIn("<flann/flann.hpp>", errors)
        self.assertIn("#include <onnxruntime_cxx_api.h> is not a dependency declared in package.xml", errors)

    def test_allows_ros_logging_macros_only(self):
        fake_ros = os.path.join(self.src, "fake_ros")
        write(fake_ros, "ros/console.h", """
            #pragma once
            namespace ros {
            namespace console { struct LogLocation { bool initialized; }; inline void print(LogLocation*, const char*) {} }
            struct Time { static Time now() { return Time(); } };
            }
            #define ROS_ERROR_THROTTLE(period, text) do { \\
              static ::ros::console::LogLocation __rosconsole_define_location__loc = {false}; \\
              static double __ros_log_throttle_last_hit__ = 0.0; \\
              ::ros::Time __ros_log_throttle_now__ = ::ros::Time::now(); \\
              static_cast<void>(__ros_log_throttle_now__); static_cast<void>(period); \\
              __ros_log_throttle_last_hit__ += 1.0; \\
              ::ros::console::print(&__rosconsole_define_location__loc, text); } while (false)
            """)
        write(self.package, "include/demo/cores/good.hpp", """
            #pragma once
            #include <ros/console.h>
            namespace demo {
            class Good {
             public:
              int process() { ROS_ERROR_THROTTLE(1.0, "gpu unavailable"); return 1; }
            };
            }
            """)
        include_dirs = [os.path.join(self.package, "include"), fake_ros]
        self.assertEqual("", "\n".join(np_check.check(self.package, "demo", self.src, include_dirs)))
        write(self.package, "include/demo/cores/good.hpp", """
            #pragma once
            #include <ros/console.h>
            #include <cstdio>
            namespace demo {
            class Good {
             public:
              int process() {
                static ::ros::console::LogLocation __ros_location = {false};
                ::ros::console::print(&__ros_location, "direct");
                ROS_ERROR_THROTTLE(1.0, std::fopen("/tmp/x", "r") ? "a" : "b");
                return 1;
              }
            };
            }
            """)
        errors = "\n".join(np_check.check(self.package, "demo", self.src, include_dirs))
        self.assertIn("static mutable variable '__ros_location'", errors)
        self.assertIn("'ros::console::print' is not allowed", errors)
        self.assertIn("'fopen' is not allowed", errors)


if __name__ == "__main__":
    unittest.main()
