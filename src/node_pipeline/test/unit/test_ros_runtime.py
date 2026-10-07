#!/usr/bin/env python3
"""Run generated common executables against an isolated ROS master."""
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import time
import unittest
import xmlrpc.client

import yaml


PIPELINE = Path(__file__).resolve().parents[2]
ROOT = next((parent for parent in PIPELINE.parents if (parent / ".git").exists()), PIPELINE.parents[1])


def stop(process):
    if process.poll() is not None:
        return
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, signum)
            process.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:
            pass
        except ProcessLookupError:
            return


def wait_for(predicate, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("timed out waiting for ROS test condition")


class RosRuntimeTest(unittest.TestCase):
    @classmethod
    def fixture(cls, name, message, body, loader=None, latched=False):
        package = cls.directory / name
        include = package / "include" / name
        (include / "cores").mkdir(parents=True, exist_ok=True)
        (package / "registrations").mkdir(exist_ok=True)
        (include / "cores/own.hpp").write_text(body)
        node = {"name": name, "inputs": {} if latched else {"sample": {
            "type": message, "topic": "/runtime/" + name + "/input",
            "limits": {"frame": "map", "stale_after_sec": 2, "receive_timeout_sec": 2}}},
            "section_topic": "/section_annotation",
            "stages": {"only": {"cores": ["own.hpp"], "section_core_map": {"0": "own.hpp"}}},
            "output": {"source": "only.result", "topic": "/runtime/" + name + "/output", "header": {}},
            "execution": {"mode": "latched" if latched else "on_input"}}
        if not latched:
            node["execution"]["trigger_input"] = "sample"
            node["output"]["header"] = {"stamp_from": "sample", "frame_id": "base_link"}
        registration = {"name": "own.hpp", "type": name + "::Own", "stage": "only",
            "inputs": [] if latched else ["sample"], "outputs": [{"name": "result", "type": message}], "allowed_issues": {}}
        if loader:
            (include / "parameters").mkdir(exist_ok=True)
            (include / "parameters/own.hpp").write_text(loader)
            registration.update(parameter_namespace="own", parameter_loader="parameters/own.hpp")
        (package / "node.yaml").write_text(yaml.safe_dump(node))
        (package / "registrations/own.yaml").write_text(yaml.safe_dump(registration))
        generated, manifest = package / "generated.cpp", package / "manifest.json"
        subprocess.run([sys.executable, str(PIPELINE / "scripts/generate_registration.py"), "--package-dir", str(package),
            "--output", str(generated), "--manifest", str(manifest)], check=True)
        includes = [package / "include", PIPELINE / "include"]
        includes += [parent / "devel/include" for parent in PIPELINE.parents if (parent / "devel/include").is_dir()]
        includes += [Path(prefix) / "include" for prefix in os.environ.get("CMAKE_PREFIX_PATH", "").split(":") if prefix]
        includes += [path / "xmlrpcpp" for path in list(includes) if (path / "xmlrpcpp/XmlRpcValue.h").is_file()]
        flags = ["-std=c++14"] + ["-I" + str(path) for path in includes]
        ros_flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "roscpp", "xmlrpcpp"], text=True))
        flags += [flag for flag in ros_flags if flag.startswith("-I")]
        object_file = package / "generated.o"
        command = ["c++"] + flags + ["-c", str(generated), "-o", str(object_file)]
        database = package / "compile_commands.json"
        database.write_text(json.dumps([{"directory": str(package), "file": str(generated), "arguments": command}]))
        print("generator + Clang AST + common main build:", name, flush=True)
        subprocess.run([sys.executable, str(PIPELINE / "scripts/check_extensions.py"), "--manifest", str(manifest),
            "--compile-commands", str(database), "--translation-unit", str(generated)], check=True)
        subprocess.run(command, check=True)
        binary = package / "node"
        subprocess.run(["c++"] + flags + [str(PIPELINE / "src/main.cpp"), str(object_file)] + ros_flags + ["-o", str(binary)], check=True)
        return binary

    @classmethod
    def setUpClass(cls):
        cls.master = None
        cls.directory = ROOT / "build" / ("ros-runtime-fixtures-" + str(os.getpid()))
        cls.directory.mkdir(parents=True, exist_ok=True)
        try:
            cls.path_node = cls.fixture("path_case", "interfaces/Path", '''#pragma once
#include <interfaces/Path.h>
namespace path_case { struct Own { interfaces::Path process(const interfaces::Path& input) {
  interfaces::Path result; result.points = input.points; return result;
} }; }
''')
            cls.string_node = cls.fixture("string_case", "std_msgs/String", '''#pragma once
#include <std_msgs/String.h>
namespace string_case {
struct Settings { std::string suffix; };
struct Own { explicit Own(Settings settings):suffix_(settings.suffix) {}
  std_msgs::String process(const std_msgs::String& input) { std_msgs::String result; result.data = input.data + suffix_; return result; }
  std::string suffix_;
}; }
''', '''#pragma once
#include <node_pipeline/sdk.hpp>
#include <string_case/cores/own.hpp>
namespace string_case { namespace parameters { struct Own {
  Settings operator()(const node_pipeline::Parameters& parameters) const {
    return {parameters.require<std::string>(parameters.get<std::string>("lookup", "suffix"))};
  }
}; }}
''')
            cls.visualizer_node = cls.fixture("visualizer_case", "std_msgs/String", '''#pragma once
#include <node_pipeline/sdk.hpp>
#include <std_msgs/String.h>
namespace visualizer_case { struct Own {
  explicit Own(const node_pipeline::Visualizer& visualizer):publisher_(visualizer.advertise<std_msgs::String>("/visualizer/test")) {}
  std_msgs::String process(const std_msgs::String& input) { publisher_.publish(input); return input; }
  node_pipeline::VisualizerPublisher<std_msgs::String> publisher_;
}; }
''')
            cls.latched_node = cls.fixture("latched_case", "tf2_msgs/TFMessage", '''#pragma once
#include <tf2_msgs/TFMessage.h>
namespace latched_case { struct Own { tf2_msgs::TFMessage process() {
  tf2_msgs::TFMessage message;
  for (const auto& child : {"sensor_a", "sensor_b"}) {
    geometry_msgs::TransformStamped transform; transform.header.frame_id = "base_link";
    transform.child_frame_id = child; transform.transform.rotation.w = 1;
    message.transforms.push_back(transform);
  }
  return message;
} }; }
''', latched=True)
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                port = reservation.getsockname()[1]
            uri = "http://127.0.0.1:" + str(port)
            os.environ.update(ROS_MASTER_URI=uri, ROS_HOSTNAME="127.0.0.1", ROS_HOME=str(cls.directory / "ros-home"))
            os.environ.pop("ROS_IP", None)
            with (cls.directory / "roscore.log").open("w") as log:
                cls.master = subprocess.Popen(["roscore", "-p", str(port)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            def ready():
                try:
                    with xmlrpc.client.ServerProxy(uri) as master:
                        return master.getPid("/runtime_contract_test")[0] == 1
                except OSError:
                    return False
            wait_for(ready, 10)
            import rospy
            cls.rospy = rospy
            rospy.init_node("runtime_contract_observer", anonymous=True, disable_signals=True)
            print("isolated ROS master:", uri, flush=True)
        except BaseException:
            if cls.master:
                stop(cls.master)
            raise

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "rospy"):
            cls.rospy.signal_shutdown("ROS runtime regression finished")
        if cls.master:
            stop(cls.master)

    def setUp(self):
        self.processes, self.handles = [], []

    def tearDown(self):
        for handle in self.handles:
            handle.unregister()
        for process in reversed(self.processes):
            stop(process)

    def start(self, binary, name, parameters=None, remaps=()):
        if parameters is not None:
            self.rospy.set_param("/" + name, parameters)
        logfile = self.directory / (name + ".log")
        with logfile.open("w") as output:
            process = subprocess.Popen([str(binary), "__name:=" + name] + list(remaps),
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process, logfile

    def fails(self, binary, name, diagnostic, parameters=None, remaps=()):
        process, logfile = self.start(binary, name, parameters, remaps)
        try:
            code = process.wait(timeout=6)
        except subprocess.TimeoutExpired:
            self.fail("invalid startup stayed alive: " + name)
        self.assertNotEqual(code, 0)
        self.assertIn(diagnostic, logfile.read_text())

    def exchange(self, binary, name, message, message_type, parameters=None, visualizer=False):
        values, visualized = [], []
        output = "/runtime/" + binary.parent.name + "/output"
        subscriber = self.rospy.Subscriber(output, message_type, values.append, queue_size=1)
        self.handles.append(subscriber)
        visualization = None
        if visualizer:
            visualization = self.rospy.Subscriber("/visualizer/test", message_type, visualized.append, queue_size=1)
            self.handles.append(visualization)
        publisher = self.rospy.Publisher("/runtime/" + binary.parent.name + "/input", message_type, queue_size=1)
        self.handles.append(publisher)
        process, logfile = self.start(binary, name, parameters)
        wait_for(lambda: publisher.get_num_connections() == 1 and subscriber.get_num_connections() == 1 and
                 (visualization is None or visualization.get_num_connections() == 1))
        if hasattr(message, "header"):
            message.header.stamp = self.rospy.Time.now()
        publisher.publish(message)
        wait_for(lambda: bool(values) and (not visualizer or bool(visualized)))
        time.sleep(0.2)
        self.assertIsNone(process.poll(), logfile.read_text())
        self.assertEqual(len(values), 1)
        if visualizer:
            self.assertEqual(len(visualized), 1)
            self.assertEqual(visualized[0], message)
        return values[0]

    def test_outer_header_injection_preserves_nested_headers(self):
        from interfaces.msg import Path as PathMessage, PathPoint
        message = PathMessage()
        message.header.frame_id = "map"
        point = PathPoint(); point.pose.header.frame_id = "map"
        point.pose.header.stamp = self.rospy.Time(12, 34)
        point.pose.header.seq = 77; point.pose.pose.orientation.w = 1
        point.pose.pose.position.x = 3; point.velocity = 5; point.acceleration = 2
        message.points = [point]
        output = self.exchange(self.path_node, "path_good", message, PathMessage)
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.header.frame_id, "base_link")
        self.assertEqual(output.points, message.points)

    def test_headerless_passthrough_and_legitimate_core_parameter(self):
        from std_msgs.msg import String
        output = self.exchange(self.string_node, "string_good", String("payload"), String, {"own": {"suffix": "-scoped"}})
        self.assertEqual(output.data, "payload-scoped")

    def test_reserved_pipeline_config_is_rejected(self):
        self.fails(self.string_node, "reserved_config", "Pipeline config belongs in node.yaml", {"execution_mode": "periodic", "own": {"suffix": "allowed"}})

    def test_loader_cannot_read_sibling_core_namespace(self):
        self.fails(self.string_node, "cross_namespace", "missing Core parameter", {"own": {"lookup": "sibling/secret"}, "sibling": {"secret": "forbidden"}})

    def test_resolved_section_alias_cannot_be_a_general_input(self):
        self.fails(self.path_node, "section_alias", "selection topic cannot be a general input", remaps=("/runtime/path_case/input:=/runtime/alias", "/section_annotation:=/runtime/alias"))

    def test_visualizer_sdk_binding_publishes(self):
        from std_msgs.msg import String
        self.exchange(self.visualizer_node, "visualizer_good", String("visualized"), String, visualizer=True)

    def test_visualizer_remapping_cannot_escape_namespace(self):
        self.fails(self.visualizer_node, "visualizer_bad", "Visualizer topic must resolve under /visualizer/", remaps=("/visualizer/test:=/forbidden/test",))

    def test_static_bundle_reaches_late_subscriber_once(self):
        from tf2_msgs.msg import TFMessage
        prefixes = [Path(path) for path in os.environ.get("CMAKE_PREFIX_PATH", "").split(":") if path]
        prefixes += [ROOT / "devel", ROOT / "install"]
        if PIPELINE.parents[1].name == "candidate":
            prefixes.append(PIPELINE.parents[2] / "validation/candidate_ws/devel")
        executable = "lib/tf_publisher/tf_static_publisher_node"
        actual = next((prefix / executable for prefix in prefixes if (prefix / executable).is_file()), None)
        if actual is not None:
            config = yaml.safe_load((PIPELINE.parent / "tf_publisher/config/tf_publisher.yaml").read_text())
            expected = set(config["static_transforms"]["children"])
            binary, topic, parameters = actual, "/tf_static", config
            print("latched late subscriber: actual TF static node", flush=True)
        else:
            binary, topic, parameters = self.latched_node, "/runtime/latched_case/output", None
            expected = {"sensor_a", "sensor_b"}
            print("latched late subscriber: generated zero-input common runner", flush=True)
        process, logfile = self.start(binary, "static_latched", parameters)
        wait_for(lambda: any(name == topic for name, _ in self.rospy.get_published_topics()))
        time.sleep(0.25)
        messages = []
        self.handles.append(self.rospy.Subscriber(topic, TFMessage, messages.append, queue_size=1))
        wait_for(lambda: bool(messages))
        time.sleep(0.2)
        self.assertIsNone(process.poll(), logfile.read_text())
        self.assertEqual(len(messages), 1)
        self.assertEqual({transform.child_frame_id for transform in messages[0].transforms}, expected)
        self.assertTrue(all(transform.header.frame_id == "base_link" and transform.header.stamp.to_nsec() == 0 for transform in messages[0].transforms))


if __name__ == "__main__":
    unittest.main(verbosity=2)
