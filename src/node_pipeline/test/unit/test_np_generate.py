"""등록된 stamp 인자와 구독 입력의 생성 계약을 검증한다."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "scripts"))
import np_generate  # noqa: E402


class StampInputTest(unittest.TestCase):
    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory()
        self.package = Path(self.workspace.name) / "demo"
        self.node = {
            "execution": {"mode": "periodic", "frequency_hz": 50, "reference_input": "reading"},
            "inputs": {"reading": {"type": "sensor_msgs/Imu", "topic": "/reading",
                                   "stale_after_sec": 1, "receive_timeout_sec": 1}},
            "stages": [{"name": "compute", "outputs": [{"name": "result", "type": "nav_msgs/Odometry"}]}],
            "output": {"topic": "/result", "frame_id": "map"},
        }
        self.registration = {"stage": "compute", "inputs": ["reading", "stamp"]}
        self.write("include/demo/cores/good.hpp", "#pragma once\n")

    def tearDown(self):
        self.workspace.cleanup()

    def write(self, relative, text):
        path = self.package / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")

    def load(self):
        self.write("config/demo/node.yaml", yaml.safe_dump(self.node))
        self.write("config/demo/registrations/good.yaml", yaml.safe_dump(self.registration))
        return np_generate.load_node(str(self.package), "demo", "demo")

    def test_accepts_stamp_at_each_argument_position(self):
        for inputs, expected in [(["stamp", "reading"], [-1, 0]),
                                 (["reading", "stamp"], [0, -1]), (["stamp"], [-1])]:
            with self.subTest(inputs=inputs):
                self.registration["inputs"] = inputs
                model = self.load()
                self.assertEqual(expected, model.stages[0].cores[0].args)
                np_generate.generate(model)

    def test_accepts_stamp_in_latched_nodes_without_subscriptions(self):
        self.node["execution"] = {"mode": "latched"}
        self.node["inputs"] = {}
        self.registration["inputs"] = ["stamp"]
        model = self.load()
        self.assertEqual([-1], model.stages[0].cores[0].args)
        np_generate.generate(model)

    def test_accepts_stamp_in_a_later_stage(self):
        self.node["stages"].insert(0, {"name": "prepare", "outputs": [{"name": "reading", "from": "reading"}]})
        model = self.load()
        self.assertEqual([0, -1], model.stages[1].cores[0].args)
        np_generate.generate(model)

    def test_rejects_duplicate_stamp_and_stamp_health_issues(self):
        self.registration["inputs"] = ["stamp", "stamp"]
        with self.assertRaises(np_generate.ConfigError):
            self.load()
        self.registration["inputs"] = ["reading", "stamp"]
        self.registration["allowed_issues"] = {"stamp": ["Stale"]}
        with self.assertRaises(np_generate.ConfigError):
            self.load()

    def test_reserves_stamp_for_subscribed_and_stage_output_names(self):
        self.node["inputs"]["stamp"] = self.node["inputs"].pop("reading")
        self.node["execution"]["reference_input"] = "stamp"
        with self.assertRaisesRegex(np_generate.ConfigError, "reserved"):
            self.load()
        self.node["inputs"]["reading"] = self.node["inputs"].pop("stamp")
        self.node["execution"]["reference_input"] = "reading"
        self.node["stages"][0]["outputs"][0]["name"] = "stamp"
        with self.assertRaisesRegex(np_generate.ConfigError, "reserved"):
            self.load()


if __name__ == "__main__":
    unittest.main()
