#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class RoutePathConfigContractTest(unittest.TestCase):
    def test_runtime_yaml_provides_one_hundred_metre_local_path(self):
        with (
            PACKAGE_ROOT
            / "config"
            / "routes"
            / "molit_2026_kcity_route_path.yaml"
        ).open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["local_path_mode"], "distance")
        self.assertEqual(config["local_path_length_m"], 100.0)
        self.assertEqual(config["local_path_point_count"], 20)


if __name__ == "__main__":
    unittest.main()
