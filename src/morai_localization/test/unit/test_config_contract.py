#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class KCityLocalizationConfigContractTest(unittest.TestCase):
    def test_kcity_yaw_uses_verified_utm_convergence(self):
        config_path = (
            PACKAGE_ROOT / "config" / "maps" / "molit_2026_kcity.yaml"
        )
        with config_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["crs"], "EPSG:32652")
        self.assertEqual(config["yaw_sign"], 1.0)
        self.assertAlmostEqual(config["yaw_offset_deg"], -1.34715, places=5)


if __name__ == "__main__":
    unittest.main()
