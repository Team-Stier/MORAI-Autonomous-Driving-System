#!/usr/bin/env python3

import math
import pathlib
import sys
import unittest


SCRIPT_DIRECTORY = (
    pathlib.Path(__file__).resolve().parents[2] / "scripts" / "analysis"
)
sys.path.insert(0, str(SCRIPT_DIRECTORY))

from analyze_tracking_bag import (  # noqa: E402
    aggregate_metric_rows,
    analyze_samples,
)


def status(
    timestamp,
    speed_mps=10.0,
    curvature_m_inv=0.0,
    brake=0.0,
):
    return {
        "t": timestamp,
        "active": True,
        "state": "ACTIVE",
        "mode": "hybrid",
        "cte": 0.0,
        "heading": 0.0,
        "steering": 0.0,
        "requested_steering": 0.0,
        "pp_steering": 0.0,
        "corrected_pp_steering": 0.0,
        "stanley_steering": 0.0,
        "speed": speed_mps,
        "target_speed": speed_mps,
        "curvature": curvature_m_inv,
        "speed_limiting_curve_distance": -1.0,
        "pp_probability": 0.85,
        "stanley_probability": 0.15,
        "effective_pp_weight": 0.85,
        "effective_stanley_weight": 0.15,
        "heading_suppression_active": False,
        "heading_suppression_weight": 0.0,
        "pp_innovation": 0.1,
        "stanley_innovation": 0.2,
        "accel": 0.0,
        "brake": brake,
        "longitudinal_state": "COAST",
    }


def odometry(timestamp, x, y, yaw=0.0):
    return {"t": timestamp, "x": x, "y": y, "yaw": yaw}


class TrackingBagAnalysisTest(unittest.TestCase):
    def test_centered_straight_trajectory_has_literal_vehicle_clearance(self):
        result = analyze_samples(
            global_path=[(0.0, 0.0), (30.0, 0.0)],
            statuses=[status(0.0), status(0.1)],
            odometry=[
                odometry(0.0, 0.0, 0.0),
                odometry(0.1, 10.0, 0.0),
            ],
            lane_half_width_m=1.5,
            vehicle_width_m=1.892,
            wheelbase_m=3.0,
            front_overhang_m=0.845,
            rear_overhang_m=0.790,
        )

        trajectory = result["trajectory"]
        self.assertAlmostEqual(trajectory["rear_center_rms_m"], 0.0)
        self.assertAlmostEqual(
            trajectory["wheel_outer_offset_max_m"], 0.946
        )
        self.assertAlmostEqual(
            trajectory["wheel_min_lane_clearance_m"], 0.554
        )
        self.assertEqual(trajectory["wheel_line_contact_samples"], 0)
        self.assertAlmostEqual(result["maximum_speed_kph"], 36.0)

    def test_lateral_offset_counts_each_wheel_line_contact_sample(self):
        result = analyze_samples(
            global_path=[(0.0, 0.0), (30.0, 0.0)],
            statuses=[status(0.0), status(0.1)],
            odometry=[
                odometry(0.0, 0.0, 0.6),
                odometry(0.1, 10.0, 0.6),
            ],
            lane_half_width_m=1.5,
            vehicle_width_m=1.892,
            wheelbase_m=3.0,
            front_overhang_m=0.845,
            rear_overhang_m=0.790,
        )

        trajectory = result["trajectory"]
        self.assertAlmostEqual(
            trajectory["wheel_outer_offset_max_m"], 1.546
        )
        self.assertAlmostEqual(
            trajectory["wheel_min_lane_clearance_m"], -0.046
        )
        self.assertEqual(trajectory["wheel_line_contact_samples"], 2)

    def test_straight_brake_count_excludes_curved_braking(self):
        result = analyze_samples(
            global_path=[(0.0, 0.0), (30.0, 0.0)],
            statuses=[
                status(0.0, curvature_m_inv=0.0, brake=0.2),
                status(0.1, curvature_m_inv=0.02, brake=0.3),
            ],
            odometry=[odometry(0.0, 0.0, 0.0)],
            lane_half_width_m=1.5,
            vehicle_width_m=1.892,
            wheelbase_m=3.0,
            front_overhang_m=0.845,
            rear_overhang_m=0.790,
        )

        self.assertEqual(result["straight_samples"], 1)
        self.assertEqual(result["straight_brake_samples"], 1)
        self.assertEqual(result["brake_samples"], 2)

    def test_heading_rotation_moves_front_outer_wheel_as_expected(self):
        result = analyze_samples(
            global_path=[(0.0, 0.0), (30.0, 0.0)],
            statuses=[status(0.0)],
            odometry=[
                odometry(0.0, 0.0, 0.0, yaw=math.pi / 2.0),
            ],
            lane_half_width_m=5.0,
            vehicle_width_m=1.892,
            wheelbase_m=3.0,
            front_overhang_m=0.845,
            rear_overhang_m=0.790,
        )

        self.assertAlmostEqual(
            result["trajectory"]["wheel_outer_offset_max_m"],
            3.0,
        )

    def test_repeated_runs_are_aggregated_with_worst_case_safety(self):
        rows = [
            {
                "label": "candidate_01",
                "wheel_contacts": 0,
                "minimum_wheel_clearance_m": 0.12,
                "cte_rms_m": 0.10,
                "cte_p95_m": 0.20,
                "cte_max_m": 0.40,
                "curved_cte_p95_m": 0.22,
                "steering_rate_rms_degps": 10.0,
                "steering_rate_p95_degps": 20.0,
                "maximum_speed_kph": 58.0,
                "straight_brake_samples": 0,
                "direct_pedal_reversals": 4,
            },
            {
                "label": "candidate_02",
                "wheel_contacts": 1,
                "minimum_wheel_clearance_m": -0.02,
                "cte_rms_m": 0.14,
                "cte_p95_m": 0.24,
                "cte_max_m": 0.50,
                "curved_cte_p95_m": 0.26,
                "steering_rate_rms_degps": 14.0,
                "steering_rate_p95_degps": 28.0,
                "maximum_speed_kph": 59.0,
                "straight_brake_samples": 2,
                "direct_pedal_reversals": 6,
            },
        ]

        aggregate = aggregate_metric_rows(rows)

        self.assertEqual(len(aggregate), 1)
        self.assertEqual(aggregate[0]["group"], "candidate")
        self.assertEqual(aggregate[0]["runs"], 2)
        self.assertEqual(aggregate[0]["safe_runs"], 1)
        self.assertEqual(aggregate[0]["wheel_contacts_total"], 1)
        self.assertAlmostEqual(
            aggregate[0]["minimum_wheel_clearance_worst_m"], -0.02
        )
        self.assertAlmostEqual(
            aggregate[0]["minimum_wheel_clearance_mean_m"], 0.05
        )
        self.assertAlmostEqual(
            aggregate[0]["cte_max_worst_m"], 0.50
        )
        self.assertAlmostEqual(
            aggregate[0]["steering_rate_p95_worst_degps"], 28.0
        )

    def test_worst_events_include_relative_time_and_position(self):
        statuses = [
            status(10.0),
            status(11.0),
        ]
        statuses[0]["cte"] = 0.1
        statuses[1]["cte"] = -0.4
        result = analyze_samples(
            global_path=[(0.0, 0.0), (30.0, 0.0)],
            statuses=statuses,
            odometry=[
                odometry(10.0, 0.0, 0.0),
                odometry(11.0, 10.0, 0.3),
            ],
            lane_half_width_m=1.5,
            vehicle_width_m=1.892,
            wheelbase_m=3.0,
            front_overhang_m=0.845,
            rear_overhang_m=0.790,
        )

        self.assertAlmostEqual(
            result["maximum_cte_event"]["elapsed_sec"], 1.0
        )
        self.assertAlmostEqual(
            result["maximum_cte_event"]["signed_cte_m"], -0.4
        )
        clearance_event = result["trajectory"][
            "minimum_wheel_clearance_event"
        ]
        self.assertAlmostEqual(clearance_event["elapsed_sec"], 1.0)
        self.assertAlmostEqual(clearance_event["x_m"], 10.0)
        self.assertAlmostEqual(clearance_event["y_m"], 0.3)
        self.assertAlmostEqual(
            clearance_event["wheel_clearance_m"], 0.254
        )

    def test_heading_suppression_samples_are_counted(self):
        statuses = [status(0.0), status(0.1)]
        statuses[1]["heading_suppression_active"] = True
        statuses[1]["heading_suppression_weight"] = 0.75

        result = analyze_samples(
            global_path=[(0.0, 0.0), (30.0, 0.0)],
            statuses=statuses,
            odometry=[odometry(0.0, 0.0, 0.0)],
        )

        self.assertEqual(result["heading_suppression_samples"], 1)
        self.assertAlmostEqual(
            result["maximum_heading_suppression_weight"], 0.75
        )


if __name__ == "__main__":
    unittest.main()
