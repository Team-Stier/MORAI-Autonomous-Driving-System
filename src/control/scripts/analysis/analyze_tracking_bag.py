#!/usr/bin/env python3

import argparse
import csv
import hashlib
import math
import pathlib
import re
from collections import Counter

import numpy as np
import yaml


def percentile(values, quantile):
    if not values:
        return float("nan")
    return float(np.percentile(np.asarray(values, dtype=float), quantile))


def rms(values):
    if not values:
        return float("nan")
    data = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(data * data)))


def wrap_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def builtin_numbers(value):
    if isinstance(value, dict):
        return {
            key: builtin_numbers(item) for key, item in value.items()
        }
    if isinstance(value, list):
        return [builtin_numbers(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


class PolylineProjector:
    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError("path points must be an Nx2 array")
        if len(self.points) < 2:
            raise ValueError("global path requires at least two points")

    def project(self, point):
        point = np.asarray(point, dtype=float)
        nearest_vertex = int(
            np.argmin(np.sum((self.points - point) ** 2, axis=1))
        )
        candidates = []
        first_segment = max(0, nearest_vertex - 4)
        last_segment = min(len(self.points) - 2, nearest_vertex + 4)
        for index in range(first_segment, last_segment + 1):
            start = self.points[index]
            delta = self.points[index + 1] - start
            length_squared = float(np.dot(delta, delta))
            if length_squared <= 1.0e-12:
                continue
            ratio = float(
                np.dot(point - start, delta) / length_squared
            )
            ratio = min(1.0, max(0.0, ratio))
            projected = start + ratio * delta
            difference = point - projected
            squared_distance = float(np.dot(difference, difference))
            candidates.append(
                (squared_distance, projected, delta, index)
            )
        if not candidates:
            raise ValueError("global path has no valid segment")
        squared_distance, projected, delta, segment_index = min(
            candidates, key=lambda item: item[0]
        )
        tangent = delta / math.hypot(delta[0], delta[1])
        difference = point - projected
        signed_offset = (
            tangent[0] * difference[1]
            - tangent[1] * difference[0]
        )
        return {
            "signed_offset_m": float(signed_offset),
            "path_yaw_rad": math.atan2(tangent[1], tangent[0]),
            "distance_m": math.sqrt(squared_distance),
            "projected_x": float(projected[0]),
            "projected_y": float(projected[1]),
            "segment_index": segment_index,
        }


def transformed_points(x, y, yaw, relative_points):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.array(
        [[cosine, -sine], [sine, cosine]],
        dtype=float,
    )
    return (
        np.asarray(relative_points, dtype=float) @ rotation.T
        + np.array([x, y], dtype=float)
    )


def summarize_status_subset(statuses):
    if not statuses:
        return {"samples": 0}
    cross_track_errors = [item["cte"] for item in statuses]
    heading_errors = [item["heading"] for item in statuses]
    steering_angles = [item["steering"] for item in statuses]
    return {
        "samples": len(statuses),
        "cte_rms_m": rms(cross_track_errors),
        "cte_p95_m": percentile(
            [abs(value) for value in cross_track_errors], 95
        ),
        "cte_max_m": max(abs(value) for value in cross_track_errors),
        "heading_rms_deg": math.degrees(rms(heading_errors)),
        "steering_rms_deg": math.degrees(rms(steering_angles)),
    }


def vehicle_relative_points(
    vehicle_width_m,
    wheelbase_m,
    front_overhang_m,
    rear_overhang_m,
):
    half_width_m = 0.5 * vehicle_width_m
    wheel_points = np.array(
        [
            [0.0, half_width_m],
            [0.0, -half_width_m],
            [wheelbase_m, half_width_m],
            [wheelbase_m, -half_width_m],
        ],
        dtype=float,
    )
    body_points = np.array(
        [
            [-rear_overhang_m, half_width_m],
            [-rear_overhang_m, -half_width_m],
            [wheelbase_m + front_overhang_m, half_width_m],
            [wheelbase_m + front_overhang_m, -half_width_m],
        ],
        dtype=float,
    )
    return wheel_points, body_points


def analyze_samples(
    global_path,
    statuses,
    odometry,
    lane_half_width_m=1.5,
    vehicle_width_m=1.892,
    wheelbase_m=3.0,
    front_overhang_m=0.845,
    rear_overhang_m=0.790,
):
    if not statuses:
        raise ValueError("analysis requires controller status samples")
    if not odometry:
        raise ValueError("analysis requires odometry samples")
    for name, value in (
        ("lane_half_width_m", lane_half_width_m),
        ("vehicle_width_m", vehicle_width_m),
        ("wheelbase_m", wheelbase_m),
        ("front_overhang_m", front_overhang_m),
        ("rear_overhang_m", rear_overhang_m),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(name + " must be finite and positive")

    required_finite_status_fields = (
        "cte",
        "heading",
        "steering",
        "speed",
        "target_speed",
        "curvature",
        "pp_probability",
        "stanley_probability",
        "accel",
        "brake",
    )
    finite_statuses = [
        item
        for item in statuses
        if all(
            math.isfinite(float(item[field]))
            for field in required_finite_status_fields
        )
    ]
    active_statuses = [
        item
        for item in finite_statuses
        if item["active"] and item["state"] == "ACTIVE"
    ]
    if not active_statuses:
        raise ValueError("analysis requires finite ACTIVE status samples")

    projector = PolylineProjector(global_path)
    wheel_relative, body_relative = vehicle_relative_points(
        vehicle_width_m,
        wheelbase_m,
        front_overhang_m,
        rear_overhang_m,
    )

    footprint_records = []
    for item in odometry:
        if not all(
            math.isfinite(float(item[field]))
            for field in ("t", "x", "y", "yaw")
        ):
            continue
        rear_projection = projector.project((item["x"], item["y"]))
        path_heading_error = wrap_angle(
            item["yaw"] - rear_projection["path_yaw_rad"]
        )

        wheel_points = transformed_points(
            item["x"],
            item["y"],
            item["yaw"],
            wheel_relative,
        )
        wheel_offsets = [
            abs(
                projector.project(point)["signed_offset_m"]
            )
            for point in wheel_points
        ]
        body_points = transformed_points(
            item["x"],
            item["y"],
            item["yaw"],
            body_relative,
        )
        body_offsets = [
            abs(
                projector.project(point)["signed_offset_m"]
            )
            for point in body_points
        ]
        wheel_outer_offset = max(wheel_offsets)
        body_outer_offset = max(body_offsets)
        footprint_records.append(
            {
                "t": item["t"],
                "x": item["x"],
                "y": item["y"],
                "yaw": item["yaw"],
                "rear_offset": rear_projection["signed_offset_m"],
                "heading_error": path_heading_error,
                "wheel_outer_offset": wheel_outer_offset,
                "wheel_clearance": (
                    lane_half_width_m - wheel_outer_offset
                ),
                "body_outer_offset": body_outer_offset,
                "body_clearance": (
                    lane_half_width_m - body_outer_offset
                ),
            }
        )
    if not footprint_records:
        raise ValueError("analysis requires finite odometry samples")

    steering_rates = []
    timestamped_steering_rates = []
    for previous, current in zip(
        active_statuses, active_statuses[1:]
    ):
        elapsed_sec = current["t"] - previous["t"]
        if 0.005 <= elapsed_sec <= 0.2:
            rate = ((current["steering"] - previous["steering"])
                    / elapsed_sec)
            steering_rates.append(rate)
            timestamped_steering_rates.append((current, rate, elapsed_sec))

    high_speed_straight_rates = [
        (item, rate, elapsed)
        for item, rate, elapsed in timestamped_steering_rates
        if item["speed"] >= (50.0 / 3.6)
        and abs(item["curvature"]) < 0.0015
        and item.get("speed_limiting_curve_distance", -1.0) < 0.0
    ]
    significant_rate_threshold = math.radians(1.0)
    previous_rate_sign = 0
    high_speed_straight_rate_reversals = 0
    for _, rate, _ in high_speed_straight_rates:
        sign = (1 if rate > significant_rate_threshold else
                -1 if rate < -significant_rate_threshold else 0)
        if sign and previous_rate_sign and sign != previous_rate_sign:
            high_speed_straight_rate_reversals += 1
        if sign:
            previous_rate_sign = sign
    high_speed_straight_duration = sum(
        elapsed for _, _, elapsed in high_speed_straight_rates
    )

    # Isolate the mild bends where a quantized reference curvature can make
    # steering repeatedly engage and release. Exclude geometrically straight
    # samples so the existing high-speed-straight metric remains independent.
    low_curvature_rates = [
        (item, rate, elapsed)
        for item, rate, elapsed in timestamped_steering_rates
        if item["speed"] >= (30.0 / 3.6)
        and 0.0015 <= abs(item["curvature"]) < 0.01
    ]
    low_curvature_rate_reversals = 0
    previous_rate_sign = 0
    previous_rate_time = None
    for item, rate, _ in low_curvature_rates:
        if (
            previous_rate_time is not None
            and item["t"] - previous_rate_time > 0.2
        ):
            previous_rate_sign = 0
        sign = (1 if rate > math.radians(0.5) else
                -1 if rate < -math.radians(0.5) else 0)
        if sign and previous_rate_sign and sign != previous_rate_sign:
            low_curvature_rate_reversals += 1
        if sign:
            previous_rate_sign = sign
        previous_rate_time = item["t"]
    low_curvature_duration = sum(
        elapsed for _, _, elapsed in low_curvature_rates
    )

    previous_nonzero_effort = 0
    direct_pedal_reversals = 0
    for item in active_statuses:
        effort = (
            1
            if item["accel"] > 1.0e-4
            else -1
            if item["brake"] > 1.0e-4
            else 0
        )
        if (
            effort
            and previous_nonzero_effort
            and effort != previous_nonzero_effort
        ):
            direct_pedal_reversals += 1
        if effort:
            previous_nonzero_effort = effort

    straight_statuses = [
        item
        for item in active_statuses
        if abs(item["curvature"]) < 0.0015
        and item.get("speed_limiting_curve_distance", -1.0) < 0.0
    ]
    curved_statuses = [
        item
        for item in active_statuses
        if abs(item["curvature"]) >= 0.01 and item["speed"] > 1.0
    ]
    high_speed_statuses = [
        item
        for item in active_statuses
        if item["speed"] >= (50.0 / 3.6)
    ]
    high_speed_straight_statuses = [
        item for item in active_statuses
        if item["speed"] >= (50.0 / 3.6)
        and abs(item["curvature"]) < 0.0015
        and item.get("speed_limiting_curve_distance", -1.0) < 0.0
    ]
    low_curvature_statuses = [
        item for item in active_statuses
        if item["speed"] >= (30.0 / 3.6)
        and 0.0015 <= abs(item["curvature"]) < 0.01
    ]
    one_second_steering_ranges_deg = []
    steering_window = []
    for item in high_speed_straight_statuses:
        if (
            steering_window
            and item["t"] - steering_window[-1]["t"] > 0.2
        ):
            steering_window = []
        steering_window.append(item)
        steering_window = [
            sample for sample in steering_window
            if item["t"] - sample["t"] <= 1.0
        ]
        if len(steering_window) >= 5:
            angles_deg = [
                math.degrees(sample["steering"])
                for sample in steering_window
            ]
            one_second_steering_ranges_deg.append(
                max(angles_deg) - min(angles_deg)
            )

    low_curvature_one_second_ranges_deg = []
    low_curvature_window = []
    for item in low_curvature_statuses:
        if (
            low_curvature_window
            and (
                item["t"] - low_curvature_window[-1]["t"] > 0.2
                or item["curvature"] *
                low_curvature_window[-1]["curvature"] <= 0.0
            )
        ):
            low_curvature_window = []
        low_curvature_window.append(item)
        low_curvature_window = [
            sample for sample in low_curvature_window
            if item["t"] - sample["t"] <= 1.0
        ]
        if len(low_curvature_window) >= 5:
            angles_deg = [
                math.degrees(sample["steering"])
                for sample in low_curvature_window
            ]
            low_curvature_one_second_ranges_deg.append(
                max(angles_deg) - min(angles_deg)
            )

    # Driver-visible chatter is a steering-angle excursion, not every tiny
    # steering-rate sign change. Count consecutive local extrema only when
    # their angle difference is at least 2 degrees.
    large_steering_excursions = 0
    previous_extremum_deg = None
    segment = []
    for item in high_speed_straight_statuses:
        if segment and item["t"] - segment[-1]["t"] > 0.2:
            segment = []
            previous_extremum_deg = None
        segment.append(item)
        if len(segment) < 3:
            continue
        before, point, after = segment[-3:]
        incoming = point["steering"] - before["steering"]
        outgoing = after["steering"] - point["steering"]
        if incoming * outgoing < 0.0:
            extremum_deg = math.degrees(point["steering"])
            if (
                previous_extremum_deg is not None
                and abs(extremum_deg - previous_extremum_deg) >= 2.0
            ):
                large_steering_excursions += 1
            previous_extremum_deg = extremum_deg

    wheel_offsets = [
        item["wheel_outer_offset"] for item in footprint_records
    ]
    wheel_clearances = [
        item["wheel_clearance"] for item in footprint_records
    ]
    body_offsets = [
        item["body_outer_offset"] for item in footprint_records
    ]
    body_clearances = [
        item["body_clearance"] for item in footprint_records
    ]
    rear_offsets = [
        item["rear_offset"] for item in footprint_records
    ]
    path_heading_errors = [
        item["heading_error"] for item in footprint_records
    ]

    state_counts = Counter(
        item.get("longitudinal_state", "")
        for item in active_statuses
    )
    state_counts.pop("", None)
    maximum_cte_status = max(
        active_statuses, key=lambda item: abs(item["cte"])
    )
    minimum_clearance_record = min(
        footprint_records, key=lambda item: item["wheel_clearance"]
    )

    return {
        "duration_sec": (
            statuses[-1]["t"] - statuses[0]["t"]
        ),
        "status_samples": len(statuses),
        "active_samples": len(active_statuses),
        "nonfinite_or_inactive_samples": (
            len(statuses) - len(active_statuses)
        ),
        "states": dict(
            Counter(item["state"] for item in statuses)
        ),
        "modes": dict(
            Counter(item["mode"] for item in statuses)
        ),
        "longitudinal_states": dict(state_counts),
        "maximum_speed_kph": (
            max(item["speed"] for item in active_statuses) * 3.6
        ),
        "maximum_target_speed_kph": (
            max(
                item["target_speed"]
                for item in active_statuses
            )
            * 3.6
        ),
        "maximum_cte_event": {
            "elapsed_sec": (
                maximum_cte_status["t"] - active_statuses[0]["t"]
            ),
            "signed_cte_m": maximum_cte_status["cte"],
            "speed_kph": maximum_cte_status["speed"] * 3.6,
            "target_speed_kph": (
                maximum_cte_status["target_speed"] * 3.6
            ),
            "curvature_m_inv": maximum_cte_status["curvature"],
        },
        "brake_samples": sum(
            item["brake"] > 1.0e-6
            for item in active_statuses
        ),
        "straight_samples": len(straight_statuses),
        "straight_brake_samples": sum(
            item["brake"] > 1.0e-6
            for item in straight_statuses
        ),
        "direct_pedal_reversals": direct_pedal_reversals,
        "heading_suppression_samples": sum(
            bool(item.get("heading_suppression_active", False))
            for item in active_statuses
        ),
        "maximum_heading_suppression_weight": max(
            float(item.get("heading_suppression_weight", 0.0))
            for item in active_statuses
        ),
        "steering_rate_rms_degps": math.degrees(
            rms(steering_rates)
        ),
        "steering_rate_p95_degps": math.degrees(
            percentile(
                [abs(value) for value in steering_rates],
                95,
            )
        ),
        "high_speed_straight_steering_rate_rms_degps": math.degrees(
            rms([rate for _, rate, _ in high_speed_straight_rates])
        ),
        "high_speed_straight_rate_reversals":
            high_speed_straight_rate_reversals,
        "high_speed_straight_rate_reversals_per_sec": (
            high_speed_straight_rate_reversals /
            high_speed_straight_duration
            if high_speed_straight_duration > 0.0 else float("nan")
        ),
        "high_speed_straight_steering_1s_peak_to_peak_p95_deg":
            percentile(one_second_steering_ranges_deg, 95),
        "high_speed_straight_steering_1s_peak_to_peak_max_deg": (
            max(one_second_steering_ranges_deg)
            if one_second_steering_ranges_deg else float("nan")
        ),
        "high_speed_straight_large_2deg_steering_excursions":
            large_steering_excursions,
        "high_speed_straight_cte_filter_delta_rms_m": rms([
            item.get("raw_cte", item["cte"]) - item["cte"]
            for item in high_speed_straight_statuses
        ]),
        "low_curvature_steering_rate_rms_degps": math.degrees(
            rms([rate for _, rate, _ in low_curvature_rates])
        ),
        "low_curvature_rate_reversals_per_sec": (
            low_curvature_rate_reversals / low_curvature_duration
            if low_curvature_duration > 0.0 else float("nan")
        ),
        "low_curvature_steering_1s_peak_to_peak_p95_deg": percentile(
            low_curvature_one_second_ranges_deg, 95
        ),
        "speed_limit_sources": {
            "curve_samples": sum(
                item.get("raw_target_speed", item["target_speed"]) <
                item.get("configured_target_speed", item["target_speed"]) - 0.02
                for item in active_statuses
            ),
            "lane_clearance_samples": sum(
                item.get("lane_urgency", 0.0) > 1.0e-6
                for item in active_statuses
            ),
            "heading_error_samples": sum(
                item.get("heading_urgency", 0.0) > 1.0e-6
                for item in active_statuses
            ),
        },
        "overall": summarize_status_subset(active_statuses),
        "straight": summarize_status_subset(straight_statuses),
        "curved": summarize_status_subset(curved_statuses),
        "high_speed": summarize_status_subset(
            high_speed_statuses
        ),
        "trajectory": {
            "rear_center_rms_m": rms(rear_offsets),
            "rear_center_p95_m": percentile(
                [abs(value) for value in rear_offsets],
                95,
            ),
            "rear_center_max_m": max(
                abs(value) for value in rear_offsets
            ),
            "path_heading_rms_deg": math.degrees(
                rms(path_heading_errors)
            ),
            "wheel_outer_offset_p95_m": percentile(
                wheel_offsets, 95
            ),
            "wheel_outer_offset_max_m": max(wheel_offsets),
            "wheel_min_lane_clearance_m": min(
                wheel_clearances
            ),
            "minimum_wheel_clearance_event": {
                "elapsed_sec": (
                    minimum_clearance_record["t"]
                    - footprint_records[0]["t"]
                ),
                "x_m": minimum_clearance_record["x"],
                "y_m": minimum_clearance_record["y"],
                "rear_offset_m": minimum_clearance_record[
                    "rear_offset"
                ],
                "heading_error_deg": math.degrees(
                    minimum_clearance_record["heading_error"]
                ),
                "wheel_clearance_m": minimum_clearance_record[
                    "wheel_clearance"
                ],
            },
            "wheel_line_contact_samples": sum(
                value <= 0.0 for value in wheel_clearances
            ),
            "body_corner_offset_p95_m": percentile(
                body_offsets, 95
            ),
            "body_corner_offset_max_m": max(body_offsets),
            "body_min_lane_clearance_m": min(
                body_clearances
            ),
            "body_line_contact_samples": sum(
                value <= 0.0 for value in body_clearances
            ),
            "lane_half_width_assumption_m": lane_half_width_m,
            "vehicle_width_assumption_m": vehicle_width_m,
            "wheelbase_assumption_m": wheelbase_m,
            "odometry_samples": len(footprint_records),
        },
        "_series": {
            "statuses": active_statuses,
            "footprints": footprint_records,
            "global_path": [
                (float(point[0]), float(point[1]))
                for point in global_path
            ],
        },
    }


def load_bag(path):
    import rosbag

    statuses = []
    odometry = []
    global_path = None
    with rosbag.Bag(str(path)) as bag:
        for topic, message, receipt in bag.read_messages(
            topics=[
                "/control/controller_status",
                "/localization/ego",
                "/global_path",
            ]
        ):
            timestamp = receipt.to_sec()
            if topic == "/global_path" and message.poses:
                global_path = [
                    (
                        pose.pose.position.x,
                        pose.pose.position.y,
                    )
                    for pose in message.poses
                ]
            elif topic == "/localization/ego":
                pose = getattr(message.pose, "pose", message.pose)
                odometry.append(
                    {
                        "t": timestamp,
                        "x": float(
                            pose.position.x
                        ),
                        "y": float(
                            pose.position.y
                        ),
                        "yaw": yaw_from_quaternion(
                            pose.orientation
                        ),
                    }
                )
            elif topic == "/control/controller_status":
                statuses.append(
                    {
                        "t": timestamp,
                        "active": bool(message.active),
                        "state": str(message.state),
                        "mode": str(message.lateral_controller),
                        "cte": float(
                            message.cross_track_error_m
                        ),
                        "raw_cte": float(getattr(
                            message,
                            "mpc_raw_cross_track_error_m",
                            message.cross_track_error_m,
                        )),
                        "heading": float(
                            message.heading_error_rad
                        ),
                        "steering": float(
                            message.steering_angle_rad
                        ),
                        "requested_steering": float(
                            message.requested_steering_angle_rad
                        ),
                        "mpc_steering_observation_blend": float(getattr(
                            message,
                            "mpc_yaw_rate_steering_estimate_blend",
                            0.0,
                        )),
                        "mpc_modeled_steering": float(getattr(
                            message,
                            "mpc_modeled_steering_angle_rad",
                            message.steering_angle_rad,
                        )),
                        "pp_steering": float(
                            message.pure_pursuit_steering_angle_rad
                        ),
                        "corrected_pp_steering": float(
                            message
                            .hybrid_corrected_pure_pursuit_steering_angle_rad
                        ),
                        "stanley_steering": float(
                            message.stanley_steering_angle_rad
                        ),
                        "speed": float(
                            message.measured_velocity_x_mps
                        ),
                        "target_speed": float(
                            message.target_speed_mps
                        ),
                        "configured_target_speed": float(
                            message.configured_target_speed_mps
                        ),
                        "raw_target_speed": float(
                            message.raw_target_speed_mps
                        ),
                        "curve_speed_limit": float(
                            message.curvature_speed_limit_mps
                        ),
                        "lane_speed_limit": float(
                            message.lane_clearance_speed_limit_mps
                        ),
                        "heading_speed_limit": float(
                            message.heading_error_speed_limit_mps
                        ),
                        "lane_urgency": float(
                            message.lane_clearance_recovery_urgency
                        ),
                        "heading_urgency": float(
                            message.heading_error_speed_limit_urgency
                        ),
                        "curvature": float(
                            message.preview_curvature_m_inv
                        ),
                        "speed_limiting_curve_distance": float(
                            message.speed_limiting_curve_distance_m
                        ),
                        "pp_probability": float(
                            message
                            .hybrid_pure_pursuit_probability
                        ),
                        "stanley_probability": float(
                            message.hybrid_stanley_probability
                        ),
                        "effective_pp_weight": float(
                            message
                            .hybrid_effective_pure_pursuit_weight
                        ),
                        "effective_stanley_weight": float(
                            message
                            .hybrid_effective_stanley_weight
                        ),
                        "recovery_weight": float(
                            getattr(
                                message,
                                "hybrid_cross_track_recovery_weight",
                                0.0,
                            )
                        ),
                        "heading_suppression_active": bool(
                            getattr(
                                message,
                                "hybrid_cross_track_recovery_heading_suppression_active",
                                False,
                            )
                        ),
                        "heading_suppression_weight": float(
                            getattr(
                                message,
                                "hybrid_cross_track_recovery_heading_suppression_weight",
                                0.0,
                            )
                        ),
                        "pp_innovation": float(
                            message.pure_pursuit_innovation_norm
                        ),
                        "stanley_innovation": float(
                            message.stanley_innovation_norm
                        ),
                        "accel": float(message.accel),
                        "brake": float(message.brake),
                        "longitudinal_state": str(
                            message.longitudinal_state
                        ),
                    }
                )
    if global_path is None:
        raise ValueError(str(path) + " has no global path")
    return {
        "global_path": global_path,
        "statuses": statuses,
        "odometry": odometry,
    }


def relative_times(items):
    if not items:
        return []
    first = items[0]["t"]
    return [item["t"] - first for item in items]


def offset_path(points, offset_m):
    path = np.asarray(points, dtype=float)
    differences = np.gradient(path, axis=0)
    norms = np.hypot(differences[:, 0], differences[:, 1])
    norms[norms < 1.0e-9] = 1.0
    normals = np.column_stack(
        (-differences[:, 1] / norms, differences[:, 0] / norms)
    )
    return path + offset_m * normals


def plot_runs(runs, output_directory, lane_half_width_m):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = plt.cm.tab10(np.linspace(0.0, 1.0, len(runs)))

    figure, axis = plt.subplots(figsize=(10, 8))
    first_path = np.asarray(
        runs[0][1]["_series"]["global_path"],
        dtype=float,
    )
    axis.plot(
        first_path[:, 0],
        first_path[:, 1],
        "k--",
        linewidth=1.2,
        label="reference path",
    )
    for sign in (-1.0, 1.0):
        boundary = offset_path(
            first_path,
            sign * lane_half_width_m,
        )
        axis.plot(
            boundary[:, 0],
            boundary[:, 1],
            color="tab:red",
            linewidth=0.9,
            alpha=0.7,
            label=(
                "assumed lane line"
                if sign < 0.0
                else None
            ),
        )
    for (label, result), color in zip(runs, colors):
        footprints = result["_series"]["footprints"]
        axis.plot(
            [item["x"] for item in footprints],
            [item["y"] for item in footprints],
            linewidth=1.2,
            color=color,
            label=label,
        )
    axis.set_title("Reference path and rear-axle trajectories")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.axis("equal")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(
        output_directory / "trajectory_comparison.png",
        dpi=160,
    )
    plt.close(figure)

    figure, axes = plt.subplots(
        3, 1, figsize=(12, 10), sharex=False
    )
    for (label, result), color in zip(runs, colors):
        statuses = result["_series"]["statuses"]
        footprints = result["_series"]["footprints"]
        status_times = relative_times(statuses)
        footprint_times = relative_times(footprints)
        axes[0].plot(
            status_times,
            [item["cte"] for item in statuses],
            color=color,
            linewidth=1.0,
            label=label,
        )
        axes[1].plot(
            footprint_times,
            [item["wheel_clearance"] for item in footprints],
            color=color,
            linewidth=1.0,
            label=label,
        )
        axes[2].plot(
            status_times,
            [item["speed"] * 3.6 for item in statuses],
            color=color,
            linewidth=1.0,
            label=label + " measured",
        )
        axes[2].plot(
            status_times,
            [
                item["target_speed"] * 3.6
                for item in statuses
            ],
            color=color,
            linewidth=0.8,
            linestyle="--",
            alpha=0.7,
            label=label + " target",
        )
    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_ylabel("CTE [m]")
    axes[1].axhline(
        0.0,
        color="tab:red",
        linewidth=1.0,
        linestyle="--",
    )
    axes[1].set_ylabel("wheel clearance [m]")
    axes[2].axhline(
        60.0,
        color="tab:red",
        linewidth=1.0,
        linestyle="--",
    )
    axes[2].set_ylabel("speed [km/h]")
    axes[2].set_xlabel("elapsed time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=7)
    figure.suptitle("Tracking error, wheel clearance, and speed")
    figure.tight_layout()
    figure.savefig(
        output_directory / "tracking_timeseries.png",
        dpi=160,
    )
    plt.close(figure)

    figure, axes = plt.subplots(
        2, 1, figsize=(12, 8), sharex=False
    )
    for (label, result), color in zip(runs, colors):
        statuses = result["_series"]["statuses"]
        times = relative_times(statuses)
        axes[0].plot(
            times,
            [
                math.degrees(item["corrected_pp_steering"])
                for item in statuses
            ],
            color=color,
            linewidth=0.7,
            linestyle=":",
            alpha=0.7,
            label=label + " corrected PP",
        )
        axes[0].plot(
            times,
            [
                math.degrees(item["stanley_steering"])
                for item in statuses
            ],
            color=color,
            linewidth=0.7,
            linestyle="--",
            alpha=0.7,
            label=label + " Stanley",
        )
        steering_deg = [
            math.degrees(item["steering"])
            for item in statuses
        ]
        axes[0].plot(
            times,
            steering_deg,
            color=color,
            linewidth=1.1,
            label=label + " final",
        )
        rate_times = []
        rates_degps = []
        for index in range(1, len(statuses)):
            elapsed_sec = times[index] - times[index - 1]
            if 0.005 <= elapsed_sec <= 0.2:
                rate_times.append(times[index])
                rates_degps.append(
                    (
                        steering_deg[index]
                        - steering_deg[index - 1]
                    )
                    / elapsed_sec
                )
        axes[1].plot(
            rate_times,
            rates_degps,
            color=color,
            linewidth=0.8,
            label=label,
        )
    axes[0].set_ylabel("steering [deg]")
    axes[1].set_ylabel("steering rate [deg/s]")
    axes[1].set_xlabel("elapsed time [s]")
    for axis in axes:
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=7)
    figure.suptitle("Hybrid steering candidates and final command")
    figure.tight_layout()
    figure.savefig(
        output_directory / "steering_timeseries.png",
        dpi=160,
    )
    plt.close(figure)

    figure, axes = plt.subplots(
        3, 1, figsize=(12, 9), sharex=False
    )
    for (label, result), color in zip(runs, colors):
        statuses = result["_series"]["statuses"]
        times = relative_times(statuses)
        axes[0].plot(
            times,
            [item["pp_probability"] for item in statuses],
            color=color,
            linewidth=0.9,
            label=label + " IMM PP",
        )
        axes[1].plot(
            times,
            [item["effective_pp_weight"] for item in statuses],
            color=color,
            linewidth=0.9,
            label=label + " effective PP",
        )
        axes[2].plot(
            times,
            [item.get("recovery_weight", 0.0) for item in statuses],
            color=color,
            linewidth=0.9,
            label=label + " recovery",
        )
        axes[2].plot(
            times,
            [
                item.get("heading_suppression_weight", 0.0)
                for item in statuses
            ],
            color=color,
            linewidth=0.8,
            linestyle="--",
            label=label + " heading suppression",
        )
    axes[0].set_ylabel("IMM PP probability")
    axes[1].set_ylabel("effective PP weight")
    axes[2].set_ylabel("override weights")
    axes[2].set_xlabel("elapsed time [s]")
    for axis in axes:
        axis.set_ylim(-0.05, 1.05)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=7)
    figure.suptitle("Hybrid model and recovery weights")
    figure.tight_layout()
    figure.savefig(
        output_directory / "hybrid_weights_timeseries.png",
        dpi=160,
    )
    plt.close(figure)

    figure, axes = plt.subplots(
        2, 1, figsize=(12, 8), sharex=False
    )
    for (label, result), color in zip(runs, colors):
        statuses = result["_series"]["statuses"]
        times = relative_times(statuses)
        axes[0].plot(
            times,
            [item["accel"] for item in statuses],
            color=color,
            linewidth=0.9,
            label=label + " accel",
        )
        axes[0].plot(
            times,
            [-item["brake"] for item in statuses],
            color=color,
            linewidth=0.9,
            linestyle="--",
            label=label + " -brake",
        )
        state_value = {
            "BRAKE": -1.0,
            "COAST": 0.0,
            "ACCEL": 1.0,
            "HARD_SPEED_BRAKE": -2.0,
        }
        axes[1].plot(
            times,
            [
                state_value.get(
                    item.get("longitudinal_state", ""),
                    float("nan"),
                )
                for item in statuses
            ],
            color=color,
            linewidth=0.8,
            label=label,
        )
    axes[0].set_ylabel("command")
    axes[1].set_ylabel("state [-2..1]")
    axes[1].set_xlabel("elapsed time [s]")
    for axis in axes:
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.grid(True, alpha=0.25)
        axis.legend(loc="best", fontsize=7)
    figure.suptitle("Longitudinal actuator commands and state")
    figure.tight_layout()
    figure.savefig(
        output_directory / "longitudinal_timeseries.png",
        dpi=160,
    )
    plt.close(figure)

    labels = [label for label, _ in runs]
    positions = np.arange(len(runs), dtype=float)
    figure, axes = plt.subplots(2, 2, figsize=(15, 10))

    clearances = [
        result["trajectory"]["wheel_min_lane_clearance_m"]
        for _, result in runs
    ]
    axes[0, 0].bar(positions, clearances, color=colors)
    axes[0, 0].axhline(
        0.0, color="tab:red", linewidth=1.2, linestyle="--"
    )
    axes[0, 0].set_ylabel("minimum wheel clearance [m]")
    axes[0, 0].set_title("Worst wheel-to-line clearance")

    cte_rms_values = [
        result["overall"]["cte_rms_m"] for _, result in runs
    ]
    cte_p95_values = [
        result["overall"]["cte_p95_m"] for _, result in runs
    ]
    bar_width = 0.38
    axes[0, 1].bar(
        positions - 0.5 * bar_width,
        cte_rms_values,
        width=bar_width,
        label="RMS",
    )
    axes[0, 1].bar(
        positions + 0.5 * bar_width,
        cte_p95_values,
        width=bar_width,
        label="p95",
    )
    axes[0, 1].set_ylabel("|CTE| [m]")
    axes[0, 1].set_title("Cross-track error")
    axes[0, 1].legend(loc="best")

    steering_rms_values = [
        result["steering_rate_rms_degps"] for _, result in runs
    ]
    steering_p95_values = [
        result["steering_rate_p95_degps"] for _, result in runs
    ]
    axes[1, 0].bar(
        positions - 0.5 * bar_width,
        steering_rms_values,
        width=bar_width,
        label="RMS",
    )
    axes[1, 0].bar(
        positions + 0.5 * bar_width,
        steering_p95_values,
        width=bar_width,
        label="p95",
    )
    axes[1, 0].set_ylabel("steering rate [deg/s]")
    axes[1, 0].set_title("Steering command variation")
    axes[1, 0].legend(loc="best")

    axes[1, 1].plot(
        first_path[:, 0],
        first_path[:, 1],
        "k--",
        linewidth=0.8,
        label="reference path",
    )
    for (label, result), color in zip(runs, colors):
        event = result["trajectory"][
            "minimum_wheel_clearance_event"
        ]
        axes[1, 1].scatter(
            [event["x_m"]],
            [event["y_m"]],
            color=color,
            s=34,
        )
        axes[1, 1].annotate(
            label,
            (event["x_m"], event["y_m"]),
            xytext=(3, 3),
            textcoords="offset points",
            fontsize=6,
        )
    axes[1, 1].set_xlabel("map x [m]")
    axes[1, 1].set_ylabel("map y [m]")
    axes[1, 1].set_title("Location of minimum clearance")
    axes[1, 1].axis("equal")

    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        axis.set_xticks(positions)
        axis.set_xticklabels(labels, rotation=35, ha="right")
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    figure.suptitle("Path-tracking tuning summary")
    figure.tight_layout()
    figure.savefig(
        output_directory / "summary_metrics.png",
        dpi=160,
    )
    plt.close(figure)


def metric_row(label, bag_path, result):
    trajectory = result["trajectory"]
    overall = result["overall"]
    curved = result["curved"]
    high_speed = result["high_speed"]
    return {
        "label": label,
        "bag_path": str(bag_path),
        "duration_sec": result["duration_sec"],
        "maximum_speed_kph": result["maximum_speed_kph"],
        "wheel_contacts": trajectory[
            "wheel_line_contact_samples"
        ],
        "minimum_wheel_clearance_m": trajectory[
            "wheel_min_lane_clearance_m"
        ],
        "rear_center_rms_m": trajectory[
            "rear_center_rms_m"
        ],
        "rear_center_max_m": trajectory[
            "rear_center_max_m"
        ],
        "cte_rms_m": overall["cte_rms_m"],
        "cte_p95_m": overall["cte_p95_m"],
        "cte_max_m": overall["cte_max_m"],
        "cte_max_elapsed_sec": result["maximum_cte_event"][
            "elapsed_sec"
        ],
        "minimum_wheel_clearance_elapsed_sec": trajectory[
            "minimum_wheel_clearance_event"
        ]["elapsed_sec"],
        "minimum_wheel_clearance_x_m": trajectory[
            "minimum_wheel_clearance_event"
        ]["x_m"],
        "minimum_wheel_clearance_y_m": trajectory[
            "minimum_wheel_clearance_event"
        ]["y_m"],
        "curved_cte_p95_m": curved.get(
            "cte_p95_m", float("nan")
        ),
        "high_speed_cte_p95_m": high_speed.get(
            "cte_p95_m", float("nan")
        ),
        "steering_rate_rms_degps": result[
            "steering_rate_rms_degps"
        ],
        "steering_rate_p95_degps": result[
            "steering_rate_p95_degps"
        ],
        "high_speed_straight_steering_rate_rms_degps": result[
            "high_speed_straight_steering_rate_rms_degps"
        ],
        "high_speed_straight_rate_reversals": result[
            "high_speed_straight_rate_reversals"
        ],
        "high_speed_straight_rate_reversals_per_sec": result[
            "high_speed_straight_rate_reversals_per_sec"
        ],
        "high_speed_straight_steering_1s_peak_to_peak_p95_deg": result[
            "high_speed_straight_steering_1s_peak_to_peak_p95_deg"
        ],
        "high_speed_straight_steering_1s_peak_to_peak_max_deg": result[
            "high_speed_straight_steering_1s_peak_to_peak_max_deg"
        ],
        "high_speed_straight_large_2deg_steering_excursions": result[
            "high_speed_straight_large_2deg_steering_excursions"
        ],
        "high_speed_straight_cte_filter_delta_rms_m": result[
            "high_speed_straight_cte_filter_delta_rms_m"
        ],
        "low_curvature_steering_rate_rms_degps": result[
            "low_curvature_steering_rate_rms_degps"
        ],
        "low_curvature_rate_reversals_per_sec": result[
            "low_curvature_rate_reversals_per_sec"
        ],
        "low_curvature_steering_1s_peak_to_peak_p95_deg": result[
            "low_curvature_steering_1s_peak_to_peak_p95_deg"
        ],
        "curve_speed_limit_samples": result["speed_limit_sources"][
            "curve_samples"
        ],
        "lane_speed_limit_samples": result["speed_limit_sources"][
            "lane_clearance_samples"
        ],
        "heading_speed_limit_samples": result["speed_limit_sources"][
            "heading_error_samples"
        ],
        "straight_samples": result["straight_samples"],
        "straight_brake_samples": result[
            "straight_brake_samples"
        ],
        "brake_samples": result["brake_samples"],
        "direct_pedal_reversals": result[
            "direct_pedal_reversals"
        ],
        "heading_suppression_samples": result[
            "heading_suppression_samples"
        ],
        "maximum_heading_suppression_weight": result[
            "maximum_heading_suppression_weight"
        ],
    }


def aggregate_metric_rows(rows):
    grouped_rows = {}
    for row in rows:
        group = re.sub(r"_\d{2}$", "", row["label"])
        grouped_rows.setdefault(group, []).append(row)

    def mean(group_rows, key):
        return float(
            np.mean([float(row[key]) for row in group_rows])
        )

    aggregates = []
    for group, group_rows in grouped_rows.items():
        aggregates.append(
            {
                "group": group,
                "runs": len(group_rows),
                "safe_runs": sum(
                    int(row["wheel_contacts"]) == 0
                    for row in group_rows
                ),
                "wheel_contacts_total": sum(
                    int(row["wheel_contacts"])
                    for row in group_rows
                ),
                "minimum_wheel_clearance_worst_m": min(
                    float(row["minimum_wheel_clearance_m"])
                    for row in group_rows
                ),
                "minimum_wheel_clearance_mean_m": mean(
                    group_rows, "minimum_wheel_clearance_m"
                ),
                "cte_rms_mean_m": mean(group_rows, "cte_rms_m"),
                "cte_p95_mean_m": mean(group_rows, "cte_p95_m"),
                "cte_max_worst_m": max(
                    float(row["cte_max_m"]) for row in group_rows
                ),
                "curved_cte_p95_mean_m": mean(
                    group_rows, "curved_cte_p95_m"
                ),
                "steering_rate_rms_mean_degps": mean(
                    group_rows, "steering_rate_rms_degps"
                ),
                "steering_rate_p95_worst_degps": max(
                    float(row["steering_rate_p95_degps"])
                    for row in group_rows
                ),
                "high_speed_straight_steering_1s_peak_to_peak_p95_worst_deg": max(
                    float(row.get(
                        "high_speed_straight_steering_1s_peak_to_peak_p95_deg",
                        float("nan"),
                    ))
                    for row in group_rows
                ),
                "high_speed_straight_large_2deg_steering_excursions_total": sum(
                    int(row.get(
                        "high_speed_straight_large_2deg_steering_excursions",
                        0,
                    ))
                    for row in group_rows
                ),
                "low_curvature_steering_1s_peak_to_peak_p95_worst_deg": max(
                    float(row.get(
                        "low_curvature_steering_1s_peak_to_peak_p95_deg",
                        float("nan"),
                    ))
                    for row in group_rows
                ),
                "low_curvature_rate_reversals_per_sec_mean": mean(
                    group_rows, "low_curvature_rate_reversals_per_sec"
                ),
                "maximum_speed_worst_kph": max(
                    float(row["maximum_speed_kph"])
                    for row in group_rows
                ),
                "straight_brake_samples_total": sum(
                    int(row["straight_brake_samples"])
                    for row in group_rows
                ),
                "direct_pedal_reversals_mean": mean(
                    group_rows, "direct_pedal_reversals"
                ),
            }
        )
    return aggregates


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_outputs(runs, bag_paths, output_directory, lane_half_width_m):
    output_directory.mkdir(parents=True, exist_ok=True)
    rows = [
        metric_row(label, path, result)
        for (label, result), path in zip(runs, bag_paths)
    ]
    with open(
        output_directory / "metrics.csv",
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(rows[0].keys()),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)

    aggregate_rows = aggregate_metric_rows(rows)
    with open(
        output_directory / "aggregate_metrics.csv",
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(aggregate_rows[0].keys()),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(aggregate_rows)
    with open(
        output_directory / "aggregate_metrics.yaml",
        "w",
        encoding="utf-8",
    ) as stream:
        yaml.safe_dump(
            builtin_numbers(aggregate_rows),
            stream,
            sort_keys=False,
            allow_unicode=True,
        )

    serializable_results = {}
    manifest = []
    for (label, result), path in zip(runs, bag_paths):
        serializable_results[label] = {
            key: value
            for key, value in result.items()
            if key != "_series"
        }
        manifest.append(
            {
                "label": label,
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "sha256": sha256_file(path),
            }
        )
    with open(
        output_directory / "metrics.yaml",
        "w",
        encoding="utf-8",
    ) as stream:
        yaml.safe_dump(
            builtin_numbers(serializable_results),
            stream,
            sort_keys=False,
            allow_unicode=True,
        )
    with open(
        output_directory / "bag_manifest.yaml",
        "w",
        encoding="utf-8",
    ) as stream:
        yaml.safe_dump(
            manifest,
            stream,
            sort_keys=False,
            allow_unicode=True,
        )
    plot_runs(runs, output_directory, lane_half_width_m)


def parse_labeled_bag(value):
    if "=" in value:
        label, raw_path = value.split("=", 1)
    else:
        raw_path = value
        label = pathlib.Path(raw_path).stem
    if not label:
        raise argparse.ArgumentTypeError("bag label cannot be empty")
    path = pathlib.Path(raw_path).expanduser().resolve()
    if not path.is_file():
        raise argparse.ArgumentTypeError(
            "bag file does not exist: " + str(path)
        )
    return label, path


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze MORAI path-tracking bags and generate CSV/PNG reports."
        )
    )
    parser.add_argument(
        "bags",
        nargs="+",
        type=parse_labeled_bag,
        metavar="LABEL=PATH",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=pathlib.Path,
    )
    parser.add_argument(
        "--lane-half-width-m",
        type=float,
        default=1.5,
    )
    parser.add_argument(
        "--vehicle-width-m",
        type=float,
        default=1.892,
    )
    parser.add_argument(
        "--wheelbase-m",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--front-overhang-m",
        type=float,
        default=0.845,
    )
    parser.add_argument(
        "--rear-overhang-m",
        type=float,
        default=0.790,
    )
    args = parser.parse_args()

    runs = []
    paths = []
    for label, path in args.bags:
        samples = load_bag(path)
        result = analyze_samples(
            samples["global_path"],
            samples["statuses"],
            samples["odometry"],
            lane_half_width_m=args.lane_half_width_m,
            vehicle_width_m=args.vehicle_width_m,
            wheelbase_m=args.wheelbase_m,
            front_overhang_m=args.front_overhang_m,
            rear_overhang_m=args.rear_overhang_m,
        )
        runs.append((label, result))
        paths.append(path)
        trajectory = result["trajectory"]
        print(
            "{}: contacts={}, clearance={:.3f} m, "
            "CTE max={:.3f} m, speed max={:.2f} km/h".format(
                label,
                trajectory["wheel_line_contact_samples"],
                trajectory["wheel_min_lane_clearance_m"],
                result["overall"]["cte_max_m"],
                result["maximum_speed_kph"],
            )
        )

    write_outputs(
        runs,
        paths,
        args.output_dir.resolve(),
        args.lane_half_width_m,
    )


if __name__ == "__main__":
    main()
