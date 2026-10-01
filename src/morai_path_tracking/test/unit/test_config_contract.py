#!/usr/bin/env python3

import unittest
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[2]


class PurePursuitConfigContractTest(unittest.TestCase):
    def test_runtime_yaml_contains_every_required_parameter(self):
        with (
            PACKAGE_ROOT
            / "config"
            / "controllers"
            / "molit_2026_path_tracking.yaml"
        ).open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)["path_tracking_controller_node"]

        expected = {
            "local_path_topic",
            "ego_topic",
            "vehicle_status_topic",
            "command_topic",
            "controller_status_topic",
            "lookahead_point_topic",
            "stanley_projection_point_topic",
            "mpc_projection_point_topic",
            "expected_frame_id",
            "expected_velocity_frame_id",
            "control_rate_hz",
            "path_timeout_sec",
            "ego_timeout_sec",
            "vehicle_status_timeout_sec",
            "maximum_input_skew_sec",
            "input_sync_queue_size",
            "minimum_control_dt_sec",
            "maximum_control_dt_sec",
            "safe_brake_command",
            "wheelbase_m",
            "vehicle_width_m",
            "lane_half_width_m",
            "lane_clearance_recovery_start_m",
            "lane_clearance_recovery_full_m",
            "lane_clearance_recovery_speed_kph",
            "heading_error_speed_limit_start_deg",
            "heading_error_speed_limit_full_deg",
            "heading_error_recovery_speed_kph",
            "lookahead_base_m",
            "lookahead_speed_gain_sec",
            "lookahead_curvature_gain_m",
            "lookahead_min_m",
            "lookahead_max_m",
            "minimum_target_distance_m",
            "maximum_steering_angle_deg",
            "lateral_controller",
            "longitudinal_controller",
            "mpc_solver",
            "mpc_test_speed_limit_kph",
            "mpc_prediction_time_sec",
            "mpc_horizon_steps",
            "mpc_time_based_horizon",
            "mpc_adaptive_prediction_time",
            "mpc_minimum_prediction_time_sec",
            "mpc_maximum_prediction_time_sec",
            "mpc_minimum_horizon_steps",
            "mpc_maximum_horizon_steps",
            "mpc_minimum_preview_distance_m",
            "mpc_prediction_speed_gain_sec_per_mps",
            "mpc_resample_distance_m",
            "mpc_maximum_steering_rate_deg_per_sec",
            "mpc_optimizer_iterations",
            "mpc_optimizer_initial_step_deg_per_sec",
            "mpc_optimizer_min_step_deg_per_sec",
            "mpc_optimizer_step_decay",
            "mpc_optimizer_gradient_epsilon_deg_per_sec",
            "mpc_optimizer_gradient_step",
            "mpc_optimizer_gradient_tolerance",
            "mpc_optimizer_line_search_steps",
            "mpc_use_warm_start",
            "mpc_use_multi_resolution",
            "mpc_use_previous_control",
            "mpc_model_actuator",
            "mpc_actuator_delay_sec",
            "mpc_actuator_time_constant_sec",
            "mpc_weight_lateral",
            "mpc_weight_heading",
            "mpc_weight_steering",
            "mpc_weight_steering_rate",
            "mpc_weight_steering_rate_change",
            "mpc_weight_terminal_lateral",
            "mpc_weight_terminal_heading",
            "mpc_adaptation_start_curvature_m_inv",
            "mpc_adaptation_full_curvature_m_inv",
            "mpc_curve_heading_weight_multiplier",
            "mpc_curve_steering_weight_multiplier",
            "mpc_curve_rate_change_weight_multiplier",
            "mpc_straight_rate_weight_multiplier",
            "mpc_straight_rate_change_weight_multiplier",
            "mpc_yaw_rate_damping_gain",
            "mpc_yaw_rate_damping_fade_curvature_m_inv",
            "mpc_swept_path_compensation_enabled",
            "mpc_swept_path_vehicle_width_m",
            "mpc_swept_path_compensation_gain",
            "mpc_swept_path_activation_start_curvature_m_inv",
            "mpc_swept_path_activation_full_curvature_m_inv",
            "mpc_swept_path_maximum_offset_m",
            "mpc_swept_path_curvature_smoothing_window_points",
            "mpc_straight_cte_filter_time_constant_sec",
            "mpc_straight_cte_filter_minimum_speed_kph",
            "mpc_straight_cte_filter_maximum_curvature_m_inv",
            "mpc_straight_cte_maximum_rate_mps",
            "mpc_straight_steering_rate_filter_time_constant_sec",
            "mpc_straight_steering_rate_filter_minimum_speed_kph",
            "mpc_straight_steering_rate_filter_maximum_curvature_m_inv",
            "mpc_straight_spatial_fit_enabled",
            "mpc_straight_spatial_fit_start_speed_kph",
            "mpc_straight_spatial_fit_full_speed_kph",
            "mpc_straight_spatial_fit_full_curvature_m_inv",
            "mpc_straight_spatial_fit_off_curvature_m_inv",
            "mpc_straight_spatial_fit_base_preview_m",
            "mpc_straight_spatial_fit_speed_gain_sec",
            "mpc_straight_spatial_fit_maximum_preview_m",
            "mpc_straight_spatial_fit_lateral_weight_multiplier",
            "mpc_curve_feedforward_seed_gain",
            "mpc_curve_feedforward_seed_lookahead_m",
            "mpc_straight_yaw_rate_steering_estimate_gain",
            "mpc_straight_yaw_rate_steering_estimate_minimum_speed_kph",
            "mpc_straight_yaw_rate_steering_estimate_filter_time_constant_sec",
            "mpc_curve_yaw_rate_steering_estimate_gain",
            "mpc_curve_yaw_rate_steering_estimate_minimum_speed_kph",
            "mpc_curve_yaw_rate_steering_estimate_activation_start_curvature_m_inv",
            "mpc_curve_yaw_rate_steering_estimate_activation_full_curvature_m_inv",
            "mpc_curve_yaw_rate_steering_estimate_blend_time_constant_sec",
            "mpc_straight_dynamic_model_enabled",
            "mpc_straight_dynamic_model_minimum_speed_kph",
            "mpc_straight_dynamic_model_maximum_curvature_m_inv",
            "mpc_weight_straight_dynamic_sideslip",
            "mpc_weight_straight_dynamic_yaw_rate",
            "mpc_compensated_path_topic",
            "stanley_gain",
            "stanley_softening_speed_mps",
            "stanley_minimum_control_speed_mps",
            "stanley_heading_window_m",
            "stanley_heading_error_gain",
            "stanley_curvature_feedforward_gain",
            "stanley_curvature_preview_distance_m",
            "stanley_yaw_rate_damping_gain_sec",
            "stanley_yaw_rate_damping_nonlinear_gain_sec2",
            "stanley_maximum_steering_rate_deg_per_sec",
            "hybrid_mass_kg",
            "hybrid_yaw_inertia_kgm2",
            "hybrid_front_cornering_stiffness_n_per_rad",
            "hybrid_rear_cornering_stiffness_n_per_rad",
            "hybrid_front_axle_to_cg_m",
            "hybrid_rear_axle_to_cg_m",
            "hybrid_process_noise_sideslip",
            "hybrid_process_noise_yaw_rate",
            "hybrid_measurement_noise_sideslip",
            "hybrid_measurement_noise_yaw_rate",
            "hybrid_initial_covariance_sideslip",
            "hybrid_initial_covariance_yaw_rate",
            "hybrid_initial_pure_pursuit_probability",
            "hybrid_initial_stanley_probability",
            "hybrid_stanley_probability_min",
            "hybrid_stanley_probability_max",
            "hybrid_transition_pure_pursuit_to_pure_pursuit",
            "hybrid_transition_pure_pursuit_to_stanley",
            "hybrid_transition_stanley_to_pure_pursuit",
            "hybrid_transition_stanley_to_stanley",
            "hybrid_transition_speed_gain",
            "hybrid_transition_reference_speed_kph",
            "hybrid_minimum_model_speed_mps",
            "hybrid_pure_pursuit_cross_track_correction_gain",
            "hybrid_curve_preview_stanley_weight_start_m_inv",
            "hybrid_curve_preview_stanley_weight_full_m_inv",
            "hybrid_curve_preview_stanley_minimum_weight",
            "hybrid_heading_lag_stanley_weight_start_deg",
            "hybrid_heading_lag_stanley_weight_full_deg",
            "hybrid_heading_lag_stanley_minimum_weight",
            "hybrid_candidate_conflict_curvature_threshold_m_inv",
            "hybrid_candidate_conflict_cross_track_threshold_m",
            "hybrid_cross_track_recovery_full_scale_m",
            "hybrid_cross_track_recovery_heading_error_suppression_start_deg",
            "hybrid_cross_track_recovery_heading_error_suppression_full_deg",
            "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio",
            "hybrid_maximum_steering_rate_deg_per_sec",
            "hybrid_low_curvature_maximum_steering_rate_deg_per_sec",
            "hybrid_full_steering_rate_curvature_m_inv",
            "hybrid_steering_return_rate_multiplier",
            "target_speed_kph",
            "minimum_curve_speed_kph",
            "maximum_lateral_acceleration_mps2",
            "curvature_speed_reduction_gain_m",
            "curvature_preview_distance_m",
            "lookahead_curvature_preview_distance_m",
            "curvature_sample_spacing_m",
            "curve_approach_deceleration_mps2",
            "curvature_epsilon_m_inv",
            "target_speed_acceleration_limit_mps2",
            "curve_target_speed_acceleration_limit_mps2",
            "target_speed_deceleration_limit_mps2",
            "target_speed_filter_time_constant_sec",
            "speed_filter_time_constant_sec",
            "speed_kp",
            "speed_ki",
            "speed_kd",
            "speed_integral_limit",
            "speed_integral_unwind_rate_per_sec",
            "speed_error_deadband_mps",
            "speed_accel_feedforward_gain_per_mps",
            "speed_coast_overspeed_kph",
            "speed_brake_overspeed_kph",
            "hard_brake_activation_speed_kph",
            "minimum_hard_brake_command",
            "maximum_accel_command",
            "maximum_brake_command",
            "longitudinal_command_rate_limit_per_sec",
            "longitudinal_mpc_solver",
            "longitudinal_mpc_horizon_steps",
            "longitudinal_mpc_optimizer_iterations",
            "longitudinal_mpc_line_search_steps",
            "longitudinal_mpc_prediction_dt_sec",
            "longitudinal_mpc_gradient_epsilon",
            "longitudinal_mpc_gradient_step",
            "longitudinal_mpc_gradient_tolerance",
            "longitudinal_mpc_coordinate_initial_step",
            "longitudinal_mpc_coordinate_minimum_step",
            "longitudinal_mpc_command_step_decay",
            "maximum_speed_kph",
            "longitudinal_mpc_acceleration_gain_mps2",
            "longitudinal_mpc_braking_gain_mps2",
            "longitudinal_mpc_actuator_time_constant_sec",
            "longitudinal_mpc_maximum_acceleration_mps2",
            "longitudinal_mpc_maximum_deceleration_mps2",
            "longitudinal_mpc_maximum_jerk_mps3",
            "longitudinal_mpc_acceleration_filter_tau_sec",
            "longitudinal_mpc_weight_speed",
            "longitudinal_mpc_weight_acceleration",
            "longitudinal_mpc_weight_jerk",
            "longitudinal_mpc_weight_command",
            "longitudinal_mpc_weight_command_rate",
            "longitudinal_mpc_weight_terminal_speed",
            "longitudinal_mpc_fallback_to_pid",
        }
        self.assertEqual(set(config), expected)
        self.assertEqual(config["vehicle_status_topic"], "/vehicle/competition_status")
        self.assertEqual(config["target_speed_kph"], 59.0)
        self.assertEqual(config["vehicle_width_m"], 1.892)
        self.assertEqual(config["lane_half_width_m"], 1.35)
        self.assertEqual(config["lane_clearance_recovery_start_m"], 0.18)
        self.assertEqual(config["lane_clearance_recovery_full_m"], 0.0)
        self.assertEqual(config["lane_clearance_recovery_speed_kph"], 10.0)
        self.assertEqual(config["heading_error_speed_limit_start_deg"], 14.0)
        self.assertEqual(config["heading_error_speed_limit_full_deg"], 20.0)
        self.assertEqual(config["heading_error_recovery_speed_kph"], 10.0)
        self.assertEqual(config["minimum_curve_speed_kph"], 15.0)
        self.assertEqual(
            config[
                "mpc_curve_yaw_rate_steering_estimate_activation_start_curvature_m_inv"
            ],
            0.020,
        )
        self.assertEqual(
            config[
                "mpc_curve_yaw_rate_steering_estimate_activation_full_curvature_m_inv"
            ],
            0.050,
        )
        self.assertEqual(
            config[
                "mpc_curve_yaw_rate_steering_estimate_blend_time_constant_sec"
            ],
            0.15,
        )
        self.assertEqual(config["maximum_lateral_acceleration_mps2"], 2.7)
        self.assertEqual(config["curvature_speed_reduction_gain_m"], 0.0)
        self.assertEqual(config["curvature_preview_distance_m"], 45.0)
        self.assertEqual(config["lookahead_curvature_preview_distance_m"], 8.0)
        self.assertEqual(config["curvature_sample_spacing_m"], 2.0)
        self.assertEqual(config["curve_approach_deceleration_mps2"], 1.7)
        self.assertEqual(config["target_speed_acceleration_limit_mps2"], 2.0)
        self.assertEqual(
            config["curve_target_speed_acceleration_limit_mps2"], 1.2
        )
        self.assertEqual(config["target_speed_deceleration_limit_mps2"], 5.0)
        self.assertEqual(config["maximum_accel_command"], 0.70)
        self.assertEqual(config["lookahead_base_m"], 4.0)
        self.assertEqual(config["lookahead_speed_gain_sec"], 0.75)
        self.assertEqual(config["lookahead_curvature_gain_m"], 8.0)
        self.assertEqual(config["lookahead_min_m"], 4.0)
        self.assertEqual(config["lookahead_max_m"], 16.0)
        self.assertEqual(config["lateral_controller"], "mpc")
        self.assertEqual(config["longitudinal_controller"], "mpc")
        self.assertEqual(config["mpc_solver"], "projected_gradient")
        self.assertEqual(
            config["longitudinal_mpc_solver"], "projected_gradient"
        )
        self.assertEqual(config["maximum_speed_kph"], 60.0)
        self.assertEqual(config["mpc_test_speed_limit_kph"], 59.0)
        self.assertEqual(config["mpc_prediction_time_sec"], 1.2)
        self.assertEqual(config["mpc_optimizer_iterations"], 10)
        self.assertEqual(config["mpc_weight_lateral"], 12.0)
        self.assertEqual(config["stanley_gain"], 2.0)
        self.assertEqual(config["stanley_softening_speed_mps"], 2.0)
        self.assertEqual(config["stanley_minimum_control_speed_mps"], 1.0)
        self.assertEqual(config["stanley_heading_window_m"], 4.0)
        self.assertEqual(config["stanley_heading_error_gain"], 0.6)
        self.assertEqual(config["stanley_curvature_feedforward_gain"], 1.0)
        self.assertEqual(config["stanley_curvature_preview_distance_m"], 8.0)
        self.assertEqual(config["stanley_yaw_rate_damping_gain_sec"], 0.1)
        self.assertEqual(
            config["stanley_yaw_rate_damping_nonlinear_gain_sec2"], 0.4
        )
        self.assertEqual(
            config["stanley_maximum_steering_rate_deg_per_sec"], 60.0
        )
        self.assertEqual(config["hybrid_mass_kg"], 2000.0)
        self.assertEqual(config["hybrid_yaw_inertia_kgm2"], 4000.0)
        self.assertEqual(
            config["hybrid_front_cornering_stiffness_n_per_rad"], 60000.0
        )
        self.assertEqual(
            config["hybrid_rear_cornering_stiffness_n_per_rad"], 60000.0
        )
        self.assertEqual(config["hybrid_front_axle_to_cg_m"], 1.5)
        self.assertEqual(config["hybrid_rear_axle_to_cg_m"], 1.5)
        self.assertEqual(config["hybrid_process_noise_sideslip"], 0.1)
        self.assertEqual(config["hybrid_process_noise_yaw_rate"], 0.01)
        self.assertEqual(config["hybrid_measurement_noise_sideslip"], 0.001)
        self.assertEqual(config["hybrid_measurement_noise_yaw_rate"], 0.001)
        self.assertEqual(
            config["hybrid_initial_pure_pursuit_probability"], 0.8
        )
        self.assertEqual(config["hybrid_initial_stanley_probability"], 0.2)
        self.assertEqual(config["hybrid_stanley_probability_min"], 0.15)
        self.assertEqual(config["hybrid_stanley_probability_max"], 0.90)
        self.assertEqual(config["hybrid_transition_speed_gain"], 0.1)
        self.assertEqual(config["hybrid_transition_reference_speed_kph"], 60.0)
        self.assertEqual(
            config["hybrid_pure_pursuit_cross_track_correction_gain"], 0.60
        )
        self.assertEqual(
            config["hybrid_curve_preview_stanley_weight_start_m_inv"], 0.02
        )
        self.assertEqual(
            config["hybrid_curve_preview_stanley_weight_full_m_inv"], 0.06
        )
        self.assertEqual(
            config["hybrid_curve_preview_stanley_minimum_weight"], 0.0
        )
        self.assertEqual(
            config["hybrid_heading_lag_stanley_weight_start_deg"], 8.0
        )
        self.assertEqual(
            config["hybrid_heading_lag_stanley_weight_full_deg"], 14.0
        )
        self.assertEqual(
            config["hybrid_heading_lag_stanley_minimum_weight"], 0.55
        )
        self.assertEqual(
            config["hybrid_candidate_conflict_curvature_threshold_m_inv"],
            0.015,
        )
        self.assertEqual(
            config["hybrid_candidate_conflict_cross_track_threshold_m"], 0.45
        )
        self.assertEqual(
            config["hybrid_cross_track_recovery_full_scale_m"], 0.55
        )
        self.assertEqual(
            config[
                "hybrid_cross_track_recovery_heading_error_suppression_start_deg"
            ],
            15.0,
        )
        self.assertEqual(
            config[
                "hybrid_cross_track_recovery_heading_error_suppression_full_deg"
            ],
            17.5,
        )
        self.assertEqual(
            config[
                "hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio"
            ],
            0.30,
        )
        self.assertEqual(
            config["hybrid_maximum_steering_rate_deg_per_sec"], 60.0
        )
        self.assertEqual(
            config["hybrid_low_curvature_maximum_steering_rate_deg_per_sec"],
            45.0,
        )
        self.assertEqual(
            config["hybrid_full_steering_rate_curvature_m_inv"], 0.015
        )
        self.assertEqual(
            config["hybrid_steering_return_rate_multiplier"], 2.0
        )
        self.assertEqual(config["speed_kp"], 0.18)
        self.assertEqual(config["speed_ki"], 0.02)
        self.assertEqual(config["speed_kd"], 0.0)
        self.assertEqual(config["speed_integral_limit"], 1.0)
        self.assertEqual(config["speed_error_deadband_mps"], 0.10)
        self.assertEqual(config["speed_accel_feedforward_gain_per_mps"], 0.008)
        self.assertEqual(config["speed_coast_overspeed_kph"], 0.2)
        self.assertEqual(config["speed_brake_overspeed_kph"], 1.8)
        self.assertEqual(config["hard_brake_activation_speed_kph"], 59.5)
        self.assertEqual(config["minimum_hard_brake_command"], 0.25)
        self.assertEqual(config["target_speed_filter_time_constant_sec"], 0.35)
        self.assertEqual(config["longitudinal_command_rate_limit_per_sec"], 2.0)
        self.assertNotIn("target_speed_mps", config)
        self.assertEqual(config["speed_filter_time_constant_sec"], 0.0)
        self.assertEqual(config["maximum_steering_angle_deg"], 40.0)


if __name__ == "__main__":
    unittest.main()
