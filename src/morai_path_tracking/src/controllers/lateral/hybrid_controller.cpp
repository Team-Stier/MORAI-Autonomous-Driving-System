#include "morai_path_tracking/controllers/lateral/hybrid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

namespace morai_path_tracking {
namespace {

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool nonNegativeFinite(double value) {
  return std::isfinite(value) && value >= 0.0;
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

std::string configurationError(const HybridConfig& config) {
  if (!std::isfinite(
          config.pure_pursuit_cross_track_correction_gain) ||
      config.pure_pursuit_cross_track_correction_gain < 0.0 ||
      config.pure_pursuit_cross_track_correction_gain > 1.0) {
    return "pure_pursuit_cross_track_correction_gain must be finite and "
           "in [0, 1]";
  }
  if (!nonNegativeFinite(
          config.candidate_conflict_curvature_threshold_m_inv)) {
    return "candidate_conflict_curvature_threshold_m_inv must be finite "
           "and non-negative";
  }
  if (!nonNegativeFinite(
          config.candidate_conflict_cross_track_threshold_m)) {
    return "candidate_conflict_cross_track_threshold_m must be finite and "
           "non-negative";
  }
  if (!positiveFinite(config.cross_track_recovery_full_scale_m)) {
    return "cross_track_recovery_full_scale_m must be finite and positive";
  }
  if (!nonNegativeFinite(
          config
              .cross_track_recovery_heading_error_suppression_start_rad)) {
    return "cross_track_recovery_heading_error_suppression_start_rad must "
           "be finite and non-negative";
  }
  if (!positiveFinite(
          config
              .cross_track_recovery_heading_error_suppression_full_rad) ||
      config.cross_track_recovery_heading_error_suppression_full_rad <=
          config.cross_track_recovery_heading_error_suppression_start_rad) {
    return "cross_track_recovery_heading_error_suppression_full_rad must "
           "be finite and greater than the suppression start";
  }
  if (!std::isfinite(
          config
              .cross_track_recovery_heading_error_maximum_suppression_ratio) ||
      config
              .cross_track_recovery_heading_error_maximum_suppression_ratio <
          0.0 ||
      config
              .cross_track_recovery_heading_error_maximum_suppression_ratio >
          1.0) {
    return "cross_track_recovery_heading_error_maximum_suppression_ratio "
           "must be finite and in [0, 1]";
  }
  if (!nonNegativeFinite(config.lane_clearance_recovery_start_m)) {
    return "lane_clearance_recovery_start_m must be finite and non-negative";
  }
  if (!nonNegativeFinite(config.lane_clearance_recovery_full_m) ||
      config.lane_clearance_recovery_full_m >=
          config.lane_clearance_recovery_start_m) {
    return "lane_clearance_recovery_full_m must be finite, non-negative, "
           "and less than lane_clearance_recovery_start_m";
  }
  if (!nonNegativeFinite(
          config.curve_preview_stanley_weight_start_m_inv)) {
    return "curve_preview_stanley_weight_start_m_inv must be finite and "
           "non-negative";
  }
  if (!positiveFinite(config.curve_preview_stanley_weight_full_m_inv) ||
      config.curve_preview_stanley_weight_full_m_inv <=
          config.curve_preview_stanley_weight_start_m_inv) {
    return "curve_preview_stanley_weight_full_m_inv must be finite and "
           "greater than curve_preview_stanley_weight_start_m_inv";
  }
  if (!std::isfinite(config.curve_preview_stanley_minimum_weight) ||
      config.curve_preview_stanley_minimum_weight < 0.0 ||
      config.curve_preview_stanley_minimum_weight > 1.0) {
    return "curve_preview_stanley_minimum_weight must be finite and in "
           "[0, 1]";
  }
  if (!nonNegativeFinite(config.heading_lag_stanley_weight_start_rad)) {
    return "heading_lag_stanley_weight_start_rad must be finite and "
           "non-negative";
  }
  if (!positiveFinite(config.heading_lag_stanley_weight_full_rad) ||
      config.heading_lag_stanley_weight_full_rad <=
          config.heading_lag_stanley_weight_start_rad) {
    return "heading_lag_stanley_weight_full_rad must be finite and greater "
           "than heading_lag_stanley_weight_start_rad";
  }
  if (!std::isfinite(config.heading_lag_stanley_minimum_weight) ||
      config.heading_lag_stanley_minimum_weight < 0.0 ||
      config.heading_lag_stanley_minimum_weight > 1.0) {
    return "heading_lag_stanley_minimum_weight must be finite and in [0, 1]";
  }
  if (!positiveFinite(config.maximum_steering_angle_rad)) {
    return "maximum_steering_angle_rad must be finite and positive";
  }
  if (!positiveFinite(config.maximum_steering_rate_rad_per_sec)) {
    return "maximum_steering_rate_rad_per_sec must be finite and positive";
  }
  if (!std::isfinite(
          config.low_curvature_maximum_steering_rate_rad_per_sec) ||
      config.low_curvature_maximum_steering_rate_rad_per_sec < 0.0 ||
      config.low_curvature_maximum_steering_rate_rad_per_sec >
          config.maximum_steering_rate_rad_per_sec) {
    return "low_curvature_maximum_steering_rate_rad_per_sec must be finite, "
           "non-negative, and no greater than "
           "maximum_steering_rate_rad_per_sec";
  }
  if (!positiveFinite(config.full_steering_rate_curvature_m_inv)) {
    return "full_steering_rate_curvature_m_inv must be finite and positive";
  }
  if (!std::isfinite(config.steering_return_rate_multiplier) ||
      config.steering_return_rate_multiplier < 1.0) {
    return "steering_return_rate_multiplier must be finite and >= 1";
  }
  if (!positiveFinite(config.imm.rear_axle_to_cg_m)) {
    return "IMM rear_axle_to_cg_m must be finite and positive";
  }
  if (!positiveFinite(config.imm.minimum_model_speed_mps)) {
    return "IMM minimum_model_speed_mps must be finite and positive";
  }
  return {};
}

HybridResult invalidResult(const std::string& error) {
  HybridResult result;
  result.error = error;
  return result;
}

}  // namespace

HybridController::HybridController(const HybridConfig& config)
    : config_(config),
      config_error_(configurationError(config)),
      stanley_controller_(config.stanley),
      imm_filter_(config.imm) {}

void HybridController::reset() {
  imm_filter_.reset();
}

HybridResult HybridController::calculate(
    const std::vector<Point2d>& path_in_vehicle_frame,
    double longitudinal_speed_mps,
    double rear_axle_lateral_velocity_mps,
    double measured_yaw_rate_radps,
    double preview_curvature_m_inv,
    double previous_steering_angle_rad,
    double dt_sec) {
  return calculate(path_in_vehicle_frame, longitudinal_speed_mps,
                   rear_axle_lateral_velocity_mps,
                   measured_yaw_rate_radps, preview_curvature_m_inv,
                   std::numeric_limits<double>::infinity(),
                   previous_steering_angle_rad, dt_sec);
}

HybridResult HybridController::calculate(
    const std::vector<Point2d>& path_in_vehicle_frame,
    double longitudinal_speed_mps,
    double rear_axle_lateral_velocity_mps,
    double measured_yaw_rate_radps,
    double preview_curvature_m_inv,
    double minimum_wheel_clearance_m,
    double previous_steering_angle_rad,
    double dt_sec) {
  if (!config_error_.empty()) {
    return invalidResult("invalid hybrid configuration: " +
                         config_error_);
  }
  if (!std::isfinite(longitudinal_speed_mps) ||
      longitudinal_speed_mps < 0.0) {
    return invalidResult(
        "longitudinal_speed_mps must be finite and non-negative");
  }
  if (!std::isfinite(rear_axle_lateral_velocity_mps) ||
      !std::isfinite(measured_yaw_rate_radps) ||
      !std::isfinite(preview_curvature_m_inv) ||
      preview_curvature_m_inv < 0.0 ||
      std::isnan(minimum_wheel_clearance_m) ||
      !std::isfinite(previous_steering_angle_rad)) {
    return invalidResult(
        "hybrid measurements and previous steering must be finite");
  }
  if (!positiveFinite(dt_sec)) {
    return invalidResult("dt_sec must be finite and positive");
  }

  HybridResult result;
  try {
    result.pure_pursuit = computePurePursuit(
        path_in_vehicle_frame, longitudinal_speed_mps,
        preview_curvature_m_inv, config_.pure_pursuit);
  } catch (const std::exception& error) {
    result.error = "Pure Pursuit configuration rejected: " +
                   std::string(error.what());
    return result;
  }
  result.stanley = stanley_controller_.calculate(
      path_in_vehicle_frame, longitudinal_speed_mps,
      measured_yaw_rate_radps, previous_steering_angle_rad, dt_sec);
  result.pure_pursuit_target = result.pure_pursuit.target;
  result.stanley_projection = result.stanley.target;

  if (!result.pure_pursuit.valid || !result.stanley.valid) {
    if (!result.pure_pursuit.valid && !result.stanley.valid) {
      result.error = "both hybrid controller candidates are invalid";
    } else if (!result.pure_pursuit.valid) {
      result.error = "Pure Pursuit hybrid candidate is invalid";
    } else {
      result.error = "Stanley hybrid candidate is invalid: " +
                     result.stanley.error;
    }
    return result;
  }

  result.cg_lateral_velocity_mps =
      rear_axle_lateral_velocity_mps +
      config_.imm.rear_axle_to_cg_m * measured_yaw_rate_radps;
  const double model_speed_mps =
      std::max(longitudinal_speed_mps,
               config_.imm.minimum_model_speed_mps);
  result.measured_sideslip_angle_rad =
      std::atan2(result.cg_lateral_velocity_mps, model_speed_mps);
  result.corrected_pure_pursuit_steering_angle_rad =
      clamp(result.pure_pursuit.steering_angle_rad +
                config_.pure_pursuit_cross_track_correction_gain *
                    result.stanley
                        .cross_track_feedback_steering_rad,
            -config_.maximum_steering_angle_rad,
            config_.maximum_steering_angle_rad);

  result.imm = imm_filter_.update(
      longitudinal_speed_mps, result.measured_sideslip_angle_rad,
      measured_yaw_rate_radps,
      result.corrected_pure_pursuit_steering_angle_rad,
      result.stanley.requested_steering_angle_rad, dt_sec);
  if (!result.imm.valid) {
    result.error = "IMM hybrid update failed: " + result.imm.error;
    return result;
  }

  result.effective_pure_pursuit_weight =
      result.imm.pure_pursuit_probability;
  result.effective_stanley_weight =
      result.imm.stanley_probability;
  const bool candidates_have_opposite_sign =
      result.corrected_pure_pursuit_steering_angle_rad *
          result.stanley.requested_steering_angle_rad <
      0.0;
  const bool curve_requires_geometric_preview =
      preview_curvature_m_inv >=
      config_.candidate_conflict_curvature_threshold_m_inv;
  const bool cross_track_error_requires_recovery =
      std::abs(result.stanley.cross_track_error_m) >=
      config_.candidate_conflict_cross_track_threshold_m;
  result.candidate_conflict_guard_active =
      candidates_have_opposite_sign &&
      (curve_requires_geometric_preview ||
       cross_track_error_requires_recovery);
  if (result.candidate_conflict_guard_active) {
    const bool stanley_follows_signed_local_curvature =
        std::abs(result.stanley.reference_curvature_m_inv) >=
            config_.candidate_conflict_curvature_threshold_m_inv &&
        result.stanley.reference_curvature_m_inv *
                result.stanley.requested_steering_angle_rad >
            0.0;
    const bool stanley_is_stronger =
        std::abs(result.stanley.requested_steering_angle_rad) >
        std::abs(result.corrected_pure_pursuit_steering_angle_rad);
    const bool stanley_reduces_heading_error =
        result.stanley.heading_error_rad *
                result.stanley.requested_steering_angle_rad >
            0.0;
    result.candidate_conflict_stanley_override_active =
        config_.curve_preview_stanley_minimum_weight > 0.0 &&
        curve_requires_geometric_preview &&
        stanley_follows_signed_local_curvature && stanley_is_stronger &&
        stanley_reduces_heading_error;
    if (result.candidate_conflict_stanley_override_active) {
      result.effective_pure_pursuit_weight = 0.0;
      result.effective_stanley_weight = 1.0;
    } else {
      result.effective_pure_pursuit_weight = 1.0;
      result.effective_stanley_weight = 0.0;
    }
  } else {
    const double absolute_cross_track_error_m =
        std::abs(result.stanley.cross_track_error_m);
    const double recovery_start_m =
        0.5 * config_.cross_track_recovery_full_scale_m;
    const bool pure_pursuit_requests_cross_track_recovery =
        result.stanley.cross_track_error_m *
            result.corrected_pure_pursuit_steering_angle_rad >
        0.0;
    const bool pure_pursuit_requests_stronger_recovery =
        std::abs(result.corrected_pure_pursuit_steering_angle_rad) >
        std::abs(result.stanley.requested_steering_angle_rad);
    if (pure_pursuit_requests_cross_track_recovery &&
        pure_pursuit_requests_stronger_recovery &&
        absolute_cross_track_error_m > recovery_start_m) {
      const double normalized_recovery =
          clamp((absolute_cross_track_error_m - recovery_start_m) /
                    (config_.cross_track_recovery_full_scale_m -
                     recovery_start_m),
                0.0, 1.0);
      const double smooth_recovery_weight =
          normalized_recovery * normalized_recovery *
          (3.0 - 2.0 * normalized_recovery);
      const double normalized_heading_suppression =
          clamp(
              (std::abs(result.stanley.heading_error_rad) -
               config_
                   .cross_track_recovery_heading_error_suppression_start_rad) /
                  (config_
                       .cross_track_recovery_heading_error_suppression_full_rad -
                   config_
                       .cross_track_recovery_heading_error_suppression_start_rad),
              0.0, 1.0);
      result.cross_track_recovery_heading_suppression_weight =
          normalized_heading_suppression *
          normalized_heading_suppression *
          (3.0 - 2.0 * normalized_heading_suppression) *
          config_
              .cross_track_recovery_heading_error_maximum_suppression_ratio;
      result.cross_track_recovery_heading_suppression_active =
          result.cross_track_recovery_heading_suppression_weight >
          0.0;
      result.cross_track_recovery_weight =
          smooth_recovery_weight *
          (1.0 -
           result
               .cross_track_recovery_heading_suppression_weight);
      const double previous_pure_pursuit_weight =
          result.effective_pure_pursuit_weight;
      result.effective_pure_pursuit_weight =
          std::max(result.effective_pure_pursuit_weight,
                   result.cross_track_recovery_weight);
      result.effective_pure_pursuit_weight =
          std::min(
              result.effective_pure_pursuit_weight,
              1.0 -
                  result
                      .cross_track_recovery_heading_suppression_weight);
      result.effective_stanley_weight =
          1.0 - result.effective_pure_pursuit_weight;
      result.cross_track_recovery_active =
          result.effective_pure_pursuit_weight >
          previous_pure_pursuit_weight;
    }
  }

  if (!result.candidate_conflict_guard_active) {
    const bool same_steering_direction =
        result.corrected_pure_pursuit_steering_angle_rad *
            result.stanley.requested_steering_angle_rad >
        0.0;
    const bool stanley_requests_stronger_curve_entry =
        std::abs(result.stanley.requested_steering_angle_rad) >
        std::abs(result.corrected_pure_pursuit_steering_angle_rad);
    if (same_steering_direction &&
        stanley_requests_stronger_curve_entry) {
      const double normalized_curve_urgency =
          clamp((preview_curvature_m_inv -
                 config_.curve_preview_stanley_weight_start_m_inv) /
                    (config_.curve_preview_stanley_weight_full_m_inv -
                     config_.curve_preview_stanley_weight_start_m_inv),
                0.0, 1.0);
      const double smooth_curve_urgency =
          normalized_curve_urgency * normalized_curve_urgency *
          (3.0 - 2.0 * normalized_curve_urgency);
      result.curve_preview_stanley_recovery_weight =
          smooth_curve_urgency *
          config_.curve_preview_stanley_minimum_weight;
      if (result.curve_preview_stanley_recovery_weight >
          result.effective_stanley_weight) {
        result.effective_stanley_weight =
            result.curve_preview_stanley_recovery_weight;
        result.effective_pure_pursuit_weight =
            1.0 - result.effective_stanley_weight;
        result.curve_preview_stanley_recovery_active = true;
      }
    }
  }

  if (!result.candidate_conflict_guard_active) {
    const bool same_steering_direction =
        result.corrected_pure_pursuit_steering_angle_rad *
            result.stanley.requested_steering_angle_rad >
        0.0;
    const bool stanley_requests_stronger_heading_recovery =
        std::abs(result.stanley.requested_steering_angle_rad) >
        std::abs(result.corrected_pure_pursuit_steering_angle_rad);
    const bool stanley_reduces_heading_error =
        result.stanley.heading_error_rad *
            result.stanley.requested_steering_angle_rad >
        0.0;
    if (same_steering_direction &&
        stanley_requests_stronger_heading_recovery &&
        stanley_reduces_heading_error) {
      const double normalized_heading_urgency =
          clamp((std::abs(result.stanley.heading_error_rad) -
                 config_.heading_lag_stanley_weight_start_rad) /
                    (config_.heading_lag_stanley_weight_full_rad -
                     config_.heading_lag_stanley_weight_start_rad),
                0.0, 1.0);
      const double smooth_heading_urgency =
          normalized_heading_urgency * normalized_heading_urgency *
          (3.0 - 2.0 * normalized_heading_urgency);
      result.heading_lag_stanley_recovery_weight =
          smooth_heading_urgency *
          config_.heading_lag_stanley_minimum_weight;
      if (result.heading_lag_stanley_recovery_weight >
          result.effective_stanley_weight) {
        result.effective_stanley_weight =
            result.heading_lag_stanley_recovery_weight;
        result.effective_pure_pursuit_weight =
            1.0 - result.effective_stanley_weight;
        result.heading_lag_stanley_recovery_active = true;
      }
    }
  }

  if (!result.candidate_conflict_guard_active &&
      std::isfinite(minimum_wheel_clearance_m)) {
    const double normalized_clearance_urgency =
        clamp((config_.lane_clearance_recovery_start_m -
               minimum_wheel_clearance_m) /
                  (config_.lane_clearance_recovery_start_m -
                   config_.lane_clearance_recovery_full_m),
              0.0, 1.0);
    result.lane_clearance_recovery_urgency =
        normalized_clearance_urgency * normalized_clearance_urgency *
        (3.0 - 2.0 * normalized_clearance_urgency);

    const double normalized_heading_suppression =
        clamp(
            (std::abs(result.stanley.heading_error_rad) -
             config_
                 .cross_track_recovery_heading_error_suppression_start_rad) /
                (config_
                     .cross_track_recovery_heading_error_suppression_full_rad -
                 config_
                     .cross_track_recovery_heading_error_suppression_start_rad),
            0.0, 1.0);
    const double heading_suppression =
        normalized_heading_suppression * normalized_heading_suppression *
        (3.0 - 2.0 * normalized_heading_suppression) *
        config_
            .cross_track_recovery_heading_error_maximum_suppression_ratio;
    const double recovery_weight =
        result.lane_clearance_recovery_urgency *
        (1.0 - heading_suppression);
    const bool pure_pursuit_recovers_cross_track =
        result.stanley.cross_track_error_m *
            result.corrected_pure_pursuit_steering_angle_rad >
        0.0;
    const bool stanley_recovers_cross_track =
        result.stanley.cross_track_error_m *
            result.stanley.requested_steering_angle_rad >
        0.0;
    const bool pure_pursuit_is_stronger =
        pure_pursuit_recovers_cross_track &&
        (!stanley_recovers_cross_track ||
         std::abs(result.corrected_pure_pursuit_steering_angle_rad) >
             std::abs(result.stanley.requested_steering_angle_rad));
    const bool stanley_is_stronger =
        stanley_recovers_cross_track &&
        (!pure_pursuit_recovers_cross_track ||
         std::abs(result.stanley.requested_steering_angle_rad) >
             std::abs(result.corrected_pure_pursuit_steering_angle_rad));
    if (pure_pursuit_is_stronger &&
        recovery_weight > result.effective_pure_pursuit_weight) {
      result.effective_pure_pursuit_weight = recovery_weight;
      result.effective_stanley_weight = 1.0 - recovery_weight;
      result.lane_clearance_recovery_active = true;
    } else if (stanley_is_stronger &&
               recovery_weight > result.effective_stanley_weight) {
      result.effective_stanley_weight = recovery_weight;
      result.effective_pure_pursuit_weight = 1.0 - recovery_weight;
      result.lane_clearance_recovery_active = true;
    }
  }

  result.requested_steering_angle_rad =
      result.effective_pure_pursuit_weight *
          result.corrected_pure_pursuit_steering_angle_rad +
      result.effective_stanley_weight *
          result.stanley.requested_steering_angle_rad;
  const double saturated_steering_angle_rad =
      clamp(result.requested_steering_angle_rad,
            -config_.maximum_steering_angle_rad,
            config_.maximum_steering_angle_rad);
  const bool returning_toward_center =
      saturated_steering_angle_rad * previous_steering_angle_rad >= 0.0 &&
      std::abs(saturated_steering_angle_rad) <
          std::abs(previous_steering_angle_rad);
  const double low_curvature_maximum_steering_rate_rad_per_sec =
      config_.low_curvature_maximum_steering_rate_rad_per_sec > 0.0
          ? config_.low_curvature_maximum_steering_rate_rad_per_sec
          : config_.maximum_steering_rate_rad_per_sec;
  const bool adaptive_rate_enabled =
      low_curvature_maximum_steering_rate_rad_per_sec <
      config_.maximum_steering_rate_rad_per_sec;
  const double normalized_rate_urgency =
      adaptive_rate_enabled
          ? clamp(preview_curvature_m_inv /
                      config_.full_steering_rate_curvature_m_inv,
                  0.0, 1.0)
          : 1.0;
  const double smooth_rate_urgency =
      normalized_rate_urgency * normalized_rate_urgency *
      (3.0 - 2.0 * normalized_rate_urgency);
  result.applied_maximum_steering_rate_rad_per_sec =
      low_curvature_maximum_steering_rate_rad_per_sec +
      smooth_rate_urgency *
          (config_.maximum_steering_rate_rad_per_sec -
           low_curvature_maximum_steering_rate_rad_per_sec);
  const double configured_return_multiplier =
      adaptive_rate_enabled
          ? 1.0 + smooth_rate_urgency *
                      (config_.steering_return_rate_multiplier - 1.0)
          : config_.steering_return_rate_multiplier;
  const double rate_multiplier =
      returning_toward_center ? configured_return_multiplier : 1.0;
  const double maximum_change_rad =
      result.applied_maximum_steering_rate_rad_per_sec * rate_multiplier *
      dt_sec;
  result.steering_angle_rad =
      clamp(saturated_steering_angle_rad,
            previous_steering_angle_rad - maximum_change_rad,
            previous_steering_angle_rad + maximum_change_rad);
  result.steering_angle_rad =
      clamp(result.steering_angle_rad,
            -config_.maximum_steering_angle_rad,
            config_.maximum_steering_angle_rad);

  if (!std::isfinite(result.cg_lateral_velocity_mps) ||
      !std::isfinite(result.measured_sideslip_angle_rad) ||
      !std::isfinite(
          result.corrected_pure_pursuit_steering_angle_rad) ||
      !std::isfinite(result.effective_pure_pursuit_weight) ||
      !std::isfinite(result.effective_stanley_weight) ||
      !std::isfinite(result.cross_track_recovery_weight) ||
      !std::isfinite(
          result
              .cross_track_recovery_heading_suppression_weight) ||
      !std::isfinite(result.lane_clearance_recovery_urgency) ||
      !std::isfinite(result.curve_preview_stanley_recovery_weight) ||
      !std::isfinite(result.heading_lag_stanley_recovery_weight) ||
      !std::isfinite(
          result.applied_maximum_steering_rate_rad_per_sec) ||
      !std::isfinite(result.requested_steering_angle_rad) ||
      !std::isfinite(result.steering_angle_rad)) {
    result.error = "hybrid calculation produced a non-finite result";
    return result;
  }
  result.valid = true;
  return result;
}

}  // namespace morai_path_tracking
