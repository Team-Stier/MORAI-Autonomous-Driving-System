#include "morai_path_tracking/controllers/lateral/mpc/mpc_lateral_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

namespace morai_path_tracking {
namespace {

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double signedCurvature(const morai_mpc::Point2d& before,
                       const morai_mpc::Point2d& point,
                       const morai_mpc::Point2d& after) {
  const double ab = std::hypot(point.x - before.x, point.y - before.y);
  const double bc = std::hypot(after.x - point.x, after.y - point.y);
  const double ac = std::hypot(after.x - before.x, after.y - before.y);
  const double denominator = ab * bc * ac;
  if (!std::isfinite(denominator) || denominator <= 1.0e-9) {
    return 0.0;
  }
  const double cross = (point.x - before.x) * (after.y - before.y) -
                       (point.y - before.y) * (after.x - before.x);
  return 2.0 * cross / denominator;
}

}  // namespace

SweptPathCompensationResult compensateSweptWheelPath(
    const std::vector<morai_mpc::Point2d>& path,
    const morai_mpc::MpcConfig& config) {
  SweptPathCompensationResult result;
  if (path.size() < 3U || !morai_mpc::valid(config)) {
    result.error = "INVALID_SWEPT_PATH_INPUT";
    return result;
  }
  result.path = path;
  if (!config.swept_path_compensation_enabled ||
      config.swept_path_gain <= 0.0 ||
      config.swept_path_maximum_offset <= 0.0) {
    result.valid = true;
    return result;
  }

  std::vector<double> raw_curvature(path.size(), 0.0);
  for (std::size_t index = 0U; index < path.size(); ++index) {
    const std::size_t before = index == 0U ? 0U : index - 1U;
    const std::size_t after =
        index + 1U < path.size() ? index + 1U : path.size() - 1U;
    if (before == index && after + 1U < path.size()) {
      raw_curvature[index] =
          signedCurvature(path[index], path[after], path[after + 1U]);
    } else if (after == index && before > 0U) {
      raw_curvature[index] =
          signedCurvature(path[before - 1U], path[before], path[index]);
    } else {
      raw_curvature[index] =
          signedCurvature(path[before], path[index], path[after]);
    }
  }

  const int half_window = config.swept_path_curvature_smoothing_window / 2;
  std::vector<double> curvature(path.size(), 0.0);
  for (std::size_t index = 0U; index < path.size(); ++index) {
    const std::size_t first =
        index > static_cast<std::size_t>(half_window)
            ? index - static_cast<std::size_t>(half_window)
            : 0U;
    const std::size_t last = std::min(
        path.size() - 1U, index + static_cast<std::size_t>(half_window));
    for (std::size_t sample = first; sample <= last; ++sample) {
      curvature[index] += raw_curvature[sample];
    }
    curvature[index] /= static_cast<double>(last - first + 1U);
  }

  for (std::size_t index = 0U; index < path.size(); ++index) {
    const std::size_t before = index == 0U ? 0U : index - 1U;
    const std::size_t after =
        index + 1U < path.size() ? index + 1U : path.size() - 1U;
    const double tangent_x = path[after].x - path[before].x;
    const double tangent_y = path[after].y - path[before].y;
    const double tangent_norm = std::hypot(tangent_x, tangent_y);
    if (!std::isfinite(tangent_norm) || tangent_norm <= 1.0e-9) {
      result.error = "DEGENERATE_SWEPT_PATH_TANGENT";
      result.path.clear();
      return result;
    }
    const double absolute_curvature = std::abs(curvature[index]);
    const double activation = clamp01(
        (absolute_curvature -
         config.swept_path_activation_start_curvature) /
        (config.swept_path_activation_full_curvature -
         config.swept_path_activation_start_curvature));
    const double smooth_activation =
        activation * activation * (3.0 - 2.0 * activation);
    const double geometric_offset =
        config.wheelbase * config.wheelbase * absolute_curvature /
        (4.0 + 2.0 * config.swept_path_vehicle_width * absolute_curvature);
    const double magnitude = std::min(
        config.swept_path_maximum_offset,
        config.swept_path_gain * smooth_activation * geometric_offset);
    const double signed_offset = std::copysign(magnitude, curvature[index]);
    result.path[index].x += -tangent_y / tangent_norm * signed_offset;
    result.path[index].y += tangent_x / tangent_norm * signed_offset;
    result.maximum_offset_m = std::max(result.maximum_offset_m, magnitude);
  }
  result.valid = true;
  return result;
}

MpcLateralController::MpcLateralController(
    const morai_mpc::MpcConfig& config)
    : config_(config),
      controller_(new morai_mpc::MpcController(config_)) {
  if (!std::isfinite(config_.resample_ds) || config_.resample_ds <= 0.0) {
    throw std::invalid_argument("mpc_resample_distance_m must be positive");
  }
}

MpcLateralResult MpcLateralController::calculate(
    const std::vector<morai_mpc::Point2d>& vehicle_path,
    double speed_mps, double measured_yaw_rate_radps,
    double previous_steering_angle_rad,
    double control_dt_sec) {
  return calculate(vehicle_path, speed_mps, 0.0, measured_yaw_rate_radps,
                   previous_steering_angle_rad, control_dt_sec);
}

MpcLateralResult MpcLateralController::calculate(
    const std::vector<morai_mpc::Point2d>& vehicle_path,
    double speed_mps, double measured_lateral_velocity_mps,
    double measured_yaw_rate_radps, double previous_steering_angle_rad,
    double control_dt_sec) {
  MpcLateralResult result;
  if (vehicle_path.size() < 2U || !std::isfinite(speed_mps) ||
      !std::isfinite(measured_lateral_velocity_mps) ||
      !std::isfinite(measured_yaw_rate_radps) || !std::isfinite(previous_steering_angle_rad) ||
      !std::isfinite(control_dt_sec) || control_dt_sec <= 0.0) {
    result.error = "INVALID_MPC_INPUT";
    return result;
  }

  try {
    std::vector<morai_mpc::Point2d> deduplicated;
    deduplicated.reserve(vehicle_path.size());
    for (const morai_mpc::Point2d& point : vehicle_path) {
      if (!morai_mpc::valid(point)) {
        result.error = "INVALID_MPC_PATH_POINT";
        return result;
      }
      if (deduplicated.empty() ||
          std::hypot(point.x - deduplicated.back().x,
                     point.y - deduplicated.back().y) > 1.0e-6) {
        deduplicated.push_back(point);
      }
    }
    if (deduplicated.size() < 2U) {
      result.error = "INSUFFICIENT_MPC_PATH_POINTS";
      return result;
    }

    const SweptPathCompensationResult compensated =
        compensateSweptWheelPath(deduplicated, config_);
    if (!compensated.valid) {
      result.error = compensated.error;
      return result;
    }
    result.compensated_path = compensated.path;
    result.swept_path_maximum_offset_m = compensated.maximum_offset_m;
    morai_mpc::ReferencePath path(
        compensated.path, false, morai_mpc::CoordinateFrame::Vehicle);
    if (path.length() > config_.resample_ds) {
      path = path.resample(config_.resample_ds);
    }
    const morai_mpc::Projection initial_projection =
        path.project({0.0, 0.0, 0.0});
    double preview_curvature = 0.0;
    if (initial_projection.valid) {
      const int preview_steps = morai_mpc::effectiveHorizon(
          config_, std::abs(speed_mps));
      for (int step = 0; step < preview_steps; ++step) {
        preview_curvature = std::max(
            preview_curvature,
            std::abs(path.sample(
                initial_projection.s + static_cast<double>(step) *
                                           std::abs(speed_mps) * config_.dt)
                         .curvature));
      }
    }
    const double raw_curve_steering_estimate_blend = morai_mpc::clamp(
        (preview_curvature -
         config_
             .curve_yaw_rate_steering_estimate_activation_start_curvature) /
            (config_
                 .curve_yaw_rate_steering_estimate_activation_full_curvature -
             config_
                 .curve_yaw_rate_steering_estimate_activation_start_curvature),
        0.0, 1.0);
    const double smooth_curve_steering_estimate_blend =
        raw_curve_steering_estimate_blend *
        raw_curve_steering_estimate_blend *
        (3.0 - 2.0 * raw_curve_steering_estimate_blend);
    if (config_.curve_yaw_rate_steering_estimate_blend_time_constant > 0.0) {
      const double alpha = 1.0 - std::exp(
          -control_dt_sec /
          config_.curve_yaw_rate_steering_estimate_blend_time_constant);
      filtered_curve_steering_estimate_blend_ +=
          alpha * (smooth_curve_steering_estimate_blend -
                   filtered_curve_steering_estimate_blend_);
    } else {
      filtered_curve_steering_estimate_blend_ =
          smooth_curve_steering_estimate_blend;
    }
    result.yaw_rate_steering_estimate_blend =
        filtered_curve_steering_estimate_blend_;
    double modeled_steering_angle_rad = previous_steering_angle_rad;
    const double steering_estimate_minimum_speed = morai_mpc::lerp(
        config_.straight_yaw_rate_steering_estimate_minimum_speed,
        config_.curve_yaw_rate_steering_estimate_minimum_speed,
        filtered_curve_steering_estimate_blend_);
    const double steering_estimate_gain = morai_mpc::lerp(
        config_.straight_yaw_rate_steering_estimate_gain,
        config_.curve_yaw_rate_steering_estimate_gain,
        filtered_curve_steering_estimate_blend_);
    if (steering_estimate_gain > 0.0 &&
        std::abs(speed_mps) >= steering_estimate_minimum_speed) {
      const double raw_yaw_rate_steering_estimate = morai_mpc::clamp(
          std::atan(
          config_.wheelbase * measured_yaw_rate_radps /
          std::max(0.1, std::abs(speed_mps))),
          -config_.max_steering, config_.max_steering);
      if (!has_filtered_yaw_rate_steering_estimate_) {
        filtered_yaw_rate_steering_estimate_rad_ =
            raw_yaw_rate_steering_estimate;
        has_filtered_yaw_rate_steering_estimate_ = true;
      } else if (
          config_.straight_yaw_rate_steering_estimate_filter_time_constant >
          0.0) {
        const double alpha = 1.0 - std::exp(
            -control_dt_sec /
            config_
                .straight_yaw_rate_steering_estimate_filter_time_constant);
        filtered_yaw_rate_steering_estimate_rad_ +=
            alpha * (raw_yaw_rate_steering_estimate -
                     filtered_yaw_rate_steering_estimate_rad_);
      } else {
        filtered_yaw_rate_steering_estimate_rad_ =
            raw_yaw_rate_steering_estimate;
      }
      // The curve model needs the freshest observation for entry response;
      // the straight model uses the filtered observation to reject IMU noise.
      const double observed_steering_angle_rad = morai_mpc::lerp(
          filtered_yaw_rate_steering_estimate_rad_,
          raw_yaw_rate_steering_estimate,
          filtered_curve_steering_estimate_blend_);
      modeled_steering_angle_rad = morai_mpc::lerp(
          previous_steering_angle_rad,
          observed_steering_angle_rad, steering_estimate_gain);
    }
    result.modeled_steering_angle_rad = modeled_steering_angle_rad;
    const morai_mpc::VehicleState vehicle{
        0.0,
        0.0,
        0.0,
        std::abs(speed_mps),
        morai_mpc::clamp(modeled_steering_angle_rad,
                         -config_.max_steering, config_.max_steering),
        measured_lateral_velocity_mps,
        measured_yaw_rate_radps};
    const morai_mpc::MpcCommand command =
        controller_->compute(vehicle, path, control_dt_sec);
    result.solver_iterations = command.iterations;
    result.solver_time_ms = command.solve_time_ms;
    result.solver_cost = command.cost;
    result.raw_steering_rate_rad_per_sec = command.raw_steering_rate;

    const morai_mpc::Projection& projection = controller_->lastProjection();
    if (projection.valid) {
      result.cross_track_error_m = projection.lateral_error;
      result.raw_cross_track_error_m = controller_->lastRawLateralError();
      result.heading_error_rad = projection.heading_error;
      result.reference_curvature_m_inv = projection.reference.curvature;
      result.projection = projection.point;
    }
    if (!command.success || !projection.valid ||
        !std::isfinite(command.steering_rate)) {
      result.error = command.message.empty() ? "MPC_SOLVER_FAILED"
                                             : command.message;
      return result;
    }

    const double reference_yaw_rate =
        std::abs(speed_mps) * projection.reference.curvature;
    const double yaw_rate_error = measured_yaw_rate_radps - reference_yaw_rate;
    const double damping_fade = 1.0 - morai_mpc::clamp(
        std::abs(projection.reference.curvature) /
            config_.yaw_rate_damping_fade_curvature,
        0.0, 1.0);
    result.steering_rate_rad_per_sec = morai_mpc::clamp(
        command.steering_rate - config_.yaw_rate_damping_gain * damping_fade *
                                    yaw_rate_error,
        -config_.max_steering_rate, config_.max_steering_rate);

    result.steering_angle_rad = morai_mpc::clamp(
        previous_steering_angle_rad + result.steering_rate_rad_per_sec * control_dt_sec,
        -config_.max_steering, config_.max_steering);
    result.valid = std::isfinite(result.steering_angle_rad);
    result.error = result.valid ? "" : "INVALID_MPC_STEERING_ANGLE";
    return result;
  } catch (const std::exception& error) {
    result.error = std::string("MPC_EXCEPTION: ") + error.what();
    return result;
  }
}

void MpcLateralController::reset() {
  controller_.reset(new morai_mpc::MpcController(config_));
  has_filtered_yaw_rate_steering_estimate_ = false;
  filtered_yaw_rate_steering_estimate_rad_ = 0.0;
  filtered_curve_steering_estimate_blend_ = 0.0;
}

}  // namespace morai_path_tracking
