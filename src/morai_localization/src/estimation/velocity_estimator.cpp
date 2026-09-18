#include "morai_localization/estimation/velocity_estimator.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace morai_localization {
namespace {

void requireFinitePositive(const char* name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) +
                                " must be finite and positive");
  }
}

}  // namespace

VelocityEstimator::VelocityEstimator(const VelocityEstimatorConfig& config)
    : config_(config) {
  requireFinitePositive("minimum_dt_sec", config_.minimum_dt_sec);
  requireFinitePositive("maximum_dt_sec", config_.maximum_dt_sec);
  requireFinitePositive("maximum_speed_mps", config_.maximum_speed_mps);
  if (config_.maximum_dt_sec <= config_.minimum_dt_sec) {
    throw std::invalid_argument(
        "maximum_dt_sec must exceed minimum_dt_sec");
  }
  if (!std::isfinite(config_.filter_time_constant_sec) ||
      config_.filter_time_constant_sec < 0.0) {
    throw std::invalid_argument(
        "filter_time_constant_sec must be finite and non-negative");
  }
}

VelocityEstimate VelocityEstimator::update(double x_m, double y_m,
                                           double stamp_sec, double yaw_rad) {
  if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(stamp_sec) || !std::isfinite(yaw_rad)) {
    clearFilter();
    return {};
  }

  if (!has_previous_sample_) {
    setBaseline(x_m, y_m, stamp_sec);
    clearFilter();
    return {};
  }

  const double dt = stamp_sec - previous_stamp_sec_;
  if (!std::isfinite(dt) || dt < config_.minimum_dt_sec ||
      dt > config_.maximum_dt_sec) {
    setBaseline(x_m, y_m, stamp_sec);
    clearFilter();
    return {};
  }

  const double vx_map = (x_m - previous_x_m_) / dt;
  const double vy_map = (y_m - previous_y_m_) / dt;
  const double cosine = std::cos(yaw_rad);
  const double sine = std::sin(yaw_rad);
  const double raw_longitudinal = cosine * vx_map + sine * vy_map;
  const double raw_lateral = -sine * vx_map + cosine * vy_map;

  if (!std::isfinite(vx_map) || !std::isfinite(vy_map) ||
      !std::isfinite(raw_longitudinal) || !std::isfinite(raw_lateral) ||
      std::hypot(vx_map, vy_map) > config_.maximum_speed_mps) {
    setBaseline(x_m, y_m, stamp_sec);
    clearFilter();
    return {};
  }

  const double alpha =
      config_.filter_time_constant_sec == 0.0
          ? 1.0
          : dt / (config_.filter_time_constant_sec + dt);
  if (!has_filtered_velocity_) {
    filtered_longitudinal_mps_ = raw_longitudinal;
    filtered_lateral_mps_ = raw_lateral;
    has_filtered_velocity_ = true;
  } else {
    filtered_longitudinal_mps_ +=
        alpha * (raw_longitudinal - filtered_longitudinal_mps_);
    filtered_lateral_mps_ += alpha * (raw_lateral - filtered_lateral_mps_);
  }
  setBaseline(x_m, y_m, stamp_sec);

  return {true, filtered_longitudinal_mps_, filtered_lateral_mps_};
}

void VelocityEstimator::reset() {
  has_previous_sample_ = false;
  clearFilter();
}

void VelocityEstimator::setBaseline(double x_m, double y_m, double stamp_sec) {
  previous_x_m_ = x_m;
  previous_y_m_ = y_m;
  previous_stamp_sec_ = stamp_sec;
  has_previous_sample_ = true;
}

void VelocityEstimator::clearFilter() {
  has_filtered_velocity_ = false;
  filtered_longitudinal_mps_ = 0.0;
  filtered_lateral_mps_ = 0.0;
}

}  // namespace morai_localization
