#include "morai_path_tracking/controllers/lateral/imm_two_model_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace morai_path_tracking {
namespace {

constexpr double kProbabilityTolerance = 1.0e-9;
constexpr double kMinimumDeterminant = 1.0e-18;
constexpr double kMinimumCovariance = 1.0e-15;

bool finiteNonNegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool positiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool probability(double value) {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

std::string configurationError(const ImmConfig& config) {
  if (!positiveFinite(config.mass_kg)) {
    return "mass_kg must be finite and positive";
  }
  if (!positiveFinite(config.yaw_inertia_kgm2)) {
    return "yaw_inertia_kgm2 must be finite and positive";
  }
  if (!positiveFinite(config.front_cornering_stiffness_n_per_rad) ||
      !positiveFinite(config.rear_cornering_stiffness_n_per_rad)) {
    return "cornering stiffness values must be finite and positive";
  }
  if (!positiveFinite(config.front_axle_to_cg_m) ||
      !positiveFinite(config.rear_axle_to_cg_m)) {
    return "axle-to-CG distances must be finite and positive";
  }
  if (!finiteNonNegative(config.process_noise_sideslip) ||
      !finiteNonNegative(config.process_noise_yaw_rate)) {
    return "process noise values must be finite and non-negative";
  }
  if (!positiveFinite(config.measurement_noise_sideslip) ||
      !positiveFinite(config.measurement_noise_yaw_rate)) {
    return "measurement noise values must be finite and positive";
  }
  if (!positiveFinite(config.initial_covariance_sideslip) ||
      !positiveFinite(config.initial_covariance_yaw_rate)) {
    return "initial covariance values must be finite and positive";
  }
  if (!probability(config.initial_pure_pursuit_probability) ||
      !probability(config.initial_stanley_probability) ||
      std::abs(config.initial_pure_pursuit_probability +
                   config.initial_stanley_probability -
               1.0) > kProbabilityTolerance) {
    return "initial probabilities must be finite, in [0,1], and sum to one";
  }
  if (!probability(config.stanley_probability_min) ||
      !probability(config.stanley_probability_max) ||
      config.stanley_probability_min > config.stanley_probability_max) {
    return "Stanley probability bounds must be ordered values in [0,1]";
  }

  const double transitions[] = {
      config.transition_pure_pursuit_to_pure_pursuit,
      config.transition_pure_pursuit_to_stanley,
      config.transition_stanley_to_pure_pursuit,
      config.transition_stanley_to_stanley,
  };
  for (const double transition : transitions) {
    if (!probability(transition)) {
      return "transition probabilities must be finite values in [0,1]";
    }
  }
  if (std::abs(config.transition_pure_pursuit_to_pure_pursuit +
                   config.transition_pure_pursuit_to_stanley -
               1.0) > kProbabilityTolerance ||
      std::abs(config.transition_stanley_to_pure_pursuit +
                   config.transition_stanley_to_stanley -
               1.0) > kProbabilityTolerance) {
    return "each transition row must sum to one";
  }
  if (!finiteNonNegative(config.transition_speed_gain) ||
      config.transition_speed_gain >
          config.transition_pure_pursuit_to_pure_pursuit ||
      config.transition_speed_gain >
          config.transition_stanley_to_pure_pursuit ||
      config.transition_pure_pursuit_to_stanley +
              config.transition_speed_gain >
          1.0 ||
      config.transition_stanley_to_stanley +
              config.transition_speed_gain >
          1.0) {
    return "transition_speed_gain produces an invalid transition matrix";
  }
  if (!positiveFinite(config.transition_reference_speed_mps)) {
    return "transition_reference_speed_mps must be finite and positive";
  }
  if (!positiveFinite(config.minimum_model_speed_mps)) {
    return "minimum_model_speed_mps must be finite and positive";
  }
  return {};
}

ImmResult invalidResult(const std::string& error) {
  ImmResult result;
  result.error = error;
  return result;
}

}  // namespace

ImmTwoModelFilter::ImmTwoModelFilter(const ImmConfig& config)
    : config_(config), config_error_(configurationError(config)) {
  reset();
}

void ImmTwoModelFilter::reset() {
  for (Branch& branch : branches_) {
    branch.state = {};
    branch.covariance = {
        config_.initial_covariance_sideslip,
        0.0,
        0.0,
        config_.initial_covariance_yaw_rate,
    };
  }
  probabilities_[0] = config_.initial_pure_pursuit_probability;
  probabilities_[1] = config_.initial_stanley_probability;
}

ImmResult ImmTwoModelFilter::update(
    double longitudinal_speed_mps, double measured_sideslip_rad,
    double measured_yaw_rate_radps, double pure_pursuit_steering_rad,
    double stanley_steering_rad, double dt_sec) {
  if (!config_error_.empty()) {
    return invalidResult("invalid IMM configuration: " + config_error_);
  }
  if (!finiteNonNegative(longitudinal_speed_mps)) {
    return invalidResult(
        "longitudinal_speed_mps must be finite and non-negative");
  }
  if (!std::isfinite(measured_sideslip_rad) ||
      !std::isfinite(measured_yaw_rate_radps) ||
      !std::isfinite(pure_pursuit_steering_rad) ||
      !std::isfinite(stanley_steering_rad)) {
    return invalidResult("IMM measurements and steering inputs must be finite");
  }
  if (!positiveFinite(dt_sec)) {
    return invalidResult("dt_sec must be finite and positive");
  }

  const double speed_mps =
      std::max(longitudinal_speed_mps, config_.minimum_model_speed_mps);
  const double speed_ratio =
      clamp(longitudinal_speed_mps /
                config_.transition_reference_speed_mps,
            0.0, 1.0);
  const double transition_shift =
      config_.transition_speed_gain * speed_ratio;
  const double transition[2][2] = {
      {
          config_.transition_pure_pursuit_to_pure_pursuit -
              transition_shift,
          config_.transition_pure_pursuit_to_stanley +
              transition_shift,
      },
      {
          config_.transition_stanley_to_pure_pursuit -
              transition_shift,
          config_.transition_stanley_to_stanley +
              transition_shift,
      },
  };

  std::array<double, 2> mixed_probabilities{};
  std::array<Branch, 2> mixed_branches{};
  for (std::size_t destination = 0U; destination < 2U; ++destination) {
    for (std::size_t source = 0U; source < 2U; ++source) {
      mixed_probabilities[destination] +=
          probabilities_[source] * transition[source][destination];
    }
    if (!positiveFinite(mixed_probabilities[destination])) {
      return invalidResult("IMM mixed probability is not positive");
    }

    double mixing_weights[2]{};
    for (std::size_t source = 0U; source < 2U; ++source) {
      mixing_weights[source] =
          probabilities_[source] * transition[source][destination] /
          mixed_probabilities[destination];
      mixed_branches[destination].state.sideslip_rad +=
          mixing_weights[source] * branches_[source].state.sideslip_rad;
      mixed_branches[destination].state.yaw_rate_radps +=
          mixing_weights[source] * branches_[source].state.yaw_rate_radps;
    }

    Matrix2 covariance{};
    for (std::size_t source = 0U; source < 2U; ++source) {
      const double d_beta =
          branches_[source].state.sideslip_rad -
          mixed_branches[destination].state.sideslip_rad;
      const double d_yaw_rate =
          branches_[source].state.yaw_rate_radps -
          mixed_branches[destination].state.yaw_rate_radps;
      const double weight = mixing_weights[source];
      covariance.m00 +=
          weight *
          (branches_[source].covariance.m00 + d_beta * d_beta);
      covariance.m01 +=
          weight *
          (branches_[source].covariance.m01 + d_beta * d_yaw_rate);
      covariance.m10 +=
          weight *
          (branches_[source].covariance.m10 + d_yaw_rate * d_beta);
      covariance.m11 +=
          weight *
          (branches_[source].covariance.m11 +
           d_yaw_rate * d_yaw_rate);
    }
    mixed_branches[destination].covariance = covariance;
  }

  const double mass = config_.mass_kg;
  const double inertia = config_.yaw_inertia_kgm2;
  const double front_stiffness =
      config_.front_cornering_stiffness_n_per_rad;
  const double rear_stiffness =
      config_.rear_cornering_stiffness_n_per_rad;
  const double front_distance = config_.front_axle_to_cg_m;
  const double rear_distance = config_.rear_axle_to_cg_m;
  const double speed_squared = speed_mps * speed_mps;

  const double a00 =
      -2.0 * (front_stiffness + rear_stiffness) /
      (mass * speed_mps);
  const double a01 =
      -1.0 -
      2.0 *
          (front_stiffness * front_distance -
           rear_stiffness * rear_distance) /
          (mass * speed_squared);
  const double a10 =
      -2.0 *
      (front_stiffness * front_distance -
       rear_stiffness * rear_distance) /
      inertia;
  const double a11 =
      -2.0 *
      (front_stiffness * front_distance * front_distance +
       rear_stiffness * rear_distance * rear_distance) /
      (inertia * speed_mps);
  const double b0 = 2.0 * front_stiffness / (mass * speed_mps);
  const double b1 =
      2.0 * front_stiffness * front_distance / inertia;
  const Matrix2 transition_matrix{
      1.0 + dt_sec * a00,
      dt_sec * a01,
      dt_sec * a10,
      1.0 + dt_sec * a11,
  };

  const double steering_inputs[2] = {
      pure_pursuit_steering_rad,
      stanley_steering_rad,
  };
  std::array<Branch, 2> updated_branches{};
  std::array<double, 2> innovation_norms{};
  std::array<double, 2> log_likelihoods{};

  for (std::size_t model = 0U; model < 2U; ++model) {
    const Branch& mixed = mixed_branches[model];
    ImmModelState predicted_state;
    predicted_state.sideslip_rad =
        transition_matrix.m00 * mixed.state.sideslip_rad +
        transition_matrix.m01 * mixed.state.yaw_rate_radps +
        dt_sec * b0 * steering_inputs[model];
    predicted_state.yaw_rate_radps =
        transition_matrix.m10 * mixed.state.sideslip_rad +
        transition_matrix.m11 * mixed.state.yaw_rate_radps +
        dt_sec * b1 * steering_inputs[model];

    const Matrix2& p = mixed.covariance;
    const Matrix2 fp{
        transition_matrix.m00 * p.m00 +
            transition_matrix.m01 * p.m10,
        transition_matrix.m00 * p.m01 +
            transition_matrix.m01 * p.m11,
        transition_matrix.m10 * p.m00 +
            transition_matrix.m11 * p.m10,
        transition_matrix.m10 * p.m01 +
            transition_matrix.m11 * p.m11,
    };
    Matrix2 predicted_covariance{
        fp.m00 * transition_matrix.m00 +
            fp.m01 * transition_matrix.m01 +
            config_.process_noise_sideslip * dt_sec,
        fp.m00 * transition_matrix.m10 +
            fp.m01 * transition_matrix.m11,
        fp.m10 * transition_matrix.m00 +
            fp.m11 * transition_matrix.m01,
        fp.m10 * transition_matrix.m10 +
            fp.m11 * transition_matrix.m11 +
            config_.process_noise_yaw_rate * dt_sec,
    };
    const double covariance_off_diagonal =
        0.5 * (predicted_covariance.m01 +
               predicted_covariance.m10);
    predicted_covariance.m01 = covariance_off_diagonal;
    predicted_covariance.m10 = covariance_off_diagonal;

    const Matrix2 innovation_covariance{
        predicted_covariance.m00 +
            config_.measurement_noise_sideslip,
        predicted_covariance.m01,
        predicted_covariance.m10,
        predicted_covariance.m11 +
            config_.measurement_noise_yaw_rate,
    };
    const double determinant =
        innovation_covariance.m00 * innovation_covariance.m11 -
        innovation_covariance.m01 * innovation_covariance.m10;
    if (!std::isfinite(determinant) ||
        determinant <= kMinimumDeterminant) {
      return invalidResult(
          "IMM innovation covariance is not positive definite");
    }
    const Matrix2 inverse_innovation{
        innovation_covariance.m11 / determinant,
        -innovation_covariance.m01 / determinant,
        -innovation_covariance.m10 / determinant,
        innovation_covariance.m00 / determinant,
    };

    const double innovation_beta =
        measured_sideslip_rad - predicted_state.sideslip_rad;
    const double innovation_yaw_rate =
        measured_yaw_rate_radps - predicted_state.yaw_rate_radps;
    const double mahalanobis_squared =
        innovation_beta *
            (inverse_innovation.m00 * innovation_beta +
             inverse_innovation.m01 * innovation_yaw_rate) +
        innovation_yaw_rate *
            (inverse_innovation.m10 * innovation_beta +
             inverse_innovation.m11 * innovation_yaw_rate);
    if (!std::isfinite(mahalanobis_squared) ||
        mahalanobis_squared < -kProbabilityTolerance) {
      return invalidResult("IMM innovation norm is invalid");
    }
    innovation_norms[model] =
        std::sqrt(std::max(0.0, mahalanobis_squared));
    log_likelihoods[model] =
        -0.5 * (std::max(0.0, mahalanobis_squared) +
                std::log(determinant));

    const Matrix2 kalman_gain{
        predicted_covariance.m00 * inverse_innovation.m00 +
            predicted_covariance.m01 * inverse_innovation.m10,
        predicted_covariance.m00 * inverse_innovation.m01 +
            predicted_covariance.m01 * inverse_innovation.m11,
        predicted_covariance.m10 * inverse_innovation.m00 +
            predicted_covariance.m11 * inverse_innovation.m10,
        predicted_covariance.m10 * inverse_innovation.m01 +
            predicted_covariance.m11 * inverse_innovation.m11,
    };
    updated_branches[model].state.sideslip_rad =
        predicted_state.sideslip_rad +
        kalman_gain.m00 * innovation_beta +
        kalman_gain.m01 * innovation_yaw_rate;
    updated_branches[model].state.yaw_rate_radps =
        predicted_state.yaw_rate_radps +
        kalman_gain.m10 * innovation_beta +
        kalman_gain.m11 * innovation_yaw_rate;

    Matrix2 updated_covariance{
        predicted_covariance.m00 -
            (kalman_gain.m00 * predicted_covariance.m00 +
             kalman_gain.m01 * predicted_covariance.m10),
        predicted_covariance.m01 -
            (kalman_gain.m00 * predicted_covariance.m01 +
             kalman_gain.m01 * predicted_covariance.m11),
        predicted_covariance.m10 -
            (kalman_gain.m10 * predicted_covariance.m00 +
             kalman_gain.m11 * predicted_covariance.m10),
        predicted_covariance.m11 -
            (kalman_gain.m10 * predicted_covariance.m01 +
             kalman_gain.m11 * predicted_covariance.m11),
    };
    const double updated_off_diagonal =
        0.5 * (updated_covariance.m01 + updated_covariance.m10);
    updated_covariance.m00 =
        std::max(updated_covariance.m00, kMinimumCovariance);
    updated_covariance.m01 = updated_off_diagonal;
    updated_covariance.m10 = updated_off_diagonal;
    updated_covariance.m11 =
        std::max(updated_covariance.m11, kMinimumCovariance);
    updated_branches[model].covariance = updated_covariance;
  }

  std::array<double, 2> updated_probabilities{};
  const double weighted_log_likelihoods[2] = {
      std::log(mixed_probabilities[0]) + log_likelihoods[0],
      std::log(mixed_probabilities[1]) + log_likelihoods[1],
  };
  const double maximum_log_likelihood =
      std::max(weighted_log_likelihoods[0],
               weighted_log_likelihoods[1]);
  const double normalized_pp =
      std::exp(weighted_log_likelihoods[0] -
               maximum_log_likelihood);
  const double normalized_stanley =
      std::exp(weighted_log_likelihoods[1] -
               maximum_log_likelihood);
  const double normalization = normalized_pp + normalized_stanley;
  if (positiveFinite(normalization)) {
    updated_probabilities[1] =
        normalized_stanley / normalization;
  } else {
    const double mixed_normalization =
        mixed_probabilities[0] + mixed_probabilities[1];
    updated_probabilities[1] =
        mixed_probabilities[1] / mixed_normalization;
  }
  updated_probabilities[1] =
      clamp(updated_probabilities[1],
            config_.stanley_probability_min,
            config_.stanley_probability_max);
  updated_probabilities[0] = 1.0 - updated_probabilities[1];

  const auto finite_state = [](const ImmModelState& state) {
    return std::isfinite(state.sideslip_rad) &&
           std::isfinite(state.yaw_rate_radps);
  };
  if (!finite_state(updated_branches[0].state) ||
      !finite_state(updated_branches[1].state) ||
      !probability(updated_probabilities[0]) ||
      !probability(updated_probabilities[1]) ||
      !std::isfinite(innovation_norms[0]) ||
      !std::isfinite(innovation_norms[1])) {
    return invalidResult("IMM update produced a non-finite result");
  }

  branches_ = updated_branches;
  probabilities_ = updated_probabilities;

  ImmResult result;
  result.valid = true;
  result.pure_pursuit_probability = probabilities_[0];
  result.stanley_probability = probabilities_[1];
  result.pure_pursuit_innovation_norm = innovation_norms[0];
  result.stanley_innovation_norm = innovation_norms[1];
  result.pure_pursuit_state = branches_[0].state;
  result.stanley_state = branches_[1].state;
  return result;
}

}  // namespace morai_path_tracking
