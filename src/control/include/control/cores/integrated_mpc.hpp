#pragma once
#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
namespace control { namespace autoware_ai_mpc { namespace autoware {} namespace amathutils {} }
namespace integrated_ai_mpc { namespace amathutils=control::autoware_ai_mpc::amathutils; namespace autoware { using namespace control::autoware_ai_mpc::autoware; } } }

#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <osqp.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace control { namespace integrated_ai_mpc {
// Kept separate from the source PID header: no PID controller is imported.
enum class LongitudinalState { kAccel, kCoast, kBrake };
}}  // namespace control::integrated_ai_mpc


#include <limits>
#include <string>
#include <vector>


namespace control { namespace integrated_ai_mpc {

struct LongitudinalMpcConfig {
  std::string solver{"osqp"};
  int horizon_steps{30};
  int optimizer_iterations{10};
  int line_search_steps{8};
  double prediction_dt_sec{0.10};
  double gradient_epsilon{0.01};
  double gradient_step{0.08};
  double gradient_tolerance{1.0e-5};
  double coordinate_initial_step{0.20};
  double coordinate_minimum_step{0.01};
  double command_step_decay{0.5};
  double maximum_accel_command{0.70};
  double maximum_brake_command{0.60};
  double command_rate_limit_per_sec{2.0};
  double acceleration_gain_mps2{3.0};
  double braking_gain_mps2{6.0};
  double actuator_time_constant_sec{0.25};
  bool physical_rollout{false};
  double constant_drag_mps2{0.0};
  double quadratic_drag_per_m{0.0};
  double maximum_acceleration_mps2{2.5};
  double maximum_deceleration_mps2{5.0};
  double maximum_jerk_mps3{3.0};
  double acceleration_estimate_filter_time_constant_sec{0.20};
  double speed_weight{8.0};
  double acceleration_weight{0.20};
  double jerk_weight{1.2};
  double command_weight{0.08};
  double command_rate_weight{2.5};
  double terminal_speed_weight{24.0};
  double stop_position_weight{1000.0};
};

struct LongitudinalReference {double speed_mps=0,acceleration_mps2=0;};
struct LongitudinalMpcResult {
  double accel{0.0};
  double brake{0.0};
  double signed_command{0.0};
  double estimated_acceleration_mps2{0.0};
  double predicted_maximum_speed_mps{0.0};
  double cost{std::numeric_limits<double>::infinity()};
  int iterations{0};
  LongitudinalState state{LongitudinalState::kCoast};
};

class LongitudinalMpc {
 public:
  explicit LongitudinalMpc(const LongitudinalMpcConfig& config);
  LongitudinalMpcResult update(double target_speed_mps,
                               double measured_speed_mps, double dt_sec,
                               const std::vector<double>& stations={},const std::vector<double>& speeds={});
  void reset();
  void setTimedReference(std::vector<LongitudinalReference> reference) {timed_reference_=std::move(reference);}


 private:
  struct RolloutResult {
    double cost{std::numeric_limits<double>::infinity()};
    double maximum_speed_mps{0.0};
  };

  void project(std::vector<double>* commands) const;
  RolloutResult rollout(const std::vector<double>& commands,
                        double target_speed_mps,
                        double measured_speed_mps) const;
  void solve(double target_speed_mps, double measured_speed_mps,
             std::vector<double>* commands, double* cost,
             double* predicted_maximum_speed_mps, int* iterations) const;

  std::vector<LongitudinalReference> timed_reference_;
  LongitudinalMpcConfig config_;
  std::vector<double> warm_start_;
  bool stop_release_pending_ = false;
  std::vector<double> reference_stations_,reference_speeds_;
  double reference_stop_station_=std::numeric_limits<double>::infinity();
  double referenceSpeed(double station,double fallback) const;
  bool has_previous_speed_{false};
  double previous_speed_mps_{0.0};
  double estimated_acceleration_mps2_{0.0};
  double previous_signed_command_{0.0};
};

}}  // namespace control::integrated_ai_mpc


namespace control { namespace integrated_ai_mpc {
// Operating values from MOLIT2026-stier1, including launch overrides.
// The supported lateral model/solver are kinematics + qpoases_hotstart.
struct LateralParameters {
  double ctrl_period = 1.0 / 30.0;
  bool enable_path_smoothing = false;
  bool enable_yaw_recalculation = true;
  int path_filter_moving_ave_num = 35;
  int path_smoothing_times = 1;
  int curvature_smoothing_num = 14;
  int morai_yaw_smoothing_num = 14;
  double traj_resample_dist = 0.25;
  double vehicle_model_wheelbase = 3.0;
  double steer_lim_deg = 40.0;
  double morai_steer_rate_limit_deg = 60.0;
  int mpc_prediction_horizon = 45;
  double mpc_prediction_sampling_time = 0.1;
  double mpc_weight_lat_error = 10.0;
  double mpc_weight_heading_error = 3.0;
  double mpc_weight_heading_error_squared_vel_coeff = 0.3;
  double mpc_weight_steering_input = 1.0;
  double mpc_weight_steering_input_squared_vel_coeff = 1.0;
  double mpc_weight_lat_jerk = 1.0;
  double mpc_weight_terminal_lat_error = 10.0;
  double mpc_weight_terminal_heading_error = 0.1;
  double mpc_zero_ff_steer_deg = 0.0;
  double steering_lpf_cutoff_hz = 3.0;
  int qpoases_max_iter = 500;
  double morai_min_prediction_length_m = 20.0;
  int morai_max_prediction_horizon = 90;
  bool morai_use_output_steering_filter = false;
  double morai_straight_lat_weight_scale = 0.2;
  double morai_curve_steer_weight_scale = 0.25;
  double morai_weight_curvature_low = 0.005;
  double morai_weight_curvature_high = 0.03;
  double morai_weight_first_steer_change = 2.0;
  double morai_straight_steer_change_scale = 1.0;
  double morai_weight_front_point_error = 0.0;
  double morai_front_overhang_m = 0.845;
};

// Shared by the MPC model and command-history predictor, as in the source adapter.
struct SteeringPredictionParameters {
  double delay_sec = 1.0 / 6.0;
  double time_constant_sec = 0.05;
  double gain = 0.77;
};

struct Parameters {
  bool dynamic_zone_enabled = false;
  double dynamic_zone_min_x_m = -75;
  double dynamic_zone_max_x_m = -40;
  double dynamic_zone_min_y_m = -100;
  double dynamic_zone_max_y_m = 85;
  double dynamic_zone_speed_mps = 40.0/3.6;
  double dynamic_zone_approach_m = 30;

  LateralParameters lateral;
  LongitudinalMpcConfig longitudinal;
  SteeringPredictionParameters steering_prediction;
  double reference_history_length_m = 4.0;
  double minimum_control_dt_sec = 0.005;
  double maximum_control_dt_sec = 0.10;
  double stationary_hold_speed_mps = 0.1;
  double stationary_hold_brake = 0.3;
};

constexpr double kPi = 3.14159265358979323846;
// The Bridge contract is fixed independently of a lower optimizer steering limit.
constexpr double kBridgeSteeringLimitRad = 40.0 * kPi / 180.0;
}}  // namespace control::integrated_ai_mpc


#include <functional>
#include <string>
#include <vector>

namespace control { namespace integrated_ai_mpc {

struct OsqpSqpConfig {
  int sqp_iterations{2};
  int maximum_iterations{1000};
  double finite_difference_epsilon{0.01};
  double hessian_regularization{1.0e-4};
  double absolute_tolerance{1.0e-4};
  double relative_tolerance{1.0e-4};
  bool polish{true};
};

struct OsqpSqpResult {
  std::vector<double> solution;
  double cost{0.0};
  int iterations{0};
};

OsqpSqpResult solveWithOsqpSqp(
    const std::function<double(const std::vector<double>&)>& objective,
    const std::vector<double>& initial,
    const std::vector<double>& constraint_matrix, int constraint_rows,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds, const OsqpSqpConfig& config);
}}  // namespace control::integrated_ai_mpc

#include <deque>
#include <utility>

namespace control { namespace integrated_ai_mpc {
// A command-history estimate, never a measured wheel-angle report.
class SteeringPrediction {
 public:
  SteeringPrediction(double delay_sec, double time_constant_sec);
  double advance(double time_sec);
  void sent(double time_sec, double effective_steering_rad);
  void synchronize(double time_sec,double physical_steering_rad) {
    time_sec_=time_sec;angle_rad_=command_rad_=physical_steering_rad;events_.clear();
  }

 private:
  double delay_sec_;
  double time_constant_sec_;
  double time_sec_ = 0.0;
  double angle_rad_ = 0.0;
  double command_rad_ = 0.0;
  std::deque<std::pair<double, double>> events_;
};
}}  // namespace control::integrated_ai_mpc

#include <control/libraries/morai_mpc/autoware/mpc_trajectory.h>
#include <control/libraries/morai_mpc/autoware/lowpass_filter.h>
#include <control/libraries/morai_mpc/autoware/vehicle_model/vehicle_model_bicycle_kinematics.h>
#include <control/libraries/morai_mpc/autoware/qp_solver/qp_solver_qpoases.h>
#include <control/libraries/morai_mpc/autoware/lane.hpp>
#include <deque>
#include <memory>

namespace control { namespace integrated_ai_mpc {
class AutowareLateral {
 public:
  AutowareLateral(const LateralParameters& parameters,
                 const SteeringPredictionParameters& steering);
  void synchronize(double steering_rad) {
    std::fill(input_buffer_.begin(),input_buffer_.end(),steering_rad);queued_command_=false;
    for(int i=0;i<100;++i)lpf_steering_cmd_.filter(steering_rad);
  }
  void sent(double steering_rad) {
    if(input_buffer_.empty())return;
    if(queued_command_)input_buffer_.back()=steering_rad;
    else {input_buffer_.push_back(steering_rad);input_buffer_.pop_front();}
    queued_command_=false;
  }
  double calculate(const control::integrated_ai_mpc::autoware::Lane& path, double speed_mps,
                   double modeled_steering_rad, double previous_steering_rad);

 private:
  void prepareTrajectory(const control::integrated_ai_mpc::autoware::Lane& path, double speed_mps);
  LateralParameters p_;
  SteeringPredictionParameters steering_;
  autoware::MPCTrajectory ref_traj_;
  autoware::Butterworth2dFilter lpf_steering_cmd_;
  std::unique_ptr<autoware::KinematicsBicycleModel> vehicle_model_ptr_;
  std::unique_ptr<autoware::QPSolverQpoasesHotstart> qpsolver_ptr_;
  std::deque<double> input_buffer_;
  bool queued_command_=false;
  Eigen::Matrix2d front_point_weight_;
};
}}  // namespace control::integrated_ai_mpc

#include <algorithm>
#include <cmath>

namespace control { namespace integrated_ai_mpc { namespace autoware {

inline int moraiPredictionSteps(int base, int maximum, double dt, double speed, double metres) {
  if (metres==0.) return base;
  const double needed=std::ceil(metres/(std::max(.5,std::abs(speed))*dt))+1.;
  // Five-step blocks reduce repeated solver resizes as speed changes.
  return std::max(base,static_cast<int>(std::min(double(maximum),5.*std::ceil(needed/5.))));
}

}}}  // namespace control::integrated_ai_mpc::autoware

#include <algorithm>
#include <cmath>

namespace control { namespace integrated_ai_mpc { namespace autoware {

// A continuous transition avoids an abrupt controller switch near a threshold.
inline double moraiCurvatureWeightBlend(double curvature, double low, double high) {
  const double t = std::max(0., std::min(1., (std::abs(curvature)-low)/(high-low)));
  return t*t*(3.-2.*t);
}

}}}  // namespace control::integrated_ai_mpc::autoware


#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <osqp.h>

namespace control { namespace integrated_ai_mpc {
namespace {

struct CscStorage {
  std::vector<c_float> values;
  std::vector<c_int> rows;
  std::vector<c_int> columns;
};

inline CscStorage denseToCsc(const std::vector<double>& dense, int row_count,
                      int column_count) {
  CscStorage result;
  result.columns.reserve(static_cast<std::size_t>(column_count + 1));
  result.columns.push_back(0);
  for (int column = 0; column < column_count; ++column) {
    for (int row = 0; row < row_count; ++row) {
      const double value = dense[static_cast<std::size_t>(row * column_count + column)];
      if (std::abs(value) > 1.0e-14) {
        result.values.push_back(static_cast<c_float>(value));
        result.rows.push_back(static_cast<c_int>(row));
      }
    }
    result.columns.push_back(static_cast<c_int>(result.values.size()));
  }
  return result;
}

}  // namespace

inline OsqpSqpResult solveWithOsqpSqp(
    const std::function<double(const std::vector<double>&)>& objective,
    const std::vector<double>& initial,
    const std::vector<double>& constraint_matrix, int constraint_rows,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds, const OsqpSqpConfig& config) {
  OsqpSqpResult result;
  const int variables = static_cast<int>(initial.size());
  result.solution = initial;
  result.cost = objective(result.solution);
  const CscStorage a_storage =
      denseToCsc(constraint_matrix, constraint_rows, variables);
  for (int outer = 0; outer < config.sqp_iterations; ++outer) {
    std::vector<double> diagonal(static_cast<std::size_t>(variables), 0.0);
    std::vector<c_float> q(static_cast<std::size_t>(variables));
    std::vector<double> gradient(static_cast<std::size_t>(variables), 0.0);
    for (int index = 0; index < variables; ++index) {
      std::vector<double> plus = result.solution;
      std::vector<double> minus = result.solution;
      plus[static_cast<std::size_t>(index)] += config.finite_difference_epsilon;
      minus[static_cast<std::size_t>(index)] -= config.finite_difference_epsilon;
      const double plus_cost = objective(plus);
      const double minus_cost = objective(minus);
      gradient[static_cast<std::size_t>(index)] =
          (plus_cost - minus_cost) /
          (2.0 * config.finite_difference_epsilon);
      const double curvature = std::max(
          config.hessian_regularization,
          (plus_cost - 2.0 * result.cost + minus_cost) /
              (config.finite_difference_epsilon * config.finite_difference_epsilon));
      diagonal[static_cast<std::size_t>(index)] = curvature;
    }
    for (int index = 0; index < variables; ++index) {
      q[static_cast<std::size_t>(index)] = static_cast<c_float>(
          gradient[static_cast<std::size_t>(index)] -
          diagonal[static_cast<std::size_t>(index)] *
              result.solution[static_cast<std::size_t>(index)]);
    }

    std::vector<c_float> p_values(diagonal.begin(), diagonal.end());
    std::vector<c_int> p_rows(static_cast<std::size_t>(variables));
    std::vector<c_int> p_columns(static_cast<std::size_t>(variables + 1));
    for (int index = 0; index < variables; ++index) {
      p_rows[static_cast<std::size_t>(index)] = index;
      p_columns[static_cast<std::size_t>(index)] = index;
    }
    p_columns[static_cast<std::size_t>(variables)] = variables;
    std::vector<c_float> lower(lower_bounds.begin(), lower_bounds.end());
    std::vector<c_float> upper(upper_bounds.begin(), upper_bounds.end());
    csc* p_matrix = csc_matrix(variables, variables,
                               static_cast<c_int>(p_values.size()), p_values.data(),
                               p_rows.data(), p_columns.data());
    csc* a_matrix = csc_matrix(constraint_rows, variables,
                               static_cast<c_int>(a_storage.values.size()),
                               const_cast<c_float*>(a_storage.values.data()),
                               const_cast<c_int*>(a_storage.rows.data()),
                               const_cast<c_int*>(a_storage.columns.data()));
    OSQPData data;
    data.n = variables;
    data.m = constraint_rows;
    data.P = p_matrix;
    data.A = a_matrix;
    data.q = q.data();
    data.l = lower.data();
    data.u = upper.data();
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.max_iter = config.maximum_iterations;
    settings.eps_abs = config.absolute_tolerance;
    settings.eps_rel = config.relative_tolerance;
    settings.polish = config.polish;
    OSQPWorkspace* workspace = nullptr;
    osqp_setup(&workspace, &data, &settings);
    osqp_solve(workspace);
    result.iterations += static_cast<int>(workspace->info->iter);
    std::vector<double> candidate(static_cast<std::size_t>(variables));
    for (int index = 0; index < variables; ++index) {
      candidate[static_cast<std::size_t>(index)] = workspace->solution->x[index];
    }
    osqp_cleanup(workspace);
    c_free(p_matrix);
    c_free(a_matrix);
    double blend = 1.0;
    bool improved = false;
    for (int line_search = 0; line_search < 8; ++line_search) {
      std::vector<double> trial = result.solution;
      for (int index = 0; index < variables; ++index) {
        trial[static_cast<std::size_t>(index)] += blend *
            (candidate[static_cast<std::size_t>(index)] - trial[static_cast<std::size_t>(index)]);
      }
      const double trial_cost = objective(trial);
      if (std::isfinite(trial_cost) && trial_cost <= result.cost) {
        result.solution = trial;
        result.cost = trial_cost;
        improved = true;
        break;
      }
      blend *= 0.5;
    }
    if (!improved) break;
  }
  return result;
}

}}  // namespace control::integrated_ai_mpc


#include <algorithm>
#include <cmath>

namespace control { namespace integrated_ai_mpc {
namespace {

inline double clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}


}  // namespace

inline LongitudinalMpc::LongitudinalMpc(const LongitudinalMpcConfig& config)
    : config_(config),
      warm_start_(static_cast<std::size_t>(config.horizon_steps), 0.0) {
}

inline void LongitudinalMpc::project(std::vector<double>* commands) const {
  double previous = previous_signed_command_;
  const double maximum_step =
      config_.command_rate_limit_per_sec * config_.prediction_dt_sec;
  for (double& command : *commands) {
    command = clamp(command, -config_.maximum_brake_command,
                    config_.maximum_accel_command);
    command = clamp(command, previous - maximum_step,
                    previous + maximum_step);
    previous = command;
  }
}

inline LongitudinalMpc::RolloutResult LongitudinalMpc::rollout(
    const std::vector<double>& commands, double target_speed_mps,
    double measured_speed_mps) const {
  RolloutResult result;
  result.cost = 0.0;
  double speed = std::max(0.0, measured_speed_mps);
  double acceleration = estimated_acceleration_mps2_;
  double previous_command = previous_signed_command_;
  double station=0;
  result.maximum_speed_mps = speed;
  const double dt = config_.prediction_dt_sec;
  const double actuator_alpha =
      1.0 - std::exp(-dt / config_.actuator_time_constant_sec);
  std::size_t reference_index=0;
  for (double command : commands) {
    double requested_acceleration =
        command >= 0.0 ? config_.acceleration_gain_mps2 * command
                       : config_.braking_gain_mps2 * command;
    if (config_.physical_rollout) {
      requested_acceleration -= config_.constant_drag_mps2 +
          config_.quadratic_drag_per_m * speed * speed;
    }
    double next_acceleration =
        acceleration + actuator_alpha * (requested_acceleration - acceleration);
    if (!config_.physical_rollout) {
      next_acceleration =
        clamp(next_acceleration, -config_.maximum_deceleration_mps2,
              config_.maximum_acceleration_mps2);
    const double maximum_acceleration_change = config_.maximum_jerk_mps3 * dt;
    next_acceleration = clamp(next_acceleration,
                              acceleration - maximum_acceleration_change,
                              acceleration + maximum_acceleration_change);
    }  // Physical rollout must not silently clip the plant to comfort limits.
    const double previous_speed=speed;
    speed = std::max(0.0, speed + next_acceleration * dt);
    station+=(previous_speed+speed)*dt/2;
    result.maximum_speed_mps = std::max(result.maximum_speed_mps, speed);
    const double stop_overshoot=std::max(0.0,station-reference_stop_station_);
    result.cost+=config_.stop_position_weight*stop_overshoot*stop_overshoot;
    const auto reference=timed_reference_.empty()?LongitudinalReference{referenceSpeed(station,target_speed_mps),0}:
        timed_reference_[std::min(reference_index,timed_reference_.size()-1)];
    ++reference_index;
    const double speed_error = reference.speed_mps-speed;
    const double acceleration_error=next_acceleration-reference.acceleration_mps2;
    const double jerk = (next_acceleration - acceleration) / dt;
    const double command_change = command - previous_command;
    result.cost += config_.speed_weight * speed_error * speed_error +
                   config_.acceleration_weight * acceleration_error *
                       acceleration_error +
                   config_.jerk_weight * jerk * jerk +
                   config_.command_weight * command * command +
                   config_.command_rate_weight * command_change *
                       command_change;
    acceleration = next_acceleration;
    previous_command = command;
  }
  const double terminal_error=(timed_reference_.empty()?referenceSpeed(station,target_speed_mps):timed_reference_.back().speed_mps)-speed;
  result.cost += config_.terminal_speed_weight * terminal_error * terminal_error;
  return result;
}

inline void LongitudinalMpc::solve(double target_speed_mps, double measured_speed_mps,
                            std::vector<double>* commands, double* cost,
                            double* predicted_maximum_speed_mps,
                            int* iterations) const {
  project(commands);
  RolloutResult best = rollout(*commands, target_speed_mps, measured_speed_mps);
  *iterations = 0;
  if (config_.solver == "osqp") {
    const int horizon = static_cast<int>(commands->size());
    const int rows = 2 * horizon;
    std::vector<double> matrix(static_cast<std::size_t>(rows * horizon), 0.0);
    std::vector<double> lower(static_cast<std::size_t>(rows));
    std::vector<double> upper(static_cast<std::size_t>(rows));
    const double maximum_step = config_.command_rate_limit_per_sec *
                                config_.prediction_dt_sec;
    for (int index = 0; index < horizon; ++index) {
      matrix[static_cast<std::size_t>(index * horizon + index)] = 1.0;
      lower[static_cast<std::size_t>(index)] = -config_.maximum_brake_command;
      upper[static_cast<std::size_t>(index)] = config_.maximum_accel_command;
      const int rate_row = horizon + index;
      matrix[static_cast<std::size_t>(rate_row * horizon + index)] = 1.0;
      if (index == 0) {
        lower[static_cast<std::size_t>(rate_row)] =
            previous_signed_command_ - maximum_step;
        upper[static_cast<std::size_t>(rate_row)] =
            previous_signed_command_ + maximum_step;
      } else {
        matrix[static_cast<std::size_t>(rate_row * horizon + index - 1)] = -1.0;
        lower[static_cast<std::size_t>(rate_row)] = -maximum_step;
        upper[static_cast<std::size_t>(rate_row)] = maximum_step;
      }
    }
    OsqpSqpConfig solver_config;
    solver_config.sqp_iterations = std::min(3, config_.optimizer_iterations);
    solver_config.finite_difference_epsilon = config_.gradient_epsilon;
    const OsqpSqpResult solved = solveWithOsqpSqp(
        [&](const std::vector<double>& candidate) {
          return rollout(candidate, target_speed_mps, measured_speed_mps).cost;
        },
        *commands, matrix, rows, lower, upper, solver_config);
    *commands = solved.solution;
    best = rollout(*commands, target_speed_mps, measured_speed_mps);
    *iterations = solved.iterations;
  } else if (config_.solver == "coordinate_search") {
    double step = config_.coordinate_initial_step;
    for (int iteration = 0; iteration < config_.optimizer_iterations;
         ++iteration) {
      for (std::size_t index = 0; index < commands->size(); ++index) {
        std::vector<double> best_commands = *commands;
        for (double sign : {-1.0, 1.0}) {
          std::vector<double> candidate = *commands;
          candidate[index] += sign * step;
          project(&candidate);
          const RolloutResult candidate_result =
              rollout(candidate, target_speed_mps, measured_speed_mps);
          if (candidate_result.cost < best.cost) {
            best = candidate_result;
            best_commands = candidate;
          }
        }
        *commands = best_commands;
      }
      *iterations = iteration + 1;
      if (step <= config_.coordinate_minimum_step) break;
      step = std::max(config_.coordinate_minimum_step,
                      step * config_.command_step_decay);
    }
  } else {
    std::vector<double> gradient(commands->size(), 0.0);
    for (int iteration = 0; iteration < config_.optimizer_iterations;
         ++iteration) {
      double norm_squared = 0.0;
      for (std::size_t index = 0; index < commands->size(); ++index) {
        std::vector<double> plus = *commands;
        std::vector<double> minus = *commands;
        plus[index] += config_.gradient_epsilon;
        minus[index] -= config_.gradient_epsilon;
        project(&plus);
        project(&minus);
        const double denominator = plus[index] - minus[index];
        gradient[index] = 0.0;
        if (std::abs(denominator) > 1.0e-12) {
          const double plus_cost =
              rollout(plus, target_speed_mps, measured_speed_mps).cost;
          const double minus_cost =
              rollout(minus, target_speed_mps, measured_speed_mps).cost;
          if (std::isfinite(plus_cost) && std::isfinite(minus_cost)) {
            gradient[index] = (plus_cost - minus_cost) / denominator;
          }
        }
        norm_squared += gradient[index] * gradient[index];
      }
      *iterations = iteration + 1;
      if (std::sqrt(norm_squared) <= config_.gradient_tolerance) break;
      bool improved = false;
      double step = config_.gradient_step;
      for (int line_search = 0; line_search < config_.line_search_steps;
           ++line_search) {
        std::vector<double> candidate = *commands;
        for (std::size_t index = 0; index < candidate.size(); ++index) {
          candidate[index] -= step * gradient[index];
        }
        project(&candidate);
        const RolloutResult candidate_result =
            rollout(candidate, target_speed_mps, measured_speed_mps);
        if (candidate_result.cost < best.cost) {
          best = candidate_result;
          *commands = candidate;
          improved = true;
          break;
        }
        step *= 0.5;
      }
      if (!improved) break;
    }
  }
  *cost = best.cost;
  *predicted_maximum_speed_mps = best.maximum_speed_mps;
}

inline LongitudinalMpcResult LongitudinalMpc::update(double target_speed_mps,
                                              double measured_speed_mps,
                                              double dt_sec,const std::vector<double>& stations,
                                              const std::vector<double>& speeds) {
  reference_stations_=stations;reference_speeds_=speeds;
  reference_stop_station_=std::numeric_limits<double>::infinity();
  for(std::size_t i=0;i<speeds.size();++i)if(speeds[i]==0 && !(i==0 && !timed_reference_.empty() && timed_reference_.front().speed_mps>0)){reference_stop_station_=stations[i];break;}
  LongitudinalMpcResult result;
  // Competition longitudinal velocity is signed. MPC controls speed magnitude;
  // reverse/gear authority remains with MORAI and the gear interlock.
  measured_speed_mps = std::abs(measured_speed_mps);
  if (has_previous_speed_) {
    const double raw_acceleration =
        (measured_speed_mps - previous_speed_mps_) / dt_sec;
    const double alpha = 1.0 - std::exp(
        -dt_sec / config_.acceleration_estimate_filter_time_constant_sec);
    estimated_acceleration_mps2_ +=
        alpha * (clamp(raw_acceleration, -config_.maximum_deceleration_mps2,
                       config_.maximum_acceleration_mps2) -
                 estimated_acceleration_mps2_);
  }
  previous_speed_mps_ = measured_speed_mps;
  has_previous_speed_ = true;

  std::vector<double> commands = warm_start_;
  // At crawl speed, the clipped zero-speed rollout has a flat all-braking basin.
  // Seed above the pedal dead zone when restarting. The same path-stop costs
  // and command/jerk limits still decide the optimized command.
  if(measured_speed_mps<1.0 && target_speed_mps>measured_speed_mps)
    for(auto& command:commands)command=std::max(
        std::min(.3,config_.maximum_accel_command),command);
  if (target_speed_mps == 0.0) stop_release_pending_ = true;
  if (measured_speed_mps < 1.0) stop_release_pending_ = false;
  const bool recovering_stop = stop_release_pending_ && target_speed_mps > measured_speed_mps;
  if (recovering_stop) stop_release_pending_ = false;
  const bool compare_coasting = recovering_stop &&
      std::any_of(commands.begin(), commands.end(), [](double command) { return command < 0.0; });
  solve(target_speed_mps, measured_speed_mps, &commands,
                       &result.cost, &result.predicted_maximum_speed_mps,
                       &result.iterations);
  // A released stop can leave SQP in an all-braking local minimum. Compare a
  // coasting initialization using the same objective, stop cost and constraints.
  if (compare_coasting) {
    auto coasting = warm_start_;
    for (auto& command : coasting) command = std::max(0.0, command);
    double coast_cost = 0.0, coast_maximum_speed = 0.0;
    int coast_iterations = 0;
    solve(target_speed_mps, measured_speed_mps, &coasting,
          &coast_cost, &coast_maximum_speed, &coast_iterations);
    result.iterations += coast_iterations;
    if (coast_cost < result.cost) {
      commands = std::move(coasting);
      result.cost = coast_cost;
      result.predicted_maximum_speed_mps = coast_maximum_speed;
    }
  }
  result.estimated_acceleration_mps2 = estimated_acceleration_mps2_;

  double command = commands.front();
  const double actual_maximum_step = config_.command_rate_limit_per_sec * dt_sec;
  command = clamp(command, previous_signed_command_ - actual_maximum_step,
                  previous_signed_command_ + actual_maximum_step);
  command = clamp(command, -config_.maximum_brake_command,
                  config_.maximum_accel_command);
  previous_signed_command_ = command;
  result.signed_command = command;
  result.accel = std::max(0.0, command);
  result.brake = std::max(0.0, -command);
  result.state = command > 1.0e-6 ? LongitudinalState::kAccel
                 : (command < -1.0e-6 ? LongitudinalState::kBrake
                                     : LongitudinalState::kCoast);
  warm_start_.assign(commands.size(), commands.back());
  for (std::size_t index = 0; index + 1 < commands.size(); ++index) {
    warm_start_[index] = commands[index + 1];
  }
  return result;
}

inline double LongitudinalMpc::referenceSpeed(double station,double fallback) const {
  if(reference_stations_.empty())return fallback;
  const auto it=std::upper_bound(reference_stations_.begin(),reference_stations_.end(),station);
  if(it==reference_stations_.begin())return reference_speeds_.front();
  if(it==reference_stations_.end())return reference_speeds_.back();
  const auto right=static_cast<std::size_t>(it-reference_stations_.begin()),left=right-1;
  const double ratio=(station-reference_stations_[left])/(reference_stations_[right]-reference_stations_[left]);
  return reference_speeds_[left]+ratio*(reference_speeds_[right]-reference_speeds_[left]);
}
inline void LongitudinalMpc::reset() {
  std::fill(warm_start_.begin(), warm_start_.end(), 0.0);
  has_previous_speed_ = false;
  stop_release_pending_ = false;
  previous_speed_mps_ = 0.0;
  estimated_acceleration_mps2_ = 0.0;
  previous_signed_command_ = 0.0;
}

}}  // namespace control::integrated_ai_mpc

#include <cmath>

namespace control { namespace integrated_ai_mpc {
inline SteeringPrediction::SteeringPrediction(double delay_sec, double time_constant_sec)
    : delay_sec_(delay_sec), time_constant_sec_(time_constant_sec) {
}

inline double SteeringPrediction::advance(double time_sec) {
  while (!events_.empty() && events_.front().first <= time_sec) {
    const auto event = events_.front();
    events_.pop_front();
    angle_rad_ = command_rad_ + (angle_rad_ - command_rad_) *
        std::exp(-(event.first - time_sec_) / time_constant_sec_);
    time_sec_ = event.first;
    command_rad_ = event.second;
  }
  angle_rad_ = command_rad_ + (angle_rad_ - command_rad_) *
      std::exp(-(time_sec - time_sec_) / time_constant_sec_);
  time_sec_ = time_sec;
  return angle_rad_;
}

inline void SteeringPrediction::sent(double time_sec, double effective_steering_rad) {
  advance(time_sec);
  events_.emplace_back(time_sec + delay_sec_, effective_steering_rad);
}
}}  // namespace control::integrated_ai_mpc

#include <control/libraries/morai_mpc/autoware/mpc_utils.h>
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Adapted from MPCFollower::calculateMPC/callbackRefPath; ROS I/O removed.
#include <algorithm>
#include <cmath>

namespace control { namespace integrated_ai_mpc {
using namespace autoware;

inline AutowareLateral::AutowareLateral(const LateralParameters& parameters,
                               const SteeringPredictionParameters& steering)
    : p_(parameters), steering_(steering),
      vehicle_model_ptr_(new KinematicsBicycleModel(p_.vehicle_model_wheelbase,
          amathutils::deg2rad(p_.steer_lim_deg), steering_.time_constant_sec, steering_.gain)),
      qpsolver_ptr_(new QPSolverQpoasesHotstart(p_.qpoases_max_iter)),
      input_buffer_(static_cast<std::size_t>(std::round(steering_.delay_sec / p_.ctrl_period)), 0.0),
      front_point_weight_(MPCUtils::frontPointWeight(p_.morai_weight_front_point_error,
          p_.vehicle_model_wheelbase + p_.morai_front_overhang_m)) {
  lpf_steering_cmd_.initialize(p_.ctrl_period, p_.steering_lpf_cutoff_hz);
}

inline void AutowareLateral::prepareTrajectory(const control::integrated_ai_mpc::autoware::Lane& path, double speed_mps) {
  // Coincident vertices describe the same geometry; omit zero-length segments
  // before distance interpolation to avoid division by zero.
  control::integrated_ai_mpc::autoware::Lane distinct;
  for (const auto& point : path.waypoints) {
    if (!distinct.waypoints.empty()) {
      const auto& previous = distinct.waypoints.back().pose.pose.position;
      const auto& current = point.pose.pose.position;
      if (std::hypot(current.x - previous.x, current.y - previous.y) == 0.0) continue;
    }
    distinct.waypoints.push_back(point);
  }
  std::vector<double> relative_time;
  MPCUtils::calcPathRelativeTime(distinct, relative_time);
  MPCTrajectory traj;
  MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(
      distinct, relative_time, p_.traj_resample_dist, traj);
  // A segment shorter than the resampling interval still has two real endpoints.
  if (traj.x.size() < 2) {
    const auto& endpoint = distinct.waypoints.back();
    const auto& position = endpoint.pose.pose.position;
    traj.push_back(position.x, position.y, position.z,
        tf2::getYaw(endpoint.pose.pose.orientation), endpoint.twist.twist.linear.x,
        0.0, relative_time.back());
  }
  MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  if (p_.enable_path_smoothing) {
    for (int i = 0; i < p_.path_smoothing_times; ++i) {
      MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.x);
      MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.y);
      MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.yaw);
      MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.vx);
    }
  }
  if (p_.enable_yaw_recalculation) {
    MPCUtils::calcTrajectoryYawFromXY(traj, p_.morai_yaw_smoothing_num);
    MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  }
  // Keep the configured window on normal paths; fit only available real points
  // on shorter paths (the upstream unsigned loop underflows otherwise).
  const int window = std::min(p_.curvature_smoothing_num,
                             static_cast<int>((traj.x.size() - 1) / 2));
  if (window > 0) MPCUtils::calcTrajectoryCurvature(traj, window);
  else traj.k.assign(traj.x.size(), 0.0);

  const int steps = moraiPredictionSteps(p_.mpc_prediction_horizon,
      p_.morai_max_prediction_horizon, p_.mpc_prediction_sampling_time,
      speed_mps, p_.morai_min_prediction_length_m);
  // Preserve the source stationary terminal extension, covering the actual
  // adaptive horizon. This does not supply additional spatial path information.
  const double extension = (steps + 1) * p_.mpc_prediction_sampling_time +
                           steering_.delay_sec + p_.ctrl_period;
  traj.vx.back() = 0.0;
  traj.push_back(traj.x.back(), traj.y.back(), traj.z.back(), traj.yaw.back(),
      0.0, traj.k.back(), traj.relative_time.back() + extension);
  ref_traj_ = std::move(traj);
}

inline double AutowareLateral::calculate(const control::integrated_ai_mpc::autoware::Lane& path, double speed_mps,
                                double modeled_steering_rad, double previous_steering_rad) {
  queued_command_=false;
  prepareTrajectory(path, speed_mps);
  // Input path is already expressed at the rear-axle base_link origin.
  geometry_msgs::Pose control_pose;
  control_pose.orientation.w = 1.0;
  const double DT = p_.mpc_prediction_sampling_time;
  const int N = moraiPredictionSteps(p_.mpc_prediction_horizon, p_.morai_max_prediction_horizon,
      DT, speed_mps, p_.morai_min_prediction_length_m);
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();


  /* calculate nearest point on reference trajectory (used as initial state) */
  unsigned int nearest_index = 0;
  double yaw_err = 0.0, dist_err = 0.0, nearest_traj_time = 0.0;
  geometry_msgs::Pose nearest_pose;
  if (!MPCUtils::calcNearestPoseInterp(ref_traj_, control_pose, nearest_pose,
      nearest_index, dist_err, yaw_err, nearest_traj_time)) {
    ROS_WARN_THROTTLE(1.0, "[control] MPC reference matching failed; retaining steering command");
    return previous_steering_rad;
  }

  /* set mpc initial time */
  const double mpc_start_time = nearest_traj_time;


  /* convert tracking x,y error to lat error */
  const double err_x = control_pose.position.x - nearest_pose.position.x;
  const double err_y = control_pose.position.y - nearest_pose.position.y;
  const double sp_yaw = tf2::getYaw(nearest_pose.orientation);
  const double err_lat = -sin(sp_yaw) * err_x + cos(sp_yaw) * err_y;

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);
  x0 << err_lat, yaw_err, modeled_steering_rad;

  /////////////// delay compensation  ///////////////
  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  Eigen::MatrixXd x_curr = x0;
  double mpc_curr_time = mpc_start_time;
  for (unsigned int i = 0; i < input_buffer_.size(); ++i)
  {
    double k = 0.0;
    double v = 0.0;
    MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.k, mpc_curr_time, k);
    MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.vx, mpc_curr_time, v);

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(v);
    vehicle_model_ptr_->setCurvature(k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, p_.ctrl_period);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = input_buffer_[i];
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += p_.ctrl_period;
  }
  x0 = x_curr; // set delay compensated initial state


  /////////////// generate mpc matrix  ///////////////
  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * Rex * (Uex - Urefex)
   * Qex = diag([Q,Q,...]), Rex = diag([R,R,...])
   */

  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_X);
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_U * N);
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * N, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  Eigen::MatrixXd Rex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd Urefex = Eigen::MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = p_.mpc_weight_lat_error;
  Q(1, 1) = p_.mpc_weight_heading_error;
  R(0, 0) = p_.mpc_weight_steering_input;

  /* resample ref_traj with mpc sampling time */
  std::vector<double> mpc_time_v;
  for (int i = 0; i < N; ++i)
  {
    mpc_time_v.push_back(mpc_curr_time + i * DT);
  }
  MPCTrajectory mpc_resampled_ref_traj;
  if (!MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_,
      mpc_time_v, mpc_resampled_ref_traj)) {
    ROS_WARN_THROTTLE(1.0, "[control] MPC horizon interpolation failed; retaining steering command");
    return previous_steering_rad;
  }

  /* predict dynamics for N times */
  double maximum_curve_blend=0;
  for (int i = 0; i < N; ++i)
  {
    const double ref_k = mpc_resampled_ref_traj.k[i];
    const double ref_vx = mpc_resampled_ref_traj.vx[i];
    const double ref_vx_squared = ref_vx * ref_vx;

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(ref_vx);
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1)
    {
      Q_adaptive(0, 0) = p_.mpc_weight_terminal_lat_error;
      Q_adaptive(1, 1) = p_.mpc_weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * p_.mpc_weight_heading_error_squared_vel_coeff;
    R_adaptive(0, 0) += ref_vx_squared * p_.mpc_weight_steering_input_squared_vel_coeff;
    const double curve_blend = moraiCurvatureWeightBlend(ref_k, p_.morai_weight_curvature_low, p_.morai_weight_curvature_high);
    maximum_curve_blend=std::max(maximum_curve_blend,curve_blend);
    Q_adaptive(0, 0) *= p_.morai_straight_lat_weight_scale + (1.0-p_.morai_straight_lat_weight_scale) * curve_blend;
    R_adaptive(0, 0) *= 1.0 + (p_.morai_curve_steer_weight_scale-1.0) * curve_blend;
    // Same reference vehicle pose, not a shifted path: e_front ~= e_y + d*e_yaw.
    // ponytail: small-angle point cost; full-width body clearance stays an audit,
    // not a collision constraint. d includes the configured front overhang.
    Q_adaptive += front_point_weight_;

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0)
    {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
    }
    else
    {
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j)
      {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) = Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    Rex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < amathutils::deg2rad(p_.mpc_zero_ff_steer_deg))
    {
      Uref(0, 0) = 0.0; // ignore curvature noise
    }

    Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;

    mpc_curr_time += DT;
  }

  // Only input-reference tracking contributes a feed-forward linear term.
  // The jerk penalty below damps actual command differences, not U-Uref differences.
  const Eigen::MatrixXd reference_input_gradient = -Urefex.transpose() * Rex;

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i)
  {
    const double v = mpc_resampled_ref_traj.vx[i];
    const double lateral_jerk_weight = v * v * p_.mpc_weight_lat_jerk;
    Rex(i, i) += lateral_jerk_weight;
    Rex(i + 1, i) -= lateral_jerk_weight;
    Rex(i, i + 1) -= lateral_jerk_weight;
    Rex(i + 1, i + 1) += lateral_jerk_weight;
  }


  /////////////// optimization ///////////////
  /*
   * solve quadratic optimization.
   * cost function: 1/2 * Uex' * H * Uex + f' * Uex
   */
  const Eigen::MatrixXd CB = Cex * Bex;
  const Eigen::MatrixXd QCB = Qex * CB;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB; // NOTE: This calculation is very heavy. searching for a good way...
  H.triangularView<Eigen::Upper>() += Rex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  Eigen::MatrixXd f = (Cex * (Aex * x0 + Wex)).transpose() * QCB + reference_input_gradient;
  // First transition spans the control period, not the longer prediction step.
  // Feedback is the Core's final bounded command, not measured tire angle.
  const double straight_scale=1+(p_.morai_straight_steer_change_scale-1)*(1-maximum_curve_blend);
  const double first_change_weight=straight_scale*p_.morai_weight_first_steer_change *
      speed_mps * speed_mps * DT / p_.ctrl_period;
  H(0,0)+=first_change_weight;
  f(0,0)-=first_change_weight*previous_steering_rad;

  /* constraint matrix : lb < U < ub, lbA < A*U < ubA */
  const double u_lim = amathutils::deg2rad(p_.steer_lim_deg);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd lbA = Eigen::MatrixXd::Zero(DIM_U * N, 1);
  Eigen::MatrixXd ubA = Eigen::MatrixXd::Zero(DIM_U * N, 1);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(DIM_U * N, -u_lim); // min steering angle
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(DIM_U * N, u_lim);  // max steering angle
  // Optional MORAI comparison constraint: enforce rate INSIDE the prediction.
  const double rate_deg = p_.morai_steer_rate_limit_deg;
  if (rate_deg > 0.0) {
    const double rate = amathutils::deg2rad(rate_deg);
    for (int i = 0; i < N; ++i) {
      A(i, i) = 1.0;
      if (i == 0) {
        const double previous = previous_steering_rad;
        lbA(i) = previous - rate * p_.ctrl_period;
        ubA(i) = previous + rate * p_.ctrl_period;
      } else {
        A(i, i - 1) = -1.0;
        lbA(i) = -rate * DT;
        ubA(i) = rate * DT;
      }
    }
  }

  Eigen::VectorXd Uex;
  if (!qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex) ||
      Uex.size() != DIM_U * N || !Uex.allFinite()) {
    ROS_WARN_THROTTLE(1.0, "[control] MPC optimization failed; retaining steering command");
    return previous_steering_rad;
  }


  /* saturation */
  const double u_sat = std::max(std::min(Uex(0), u_lim), -u_lim);

  /* filtering */
  // With rate bounds referenced to the last sent command, an extra output LPF
  // silently reduces achievable slew and adds dynamics absent from the model.
  const double u_filtered = p_.morai_use_output_steering_filter ? lpf_steering_cmd_.filter(u_sat) : u_sat;

  // The Core replaces the newest entry with its final bounded command.
  if (!input_buffer_.empty()) {
    input_buffer_.push_back(u_filtered);
    input_buffer_.pop_front();
    queued_command_=true;
  }
  return u_filtered;
}

}}  // namespace control::integrated_ai_mpc

#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace control {
// Geometry retained from the same previously supplied reference, never ego breadcrumbs.
class IntegratedMpcPathHistory {
 public:
  void clear(){points_.clear();}
  interfaces::Path update(const interfaces::Path& path, const nav_msgs::Odometry& ego, double length_m) {
    tf2::Transform world;
    tf2::fromMsg(ego.pose.pose, world);
    const auto inverse = world.inverse();
    interfaces::Path result = path;
    if (length_m > 0 && points_.size() > 1) {
      tf2::Transform first;
      tf2::fromMsg(path.points.front().pose.pose, first);
      const auto origin = (world * first).getOrigin();
      double best = std::numeric_limits<double>::infinity(), station = 0, nearest = 0;
      std::vector<double> stations(points_.size(), 0);
      for (std::size_t i = 1; i < points_.size(); ++i) {
        const auto delta = points_[i].getOrigin() - points_[i-1].getOrigin();
        const double squared = delta.x()*delta.x()+delta.y()*delta.y();
        const double segment = std::sqrt(squared);
        stations[i] = station + segment;
        if (squared > 0) {
          const auto relative = origin - points_[i-1].getOrigin();
          const double t = std::max(0.0, std::min(1.0,
              (relative.x()*delta.x()+relative.y()*delta.y())/squared));
          const auto error = points_[i-1].getOrigin()+t*delta-origin;
          const double distance = error.x()*error.x()+error.y()*error.y();
          if (distance < best) {best=distance;nearest=station+t*segment;}
        }
        station += segment;
      }
      // A different reference corridor must not inherit an unrelated historical tail.
      if (best < 0.25) {
        std::vector<interfaces::PathPoint> prefix;
        for (double d = std::max(0.0, nearest-length_m); d < nearest-0.01; d += 0.25) {
          const auto upper = std::upper_bound(stations.begin(), stations.end(), d);
          const std::size_t right = std::min(stations.size()-1,
              std::max<std::size_t>(1, upper-stations.begin()));
          const std::size_t left = right-1;
          const double t = (d-stations[left])/(stations[right]-stations[left]);
          tf2::Transform pose;
          pose.setOrigin(points_[left].getOrigin().lerp(points_[right].getOrigin(), t));
          pose.setRotation(points_[left].getRotation().slerp(points_[right].getRotation(), t));
          auto point = path.points.front();
          tf2::toMsg(inverse*pose, point.pose.pose);
          prefix.push_back(point);
        }
        result.points.insert(result.points.begin(), prefix.begin(), prefix.end());
      }
    }
    points_.clear();
    // Bound memory by spatial sampling even when repeated calls advance millimetres.
    for (const auto& point : result.points) {
      tf2::Transform pose;
      tf2::fromMsg(point.pose.pose, pose);
      pose = world*pose;
      if (points_.empty() || (pose.getOrigin()-points_.back().getOrigin()).length() >= 0.2)
        points_.push_back(pose);
    }
    return result;
  }
 private:
  std::vector<tf2::Transform> points_;
};

class IntegratedMpc {
 public:
  explicit IntegratedMpc(integrated_ai_mpc::Parameters parameters = {})
      : parameters_(std::move(parameters)),
        lateral_(parameters_.lateral, parameters_.steering_prediction),
        longitudinal_(parameters_.longitudinal),
        steering_(parameters_.steering_prediction.delay_sec,
                  parameters_.steering_prediction.time_constant_sec) {}

  static std::vector<integrated_ai_mpc::LongitudinalReference> referenceHorizon(
      const interfaces::Path& path, double elapsed_sec,
      const integrated_ai_mpc::LongitudinalMpcConfig& config) {
    using Reference = integrated_ai_mpc::LongitudinalReference;
    std::vector<Reference> references(static_cast<std::size_t>(config.horizon_steps));
    const auto after = std::upper_bound(path.points.begin(), path.points.end(), elapsed_sec,
        [](double t, const interfaces::PathPoint& point) {
          return t < point.time_from_start.toSec();
        });
    if (after != path.points.begin() && after != path.points.end() &&
        (after - 1)->velocity == 0.0 && after->velocity == 0.0)
      return references;  // 계획상 대기 중에는 이후 출발을 미리 참조하지 않는다.
    double next_stop_sec = std::numeric_limits<double>::infinity();
    for (auto it = after; it != path.points.end(); ++it) {
      if (it->velocity == 0.0) {
        next_stop_sec = it->time_from_start.toSec();
        break;
      }
    }
    for (std::size_t k = 0; k < references.size(); ++k) {
      const double time = elapsed_sec + (k + 1) * config.prediction_dt_sec;
      if (time >= next_stop_sec || time >= path.points.back().time_from_start.toSec())
        continue;
      const auto right = std::upper_bound(path.points.begin(), path.points.end(), time,
          [](double t, const interfaces::PathPoint& point) {
            return t < point.time_from_start.toSec();
          });
      const auto& a = *(right - 1);
      const auto& b = *right;
      const double h = b.time_from_start.toSec() - a.time_from_start.toSec();
      const double u = (time - a.time_from_start.toSec()) / h;
      const double u2 = u * u, u3 = u2 * u;
      const double velocity = (2*u3 - 3*u2 + 1)*a.velocity +
          (u3 - 2*u2 + u)*h*a.acceleration + (-2*u3 + 3*u2)*b.velocity +
          (u3 - u2)*h*b.acceleration;
      const double acceleration = (6*u2 - 6*u)*a.velocity/h +
          (3*u2 - 4*u + 1)*a.acceleration + (-6*u2 + 6*u)*b.velocity/h +
          (3*u2 - 2*u)*b.acceleration;
      references[k] = {
          std::max(std::min(a.velocity,b.velocity),
                   std::min(std::max(a.velocity,b.velocity),velocity)),
          std::max(std::min(a.acceleration,b.acceleration),
                   std::min(std::max(a.acceleration,b.acceleration),acceleration))};
    }
    return references;
  }

  morai_msgs::CtrlCmd process(const interfaces::Path& path, const nav_msgs::Odometry& ego,
                             const interfaces::CompetitionVehicleStatus& status,const ros::Time& control_stamp) {
    ROS_INFO_ONCE("[control] executing integrated_mpc.hpp (current lateral tuning, timed longitudinal reference)");
    const auto stamp = control_stamp.toNSec();
    if(has_previous_stamp_ && (stamp<previous_stamp_ ||
        (static_cast<double>(stamp)-static_cast<double>(previous_stamp_))*1e-9>1.0)) {
      // A different selected Core has controlled the vehicle in the meantime.
      // MORAI reports physical wheel steering with the opposite ROS yaw sign.
      const double physical=-status.steering_angle_rad;
      const double limit=parameters_.lateral.steer_lim_deg*integrated_ai_mpc::kPi/180.;
      previous_steering_rad_=std::max(-limit,std::min(limit,physical/parameters_.steering_prediction.gain));
      lateral_=integrated_ai_mpc::AutowareLateral(parameters_.lateral,parameters_.steering_prediction);
      lateral_.synchronize(previous_steering_rad_);
      steering_.synchronize(path.header.stamp.toSec(),physical);
      path_history_.clear();longitudinal_.reset();
      has_previous_stamp_=false;has_previous_lateral_stamp_=false;
    }
    const double dt = has_previous_stamp_ ?
        std::max(parameters_.minimum_control_dt_sec,
          std::min(parameters_.maximum_control_dt_sec,
            (static_cast<double>(stamp) - static_cast<double>(previous_stamp_)) * 1e-9)) :
        parameters_.lateral.ctrl_period;
    previous_stamp_ = stamp;
    has_previous_stamp_ = true;

    const auto nearest = std::min_element(path.points.begin(), path.points.end(),
        [](const auto& a, const auto& b) {
          const auto& pa = a.pose.pose.position;
          const auto& pb = b.pose.pose.position;
          return pa.x * pa.x + pa.y * pa.y < pb.x * pb.x + pb.y * pb.y;
        });
    // The source Autoware adapter uses observed odometry speed throughout the
    // lateral reference. The injected future velocity/acceleration profile is
    // not a longitudinal horizon reference for this controller.
    // The original  lateral controller advances steering state using
    // the path epoch. Keep that time base independent of longitudinal v(t) age.
    const auto lateral_stamp=path.header.stamp.toNSec();
    const double lateral_dt=has_previous_lateral_stamp_ ?
        std::max(parameters_.minimum_control_dt_sec,std::min(parameters_.maximum_control_dt_sec,
          (static_cast<double>(lateral_stamp)-static_cast<double>(previous_lateral_stamp_))*1e-9)) : parameters_.lateral.ctrl_period;
    const double lateral_time=path.header.stamp.toSec();
    previous_lateral_stamp_=lateral_stamp;has_previous_lateral_stamp_=true;
    const double speed_mps = ego.twist.twist.linear.x;
    control::integrated_ai_mpc::autoware::Lane waypoints;
    interfaces::Path lateral_path=path;
    if(!path.lateral_reference.empty()) {
      lateral_path.points.clear();lateral_path.points.reserve(path.lateral_reference.size());
      for(const auto& pose:path.lateral_reference) {
        interfaces::PathPoint point;point.pose.header=path.header;point.pose.pose=pose;
        lateral_path.points.push_back(point);
      }
    }
    const auto reference = path_history_.update(lateral_path, ego, parameters_.reference_history_length_m);
    waypoints.waypoints.reserve(reference.points.size());
    for (const auto& point : reference.points) {
      control::integrated_ai_mpc::autoware::Waypoint waypoint;
      waypoint.pose = point.pose;
      waypoint.twist.twist.linear.x = std::max(0.5, std::abs(speed_mps));
      waypoints.waypoints.push_back(waypoint);
    }
    const bool stationary_wait=path.points.back().time_from_start.toSec()>0 &&
        std::all_of(path.points.begin(),path.points.end(),
          [](const interfaces::PathPoint& p){return p.velocity==0 && p.acceleration==0;});
    const double requested = stationary_wait ? previous_steering_rad_ : lateral_.calculate(waypoints, speed_mps,
        steering_.advance(lateral_time), previous_steering_rad_);
    const double limit = parameters_.lateral.steer_lim_deg * integrated_ai_mpc::kPi / 180.0;
    const double step = parameters_.lateral.morai_steer_rate_limit_deg *
                        integrated_ai_mpc::kPi / 180.0 * lateral_dt;
    double angle = std::max(-limit, std::min(limit, requested));
    angle = std::max(previous_steering_rad_ - step,
                     std::min(previous_steering_rad_ + step, angle));
    lateral_.sent(angle);
    const auto& position = ego.pose.pose.position;
    const bool dynamic_zone = parameters_.dynamic_zone_enabled &&
        position.x >= parameters_.dynamic_zone_min_x_m && position.x <= parameters_.dynamic_zone_max_x_m &&
        position.y >= parameters_.dynamic_zone_min_y_m && position.y <= parameters_.dynamic_zone_max_y_m;
    const bool approach = parameters_.dynamic_zone_enabled &&
        position.x >= parameters_.dynamic_zone_min_x_m && position.x <= parameters_.dynamic_zone_max_x_m &&
        position.y >= parameters_.dynamic_zone_min_y_m-parameters_.dynamic_zone_approach_m &&
        position.y <= parameters_.dynamic_zone_max_y_m;
    const bool timed=path.points.size()>1 && path.points.back().time_from_start.toSec()>0;
    auto time_reference=timed?referenceHorizon(path,std::max(0.,control_stamp.toSec()-path.header.stamp.toSec()),parameters_.longitudinal):std::vector<integrated_ai_mpc::LongitudinalReference>{};
    if(approach)for(auto& point:time_reference)point.speed_mps=std::min(point.speed_mps,parameters_.dynamic_zone_speed_mps);
    const double time_target=time_reference.empty()?nearest->velocity:time_reference.front().speed_mps;
    longitudinal_.setTimedReference(std::move(time_reference));
    const double target_speed = approach ? std::min(time_target, parameters_.dynamic_zone_speed_mps) : time_target;
    std::vector<double> stations{0},speeds{target_speed};double distance=0;
    for(auto point=nearest+1;point!=path.points.end();++point) {
      const auto& a=(point-1)->pose.pose.position;const auto& b=point->pose.pose.position;
      distance+=std::hypot(b.x-a.x,b.y-a.y);
      if(distance<=stations.back()+1e-6)continue;
      stations.push_back(distance);speeds.push_back(approach?std::min(point->velocity,parameters_.dynamic_zone_speed_mps):point->velocity);
    }
    const auto pedals = longitudinal_.update(target_speed, status.velocity_x_mps, dt,stations,speeds);
    const bool emergency = dynamic_zone && std::all_of(path.points.begin(), path.points.end(),
        [](const interfaces::PathPoint& point) {return point.velocity == 0;});

    previous_steering_rad_ = angle;
    steering_.sent(lateral_time, angle * parameters_.steering_prediction.gain);
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = angle / integrated_ai_mpc::kBridgeSteeringLimitRad;
    command.accel = pedals.accel;
    command.brake = pedals.brake;
    if (target_speed == 0.0 && std::abs(status.velocity_x_mps) <= parameters_.stationary_hold_speed_mps) {
      command.accel = 0;
      command.brake = std::max(command.brake, parameters_.stationary_hold_brake);
    }
    if (emergency) {command.accel=0;command.brake=1;}
    return command;
  }

 private:
  integrated_ai_mpc::Parameters parameters_;
  IntegratedMpcPathHistory path_history_;
  bool has_previous_lateral_stamp_=false;
  std::uint64_t previous_lateral_stamp_=0;
  integrated_ai_mpc::AutowareLateral lateral_;
  integrated_ai_mpc::LongitudinalMpc longitudinal_;
  integrated_ai_mpc::SteeringPrediction steering_;
  std::uint64_t previous_stamp_ = 0;
  bool has_previous_stamp_ = false;
  double previous_steering_rad_ = 0.0;
};
}  // namespace control
