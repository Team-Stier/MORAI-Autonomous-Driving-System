#pragma once
#include <ros/console.h>
#include <osqp.h>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>


namespace control { namespace autoware_ai_mpc {
// Kept separate from the source PID header: no PID controller is imported.
enum class LongitudinalState { kAccel, kCoast, kBrake };
}}  // namespace control::autoware_ai_mpc


#include <deque>
#include <limits>
#include <string>
#include <sstream>
#include <vector>


namespace control { namespace autoware_ai_mpc {

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
  double high_speed_braking_gain_mps2{0.0};  // 0: original constant braking gain.
  double braking_gain_transition_low_mps{0.0};
  double braking_gain_transition_high_mps{0.0};
  double strong_brake_transition_low{0.0};
  double strong_brake_transition_high{0.0};
  double actuator_time_constant_sec{0.25};
  bool physical_rollout{false};
  double constant_drag_mps2{0.0};
  double quadratic_drag_per_m{0.0};
  double high_speed_acceleration_gain_mps2{0.0};  // 0: original drive model.
  double high_speed_constant_drag_mps2{0.0};
  double high_speed_quadratic_drag_per_m{0.0};
  double drive_model_transition_low_mps{0.0};
  double drive_model_transition_high_mps{0.0};
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
};

// #MPC_HORIZON_REFERENCE
// 호출자는 각 예측 상태 시각(t + (k+1)*prediction_dt_sec)의 계획 속도(m/s)와
// 가속도(m/s²)를 전달한다. 경로점 번호를 시간으로 가정해서는 안 된다.
// 속도·가속도 오차만 시간별 참조로 바꾸며 기존 모델과 페달·변화율 제약은 유지한다.
// ROS 경로의 예정 시각 계약을 연결하기 전에는 기존 scalar 호출을 그대로 사용한다.
struct LongitudinalReference {
  double speed_mps{0.0};
  double acceleration_mps2{0.0};
};

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
                               double measured_speed_mps, double dt_sec);
  // references는 유효한 horizon_steps개의 예측 상태 참조다.
  LongitudinalMpcResult updateReference(
      const std::vector<LongitudinalReference>& references,
      double measured_speed_mps, double dt_sec);
  void reset();

 private:
  struct RolloutResult {
    double cost{std::numeric_limits<double>::infinity()};
    double maximum_speed_mps{0.0};
  };

  void project(std::vector<double>* commands) const;
  RolloutResult rollout(const std::vector<double>& commands,
                        const std::vector<LongitudinalReference>& references,
                        double measured_speed_mps) const;
  void solve(const std::vector<LongitudinalReference>& references, double measured_speed_mps,
             std::vector<double>* commands, double* cost,
             double* predicted_maximum_speed_mps, int* iterations) const;

  LongitudinalMpcConfig config_;
  std::vector<double> warm_start_;
  bool has_previous_speed_{false};
  double previous_speed_mps_{0.0};
  double estimated_acceleration_mps2_{0.0};
  double previous_signed_command_{0.0};
};

}}  // namespace control::autoware_ai_mpc


namespace control { namespace autoware_ai_mpc {
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
  LateralParameters lateral;
  LongitudinalMpcConfig longitudinal;
  SteeringPredictionParameters steering_prediction;
  double minimum_control_dt_sec = 0.005;
  double maximum_control_dt_sec = 0.10;
};

constexpr double kPi = 3.14159265358979323846;
// The Bridge contract is fixed independently of a lower optimizer steering limit.
constexpr double kBridgeSteeringLimitRad = 40.0 * kPi / 180.0;
}}  // namespace control::autoware_ai_mpc


#include <functional>
#include <string>
#include <vector>

namespace control { namespace autoware_ai_mpc {

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
}}  // namespace control::autoware_ai_mpc

#include <deque>
#include <utility>

namespace control { namespace autoware_ai_mpc {
// A command-history estimate, never a measured wheel-angle report.
class SteeringPrediction {
 public:
  SteeringPrediction(double delay_sec, double time_constant_sec);
  double advance(double time_sec);
  void sent(double time_sec, double effective_steering_rad);

 private:
  double delay_sec_;
  double time_constant_sec_;
  double time_sec_ = 0.0;
  double angle_rad_ = 0.0;
  double command_rad_ = 0.0;
  std::deque<std::pair<double, double>> events_;
};
}}  // namespace control::autoware_ai_mpc

#include <control/libraries/morai_mpc/autoware/mpc_trajectory.h>
#include <control/libraries/morai_mpc/autoware/lowpass_filter.h>
#include <control/libraries/morai_mpc/autoware/vehicle_model/vehicle_model_bicycle_kinematics.h>
#include <control/libraries/morai_mpc/autoware/qp_solver/qp_solver_qpoases.h>
#include <control/libraries/morai_mpc/autoware/lane.hpp>
#include <deque>
#include <memory>

namespace control { namespace autoware_ai_mpc {
class AutowareLateral {
 public:
  AutowareLateral(const LateralParameters& parameters,
                 const SteeringPredictionParameters& steering);
  double calculate(const control::autoware_ai_mpc::autoware::Lane& path, double speed_mps,
                   double modeled_steering_rad, double previous_steering_rad);

 private:
  void prepareTrajectory(const control::autoware_ai_mpc::autoware::Lane& path, double speed_mps);
  LateralParameters p_;
  SteeringPredictionParameters steering_;
  autoware::MPCTrajectory ref_traj_;
  autoware::Butterworth2dFilter lpf_steering_cmd_;
  std::unique_ptr<autoware::KinematicsBicycleModel> vehicle_model_ptr_;
  std::unique_ptr<autoware::QPSolverQpoasesHotstart> qpsolver_ptr_;
  std::deque<double> input_buffer_;
  Eigen::Matrix2d front_point_weight_;
};
}}  // namespace control::autoware_ai_mpc

#include <algorithm>
#include <cmath>

namespace control { namespace autoware_ai_mpc { namespace autoware {

inline int moraiPredictionSteps(int base, int maximum, double dt, double speed, double metres) {
  if (metres==0.) return base;
  const double needed=std::ceil(metres/(std::max(.5,std::abs(speed))*dt))+1.;
  // Five-step blocks reduce repeated solver resizes as speed changes.
  return std::max(base,static_cast<int>(std::min(double(maximum),5.*std::ceil(needed/5.))));
}

}}}  // namespace control::autoware_ai_mpc::autoware

#include <algorithm>
#include <cmath>

namespace control { namespace autoware_ai_mpc { namespace autoware {

// A continuous transition avoids an abrupt controller switch near a threshold.
inline double moraiCurvatureWeightBlend(double curvature, double low, double high) {
  const double t = std::max(0., std::min(1., (std::abs(curvature)-low)/(high-low)));
  return t*t*(3.-2.*t);
}

}}}  // namespace control::autoware_ai_mpc::autoware


#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

#include <osqp.h>

namespace control { namespace autoware_ai_mpc {
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
    // #MPC_LONGITUDINAL_COUPLING
    // 각 미래 페달은 이후 여러 시점의 속도·가속도에 함께 영향을 준다.
    // 대각 곡률만 사용하면 이 상호작용을 잃어 같은 참조에서도 시작값에 따라
    // 불필요한 제동이 남는다. 기존 목적함수의 혼합 편미분을 계산하여
    // QP의 전체 이차항에 넣는다. 모델·가중치·페달·변화율 제약은 유지한다.
    Eigen::MatrixXd hessian(variables, variables);
    const double epsilon = config.finite_difference_epsilon;
    for (int row = 0; row < variables; ++row) {
      hessian(row, row) = diagonal[static_cast<std::size_t>(row)];
      for (int column = 0; column < row; ++column) {
        auto plus_plus = result.solution;
        auto plus_minus = result.solution;
        auto minus_plus = result.solution;
        auto minus_minus = result.solution;
        plus_plus[row] += epsilon;
        plus_plus[column] += epsilon;
        plus_minus[row] += epsilon;
        plus_minus[column] -= epsilon;
        minus_plus[row] -= epsilon;
        minus_plus[column] += epsilon;
        minus_minus[row] -= epsilon;
        minus_minus[column] -= epsilon;
        const double mixed = (objective(plus_plus) - objective(plus_minus) -
            objective(minus_plus) + objective(minus_minus)) /
            (4.0 * epsilon * epsilon);
        hessian(row, column) = mixed;
        hessian(column, row) = mixed;
      }
    }
    // 가속·제동 전환과 비선형 모델의 국소 곡률을 OSQP가 수용하는
    // 양의 정부호 이차항으로 정규화한다. 실제 목적함수의 기존 line search로
    // 개선된 해만 채택하며, 도로·객체 판단이나 속도 참조는 바꾸지 않는다.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(hessian);
    hessian = eigen.eigenvectors() *
        eigen.eigenvalues().cwiseMax(config.hessian_regularization).asDiagonal() *
        eigen.eigenvectors().transpose();
    for (int row = 0; row < variables; ++row) {
      double linear = gradient[static_cast<std::size_t>(row)];
      for (int column = 0; column < variables; ++column) {
        linear -= hessian(row, column) * result.solution[column];
      }
      q[static_cast<std::size_t>(row)] = static_cast<c_float>(linear);
    }
    // OSQP의 P 행렬은 CSC 형식의 상삼각 성분만 전달한다.
    std::vector<c_float> p_values;
    std::vector<c_int> p_rows;
    std::vector<c_int> p_columns;
    for (int column = 0; column < variables; ++column) {
      p_columns.push_back(static_cast<c_int>(p_values.size()));
      for (int row = 0; row <= column; ++row) {
        p_values.push_back(static_cast<c_float>(hessian(row, column)));
        p_rows.push_back(row);
      }
    }
    p_columns.push_back(static_cast<c_int>(p_values.size()));
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

}}  // namespace control::autoware_ai_mpc


#include <algorithm>
#include <cmath>

namespace control { namespace autoware_ai_mpc {
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
    const std::vector<double>& commands,
    const std::vector<LongitudinalReference>& references,
    double measured_speed_mps) const {
  RolloutResult result;
  result.cost = 0.0;
  double speed = std::max(0.0, measured_speed_mps);
  double acceleration = estimated_acceleration_mps2_;
  double previous_command = previous_signed_command_;
  result.maximum_speed_mps = speed;
  const double dt = config_.prediction_dt_sec;
  const double actuator_alpha =
      1.0 - std::exp(-dt / config_.actuator_time_constant_sec);
  for (std::size_t index = 0; index < commands.size(); ++index) {
    const double command = commands[index];
    const auto& reference = references[index];
    // #SPEED_DEPENDENT_BRAKING_MODEL
    // MORAI의 일정 제동 입력 측정에서 6~8m/s와 4~5m/s의 응답이 달랐다.
    // 5~6m/s의 전환 응답은 불확실하여 기존 계수를 유지하고 실측한 상부 영역만
    // 연결한다. 높은 속도의 제동 효과를 저속 계수로 계산하면 작은 페달로 충분히
    // 감속한다고 예측하여 실제 차량이 곡선 진입 전에 속도를 줄이지 못한다.
    // 별도 계수를 설정한 physical rollout에서만, 각 예측 상태의 전방 속도로
    // 두 계수를 속도와 제동 크기의 C1 연속 smoothstep으로 보간한다.
    // 별도의 0.3 제동 실측에서 작은 제동보다 큰 효과를 확인했다. 강한 제동은
    // 원래 계수로 연속 복귀시켜 작은 제동 계수를 모든 페달에 적용하지 않는다.
    // 기존 최대 페달 자체는 바꾸지 않는다. 실제 속도를 임계값으로 자르거나
    // 참조 속도·가속도에 값을 더하지 않는다. 높은 속도 계수 0은 원래 식이다.
    // Coast·가속 모델과 페달/변화율 한도·정지 유지·Watchdog은 그대로다.
    // 이 계수는 시뮬레이터의 유한 속도 범위 실측에 근거한 모델이며, 다른
    // 차량·넓은 속도 범위에는 재측정이 필요하다. GT 객체/ID와는 독립적이다.
    double braking_gain = config_.braking_gain_mps2;
    if (command < 0.0 && config_.physical_rollout &&
        config_.high_speed_braking_gain_mps2 > 0.0) {
      const double ratio = clamp((speed - config_.braking_gain_transition_low_mps) /
          (config_.braking_gain_transition_high_mps - config_.braking_gain_transition_low_mps), 0.0, 1.0);
      const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
      const double pressure_ratio = clamp((-command - config_.strong_brake_transition_low) /
          (config_.strong_brake_transition_high - config_.strong_brake_transition_low), 0.0, 1.0);
      const double pressure_blend = pressure_ratio * pressure_ratio * (3.0 - 2.0 * pressure_ratio);
      braking_gain += blend * (1.0 - pressure_blend) *
          (config_.high_speed_braking_gain_mps2 - braking_gain);
    }
    // #SPEED_DEPENDENT_DRIVE_MODEL
    // 세 번의 Ioniq5 주행에서 6m/s 이상 가속 응답은 기존 5.74보다 작은
    // 약 2.7m/s²/페달로 관측되었다. 기존 고속 저항 예측도 실제보다 커서,
    // 감속 참조에 낸 양의 페달이 모델에서는 감속, 실제로는 가속이 되었다.
    // physical rollout에서 명시한 속도 구간의 smoothstep으로 가속 이득과
    // 저항을 연속 연결한다. 속도는 예측 차량 상태이고 페달은 정규화 값이다.
    // 구간 아래의 기존 모델·실측 초기 상태·최적화 참조·게인/한도·제동 이득은
    // 유지한다. 새 이득 0은 기존 계산이며 다른 차량에는 재측정이 필요하다.
    // 관측 ID·GT·LiDAR와 무관한 차량 모델 보정으로, 객체나 경로를 수정하지 않는다.
    double acceleration_gain=config_.acceleration_gain_mps2;
    double constant_drag=config_.constant_drag_mps2;
    double quadratic_drag=config_.quadratic_drag_per_m;
    if(config_.physical_rollout && config_.high_speed_acceleration_gain_mps2>0) {
      const double ratio=clamp((speed-config_.drive_model_transition_low_mps)/
          (config_.drive_model_transition_high_mps-config_.drive_model_transition_low_mps),0.0,1.0);
      const double blend=ratio*ratio*(3-2*ratio);
      acceleration_gain+=(config_.high_speed_acceleration_gain_mps2-acceleration_gain)*blend;
      constant_drag+=(config_.high_speed_constant_drag_mps2-constant_drag)*blend;
      quadratic_drag+=(config_.high_speed_quadratic_drag_per_m-quadratic_drag)*blend;
    }
    double requested_acceleration =
        command >= 0.0 ? acceleration_gain * command
                       : braking_gain * command;
    if (config_.physical_rollout) {
      requested_acceleration -= constant_drag + quadratic_drag * speed * speed;
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
    speed = std::max(0.0, speed + next_acceleration * dt);
    result.maximum_speed_mps = std::max(result.maximum_speed_mps, speed);
    const double speed_error = reference.speed_mps - speed;
    const double acceleration_error = next_acceleration - reference.acceleration_mps2;
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
  const double terminal_error = references.back().speed_mps - speed;
  result.cost += config_.terminal_speed_weight * terminal_error * terminal_error;
  return result;
}

inline void LongitudinalMpc::solve(const std::vector<LongitudinalReference>& references,
                            double measured_speed_mps,
                            std::vector<double>* commands, double* cost,
                            double* predicted_maximum_speed_mps,
                            int* iterations) const {
  project(commands);
  RolloutResult best = rollout(*commands, references, measured_speed_mps);
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
          return rollout(candidate, references, measured_speed_mps).cost;
        },
        *commands, matrix, rows, lower, upper, solver_config);
    *commands = solved.solution;
    best = rollout(*commands, references, measured_speed_mps);
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
              rollout(candidate, references, measured_speed_mps);
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
              rollout(plus, references, measured_speed_mps).cost;
          const double minus_cost =
              rollout(minus, references, measured_speed_mps).cost;
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
            rollout(candidate, references, measured_speed_mps);
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
                                              double dt_sec) {
  // 기존 scalar API는 동일한 속도·가속도 0 참조를 반복한다.
  // 참조를 아직 연결하지 않은 모든 호출자의 계산과 warm start를 보존한다.
  return updateReference(std::vector<LongitudinalReference>(
      static_cast<std::size_t>(config_.horizon_steps), {target_speed_mps, 0.0}),
      measured_speed_mps, dt_sec);
}

inline LongitudinalMpcResult LongitudinalMpc::updateReference(
    const std::vector<LongitudinalReference>& references,
    double measured_speed_mps, double dt_sec) {
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
  solve(references, measured_speed_mps, &commands,
                       &result.cost, &result.predicted_maximum_speed_mps,
                       &result.iterations);
  result.estimated_acceleration_mps2 = estimated_acceleration_mps2_;

  // #STANDSTILL_HOLD
  // 예측 구간 전체가 정지 참조이고 실측 속도가 0.05m/s 이내이면 정지를
  // 유지한다. 기존 저항 모델은 v=0에도 가속 페달을 보상해 재출발시켰다.
  // 양의 출발 참조가 하나라도 있거나 아직 움직이는 동안에는 원래 MPC를 쓴다.
  // 제동 목표는 0.05이며 기존 최대 페달과 변화율 한도로 제한한다.
  // 실제 적용 명령은 previous_signed_command_에 보존한다. 미래 탐색의
  // warm start는 중립으로 두어 제동 구간에 갇혀 다음 출발이 막히지 않게 한다.
  const bool stationary_hold = measured_speed_mps <= 0.05 &&
      std::all_of(references.begin(), references.end(), [](const LongitudinalReference& r) {
        return r.speed_mps == 0.0 && r.acceleration_mps2 == 0.0;
      });
  double command = stationary_hold ? -std::min(0.05,config_.maximum_brake_command) :
                                    commands.front();
  const double actual_maximum_step = config_.command_rate_limit_per_sec * dt_sec;
  command = clamp(command, previous_signed_command_ - actual_maximum_step,
                  previous_signed_command_ + actual_maximum_step);
  command = clamp(command, -config_.maximum_brake_command,
                  config_.maximum_accel_command);
  if (stationary_hold) std::fill(commands.begin(),commands.end(),0.0);
  // #LONGITUDINAL_SNAPSHOT: 제동 0.05 초과의 실제 최적화 입력만 DEBUG로 기록한다.
  // 참조·추정 가속도·이전 명령·탐색 초기값을 같은 호출에서 보존하여 재생하며,
  // 계산 상태·반환 명령·한도를 바꾸지 않는다. DEBUG가 꺼지면 문자열도 만들지 않는다.
  ROS_DEBUG_STREAM_COND_NAMED(command < -0.05 && measured_speed_mps > 2.0,
      "longitudinal_snapshot", ([&] {
        std::ostringstream out;out.precision(17);
        out << "optimizer dt=" << dt_sec << " speed=" << measured_speed_mps
            << " estimated_acceleration=" << estimated_acceleration_mps2_
            << " previous_command=" << previous_signed_command_
            << " selected_command=" << command << " cost=" << result.cost
            << " iterations=" << result.iterations << " references=";
        for(const auto& r:references) out << r.speed_mps << ',' << r.acceleration_mps2 << ';';
        out << " warm_start=";
        for(double u:warm_start_) out << u << ';';
        out << " optimized=";
        for(double u:commands) out << u << ';';
        return out.str();
      }()));
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

inline void LongitudinalMpc::reset() {
  std::fill(warm_start_.begin(), warm_start_.end(), 0.0);
  has_previous_speed_ = false;
  previous_speed_mps_ = 0.0;
  estimated_acceleration_mps2_ = 0.0;
  previous_signed_command_ = 0.0;
}

}}  // namespace control::autoware_ai_mpc

#include <cmath>

namespace control { namespace autoware_ai_mpc {
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
}}  // namespace control::autoware_ai_mpc

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

namespace control { namespace autoware_ai_mpc {
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

inline void AutowareLateral::prepareTrajectory(const control::autoware_ai_mpc::autoware::Lane& path, double speed_mps) {
  // Coincident vertices describe the same geometry; omit zero-length segments
  // before distance interpolation to avoid division by zero.
  control::autoware_ai_mpc::autoware::Lane distinct;
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

inline double AutowareLateral::calculate(const control::autoware_ai_mpc::autoware::Lane& path, double speed_mps,
                                double modeled_steering_rad, double previous_steering_rad) {
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
  double yaw_err, dist_err, nearest_traj_time;
  geometry_msgs::Pose nearest_pose;
  MPCUtils::calcNearestPoseInterp(ref_traj_, control_pose, nearest_pose, nearest_index, dist_err, yaw_err, nearest_traj_time);

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
  MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_, mpc_time_v, mpc_resampled_ref_traj);

  /* predict dynamics for N times */
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
  const double first_change_weight=p_.morai_weight_first_steer_change *
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
  qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex);


  /* saturation */
  const double u_sat = std::max(std::min(Uex(0), u_lim), -u_lim);

  /* filtering */
  // With rate bounds referenced to the last sent command, an extra output LPF
  // silently reduces achievable slew and adds dynamics absent from the model.
  const double u_filtered = p_.morai_use_output_steering_filter ? lpf_steering_cmd_.filter(u_sat) : u_sat;

  // #MPC_LATERAL_DIAGNOSTIC
  // 경계 접근 전에 추종 오차·선행 조향과 첫 명령 억제 비용을 실측 기록과 비교한다.
  // DEBUG가 꺼져 있으면 문자열을 만들지 않으며 계산·명령·제약을 바꾸지 않는다.
  ROS_DEBUG_STREAM_NAMED("lateral_snapshot", "lateral speed=" << speed_mps
      << " ey=" << err_lat << " epsi=" << yaw_err
      << " modeled_steer=" << modeled_steering_rad
      << " previous=" << previous_steering_rad << " requested=" << u_filtered
      << " ref_k=" << mpc_resampled_ref_traj.k.front()
      << " ref_u=" << Urefex(0,0) << " first_change_weight=" << first_change_weight
      << " H00=" << H(0,0) << " R00=" << Rex(0,0)
      << " predicted_ey=" << x0(0) << " predicted_epsi=" << x0(1));

  // Keep the original optimizer-output delay buffer. The next call's first
  // transition separately references the final bounded Core command.
  if (!input_buffer_.empty()) {
    input_buffer_.push_back(u_filtered);
    input_buffer_.pop_front();
  }
  return u_filtered;
}

}}  // namespace control::autoware_ai_mpc

#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <morai_msgs/CtrlCmd.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace control {
class MoraiMpc {
 public:
  explicit MoraiMpc(autoware_ai_mpc::Parameters parameters = {})
      : parameters_(std::move(parameters)),
        lateral_(parameters_.lateral, parameters_.steering_prediction),
        longitudinal_(parameters_.longitudinal),
        steering_(parameters_.steering_prediction.delay_sec,
                  parameters_.steering_prediction.time_constant_sec) {}

  // #MPC_PROFILE_TIME
  // 최종 velocity_injected를 Path.header.stamp 기준의 시간 궤적으로 읽는다.
  // 점 간격은 불균일하며 pose.header.stamp는 관측 기준 시각이므로 사용하지 않는다.
  // v와 a를 끝점으로 갖는 Hermite 보간을 사용하고 끝점 범위를 넘지 않게 제한한다.
  // 현재 대기 구간은 기존 0속도 제어를 유지한다. 미래 정지점을 넘은 출발 참조는
  // 그 정지 시각을 지나기 전에는 사용하지 않는다. 계획 끝 밖으로 진행을 외삽하지 않는다.
  static std::vector<autoware_ai_mpc::LongitudinalReference> referenceHorizon(
      const interfaces::Path& path, double elapsed_sec,
      const autoware_ai_mpc::LongitudinalMpcConfig& config) {
    using Reference = autoware_ai_mpc::LongitudinalReference;
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
                             const interfaces::CompetitionVehicleStatus& status,
                             const ros::Time& control_stamp,
                             const interfaces::Path& planner_path) {
    // 원본 경로는 필수 계약 검사에만 사용하며 제어 계산은 주입 경로를 따른다.
    static_cast<void>(planner_path);
    ROS_INFO_ONCE("[control] executing morai_mpc.hpp with timed horizon reference");
    const auto stamp = control_stamp.toNSec();
    const double dt = has_previous_stamp_ ?
        std::max(parameters_.minimum_control_dt_sec,
          std::min(parameters_.maximum_control_dt_sec,
            (static_cast<double>(stamp) - static_cast<double>(previous_stamp_)) * 1e-9)) :
        parameters_.lateral.ctrl_period;
    const double time_sec = control_stamp.toSec();
    previous_stamp_ = stamp;
    has_previous_stamp_ = true;

    // 횡방향 Autoware 계산에는 기존과 같이 관측 속도를 사용한다.
    const double speed_mps = ego.twist.twist.linear.x;
    // #FRAME_TIME_ALIGNMENT
    // 상위가 실제 사용한 map→경로 frame 변환으로 현재 차체 좌표에 재표현한다.
    // 경로 계획 시각과 TF 시각이 같다고 가정하거나 과거 TF를 추정하지 않는다.
    // ego 이력은 현재 계산 시각의 자세에만 사용하고 속도·예정 시각은 유지한다.
    recordEgoPose(ego);
    tf2::Transform map_from_reference;
    tf2::fromMsg(path.map_from_path_frame.transform,map_from_reference);
    const auto current_from_reference=poseAt(time_sec).inverse()*map_from_reference;
    control::autoware_ai_mpc::autoware::Lane waypoints;
    waypoints.waypoints.reserve(path.points.size());
    for (const auto& point : path.points) {
      control::autoware_ai_mpc::autoware::Waypoint waypoint;
      waypoint.pose = point.pose;
      tf2::Transform reference_from_point;
      tf2::fromMsg(point.pose.pose,reference_from_point);
      tf2::toMsg(current_from_reference*reference_from_point,waypoint.pose.pose);
      waypoint.twist.twist.linear.x = std::max(0.5, std::abs(speed_mps));
      waypoints.waypoints.push_back(waypoint);
    }
    // #MEASURED_LATERAL_STATE
    // 차체가 이미 회전 중이어도 새 경로가 현재 자세에서 시작하면 횡/방향
    // 오차는 거의 0이다. 과거 명령만으로 추정한 조향 상태까지 틀리면 실제
    // 회전을 놓친다. 같은 ego의 body yaw rate와 전방 속도로 뒷차축 운동에
    // 해당하는 등가 조향 atan(L*r/v)를 구해 기존 kinematic MPC 초기 상태에
    // 넣는다. 실제 타이어 각도 측정으로 해석하지 않는다. 자세 전파와 같이
    // 관측 body twist를 계산 시각까지 일정하게 두는 근사이며 새 센서가 없다.
    // 0.5m/s 미만에서는 yaw/v가 불안정하므로 기존 명령 기반 상태를 사용한다.
    // 예측 모델·게인·입력 지연 보상·조향/변화율 한도·종방향 계산은 유지한다.
    // GT 객체나 지도 ID에 의존하지 않아 LiDAR 검출로 바뀌어도 같은 계산이다.
    const double predicted_steering_rad=steering_.advance(time_sec);
    const double initial_effective_steering_rad=std::abs(speed_mps)>=.5 ?
        std::atan(parameters_.lateral.vehicle_model_wheelbase*ego.twist.twist.angular.z/speed_mps) :
        predicted_steering_rad;
    const double requested = lateral_.calculate(waypoints, speed_mps,
        initial_effective_steering_rad, previous_steering_rad_);
    const double limit = parameters_.lateral.steer_lim_deg * autoware_ai_mpc::kPi / 180.0;
    const double step = parameters_.lateral.morai_steer_rate_limit_deg *
                        autoware_ai_mpc::kPi / 180.0 * dt;
    double angle = std::max(-limit, std::min(limit, requested));
    angle = std::max(previous_steering_rad_ - step,
                     std::min(previous_steering_rad_ + step, angle));
    const auto references = referenceHorizon(path, (control_stamp - path.header.stamp).toSec(),
                                            parameters_.longitudinal);
    const auto pedals = longitudinal_.updateReference(references, status.velocity_x_mps, dt);
    ROS_DEBUG_STREAM_COND_NAMED(pedals.brake > 0.05 && status.velocity_x_mps > 2.0,
        "longitudinal_snapshot", "input_stamps control_ns=" << control_stamp.toNSec()
        << " path_ns=" << path.header.stamp.toNSec()
        << " status_ns=" << status.header.stamp.toNSec()
        << " ego_ns=" << ego.header.stamp.toNSec());

    ROS_DEBUG_STREAM_NAMED("lateral_snapshot", "lateral_input control_ns=" << control_stamp.toNSec()
        << " path_ns=" << path.header.stamp.toNSec() << " ego_ns=" << ego.header.stamp.toNSec()
        << " status_ns=" << status.header.stamp.toNSec()
        << " measured_yaw_rate=" << ego.twist.twist.angular.z
        << " status_speed=" << status.velocity_x_mps << " bounded=" << angle
        << " predicted_steer=" << predicted_steering_rad
        << " initial_effective_steer=" << initial_effective_steering_rad);
    previous_steering_rad_ = angle;
    steering_.sent(time_sec, angle * parameters_.steering_prediction.gain);
    morai_msgs::CtrlCmd command;
    command.longlCmdType = 1;
    command.steering = angle / autoware_ai_mpc::kBridgeSteeringLimitRad;
    command.accel = pedals.accel;
    command.brake = pedals.brake;
    return command;
  }

 private:
  struct EgoPoseSample {
    double time_sec;
    tf2::Transform map_from_base;
    geometry_msgs::Twist body_twist;
  };
  std::deque<EgoPoseSample> ego_poses_;
  void recordEgoPose(const nav_msgs::Odometry& ego) {
    const double stamp=ego.header.stamp.toSec();
    if(!ego_poses_.empty() && stamp==ego_poses_.back().time_sec) return;
    // 입력이 이력 범위(1초)보다 오래 끊긴 구간을 양 끝 pose로 보간하지 않는다.
    // 복귀 뒤에는 첫 관측에서의 기존 body twist 전파 규칙으로 기준을 다시 잡는다.
    if(!ego_poses_.empty() &&
       (stamp<ego_poses_.back().time_sec || stamp-ego_poses_.back().time_sec>1.0))
      ego_poses_.clear();
    tf2::Transform pose;tf2::fromMsg(ego.pose.pose,pose);
    ego_poses_.push_back({stamp,pose,ego.twist.twist});
    while(ego_poses_.size()>2 && ego_poses_[1].time_sec<stamp-1) ego_poses_.pop_front();
  }
  static tf2::Transform propagatePose(const EgoPoseSample& sample,double stamp) {
    const double dt=stamp-sample.time_sec;
    const auto& twist=sample.body_twist;
    const tf2::Vector3 velocity(twist.linear.x,twist.linear.y,twist.linear.z);
    const tf2::Vector3 omega(twist.angular.x,twist.angular.y,twist.angular.z);
    const double rate=omega.length(),angle=rate*dt;
    double first=dt*dt/2,second=dt*dt*dt/6;
    if(std::abs(angle)>1e-5) {
      first=(1-std::cos(angle))/(rate*rate);
      second=(angle-std::sin(angle))/(rate*rate*rate);
    }
    const auto displacement=velocity*dt+omega.cross(velocity)*first+
                            omega.cross(omega.cross(velocity))*second;
    tf2::Quaternion rotation(0,0,0,1);
    if(rate>0) rotation=tf2::Quaternion(omega/rate,angle);
    return sample.map_from_base*tf2::Transform(rotation,displacement);
  }
  tf2::Transform poseAt(double stamp) const {
    if(stamp<=ego_poses_.front().time_sec) return propagatePose(ego_poses_.front(),stamp);
    for(std::size_t i=1;i<ego_poses_.size();++i) {
      const auto& before=ego_poses_[i-1];const auto& after=ego_poses_[i];
      if(stamp<=after.time_sec) {
        const double ratio=(stamp-before.time_sec)/(after.time_sec-before.time_sec);
        return tf2::Transform(before.map_from_base.getRotation().slerp(after.map_from_base.getRotation(),ratio),
            before.map_from_base.getOrigin().lerp(after.map_from_base.getOrigin(),ratio));
      }
    }
    return propagatePose(ego_poses_.back(),stamp);
  }
  autoware_ai_mpc::Parameters parameters_;
  autoware_ai_mpc::AutowareLateral lateral_;
  autoware_ai_mpc::LongitudinalMpc longitudinal_;
  autoware_ai_mpc::SteeringPrediction steering_;
  std::uint64_t previous_stamp_ = 0;
  bool has_previous_stamp_ = false;
  double previous_steering_rad_ = 0.0;
};
}  // namespace control
