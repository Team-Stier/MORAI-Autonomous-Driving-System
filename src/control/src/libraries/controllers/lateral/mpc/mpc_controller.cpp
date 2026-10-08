#include "control/libraries/tracking/controllers/lateral/mpc/mpc_controller.hpp"
#include "control/libraries/tracking/common/osqp_sqp_solver.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
namespace morai_mpc {
namespace {

void addSquaredAffineCost(const std::vector<double>& coefficients,
                          double constant, double weight,
                          std::vector<double>* hessian,
                          std::vector<double>* gradient) {
  const int variables = static_cast<int>(coefficients.size());
  for (int row = 0; row < variables; ++row) {
    (*gradient)[static_cast<std::size_t>(row)] +=
        2.0 * weight * constant * coefficients[static_cast<std::size_t>(row)];
    for (int column = 0; column < variables; ++column) {
      (*hessian)[static_cast<std::size_t>(row * variables + column)] +=
          2.0 * weight * coefficients[static_cast<std::size_t>(row)] *
          coefficients[static_cast<std::size_t>(column)];
    }
  }
}

bool fitQuadraticPath(const std::vector<Point2d>& samples,
                      double x_scale, double* intercept,
                      double* slope) {
  if (samples.size() < 3U || !finite(x_scale) || x_scale <= 0.0 ||
      intercept == nullptr || slope == nullptr) {
    return false;
  }
  double augmented[3][4]{};
  for (const Point2d& sample : samples) {
    const double q = sample.x / x_scale;
    const double basis[3]{1.0, q, q * q};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        augmented[row][column] += basis[row] * basis[column];
      }
      augmented[row][3] += basis[row] * sample.y;
    }
  }
  for (int pivot = 0; pivot < 3; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < 3; ++row) {
      if (std::abs(augmented[row][pivot]) >
          std::abs(augmented[best][pivot])) {
        best = row;
      }
    }
    if (std::abs(augmented[best][pivot]) < 1.0e-10) return false;
    if (best != pivot) {
      for (int column = pivot; column < 4; ++column) {
        std::swap(augmented[pivot][column], augmented[best][column]);
      }
    }
    const double divisor = augmented[pivot][pivot];
    for (int column = pivot; column < 4; ++column) {
      augmented[pivot][column] /= divisor;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == pivot) continue;
      const double factor = augmented[row][pivot];
      for (int column = pivot; column < 4; ++column) {
        augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
  }
  *intercept = augmented[0][3];
  *slope = augmented[1][3] / x_scale;
  return finite(*intercept) && finite(*slope);
}

}  // namespace

MpcController::MpcController(const MpcConfig& c,std::unique_ptr<Optimizer> o):config_(c),optimizer_(std::move(o)),warm_start_(c.horizon,0.0),active_w_lateral_(c.w_lateral),active_w_heading_(c.w_heading),active_w_steering_(c.w_steering),active_w_rate_(c.w_rate),active_w_rate_change_(c.w_rate_change),active_w_terminal_lateral_(c.w_terminal_lateral),active_w_terminal_heading_(c.w_terminal_heading),active_horizon_(c.horizon){if(!valid(c))throw std::invalid_argument("invalid MPC config");if(!optimizer_){if(c.solver=="osqp")optimizer_.reset(new OsqpSqpSearch);else if(c.solver=="projected_gradient")optimizer_.reset(new ProjectedGradientSearch);else optimizer_.reset(new ProjectedCoordinateSearch);}int delay_ticks=static_cast<int>(std::lround(c.actuator_delay/c.dt));actuator_delay_line_.assign(static_cast<std::size_t>(std::max(0,delay_ticks)),0.0);}
double MpcController::rolloutCost(const MpcState& initial,double start_s,const ReferencePath& path,const std::vector<double>& u,double previous_control,double speed)const{
  MpcState x=initial;double s=start_s,cost=0,prev=previous_control,applied_rate=predicted_applied_rate_;auto delay_line=actuator_delay_line_;
  for(int k=0;k<active_horizon_;++k){auto ref=path.sample(s);double dref=std::atan(config_.wheelbase*ref.curvature);double uk=u[static_cast<std::size_t>(k)];
    cost+=active_w_lateral_*x.lateral_error*x.lateral_error+active_w_heading_*x.heading_error*x.heading_error+active_w_steering_*(x.steering-dref)*(x.steering-dref)+active_w_rate_*uk*uk+active_w_rate_change_*(uk-prev)*(uk-prev);
    double denom=1.0-ref.curvature*x.lateral_error;if(std::abs(denom)<0.2)denom=denom<0?-0.2:0.2;
    x.lateral_error+=config_.dt*speed*std::sin(x.heading_error);
    x.heading_error=normalizeAngle(x.heading_error+config_.dt*(speed/config_.wheelbase*std::tan(x.steering)-speed*ref.curvature*std::cos(x.heading_error)/denom));
    double predicted_rate=uk;
    if(config_.optimizer_model_actuator&&(config_.actuator_delay>0||config_.actuator_time_constant>0)){
      double delayed=uk;if(!delay_line.empty()){delayed=delay_line.front();delay_line.pop_front();delay_line.push_back(uk);}
      if(config_.actuator_time_constant>0){double alpha=1.0-std::exp(-config_.dt/config_.actuator_time_constant);applied_rate+=alpha*(delayed-applied_rate);}else applied_rate=delayed;
      predicted_rate=applied_rate;
    }
    x.steering=clamp(x.steering+config_.dt*predicted_rate,-config_.max_steering,config_.max_steering);s+=config_.dt*speed*std::cos(x.heading_error)/denom;prev=uk;
    if(!finite(x.lateral_error)||std::abs(x.lateral_error)>100)return 1e100;
  }
  cost+=active_w_terminal_lateral_*x.lateral_error*x.lateral_error+active_w_terminal_heading_*x.heading_error*x.heading_error;return cost;
}

OptimizationResult MpcController::solveLinearizedOsqp(
    const MpcState& initial, double start_s, const ReferencePath& path,
    const std::vector<double>& seed, double previous_control,
    double speed) const {
  OptimizationResult result;
  const int horizon = active_horizon_;
  // The straight dynamic bicycle model already represents the physical yaw
  // response. Treating that response as an additional steering-actuator delay
  // double-counts the lag and was observed to amplify weaving.
  const bool model_actuator =
      config_.optimizer_model_actuator && !active_dynamic_model_;
  const int delay_states = model_actuator
                               ? static_cast<int>(actuator_delay_line_.size())
                               : 0;
  const int base_state_count = active_dynamic_model_ ? 5 : 3;
  const int state_count =
      base_state_count + (model_actuator ? 1 + delay_states : 0);
  constexpr int kLateral = 0;
  constexpr int kHeading = 1;
  constexpr int kSteering = 2;
  constexpr int kSideslip = 3;
  constexpr int kYawRate = 4;
  const int applied_rate_index = base_state_count;
  const int delay_start_index = applied_rate_index + 1;
  std::vector<double> state(static_cast<std::size_t>(state_count), 0.0);
  state[kLateral] = initial.lateral_error;
  state[kHeading] = initial.heading_error;
  state[kSteering] = initial.steering;
  if (active_dynamic_model_) {
    state[kSideslip] = initial.sideslip;
    state[kYawRate] = initial.yaw_rate;
  }
  if (model_actuator) {
    state[static_cast<std::size_t>(applied_rate_index)] =
        predicted_applied_rate_;
    int delay_index = 0;
    for (double delayed_command : actuator_delay_line_) {
      state[static_cast<std::size_t>(delay_start_index + delay_index)] =
          delayed_command;
      ++delay_index;
    }
  }

  // Each state is represented as response*u + state. Condensing the dynamics
  // this way yields the exact dense temporal Hessian for the linearized model.
  std::vector<double> response(
      static_cast<std::size_t>(state_count * horizon), 0.0);
  std::vector<double> hessian(
      static_cast<std::size_t>(horizon * horizon), 0.0);
  std::vector<double> gradient(static_cast<std::size_t>(horizon), 0.0);
  const int constraint_rows = 2 * horizon;
  std::vector<double> constraints(
      static_cast<std::size_t>(constraint_rows * horizon), 0.0);
  std::vector<double> lower(static_cast<std::size_t>(constraint_rows), 0.0);
  std::vector<double> upper(static_cast<std::size_t>(constraint_rows), 0.0);
  for (int index = 0; index < horizon; ++index) {
    constraints[static_cast<std::size_t>(index * horizon + index)] = 1.0;
    lower[static_cast<std::size_t>(index)] = -config_.max_steering_rate;
    upper[static_cast<std::size_t>(index)] = config_.max_steering_rate;
  }

  auto stateRow = [&](int state_index) {
    return std::vector<double>(
        response.begin() + static_cast<std::ptrdiff_t>(state_index * horizon),
        response.begin() +
            static_cast<std::ptrdiff_t>((state_index + 1) * horizon));
  };

  for (int step = 0; step < horizon; ++step) {
    const ReferencePoint reference =
        path.sample(start_s + static_cast<double>(step) * speed * config_.dt);
    const double desired_steering =
        std::atan(config_.wheelbase * reference.curvature);
    addSquaredAffineCost(stateRow(kLateral), state[kLateral],
                         active_w_lateral_, &hessian, &gradient);
    addSquaredAffineCost(stateRow(kHeading), state[kHeading],
                         active_w_heading_, &hessian, &gradient);
    addSquaredAffineCost(stateRow(kSteering),
                         state[kSteering] - desired_steering,
                         active_w_steering_, &hessian, &gradient);
    if (active_dynamic_model_) {
      addSquaredAffineCost(stateRow(kSideslip), state[kSideslip],
                           config_.straight_dynamic_sideslip_weight,
                           &hessian, &gradient);
      addSquaredAffineCost(
          stateRow(kYawRate),
          state[kYawRate] - speed * reference.curvature,
          config_.straight_dynamic_yaw_rate_weight,
          &hessian, &gradient);
    }
    hessian[static_cast<std::size_t>(step * horizon + step)] +=
        2.0 * active_w_rate_;
    if (step == 0) {
      hessian[0] += 2.0 * active_w_rate_change_;
      gradient[0] -= 2.0 * active_w_rate_change_ * previous_control;
    } else {
      const int previous = step - 1;
      hessian[static_cast<std::size_t>(step * horizon + step)] +=
          2.0 * active_w_rate_change_;
      hessian[static_cast<std::size_t>(previous * horizon + previous)] +=
          2.0 * active_w_rate_change_;
      hessian[static_cast<std::size_t>(step * horizon + previous)] -=
          2.0 * active_w_rate_change_;
      hessian[static_cast<std::size_t>(previous * horizon + step)] -=
          2.0 * active_w_rate_change_;
    }

    std::vector<double> transition(
        static_cast<std::size_t>(state_count * state_count), 0.0);
    std::vector<double> input(static_cast<std::size_t>(state_count), 0.0);
    std::vector<double> affine(static_cast<std::size_t>(state_count), 0.0);
    transition[static_cast<std::size_t>(kLateral * state_count + kLateral)] =
        1.0;
    transition[static_cast<std::size_t>(kLateral * state_count + kHeading)] =
        config_.dt * speed;
    transition[static_cast<std::size_t>(kHeading * state_count + kHeading)] =
        1.0;
    if (active_dynamic_model_) {
      transition[static_cast<std::size_t>(kLateral * state_count +
                                           kSideslip)] = config_.dt * speed;
      transition[static_cast<std::size_t>(kHeading * state_count + kYawRate)] =
          config_.dt;
      affine[kHeading] = -config_.dt * speed * reference.curvature;

      const double mass = config_.dynamic_mass;
      const double inertia = config_.dynamic_yaw_inertia;
      const double front_stiffness =
          config_.dynamic_front_cornering_stiffness;
      const double rear_stiffness =
          config_.dynamic_rear_cornering_stiffness;
      const double front_distance = config_.dynamic_front_axle_to_cg;
      const double rear_distance = config_.dynamic_rear_axle_to_cg;
      const double speed_squared = speed * speed;
      const double a00 =
          -2.0 * (front_stiffness + rear_stiffness) / (mass * speed);
      const double a01 =
          -1.0 -
          2.0 * (front_stiffness * front_distance -
                 rear_stiffness * rear_distance) /
              (mass * speed_squared);
      const double a10 =
          -2.0 * (front_stiffness * front_distance -
                 rear_stiffness * rear_distance) /
          inertia;
      const double a11 =
          -2.0 * (front_stiffness * front_distance * front_distance +
                 rear_stiffness * rear_distance * rear_distance) /
          (inertia * speed);
      const double b0 = 2.0 * front_stiffness / (mass * speed);
      const double b1 =
          2.0 * front_stiffness * front_distance / inertia;
      transition[static_cast<std::size_t>(kSideslip * state_count +
                                           kSideslip)] =
          1.0 + config_.dt * a00;
      transition[static_cast<std::size_t>(kSideslip * state_count +
                                           kYawRate)] = config_.dt * a01;
      transition[static_cast<std::size_t>(kSideslip * state_count +
                                           kSteering)] = config_.dt * b0;
      transition[static_cast<std::size_t>(kYawRate * state_count +
                                           kSideslip)] = config_.dt * a10;
      transition[static_cast<std::size_t>(kYawRate * state_count + kYawRate)] =
          1.0 + config_.dt * a11;
      transition[static_cast<std::size_t>(kYawRate * state_count +
                                           kSteering)] = config_.dt * b1;
    } else {
      const double cosine = std::cos(desired_steering);
      const double steering_slope =
          config_.dt * speed /
          (config_.wheelbase * std::max(0.1, cosine * cosine));
      transition[static_cast<std::size_t>(kHeading * state_count +
                                           kSteering)] = steering_slope;
      affine[kHeading] =
          config_.dt * (speed / config_.wheelbase) *
              (std::tan(desired_steering) -
               desired_steering / std::max(0.1, cosine * cosine)) -
          config_.dt * speed * reference.curvature;
    }

    if (!model_actuator) {
      transition[static_cast<std::size_t>(kSteering * state_count + kSteering)] =
          1.0;
      input[kSteering] = config_.dt;
    } else {
      const double alpha = config_.actuator_time_constant > 0.0
                               ? 1.0 - std::exp(-config_.dt /
                                                config_.actuator_time_constant)
                               : 1.0;
      transition[static_cast<std::size_t>(kSteering * state_count + kSteering)] =
          1.0;
      transition[static_cast<std::size_t>(kSteering * state_count +
                                           applied_rate_index)] =
          config_.dt * (1.0 - alpha);
      transition[static_cast<std::size_t>(applied_rate_index * state_count +
                                           applied_rate_index)] =
          1.0 - alpha;
      if (delay_states > 0) {
        transition[static_cast<std::size_t>(kSteering * state_count +
                                             delay_start_index)] =
            config_.dt * alpha;
        transition[static_cast<std::size_t>(applied_rate_index * state_count +
                                             delay_start_index)] =
            alpha;
        for (int delay = 0; delay + 1 < delay_states; ++delay) {
          transition[static_cast<std::size_t>(
              (delay_start_index + delay) * state_count +
              delay_start_index + delay + 1)] =
              1.0;
        }
        input[static_cast<std::size_t>(delay_start_index + delay_states - 1)] =
            1.0;
      } else {
        input[kSteering] = config_.dt * alpha;
        input[static_cast<std::size_t>(applied_rate_index)] = alpha;
      }
    }

    std::vector<double> next_state(static_cast<std::size_t>(state_count), 0.0);
    std::vector<double> next_response(
        static_cast<std::size_t>(state_count * horizon), 0.0);
    for (int row = 0; row < state_count; ++row) {
      next_state[static_cast<std::size_t>(row)] =
          affine[static_cast<std::size_t>(row)];
      for (int column = 0; column < state_count; ++column) {
        const double value = transition[
            static_cast<std::size_t>(row * state_count + column)];
        next_state[static_cast<std::size_t>(row)] +=
            value * state[static_cast<std::size_t>(column)];
        for (int control = 0; control < horizon; ++control) {
          next_response[static_cast<std::size_t>(row * horizon + control)] +=
              value * response[
                          static_cast<std::size_t>(column * horizon + control)];
        }
      }
      next_response[static_cast<std::size_t>(row * horizon + step)] +=
          input[static_cast<std::size_t>(row)];
    }
    const int steering_constraint_row = horizon + step;
    for (int control = 0; control < horizon; ++control) {
      constraints[static_cast<std::size_t>(steering_constraint_row * horizon +
                                           control)] =
          next_response[static_cast<std::size_t>(kSteering * horizon + control)];
    }
    lower[static_cast<std::size_t>(steering_constraint_row)] =
        -config_.max_steering - next_state[kSteering];
    upper[static_cast<std::size_t>(steering_constraint_row)] =
        config_.max_steering - next_state[kSteering];
    state = std::move(next_state);
    response = std::move(next_response);
  }

  addSquaredAffineCost(stateRow(kLateral), state[kLateral],
                       active_w_terminal_lateral_, &hessian, &gradient);
  addSquaredAffineCost(stateRow(kHeading), state[kHeading],
                       active_w_terminal_heading_, &hessian, &gradient);
  for (int index = 0; index < horizon; ++index) {
    hessian[static_cast<std::size_t>(index * horizon + index)] += 1.0e-6;
  }
  control::tracking::OsqpQpConfig qp_config;
  qp_config.maximum_iterations = 1000;
  qp_config.warm_start = config_.optimizer_use_warm_start;
  const control::tracking::OsqpSqpResult solved =
      control::tracking::solveQuadraticProgram(
          hessian, gradient, constraints, constraint_rows, lower, upper,
          seed, qp_config);
  result.success = solved.success;
  result.controls = solved.solution;
  result.iterations = solved.iterations;
  result.message = solved.message;
  if (result.success) {
    result.cost = rolloutCost(initial, start_s, path, result.controls,
                              previous_control, speed);
  }
  return result;
}
std::vector<ReferencePoint> MpcController::referenceHorizon(double start_s,const ReferencePath& path)const{
  if(path.empty())throw std::invalid_argument("empty reference path");
  int horizon=effectiveHorizon(config_,config_.speed);std::vector<ReferencePoint> refs;refs.reserve(static_cast<std::size_t>(horizon));
  for(int k=0;k<horizon;++k)refs.push_back(path.sample(start_s+k*config_.speed*config_.dt));
  return refs;
}
MpcCommand MpcController::compute(const VehicleState& v,const ReferencePath& path,double measurement_dt_sec){
  auto begin=std::chrono::steady_clock::now();MpcCommand cmd;
  if(!valid(v)||path.empty()||!finite(measurement_dt_sec)||measurement_dt_sec<=0.0){cmd.message="차량 상태, 경로 또는 측정 주기가 유효하지 않음";return cmd;}
  last_progress_=progress_tracker_.project(path,{v.x,v.y,v.yaw},v.speed);last_projection_=last_progress_.projection;if(!last_projection_.valid){cmd.message="경로 투영 실패";return cmd;}
  active_horizon_=effectiveHorizon(config_,v.speed);if(warm_start_.size()!=static_cast<std::size_t>(active_horizon_))warm_start_.resize(static_cast<std::size_t>(active_horizon_),warm_start_.empty()?0.0:warm_start_.back());
  double preview_curvature=0.0;
  for(int k=0;k<active_horizon_;++k){preview_curvature=std::max(preview_curvature,std::abs(path.sample(last_projection_.s+k*v.speed*config_.dt).curvature));}
  const double curve_blend=clamp(
      (preview_curvature-config_.adaptation_start_curvature)/
          (config_.adaptation_full_curvature-
           config_.adaptation_start_curvature),
      0.0,1.0);
  last_raw_lateral_error_=last_projection_.lateral_error;
  double controller_lateral_error=last_raw_lateral_error_;
  double spatial_fit_blend=0.0;
  if(config_.straight_spatial_fit_enabled){
    const double speed_span=std::max(1.0e-6,
        config_.straight_spatial_fit_full_speed-
        config_.straight_spatial_fit_start_speed);
    double speed_blend=clamp((std::abs(v.speed)-
        config_.straight_spatial_fit_start_speed)/speed_span,0.0,1.0);
    speed_blend=speed_blend*speed_blend*(3.0-2.0*speed_blend);
    const double curvature_span=
        config_.straight_spatial_fit_off_curvature-
        config_.straight_spatial_fit_full_curvature;
    double curvature_blend=1.0-clamp((preview_curvature-
        config_.straight_spatial_fit_full_curvature)/curvature_span,0.0,1.0);
    curvature_blend=curvature_blend*curvature_blend*
        (3.0-2.0*curvature_blend);
    spatial_fit_blend=speed_blend*curvature_blend;
    if(spatial_fit_blend>0.0){
      const double preview_distance=std::min(
          config_.straight_spatial_fit_maximum_preview,
          config_.straight_spatial_fit_base_preview+
              config_.straight_spatial_fit_speed_gain*std::abs(v.speed));
      const int sample_count=std::max(3,
          static_cast<int>(std::ceil(preview_distance))+1);
      const double cosine=std::cos(v.yaw),sine=std::sin(v.yaw);
      std::vector<Point2d> local_samples;
      local_samples.reserve(static_cast<std::size_t>(sample_count));
      double sum_x=0.0,sum_y=0.0,sum_xx=0.0,sum_xy=0.0;
      for(int sample_index=0;sample_index<sample_count;++sample_index){
        const double ratio=static_cast<double>(sample_index)/
            static_cast<double>(sample_count-1);
        const ReferencePoint reference=path.sample(
            last_projection_.s+ratio*preview_distance);
        const double dx=reference.x-v.x,dy=reference.y-v.y;
        const double local_x=cosine*dx+sine*dy;
        const double local_y=-sine*dx+cosine*dy;
        local_samples.push_back({local_x,local_y});
        sum_x+=local_x;sum_y+=local_y;
        sum_xx+=local_x*local_x;sum_xy+=local_x*local_y;
      }
      const double count=static_cast<double>(sample_count);
      const double denominator=count*sum_xx-sum_x*sum_x;
      double linear_intercept=0.0,linear_slope=0.0;
      const bool linear_fit_valid=std::abs(denominator)>1.0e-9;
      if(linear_fit_valid){
        linear_slope=(count*sum_xy-sum_x*sum_y)/denominator;
        linear_intercept=(sum_y-linear_slope*sum_x)/count;
      }
      double quadratic_intercept=0.0,quadratic_slope=0.0;
      const bool quadratic_fit_valid=fitQuadraticPath(
          local_samples,preview_distance,&quadratic_intercept,
          &quadratic_slope);
      const bool fit_valid=linear_fit_valid||quadratic_fit_valid;
      if(fit_valid){
        // An endpoint derivative from a quadratic fit preserves a real curve,
        // but it is unnecessarily sensitive to the short RDDF segment knots
        // on a straight.  The line fit supplies Pure-Pursuit-like spatial
        // averaging there; the existing curvature adaptation brings the
        // quadratic tangent back continuously before meaningful curves.
        double intercept=quadratic_intercept;
        double slope=quadratic_slope;
        if(linear_fit_valid&&quadratic_fit_valid){
          intercept=lerp(linear_intercept,quadratic_intercept,curve_blend);
          const double linear_heading=-std::atan(linear_slope);
          const double quadratic_heading=-std::atan(quadratic_slope);
          const double fitted_heading_error=normalizeAngle(
              linear_heading+curve_blend*normalizeAngle(
                  quadratic_heading-linear_heading));
          slope=-std::tan(fitted_heading_error);
        }else if(linear_fit_valid){
          intercept=linear_intercept;
          slope=linear_slope;
        }
        const double fitted_lateral_error=-intercept/std::hypot(1.0,slope);
        const double fitted_heading_error=-std::atan(slope);
        controller_lateral_error=lerp(last_raw_lateral_error_,
            fitted_lateral_error,spatial_fit_blend);
        last_projection_.heading_error=normalizeAngle(
            last_projection_.heading_error+spatial_fit_blend*normalizeAngle(
                fitted_heading_error-last_projection_.heading_error));
      }
    }
  }
  const bool filter_lateral_error=(config_.straight_cte_filter_time_constant>0.0||
      config_.straight_cte_maximum_rate>0.0)&&
      std::abs(v.speed)>=config_.straight_cte_filter_minimum_speed&&
      preview_curvature<=config_.straight_cte_filter_maximum_curvature;
  if(!has_filtered_lateral_error_){filtered_lateral_error_=controller_lateral_error;has_filtered_lateral_error_=true;}
  if(filter_lateral_error){
    double bounded_lateral_error=controller_lateral_error;
    if(config_.straight_cte_maximum_rate>0.0){
      const double maximum_delta=config_.straight_cte_maximum_rate*measurement_dt_sec;
      bounded_lateral_error=clamp(controller_lateral_error,
          filtered_lateral_error_-maximum_delta,
          filtered_lateral_error_+maximum_delta);
    }
    if(config_.straight_cte_filter_time_constant>0.0){
      const double alpha=1.0-std::exp(-measurement_dt_sec/config_.straight_cte_filter_time_constant);
      filtered_lateral_error_+=alpha*(bounded_lateral_error-filtered_lateral_error_);
    }else filtered_lateral_error_=bounded_lateral_error;
  }else filtered_lateral_error_=controller_lateral_error;
  last_projection_.lateral_error=filtered_lateral_error_;
  active_dynamic_model_=config_.straight_dynamic_model_enabled&&
      std::abs(v.speed)>=config_.straight_dynamic_model_minimum_speed&&
      preview_curvature<=config_.straight_dynamic_model_maximum_curvature;
  const double sideslip=std::abs(v.speed)>0.1?
      std::atan2(v.lateral_velocity,std::abs(v.speed)):0.0;
  MpcState x{filtered_lateral_error_,last_projection_.heading_error,v.steering,
             sideslip,v.yaw_rate};
  active_w_lateral_=config_.w_lateral*lerp(1.0,
      config_.straight_spatial_fit_lateral_weight_multiplier,
      spatial_fit_blend);
  active_w_heading_=config_.w_heading*lerp(1.0,config_.curve_heading_weight_multiplier,curve_blend);
  active_w_steering_=config_.w_steering*lerp(1.0,config_.curve_steering_weight_multiplier,curve_blend);
  active_w_rate_=config_.w_rate*lerp(config_.straight_rate_weight_multiplier,1.0,curve_blend);
  active_w_rate_change_=config_.w_rate_change*lerp(config_.straight_rate_change_weight_multiplier,config_.curve_rate_change_weight_multiplier,curve_blend);
  active_w_terminal_lateral_=config_.w_terminal_lateral*lerp(1.0,
      config_.straight_spatial_fit_lateral_weight_multiplier,
      spatial_fit_blend);
  active_w_terminal_heading_=config_.w_terminal_heading*lerp(1.0,config_.curve_heading_weight_multiplier,curve_blend);
  const double continuity_reference=config_.use_previous_control_rate_change?previous_control_:0.0;
  auto objective=[&](const std::vector<double>& u){return rolloutCost(x,last_projection_.s,path,u,continuity_reference,v.speed);};
  std::vector<double> solver_seed=warm_start_;
  if(config_.curve_feedforward_seed_gain>0.0&&curve_blend>0.0){
    double predicted_steering=v.steering;
    for(int k=0;k<active_horizon_;++k){
      const double sample_s=last_projection_.s+k*v.speed*config_.dt+config_.curve_feedforward_seed_lookahead;
      const double target_steering=std::atan(config_.wheelbase*path.sample(sample_s).curvature);
      const double target_rate=clamp((target_steering-predicted_steering)/config_.dt,-config_.max_steering_rate,config_.max_steering_rate);
      const std::size_t index=static_cast<std::size_t>(k);
      solver_seed[index]=lerp(solver_seed[index],target_rate,config_.curve_feedforward_seed_gain*curve_blend);
      predicted_steering=clamp(predicted_steering+solver_seed[index]*config_.dt,-config_.max_steering,config_.max_steering);
    }
  }
  MpcConfig solve_config=config_;solve_config.horizon=active_horizon_;auto r=config_.solver=="osqp"?solveLinearizedOsqp(x,last_projection_.s,path,solver_seed,continuity_reference,v.speed):optimizer_->solve(objective,solver_seed,v.steering,solve_config);cmd.success=r.success;cmd.iterations=r.iterations;cmd.cost=r.cost;cmd.message=r.message;
  if(r.success&&!r.controls.empty()){cmd.raw_steering_rate=clamp(r.controls.front(),-config_.max_steering_rate,config_.max_steering_rate);warm_start_=shiftedWarmStart(r.controls,active_horizon_,config_.optimizer_use_warm_start);}else{cmd.raw_steering_rate=clamp(-v.steering/config_.dt,-config_.max_steering_rate,config_.max_steering_rate);warm_start_.assign(static_cast<std::size_t>(active_horizon_),0.0);}
  const bool filter_steering_rate=config_.straight_steering_rate_filter_time_constant>0.0&&
      std::abs(v.speed)>=config_.straight_steering_rate_filter_minimum_speed&&
      preview_curvature<=config_.straight_steering_rate_filter_maximum_curvature;
  if(!has_filtered_steering_rate_){filtered_steering_rate_=cmd.raw_steering_rate;has_filtered_steering_rate_=true;}
  if(filter_steering_rate){
    const double alpha=1.0-std::exp(-measurement_dt_sec/config_.straight_steering_rate_filter_time_constant);
    filtered_steering_rate_+=alpha*(cmd.raw_steering_rate-filtered_steering_rate_);
  }else filtered_steering_rate_=cmd.raw_steering_rate;
  cmd.steering_rate=clamp(filtered_steering_rate_,-config_.max_steering_rate,config_.max_steering_rate);
  previous_control_=cmd.steering_rate;
  if(config_.optimizer_model_actuator&&(config_.actuator_delay>0||config_.actuator_time_constant>0)){
    double delayed=cmd.steering_rate;if(!actuator_delay_line_.empty()){delayed=actuator_delay_line_.front();actuator_delay_line_.pop_front();actuator_delay_line_.push_back(cmd.steering_rate);}
    if(config_.actuator_time_constant>0){double alpha=1.0-std::exp(-config_.dt/config_.actuator_time_constant);predicted_applied_rate_+=alpha*(delayed-predicted_applied_rate_);}else predicted_applied_rate_=delayed;
  }else predicted_applied_rate_=cmd.steering_rate;
  cmd.solve_time_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return cmd;
}
}  // namespace morai_mpc
