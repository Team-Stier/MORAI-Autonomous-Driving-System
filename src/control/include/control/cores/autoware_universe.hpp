#pragma once
#include <ros/console.h>
#include <osqp.h>

#include <control/libraries/autoware_universe/mpc/mpc.hpp>
#include <control/libraries/autoware_universe/pid/pid_longitudinal_controller.hpp>
namespace control { namespace universe {
namespace lateral = autoware::motion::control::mpc_lateral_controller;
namespace longitudinal = autoware::motion::control::pid_longitudinal_controller;
struct Parameters {
  lateral::MPCParam lateral;
  lateral::TrajectoryFilteringParam trajectory_filter;
  longitudinal::PidLongitudinalControllerConfig longitudinal;
  double wheelbase_m = 3.0, maximum_steering_angle_rad = 0.6981317007977318;
  double nominal_dt_sec = 0.05, minimum_control_dt_sec = 0.005, maximum_control_dt_sec = 0.10;
  double steering_lpf_cutoff_hz = 3.0, error_deriv_lpf_cutoff_hz = 5.0;
  double stop_state_entry_ego_speed = 0.001, stop_state_entry_target_speed = 0.001;
  double converged_steer_rad = 0.1, new_traj_duration_time = 1.0, new_traj_end_dist = 0.3;
  double mpc_converged_threshold_rps = 0.01;
  bool use_steer_prediction = false, keep_steer_control_until_converged = true;
  double acceleration_to_pedal_mps2 = 3.0, braking_to_pedal_mps2 = 6.0;
  double maximum_accel_pedal = 0.7, maximum_brake_pedal = 0.6;
  std::vector<std::pair<double,double>> steer_rate_by_curvature{{0.001,40*M_PI/180},{0.002,50*M_PI/180},{0.01,60*M_PI/180}};
  std::vector<std::pair<double,double>> steer_rate_by_velocity{{10,60*M_PI/180},{15,50*M_PI/180},{20,40*M_PI/180}};
};
Parameters defaultParameters();
}}

#include <control/libraries/autoware_universe/mpc/qp_solver/qp_solver_interface.hpp>
#include <memory>
namespace control { namespace universe {
// OSQP 0.6 C API implementation of Universe's QP interface.
class OsqpSolver final : public autoware::motion::control::mpc_lateral_controller::QPSolverInterface {
 public:
  OsqpSolver();
  ~OsqpSolver() override;
  autoware::motion::control::mpc_lateral_controller::QPSolverResult solve(
    const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&,
    const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
    const Eigen::VectorXd&, Eigen::VectorXd&) override;
  int64_t getTakenIter() const override;
  double getObjVal() const override;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}}

#include <memory>
namespace control { namespace universe {
class Controller {
 public:
  explicit Controller(const Parameters&);
  ~Controller();
  Controller(Controller&&) noexcept;
  Controller& operator=(Controller&&) noexcept;
  std::pair<autoware_control_msgs::msg::Lateral, autoware_control_msgs::msg::Longitudinal> calculate(
    const autoware_planning_msgs::msg::Trajectory&, const nav_msgs::Odometry&,
    double measured_steering_rad, double measured_acceleration_mps2, const ros::Time&);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}}

namespace control { namespace universe {
inline Parameters defaultParameters() {
  Parameters p{};
  p.trajectory_filter.traj_resample_dist = 0.1;
  p.use_steer_prediction = false;
  p.trajectory_filter.enable_path_smoothing = false;
  p.trajectory_filter.path_filter_moving_ave_num = 25;
  p.trajectory_filter.curvature_smoothing_num_traj = 15;
  p.trajectory_filter.curvature_smoothing_num_ref_steer = 15;
  p.trajectory_filter.extend_trajectory_for_end_yaw_control = false;
  p.lateral.prediction_horizon = 50;
  p.lateral.prediction_dt = 0.1;
  p.lateral.nominal_weight.lat_error = 1.0;
  p.lateral.nominal_weight.heading_error = 0.0;
  p.lateral.nominal_weight.heading_error_squared_vel = 0.3;
  p.lateral.nominal_weight.steering_input = 1.0;
  p.lateral.nominal_weight.steering_input_squared_vel = 0.25;
  p.lateral.nominal_weight.lat_jerk = 0.1;
  p.lateral.nominal_weight.steer_rate = 0.0;
  p.lateral.nominal_weight.steer_acc = 0.000001;
  p.lateral.low_curvature_weight.lat_error = 0.1;
  p.lateral.low_curvature_weight.heading_error = 0.0;
  p.lateral.low_curvature_weight.heading_error_squared_vel = 0.3;
  p.lateral.low_curvature_weight.steering_input = 1.0;
  p.lateral.low_curvature_weight.steering_input_squared_vel = 0.25;
  p.lateral.low_curvature_weight.lat_jerk = 0.0;
  p.lateral.low_curvature_weight.steer_rate = 0.0;
  p.lateral.low_curvature_weight.steer_acc = 0.000001;
  p.lateral.low_curvature_thresh_curvature = 0.0;
  p.lateral.nominal_weight.terminal_lat_error = 1.0;
  p.lateral.nominal_weight.terminal_heading_error = 0.1;
  p.lateral.zero_ff_steer_deg = 0.5;
  p.lateral.acceleration_limit = 2.0;
  p.lateral.velocity_time_constant = 0.3;
  p.lateral.min_prediction_length = 5.0;
  p.lateral.input_delay = 0.24;
  p.lateral.steer_tau = 0.27;
  p.steering_lpf_cutoff_hz = 3.0;
  p.error_deriv_lpf_cutoff_hz = 5.0;
  p.stop_state_entry_ego_speed = 0.001;
  p.stop_state_entry_target_speed = 0.001;
  p.converged_steer_rad = 0.1;
  p.keep_steer_control_until_converged = true;
  p.new_traj_duration_time = 1.0;
  p.new_traj_end_dist = 0.3;
  p.mpc_converged_threshold_rps = 0.01;
  p.longitudinal.delay_compensation_time = 0.1;
  p.longitudinal.enable_smooth_stop = true;
  p.longitudinal.enable_overshoot_emergency = true;
  p.longitudinal.enable_large_tracking_error_emergency = true;
  p.longitudinal.enable_slope_compensation = true;
  p.longitudinal.enable_keep_stopped_until_steer_convergence = true;
  p.longitudinal.state_transition_params.drive_state_stop_dist = 0.5;
  p.longitudinal.state_transition_params.drive_state_offset_stop_dist = 1.0;
  p.longitudinal.state_transition_params.stopping_state_stop_dist = 0.5;
  p.longitudinal.state_transition_params.stopped_state_entry_duration_time = 0.1;
  p.longitudinal.state_transition_params.stopped_state_entry_vel = 0.01;
  p.longitudinal.state_transition_params.stopped_state_entry_acc = 0.1;
  p.longitudinal.state_transition_params.emergency_state_overshoot_stop_dist = 1.5;
  p.longitudinal.pid_gains.kp = 1.0;
  p.longitudinal.pid_gains.ki = 0.1;
  p.longitudinal.pid_gains.kd = 0.0;
  p.longitudinal.pid_limits.max_out = 1.0;
  p.longitudinal.pid_limits.min_out = -1.0;
  p.longitudinal.pid_limits.max_p_effort = 1.0;
  p.longitudinal.pid_limits.min_p_effort = -1.0;
  p.longitudinal.pid_limits.max_i_effort = 0.3;
  p.longitudinal.pid_limits.min_i_effort = -0.3;
  p.longitudinal.pid_limits.max_d_effort = 0.0;
  p.longitudinal.pid_limits.min_d_effort = 0.0;
  p.longitudinal.lpf_vel_error_gain = 0.9;
  p.longitudinal.enable_integration_at_low_speed = false;
  p.longitudinal.current_vel_threshold_pid_integrate = 0.5;
  p.longitudinal.time_threshold_before_pid_integrate = 2.0;
  p.longitudinal.ff_scale_min = 0.5;
  p.longitudinal.ff_scale_max = 2.0;
  p.longitudinal.enable_brake_keeping_before_stop = false;
  p.longitudinal.brake_keeping_acc = -0.2;
  p.longitudinal.smooth_stop_params.max_strong_acc = -0.5;
  p.longitudinal.smooth_stop_params.min_strong_acc = -0.8;
  p.longitudinal.smooth_stop_params.weak_acc = -0.3;
  p.longitudinal.smooth_stop_params.weak_stop_acc = -0.8;
  p.longitudinal.smooth_stop_params.strong_stop_acc = -3.4;
  p.longitudinal.smooth_stop_params.min_fast_vel = 0.5;
  p.longitudinal.smooth_stop_params.min_running_vel = 0.01;
  p.longitudinal.smooth_stop_params.min_running_acc = 0.01;
  p.longitudinal.smooth_stop_params.weak_stop_time = 0.8;
  p.longitudinal.smooth_stop_params.weak_stop_dist = -0.3;
  p.longitudinal.smooth_stop_params.strong_stop_dist = -0.5;
  p.longitudinal.stopped_state_params.vel = 0.0;
  p.longitudinal.stopped_state_params.acc = -3.4;
  p.longitudinal.emergency_state_params.vel = 0.0;
  p.longitudinal.emergency_state_params.acc = -5.0;
  p.longitudinal.emergency_state_params.jerk = -3.0;
  p.longitudinal.lpf_acc_error_gain = 0.98;
  p.longitudinal.acc_feedback_gain = 0.0;
  p.longitudinal.max_acc = 3.0;
  p.longitudinal.min_acc = -5.0;
  p.longitudinal.max_jerk = 2.0;
  p.longitudinal.min_jerk = -5.0;
  p.longitudinal.max_acc_cmd_diff = 50.0;
  p.longitudinal.adaptive_trajectory_velocity_th = 1.0;
  p.longitudinal.lpf_pitch_gain = 0.95;
  p.longitudinal.max_pitch_rad = 0.1;
  p.longitudinal.min_pitch_rad = -0.1;
  p.longitudinal.ego_nearest_dist_threshold = 3.0;
  p.longitudinal.ego_nearest_yaw_threshold = 1.5707963267948966;
  p.lateral.low_curvature_weight.terminal_lat_error = p.lateral.nominal_weight.terminal_lat_error;
  p.lateral.low_curvature_weight.terminal_heading_error = p.lateral.nominal_weight.terminal_heading_error;
  p.longitudinal.wheel_base = p.wheelbase_m;
  p.longitudinal.front_overhang = 0.9;
  p.longitudinal.longitudinal_ctrl_period = p.nominal_dt_sec;
  p.longitudinal.minimum_control_dt_sec = p.minimum_control_dt_sec;
  p.longitudinal.maximum_control_dt_sec = p.maximum_control_dt_sec;
  return p;
}
}}

#include <osqp.h>
#include <vector>
#include <limits>
namespace control { namespace universe {
struct OsqpSolver::Impl {
  OSQPWorkspace* work = nullptr;
  std::vector<c_float> px, ax, q, l, u;
  std::vector<c_int> pi, pp, ai, ap;
  ~Impl() { if (work) osqp_cleanup(work); }
};
inline OsqpSolver::OsqpSolver() : impl_(new Impl) {}
inline OsqpSolver::~OsqpSolver() = default;
inline autoware::motion::control::mpc_lateral_controller::QPSolverResult OsqpSolver::solve(
  const Eigen::MatrixXd& h, const Eigen::MatrixXd& f, const Eigen::MatrixXd& a,
  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lba,
  const Eigen::VectorXd& uba, Eigen::VectorXd& result) {
  auto& s=*impl_; const c_int n=h.rows(), m=n+a.rows();
  s.px.clear();s.pi.clear();s.pp.clear();s.ax.clear();s.ai.clear();s.ap.clear();
  // Keep a fixed sparsity pattern for OSQP matrix updates and warm starts.
  for(c_int col=0;col<n;++col) {
    s.pp.push_back(s.px.size());
    for(c_int row=0;row<=col;++row) { s.pi.push_back(row);s.px.push_back(h(row,col)); }
    s.ap.push_back(s.ax.size());s.ai.push_back(col);s.ax.push_back(1.0);
    for(c_int row=0;row<a.rows();++row) { s.ai.push_back(n+row);s.ax.push_back(a(row,col)); }
  }
  s.pp.push_back(s.px.size());s.ap.push_back(s.ax.size());
  s.q.resize(n);s.l.resize(m);s.u.resize(m);
  for(c_int i=0;i<n;++i) {s.q[i]=f(i);s.l[i]=lb(i);s.u[i]=ub(i);}
  for(c_int i=0;i<a.rows();++i) {s.l[n+i]=lba(i);s.u[n+i]=uba(i);}
  if(s.work && (s.work->data->n!=n || s.work->data->m!=m)) {osqp_cleanup(s.work);s.work=nullptr;}
  if(!s.work) {
    csc p{static_cast<c_int>(s.px.size()),n,n,s.pp.data(),s.pi.data(),s.px.data(),-1};
    csc c{static_cast<c_int>(s.ax.size()),m,n,s.ap.data(),s.ai.data(),s.ax.data(),-1};
    OSQPData data{}; data.n=n;data.m=m;data.P=&p;data.A=&c;data.q=s.q.data();data.l=s.l.data();data.u=s.u.data();
    OSQPSettings settings;osqp_set_default_settings(&settings);
    settings.verbose=0;settings.eps_abs=std::numeric_limits<c_float>::epsilon();
    settings.eps_rel=1e-4;settings.eps_prim_inf=1e-4;settings.eps_dual_inf=1e-4;
    settings.alpha=1.6;settings.max_iter=4000;
    settings.polish=1;settings.warm_start=1;
    osqp_setup(&s.work,&data,&settings);
  } else {
    osqp_update_P_A(s.work,s.px.data(),nullptr,s.px.size(),s.ax.data(),nullptr,s.ax.size());
    osqp_update_lin_cost(s.work,s.q.data());
    osqp_update_bounds(s.work,s.l.data(),s.u.data());
  }
  osqp_solve(s.work);
  result.resize(n);for(c_int i=0;i<n;++i)result(i)=s.work->solution->x[i];
  return {s.work->info->status_val == OSQP_SOLVED, s.work->info->status};
}
inline int64_t OsqpSolver::getTakenIter() const {return impl_->work ? impl_->work->info->iter : 0;}
inline double OsqpSolver::getObjVal() const {return impl_->work ? impl_->work->info->obj_val : 0.0;}
}}

// Copyright 2018-2023 The Autoware Foundation, TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// Calculation-only adaptation of mpc_lateral_controller.cpp's stop/convergence wrapper.
#include <control/libraries/autoware_universe/mpc/vehicle_model/vehicle_model_bicycle_kinematics.hpp>
#include <control/libraries/autoware_universe/motion_utils.hpp>
#include <deque>
namespace control { namespace universe {
struct Controller::Impl {
  Parameters params;
  lateral::MPC mpc;
  longitudinal::PidLongitudinalController pid;
  bool initialized = false, previous_steer_converged = false, history_filled = false;
  autoware_control_msgs::msg::Lateral previous_command;
  std::deque<std::pair<ros::Time,double>> steering_history;
  std::deque<std::pair<ros::Time,geometry_msgs::Point>> trajectory_ends;
  explicit Impl(const Parameters& p) : params(p), pid(p.longitudinal) {
    mpc.m_param=p.lateral;
    mpc.m_ctrl_period=p.nominal_dt_sec;
    mpc.m_steer_lim=p.maximum_steering_angle_rad;
    mpc.m_use_steer_prediction=p.use_steer_prediction;
    mpc.ego_nearest_dist_threshold=p.longitudinal.ego_nearest_dist_threshold;
    mpc.ego_nearest_yaw_threshold=p.longitudinal.ego_nearest_yaw_threshold;
    const auto delay_steps=static_cast<size_t>(std::round(p.lateral.input_delay/p.nominal_dt_sec));
    mpc.m_param.input_delay=delay_steps*p.nominal_dt_sec;
    mpc.m_input_buffer.assign(delay_steps,0.0);
    mpc.m_steer_rate_lim_map_by_curvature=p.steer_rate_by_curvature;
    mpc.m_steer_rate_lim_map_by_velocity=p.steer_rate_by_velocity;
    mpc.setVehicleModel(std::make_shared<lateral::KinematicsBicycleModel>(p.wheelbase_m,p.maximum_steering_angle_rad,p.lateral.steer_tau));
    mpc.setQPSolver(std::make_shared<OsqpSolver>());
    mpc.initializeSteeringPredictor();
    mpc.initializeLowPassFilters(p.steering_lpf_cutoff_hz,p.error_deriv_lpf_cutoff_hz);
  }
  bool steerConverged(double command, double measured, const geometry_msgs::Point& end) const {
    for(const auto& old:trajectory_ends)
      if(autoware_utils::calc_distance2d(old.second,end)>params.new_traj_end_dist) return false;
    return std::abs(command-measured)<params.converged_steer_rad;
  }
  bool mpcConverged() const {
    if(steering_history.size()<2 || !history_filled) return false;
    double min=steering_history.front().second, max=min;
    for(const auto& old:steering_history) {min=std::min(min,old.second);max=std::max(max,old.second);}
    return max-min<params.mpc_converged_threshold_rps;
  }
};
inline Controller::Controller(const Parameters& p) : impl_(new Impl(p)) {}
inline Controller::~Controller() = default;
inline Controller::Controller(Controller&&) noexcept = default;
inline Controller& Controller::operator=(Controller&&) noexcept = default;
inline std::pair<autoware_control_msgs::msg::Lateral, autoware_control_msgs::msg::Longitudinal> Controller::calculate(
  const autoware_planning_msgs::msg::Trajectory& trajectory, const nav_msgs::Odometry& odometry,
  double measured_steering, double measured_acceleration, const ros::Time& stamp) {
  auto& s=*impl_;
  const auto& end=trajectory.points.back().pose.position;
  s.trajectory_ends.emplace_back(stamp,end);
  while((stamp-s.trajectory_ends.front().first).toSec()>=s.params.new_traj_duration_time)
    s.trajectory_ends.pop_front();
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.steering_tire_angle=static_cast<float>(measured_steering);
  steering.stamp=stamp;
  if(!s.initialized) {s.previous_command.steering_tire_angle=steering.steering_tire_angle;s.initialized=true;}
  s.mpc.setReferenceTrajectory(trajectory,s.params.trajectory_filter,odometry);
  const auto solution=s.mpc.calculateMPC(steering,odometry,stamp);
  auto lateral_command=solution.ctrl_cmd;
  s.steering_history.emplace_back(stamp,lateral_command.steering_tire_angle);
  if((stamp-s.steering_history.front().first).toSec()>=1.0) {
    s.history_filled=true;
    while((stamp-s.steering_history.front().first).toSec()>1.0) s.steering_history.pop_front();
  } else s.history_filled=false;
  const auto nearest=autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points,odometry.pose.pose,s.mpc.ego_nearest_dist_threshold,s.mpc.ego_nearest_yaw_threshold);
  double target_speed=trajectory.points[nearest].longitudinal_velocity_mps, covered=0.0;
  for(size_t i=nearest+1;i<trajectory.points.size();++i) {
    target_speed=std::min(target_speed,trajectory.points[i].longitudinal_velocity_mps);
    covered+=autoware_utils::calc_distance2d(trajectory.points[i-1].pose,trajectory.points[i].pose);
    if(covered>1.0) break;
  }
  const bool stopped=std::abs(odometry.twist.twist.linear.x)<=s.params.stop_state_entry_ego_speed &&
    std::abs(target_speed)<s.params.stop_state_entry_target_speed &&
    (!s.params.keep_steer_control_until_converged || s.steerConverged(s.previous_command.steering_tire_angle,measured_steering,end));
  bool next_converged=false;
  if(stopped) {
    for(auto& v:s.mpc.m_input_buffer) v=s.previous_command.steering_tire_angle;
    s.mpc.m_raw_steer_cmd_prev=s.previous_command.steering_tire_angle;
    lateral_command=s.previous_command;
  } else {
    next_converged=s.mpcConverged() && s.steerConverged(lateral_command.steering_tire_angle,measured_steering,end);
    s.previous_command=lateral_command;
  }
  autoware::motion::control::trajectory_follower::InputData input;
  input.current_trajectory=trajectory;input.current_odometry=odometry;
  input.current_accel.accel.accel.linear.x=measured_acceleration;
  const auto longitudinal_command=s.pid.run(input,stamp,s.previous_steer_converged).output.control_cmd;
  // Universe trajectory follower sync applies after both controllers have run.
  s.previous_steer_converged=next_converged;
  return {lateral_command,longitudinal_command};
}
}}

#include <interfaces/Path.h>
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/CtrlCmd.h>
namespace control {
class AutowareUniverse {
 public:
  explicit AutowareUniverse(universe::Parameters parameters = universe::defaultParameters())
    : parameters_(std::move(parameters)), controller_(parameters_) {}
  morai_msgs::CtrlCmd process(const interfaces::Path& path, const nav_msgs::Odometry& ego,
    const interfaces::CompetitionVehicleStatus& competition, const sensor_msgs::Imu& imu) {
    ROS_INFO_ONCE("[control] executing autoware_universe.hpp");
    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.header=path.header;trajectory.header.frame_id="map";
    tf2::Transform map_from_base;tf2::fromMsg(ego.pose.pose,map_from_base);
    trajectory.points.reserve(path.points.size());
    for(const auto& point:path.points) {
      autoware_planning_msgs::msg::TrajectoryPoint converted;
      tf2::Transform local_pose;tf2::fromMsg(point.pose.pose,local_pose);
      tf2::toMsg(map_from_base*local_pose,converted.pose);
      converted.longitudinal_velocity_mps=point.velocity;
      converted.acceleration_mps2=point.acceleration;
      trajectory.points.push_back(converted);
    }
    auto odometry=ego;
    odometry.twist.twist.linear.x=competition.velocity_x_mps;
    // IMU mounting RPY=0: signed +X is the longitudinal acceleration.
    // Native Competition is left-negative; Universe uses left-positive wheel angles.
    const auto commands=controller_.calculate(trajectory,odometry,-competition.steering_angle_rad,
      imu.linear_acceleration.x,path.header.stamp);
    const double acceleration=commands.second.acceleration;
    morai_msgs::CtrlCmd output;
    output.longlCmdType=1;
    output.steering=universe::clamp(commands.first.steering_tire_angle/parameters_.maximum_steering_angle_rad,-1.0,1.0);
    output.accel=universe::clamp(acceleration/parameters_.acceleration_to_pedal_mps2,0.0,parameters_.maximum_accel_pedal);
    output.brake=universe::clamp(-acceleration/parameters_.braking_to_pedal_mps2,0.0,parameters_.maximum_brake_pedal);
    return output;
  }
 private:
  universe::Parameters parameters_;
  universe::Controller controller_;
};
}
