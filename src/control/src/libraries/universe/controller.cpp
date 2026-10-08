// Copyright 2018-2023 The Autoware Foundation, TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// Calculation-only adaptation of mpc_lateral_controller.cpp's stop/convergence wrapper.
#include <control/libraries/universe/controller.hpp>
#include <control/libraries/universe/mpc/vehicle_model/vehicle_model_bicycle_kinematics.hpp>
#include <control/libraries/universe/motion_utils.hpp>
#include <control/libraries/universe/osqp_solver.hpp>
#include <deque>
#include <stdexcept>
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
Controller::Controller(const Parameters& p) : impl_(new Impl(p)) {}
Controller::~Controller() = default;
Controller::Controller(Controller&&) noexcept = default;
Controller& Controller::operator=(Controller&&) noexcept = default;
std::pair<autoware_control_msgs::msg::Lateral, autoware_control_msgs::msg::Longitudinal> Controller::calculate(
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
  if(!solution.result) throw std::runtime_error("Universe MPC: "+solution.reason);
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
