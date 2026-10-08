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
#include <control/libraries/tracking/morai_mpc/autoware_lateral.hpp>
#include <control/libraries/tracking/morai_mpc/autoware/mpc_utils.h>
#include <control/libraries/tracking/morai_mpc/autoware/curvature_weight_blend.h>
#include <control/libraries/tracking/morai_mpc/autoware/prediction_length.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control { namespace morai_mpc {
using namespace autoware;

AutowareLateral::AutowareLateral(const LateralParameters& parameters,
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

void AutowareLateral::prepareTrajectory(const control::morai_mpc::autoware::Lane& path, double speed_mps) {
  // Coincident vertices describe the same geometry; omit zero-length segments
  // before distance interpolation to avoid division by zero.
  control::morai_mpc::autoware::Lane distinct;
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
      if (!MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.x) ||
          !MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.y) ||
          !MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.yaw) ||
          !MoveAverageFilter::filt_vector(p_.path_filter_moving_ave_num, traj.vx)) {
        throw std::runtime_error("Autoware trajectory smoothing failed");
      }
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

double AutowareLateral::calculate(const control::morai_mpc::autoware::Lane& path, double speed_mps,
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
  if (!MPCUtils::calcNearestPoseInterp(ref_traj_, control_pose, nearest_pose, nearest_index, dist_err, yaw_err, nearest_traj_time))
  {
    throw std::runtime_error("Autoware MPC numerical calculation failed");
  };

  /* set mpc initial time */
  const double mpc_start_time = nearest_traj_time;

  /* check trajectory length */
  const double mpc_end_time = mpc_start_time + (N - 1) * DT + steering_.delay_sec + p_.ctrl_period;
  if (mpc_end_time > ref_traj_.relative_time.back())
  {
    throw std::runtime_error("Autoware MPC numerical calculation failed");
  }

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
    if (!MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.k, mpc_curr_time, k) ||
        !MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.vx, mpc_curr_time, v))
    {
      throw std::runtime_error("Autoware MPC numerical calculation failed");
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(v);
    vehicle_model_ptr_->setCurvature(k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, p_.ctrl_period);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = input_buffer_.at(i);
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
  if (!MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_, mpc_time_v, mpc_resampled_ref_traj))
  {
    throw std::runtime_error("Autoware MPC numerical calculation failed");
  }

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

  if (!Aex.allFinite() || !Bex.allFinite() || !Cex.allFinite() || !Wex.allFinite())
  {
    throw std::runtime_error("Autoware MPC numerical calculation failed");
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
  if (!qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex))
  {
    throw std::runtime_error("Autoware MPC numerical calculation failed");
  }

  if (!Uex.allFinite())
  {
    throw std::runtime_error("Autoware MPC numerical calculation failed");
  }

  /* saturation */
  const double u_sat = std::max(std::min(Uex(0), u_lim), -u_lim);

  /* filtering */
  // With rate bounds referenced to the last sent command, an extra output LPF
  // silently reduces achievable slew and adds dynamics absent from the model.
  const double u_filtered = p_.morai_use_output_steering_filter ? lpf_steering_cmd_.filter(u_sat) : u_sat;

  // Keep the original optimizer-output delay buffer. The next call's first
  // transition separately references the final bounded Core command.
  if (!input_buffer_.empty()) {
    input_buffer_.push_back(u_filtered);
    input_buffer_.pop_front();
  }
  return u_filtered;
}

}}  // namespace control::morai_mpc
