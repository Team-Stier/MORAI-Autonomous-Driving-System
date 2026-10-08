#pragma once
#include <control/libraries/tracking/morai_mpc/parameters.hpp>
#include <control/libraries/tracking/morai_mpc/autoware/mpc_trajectory.h>
#include <control/libraries/tracking/morai_mpc/autoware/lowpass_filter.h>
#include <control/libraries/tracking/morai_mpc/autoware/vehicle_model/vehicle_model_bicycle_kinematics.h>
#include <control/libraries/tracking/morai_mpc/autoware/qp_solver/qp_solver_qpoases.h>
#include <control/libraries/tracking/morai_mpc/autoware/lane.hpp>
#include <deque>
#include <memory>

namespace control { namespace morai_mpc {
class AutowareLateral {
 public:
  AutowareLateral(const LateralParameters& parameters,
                 const SteeringPredictionParameters& steering);
  double calculate(const control::morai_mpc::autoware::Lane& path, double speed_mps,
                   double modeled_steering_rad, double previous_steering_rad);

 private:
  void prepareTrajectory(const control::morai_mpc::autoware::Lane& path, double speed_mps);
  LateralParameters p_;
  SteeringPredictionParameters steering_;
  autoware::MPCTrajectory ref_traj_;
  autoware::Butterworth2dFilter lpf_steering_cmd_;
  std::unique_ptr<autoware::KinematicsBicycleModel> vehicle_model_ptr_;
  std::unique_ptr<autoware::QPSolverQpoasesHotstart> qpsolver_ptr_;
  std::deque<double> input_buffer_;
  Eigen::Matrix2d front_point_weight_;
};
}}  // namespace control::morai_mpc
