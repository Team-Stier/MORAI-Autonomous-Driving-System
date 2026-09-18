#pragma once

#include <memory>
#include <string>
#include <vector>

#include "morai_path_tracking/controllers/lateral/mpc/mpc_controller.hpp"

namespace morai_path_tracking {

struct MpcLateralResult {
  bool valid{false};
  double steering_angle_rad{0.0};
  double steering_rate_rad_per_sec{0.0};
  double raw_steering_rate_rad_per_sec{0.0};
  double cross_track_error_m{0.0};
  double raw_cross_track_error_m{0.0};
  double heading_error_rad{0.0};
  double reference_curvature_m_inv{0.0};
  double yaw_rate_steering_estimate_blend{0.0};
  double modeled_steering_angle_rad{0.0};
  morai_mpc::Point2d projection;
  int solver_iterations{0};
  double solver_time_ms{0.0};
  double solver_cost{0.0};
  double swept_path_maximum_offset_m{0.0};
  std::vector<morai_mpc::Point2d> compensated_path;
  std::string error;
};

struct SweptPathCompensationResult {
  bool valid{false};
  double maximum_offset_m{0.0};
  std::vector<morai_mpc::Point2d> path;
  std::string error;
};

SweptPathCompensationResult compensateSweptWheelPath(
    const std::vector<morai_mpc::Point2d>& path,
    const morai_mpc::MpcConfig& config);

// ROS 경로 추종 노드와 팀원 MPC 연산부 사이의 얇은 계약 어댑터다.
// 입력 경로는 base_link(x 전방, y 좌측), 출력은 절대 전륜 조향각[rad]이다.
class MpcLateralController {
 public:
  explicit MpcLateralController(const morai_mpc::MpcConfig& config);

  MpcLateralResult calculate(
      const std::vector<morai_mpc::Point2d>& vehicle_path,
      double speed_mps, double measured_yaw_rate_radps,
      double previous_steering_angle_rad,
      double control_dt_sec);
  MpcLateralResult calculate(
      const std::vector<morai_mpc::Point2d>& vehicle_path,
      double speed_mps, double measured_lateral_velocity_mps,
      double measured_yaw_rate_radps, double previous_steering_angle_rad,
      double control_dt_sec);
  void reset();

 private:
  morai_mpc::MpcConfig config_;
  std::unique_ptr<morai_mpc::MpcController> controller_;
  bool has_filtered_yaw_rate_steering_estimate_{false};
  double filtered_yaw_rate_steering_estimate_rad_{0.0};
  double filtered_curve_steering_estimate_blend_{0.0};
};

}  // namespace morai_path_tracking
