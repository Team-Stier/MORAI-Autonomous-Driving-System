#pragma once
#include <control/libraries/tracking/parameters.hpp>
namespace control { namespace tracking {
// 수치 알고리즘 실패는 예외로 전달한다. 입력 정책과 안전 명령은 ROS adapter가 소유한다.
double calculateSteering(const Parameters& parameters, MpcLateralController& mpc,
                         StanleyController& stanley, HybridController& hybrid,
                         const std::vector<Point2d>& path, double speed_mps,
                         double lateral_velocity_mps, double yaw_rate_radps,
                         double previous_steering_rad, double dt_sec);
LongitudinalCommand calculatePedals(const Parameters& parameters, LongitudinalMpc& mpc,
                                   LongitudinalPid& pid, double target_speed_mps,
                                   double speed_mps, double dt_sec);
} }
