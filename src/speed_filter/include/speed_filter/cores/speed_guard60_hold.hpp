#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <algorithm>
#include <cmath>

namespace speed_filter {
struct SpeedGuard60HoldParameters {
  double speed_limit_kph = 60.0;
  // 예측 속도 = 현재 속도 + max(0, 가속도 추정) × prediction_horizon_sec.
  double prediction_horizon_sec = 0.3;
  double acceleration_filter_time_constant_sec = 0.2;
  // 예측 속도가 accel_taper_start_kph~accel_cutoff_kph 구간이면 accel을 1→0으로 줄인다.
  double accel_taper_start_kph = 58.0;
  double accel_cutoff_kph = 59.2;
  // 예측 속도가 brake_start_kph~brake_full_kph 구간이면 brake를 0→maximum_guard_brake로 늘린다.
  double brake_start_kph = 59.5;
  double brake_full_kph = 59.9;
  double maximum_guard_brake = 0.15;
};

// 60 km/h 초과 전에 예측 속도로 accel을 줄이고 약하게 제동한다. 초과하면 전제동한다.
// 정지선 대기 구간용으로 장기 정지 후 출발 보조를 하지 않는다.
class SpeedGuard60Hold {
 public:
  explicit SpeedGuard60Hold(SpeedGuard60HoldParameters parameters) : parameters_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    const double now_sec = status.header.stamp.toSec();
    const double speed_mps = std::fabs(status.signed_velocity_mps);
    // ROS 시각이 되돌아가면 가속도 추정 기준을 다시 잡는다.
    if (now_sec < previous_stamp_sec_) {
      has_previous_speed_ = false;
      acceleration_mps2_ = 0;
    }
    updateAcceleration(now_sec, speed_mps);
    previous_stamp_sec_ = now_sec;

    auto output = command;
    if (speed_mps > parameters_.speed_limit_kph / 3.6) {
      output.accel = 0;
      output.brake = 1;
      return output;
    }
    const double predicted_kph =
        (speed_mps + std::max(0.0, acceleration_mps2_) * parameters_.prediction_horizon_sec) * 3.6;
    output.accel *= 1.0 - ramp(predicted_kph, parameters_.accel_taper_start_kph, parameters_.accel_cutoff_kph);
    const double guard_brake = parameters_.maximum_guard_brake *
        ramp(predicted_kph, parameters_.brake_start_kph, parameters_.brake_full_kph);
    if (guard_brake > 0) {
      output.accel = 0;
      output.brake = std::max(output.brake, guard_brake);
    }
    return output;
  }

 private:
  // value가 lower 이하이면 0, upper 이상이면 1, 사이는 선형.
  static double ramp(double value, double lower, double upper) {
    return std::max(0.0, std::min(1.0, (value - lower) / (upper - lower)));
  }

  void updateAcceleration(double now_sec, double speed_mps) {
    if (has_previous_speed_) {
      const double dt_sec = now_sec - previous_speed_stamp_sec_;
      // 같은 상태 메시지가 다시 들어오면 추정을 갱신하지 않는다.
      if (dt_sec <= 0) return;
      const double raw_mps2 = (speed_mps - previous_speed_mps_) / dt_sec;
      const double alpha = 1.0 - std::exp(-dt_sec / parameters_.acceleration_filter_time_constant_sec);
      acceleration_mps2_ += alpha * (raw_mps2 - acceleration_mps2_);
    }
    has_previous_speed_ = true;
    previous_speed_mps_ = speed_mps;
    previous_speed_stamp_sec_ = now_sec;
  }

  SpeedGuard60HoldParameters parameters_;
  double previous_stamp_sec_ = 0;
  bool has_previous_speed_ = false;
  double previous_speed_mps_ = 0;
  double previous_speed_stamp_sec_ = 0;
  double acceleration_mps2_ = 0;
};
}  // namespace speed_filter
