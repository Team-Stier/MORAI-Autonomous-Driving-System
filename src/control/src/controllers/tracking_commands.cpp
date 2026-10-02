#include <control/tracking/commands.hpp>
#include <control/tracking/controllers/lateral/mpc/path.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace control { namespace tracking {
double calculateSteering(const Parameters& p, MpcLateralController& mpc,
                         StanleyController& stanley, HybridController& hybrid,
                         const std::vector<Point2d>& path, double speed,
                         double lateral_velocity, double yaw_rate,
                         double previous_steering, double dt) {
  double steering = 0;
  if (p.lateral_controller == "mpc") {
    std::vector<morai_mpc::Point2d> points;
    points.reserve(path.size());
    for (const auto& point : path) points.push_back({point.x, point.y});
    const auto result = mpc.calculate(points, speed, lateral_velocity, yaw_rate, previous_steering, dt);
    if (!result.valid) throw std::runtime_error(result.error);
    steering = result.steering_angle_rad;
  } else if (p.lateral_controller == "stanley") {
    const auto result = stanley.calculate(path, std::abs(speed), yaw_rate, previous_steering, dt);
    if (!result.valid) throw std::runtime_error(result.error);
    steering = result.steering_angle_rad;
  } else {
    std::vector<morai_mpc::Point2d> points;
    points.reserve(path.size());
    for (const auto& point : path) points.push_back({point.x, point.y});
    const morai_mpc::ReferencePath reference(points, false, morai_mpc::CoordinateFrame::Vehicle);
    const auto projection = reference.project({0, 0, 0});
    if (!projection.valid) throw std::runtime_error("path projection failed");
    double curvature = 0;
    const double end = std::min(reference.length(), projection.s + p.curvature_preview_distance_m);
    for (double s = projection.s; s <= end; s += p.curvature_sample_spacing_m)
      curvature = std::max(curvature, std::abs(reference.sample(s).curvature));
    if (p.lateral_controller == "pure_pursuit") {
      const auto result = computePurePursuit(path, speed, curvature, p.pure_pursuit);
      if (!result.valid) throw std::runtime_error("no lookahead target");
      steering = result.steering_angle_rad;
    } else {
      const auto corridor = estimateWheelCorridor(path, p.wheel_corridor);
      if (!corridor.valid) throw std::runtime_error(corridor.error);
      const auto result = hybrid.calculate(path, std::abs(speed), lateral_velocity, yaw_rate,
                                           curvature, corridor.minimum_clearance_m, previous_steering, dt);
      if (!result.valid) throw std::runtime_error(result.error);
      steering = result.steering_angle_rad;
    }
  }
  if (!std::isfinite(steering)) throw std::runtime_error("nonfinite steering solution");
  return steering;
}
LongitudinalCommand calculatePedals(const Parameters& p, LongitudinalMpc& mpc,
                                   LongitudinalPid& pid, double target_speed,
                                   double speed, double dt) {
  LongitudinalCommand command;
  if (p.longitudinal_controller == "mpc") {
    const auto result = mpc.update(target_speed, speed, dt);
    if (!result.valid) throw std::runtime_error(result.error);
    command = {result.accel, result.brake, result.state};
  } else {
    command = pid.update(target_speed, speed, dt);
  }
  if (!std::isfinite(command.accel) || !std::isfinite(command.brake) ||
      command.accel < 0 || command.brake < 0 || (command.accel > 0 && command.brake > 0))
    throw std::runtime_error("invalid pedal solution");
  return command;
}
} }
