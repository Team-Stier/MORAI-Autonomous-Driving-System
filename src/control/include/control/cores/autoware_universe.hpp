#pragma once
#include <control/universe/controller.hpp>
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
