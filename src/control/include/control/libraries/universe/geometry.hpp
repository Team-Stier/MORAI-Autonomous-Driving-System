// Copyright 2022 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0
// ROS 1 equivalents of the Autoware geometry helpers used by the port.
#pragma once
#include <control/libraries/universe/compat.hpp>
#include <Eigen/Geometry>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <stdexcept>
namespace autoware_utils {
inline const geometry_msgs::Point& get_point(const geometry_msgs::Point& p) { return p; }
inline const geometry_msgs::Point& get_point(const geometry_msgs::Pose& p) { return p.position; }
inline const geometry_msgs::Point& get_point(const autoware_planning_msgs::msg::TrajectoryPoint& p) { return p.pose.position; }
inline const geometry_msgs::Pose& get_pose(const autoware_planning_msgs::msg::TrajectoryPoint& p) { return p.pose; }
template<class A, class B> inline double calc_squared_distance2d(const A& a, const B& b) {
  const auto& p = get_point(a); const auto& q = get_point(b);
  return (p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y);
}
template<class A, class B> inline double calc_distance2d(const A& a, const B& b) { return std::sqrt(calc_squared_distance2d(a,b)); }
template<class A, class B> inline double calc_distance3d(const A& a, const B& b) {
  const double dz = get_point(a).z-get_point(b).z;
  return std::sqrt(calc_squared_distance2d(a,b)+dz*dz);
}
inline double normalize_radian(double a, double min = -M_PI) {
  const double n = std::fmod(a-min, 2*M_PI);
  return n < 0 ? n+2*M_PI+min : n+min;
}
inline double deg2rad(double a) { return a*M_PI/180.0; }
inline double rad2deg(double a) { return a*180.0/M_PI; }
inline double calc_yaw_deviation(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
  return normalize_radian(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation));
}
inline bool is_driving_forward(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
  return std::abs(normalize_radian(std::atan2(b.position.y-a.position.y,b.position.x-a.position.x)-tf2::getYaw(a.orientation))) < M_PI_2;
}
inline geometry_msgs::Quaternion create_quaternion_from_yaw(double yaw) {
  tf2::Quaternion q; q.setRPY(0,0,yaw); return tf2::toMsg(q);
}
inline double calc_curvature(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c) {
  const double denominator = calc_distance2d(a,b)*calc_distance2d(b,c)*calc_distance2d(c,a);
  if (denominator < 1e-10) throw std::runtime_error("curvature points coincide");
  return 2*((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/denominator;
}
inline double calc_elevation_angle(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  return std::atan2(b.z-a.z,calc_distance2d(a,b));
}
inline geometry_msgs::Pose calc_offset_pose(const geometry_msgs::Pose& pose, double x, double y, double z, double yaw = 0.0) {
  tf2::Transform t; tf2::fromMsg(pose,t);
  const auto p = t*tf2::Vector3(x,y,z);
  geometry_msgs::Pose out=pose; out.position.x=p.x(); out.position.y=p.y(); out.position.z=p.z();
  tf2::Quaternion q; q.setRPY(0,0,yaw);out.orientation=tf2::toMsg(t.getRotation()*q);return out;
}
}
namespace autoware_utils_geometry = autoware_utils;
