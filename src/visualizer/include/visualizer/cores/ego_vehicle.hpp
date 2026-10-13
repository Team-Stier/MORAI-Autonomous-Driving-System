#pragma once
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <array>

namespace visualizer {
struct EgoVehicleParameters {
  bool gt = false;
  bool flatten_z = true;
};

class EgoVehicle {
 public:
  explicit EgoVehicle(EgoVehicleParameters parameters = {}) : parameters_(parameters) {}

  visualization_msgs::Marker process(const nav_msgs::Odometry& ego) {
    using Marker = visualization_msgs::Marker;
    const auto& pose = ego.pose.pose;
    tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w);
    rotation.normalize();
    const tf2::Vector3 low(-0.7899996042251587, -1.0804176330566406, -0.3684999644756317);
    const tf2::Vector3 high(3.845008373260498, 1.080417275428772, 1.2755463123321533);
    const double body_z = parameters_.flatten_z ? 0.0 : pose.position.z;
    const auto point = [&](const tf2::Vector3& local, double origin_z) {
      const auto rotated = tf2::quatRotate(rotation, local);
      geometry_msgs::Point result;
      result.x = pose.position.x + rotated.x();
      result.y = pose.position.y + rotated.y();
      result.z = origin_z + rotated.z();
      return result;
    };
    const std::array<tf2::Vector3, 8> corners = {{
      {high.x(), high.y(), low.z()}, {high.x(), low.y(), low.z()},
      {low.x(), low.y(), low.z()}, {low.x(), high.y(), low.z()},
      {high.x(), high.y(), high.z()}, {high.x(), low.y(), high.z()},
      {low.x(), low.y(), high.z()}, {low.x(), high.y(), high.z()}
    }};
    std::array<geometry_msgs::Point, 8> world;
    for (std::size_t index = 0; index < corners.size(); ++index)
      world[index] = point(corners[index], body_z);
    constexpr std::array<int, 24> edges = {{
      0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7
    }};
    Marker output;
    output.ns = parameters_.gt ? "gt" : "ego";
    output.type = Marker::LINE_LIST;
    output.action = Marker::ADD;
    output.pose.orientation.w = 1.0;
    output.scale.x = 0.08;
    output.color.r = parameters_.gt ? 1.0 : 0.0;
    output.color.g = parameters_.gt ? 0.35 : 0.85;
    output.color.b = parameters_.gt ? 0.05 : 1.0;
    output.color.a = 0.55;
    output.lifetime = ros::Duration(0.5);
    output.points.reserve(30);
    for (const auto index : edges) output.points.push_back(world[index]);
    const double heading_z = (parameters_.flatten_z ? 0.0 : pose.position.z) + 1.8;
    const auto origin = point(tf2::Vector3(0, 0, 0), heading_z);
    const auto tip = point(tf2::Vector3(5.5, 0, 0), heading_z);
    const auto left = point(tf2::Vector3(4.8, 0.4, 0), heading_z);
    const auto right = point(tf2::Vector3(4.8, -0.4, 0), heading_z);
    output.points.insert(output.points.end(), {origin, tip, tip, left, tip, right});
    return output;
  }

 private:
  const EgoVehicleParameters parameters_;
};
}  // namespace visualizer
