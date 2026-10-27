#pragma once
#include <interfaces/Objects.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <array>
#include <node_pipeline/extension.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <string>

namespace visualizer {
struct ObjectBoxesParameters {
  int prediction_horizon = -1;
  bool flatten_z = false;
};

class ObjectBoxes {
 public:
  explicit ObjectBoxes(ObjectBoxesParameters parameters = {}) : parameters_(parameters) {}

  visualization_msgs::Marker process(const interfaces::Objects& input) {
    using Marker = visualization_msgs::Marker;
    const bool prediction = parameters_.prediction_horizon >= 0;
    const float progress = prediction ? parameters_.prediction_horizon / 9.0f : 0.0f;
    Marker marker;
    marker.ns = prediction ? "predicted_objects_" + std::to_string(parameters_.prediction_horizon) : "objects";
    marker.id = 0;
    marker.type = Marker::LINE_LIST;
    marker.action = input.objects.empty() ? Marker::DELETE : Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = prediction ? 0.08 : 0.28;
    marker.color.r = prediction ? progress : 0.95f;
    marker.color.g = prediction ? 1.0f - 0.65f * progress : 0.15f;
    marker.color.b = prediction ? 0.15f : 1.0f;
    marker.color.a = prediction ? 0.25f : 1.0f;
    marker.frame_locked = false;
    marker.lifetime = ros::Duration(0.5);
    marker.points.reserve(input.objects.size() * 24);
    visualization_msgs::MarkerArray details;
    for (const auto& object : input.objects) {
      std_msgs::ColorRGBA color=marker.color;
      if(!prediction) {
        if(!object.velocity_available){color.r=1;color.g=.85;color.b=.1;}
        else if(std::hypot(object.velocity.x,object.velocity.y)>.8){color.r=1;color.g=.2;color.b=.15;}
        else {color.r=.15;color.g=.55;color.b=1;}
        Marker label;label.header=input.header;label.ns="detected_object_details";label.id=static_cast<int>(object.id);
        label.type=Marker::TEXT_VIEW_FACING;label.action=Marker::ADD;label.pose=object.bbox_pose;
        label.pose.position.z=(parameters_.flatten_z ? object.bbox_size.z/2 : object.bbox_pose.position.z)+object.bbox_size.z/2+.65;
        label.pose.orientation={};label.pose.orientation.w=1;label.scale.z=.55;label.color=color;label.color.a=1;
        label.lifetime=ros::Duration(.5);
        std::ostringstream text;text<<"ID "<<object.id<<"  ";
        if(object.velocity_available)text<<std::fixed<<std::setprecision(1)<<std::hypot(object.velocity.x,object.velocity.y)*3.6<<" km/h";
        else text<<"speed unknown";
        text<<"\n"<<std::fixed<<std::setprecision(1)<<object.bbox_size.x<<" x "<<object.bbox_size.y<<" m";
        label.text=text.str();details.markers.push_back(label);
      }
      const auto& pose = object.bbox_pose;
      const auto& size = object.bbox_size;
      const auto& q = pose.orientation;
      const tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
      std::array<geometry_msgs::Point, 8> corners;
      for (unsigned i = 0; i < corners.size(); ++i) {
        const tf2::Vector3 local((i & 1 ? 0.5 : -0.5) * size.x,
                                (i & 2 ? 0.5 : -0.5) * size.y,
                                (i & 4 ? 0.5 : -0.5) * size.z);
        const auto rotated = tf2::quatRotate(rotation, local);
        corners[i].x = pose.position.x + rotated.x();
        corners[i].y = pose.position.y + rotated.y();
        corners[i].z = (parameters_.flatten_z ? size.z / 2.0 : pose.position.z) + rotated.z();
      }
      for (unsigned i = 0; i < corners.size(); ++i) {
        for (unsigned axis = 0; axis < 3; ++axis) {
          const unsigned neighbor = i ^ (1u << axis);
          if (neighbor <= i) continue;
          marker.points.push_back(corners[i]);
          marker.points.push_back(corners[neighbor]);
          if(!prediction){marker.colors.push_back(color);marker.colors.push_back(color);}
        }
      }
    }
    if(!prediction)details_visualizer_.publish(details);
    return marker;
  }

 private:
  node_pipeline::Visualizer<visualization_msgs::MarkerArray> details_visualizer_{"object_details"};
  ObjectBoxesParameters parameters_;
};
}  // namespace visualizer
