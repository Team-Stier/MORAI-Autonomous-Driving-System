#pragma once
#include <node_pipeline/components.hpp>
#include <nav_msgs/Odometry.h>
#include <interfaces/SectionAnnotation.h>
#include <tf2_msgs/TFMessage.h>
namespace tf_publisher {
using Data = node_pipeline::Data<nav_msgs::Odometry>;
using Message = tf2_msgs::TFMessage;
}  // namespace tf_publisher
