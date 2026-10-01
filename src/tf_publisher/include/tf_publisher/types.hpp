#pragma once
#include <node_pipeline/components.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <interfaces/SectionAnnotation.h>
#include <tf2_msgs/TFMessage.h>
namespace tf_publisher {
using Data = node_pipeline::Data<geometry_msgs::PoseStamped>;
using Message = tf2_msgs::TFMessage;
}  // namespace tf_publisher
