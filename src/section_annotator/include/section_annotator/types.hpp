#pragma once
#include <node_pipeline/components.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <interfaces/SectionAnnotation.h>
namespace section_annotator {
using Data = node_pipeline::Data<geometry_msgs::PoseStamped>;
using Message = interfaces::SectionAnnotation;
}  // namespace section_annotator
