#pragma once
#include <node_pipeline/detail/runtime.hpp>
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Objects.h>
#include <interfaces/Path.h>
#include <interfaces/SectionAnnotation.h>
#include <interfaces/TrafficLight.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <std_msgs/Float64.h>
#include <cstring>
#include <ros/message_traits.h>

namespace node_pipeline { namespace detail {
inline HealthLimits numericLimits() {
  HealthLimits limits;
  limits.lower = -std::numeric_limits<double>::max(); limits.upper = std::numeric_limits<double>::max();
  limits.frame.clear();
  return limits;
}
inline void inspectQuaternion(const geometry_msgs::Quaternion& value, std::vector<Kind>& issues) {
  if (std::abs(value.x*value.x + value.y*value.y + value.z*value.z + value.w*value.w - 1) > 1e-6)
    issues.push_back(Kind::OutRangePositive);
}
template<class Message, bool HasHeader=ros::message_traits::HasHeader<Message>::value> struct RosMessageAccess {
  static std::int64_t stamp(const Message&) { return 0; }
  static bool hasStamp() { return false; }
};
template<class Message> struct RosMessageAccess<Message, true> {
  static std::int64_t stamp(const Message& message) { return message.header.stamp.toNSec(); }
  static bool hasStamp() { return true; }
};
template<> struct MessageTraits<std_msgs::Float64> {
  static void inspect(const std_msgs::Float64& message, const HealthLimits& limits, std::vector<Kind>& issues) {
    NumericInspector<std_msgs::Float64>::inspect(message, limits, issues);
  }
};
template<> struct RosMessageAccess<tf2_msgs::TFMessage> {
  static std::int64_t stamp(const tf2_msgs::TFMessage& message) {
    return message.transforms.empty() ? 0 : message.transforms.front().header.stamp.toNSec();
  }
  static bool hasStamp() { return true; }
};
template<> struct MessageTraits<sensor_msgs::NavSatFix> {
  static void inspect(const sensor_msgs::NavSatFix& message, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(message.header.frame_id, limits, issues);
    NumericInspector<sensor_msgs::NavSatFix>::inspect(message, limits, issues);
  }
};
template<> struct MessageTraits<sensor_msgs::Imu> {
  static void inspect(const sensor_msgs::Imu& message, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(message.header.frame_id, limits, issues);
    NumericInspector<geometry_msgs::Quaternion>::inspect(message.orientation, limits, issues);
    NumericInspector<geometry_msgs::Vector3>::inspect(message.angular_velocity, limits, issues);
    NumericInspector<geometry_msgs::Vector3>::inspect(message.linear_acceleration, limits, issues);
  }
};
template<> struct MessageTraits<interfaces::CompetitionVehicleStatus> {
  static void inspect(const interfaces::CompetitionVehicleStatus& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    NumericInspector<interfaces::CompetitionVehicleStatus>::inspect(m, limits, issues);
  }
};
template<> struct MessageTraits<nav_msgs::Odometry> {
  static void inspect(const nav_msgs::Odometry& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    if (!limits.frame.empty() && m.child_frame_id != "base_link") issues.push_back(Kind::FrameMismatch);
    NumericInspector<geometry_msgs::Pose>::inspect(m.pose.pose, limits, issues);
    NumericInspector<geometry_msgs::Twist>::inspect(m.twist.twist, limits, issues);
    inspectQuaternion(m.pose.pose.orientation, issues);
  }
};
template<> struct MessageTraits<interfaces::Path> {
  static void inspect(const interfaces::Path& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    for (const auto& point : m.points) inspectFrame(point.pose.header.frame_id, limits, issues);
    NumericInspector<interfaces::Path>::inspect(m, limits, issues);
    for (const auto& point : m.points) inspectQuaternion(point.pose.pose.orientation, issues);
  }
};
template<> struct MessageTraits<sensor_msgs::CompressedImage> {
  static void inspect(const sensor_msgs::CompressedImage& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    NumericInspector<sensor_msgs::CompressedImage>::inspect(m, limits, issues);
  }
};
template<> struct MessageTraits<interfaces::TrafficLight> {
  static void inspect(const interfaces::TrafficLight& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    NumericInspector<interfaces::TrafficLight>::inspect(m, limits, issues);
    if (m.state > interfaces::TrafficLight::RED_YELLOW_GREEN_LEFT) issues.push_back(Kind::OutRangePositive);
  }
};
template<> struct MessageTraits<interfaces::SectionAnnotation> {
  static void inspect(const interfaces::SectionAnnotation& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    NumericInspector<interfaces::SectionAnnotation>::inspect(m, limits, issues);
    if (m.state > interfaces::SectionAnnotation::SECTION_16) issues.push_back(Kind::OutRangePositive);
  }
};
template<> struct MessageTraits<morai_msgs::CtrlCmd> {
  static void inspect(const morai_msgs::CtrlCmd& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    NumericInspector<morai_msgs::CtrlCmd>::inspect(m, limits, issues);
    if (m.accel < 0 || m.brake < 0 || m.steering < -1) issues.push_back(Kind::OutRangeNegative);
    if (m.accel > 1 || m.brake > 1 || m.steering > 1) issues.push_back(Kind::OutRangePositive);
  }
};
template<> struct MessageTraits<tf2_msgs::TFMessage> {
  static void inspect(const tf2_msgs::TFMessage& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    for (const auto& transform : m.transforms) inspectFrame(transform.header.frame_id, limits, issues);
    NumericInspector<tf2_msgs::TFMessage>::inspect(m, limits, issues);
    for (const auto& transform : m.transforms) inspectQuaternion(transform.transform.rotation, issues);
  }
};
template<> struct MessageTraits<sensor_msgs::PointCloud2> {
  static void inspect(const sensor_msgs::PointCloud2& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    NumericInspector<sensor_msgs::PointCloud2>::inspect(m, limits, issues);
    if (m.is_bigendian || m.point_step == 0 ||
        static_cast<std::uint64_t>(m.width)*m.point_step > m.row_step ||
        static_cast<std::uint64_t>(m.height)*m.row_step > m.data.size()) return;
    for (const auto& field : m.fields) {
      if (field.name != "x" && field.name != "y" && field.name != "z" && field.name != "time") continue;
      if (field.datatype != sensor_msgs::PointField::FLOAT32 || field.count != 1 ||
          static_cast<std::uint64_t>(field.offset)+sizeof(float) > m.point_step) continue;
      for (std::uint32_t row = 0; row < m.height; ++row) for (std::uint32_t col = 0; col < m.width; ++col) {
        float value;
        std::memcpy(&value, m.data.data()+static_cast<std::size_t>(row)*m.row_step+
          static_cast<std::size_t>(col)*m.point_step+field.offset, sizeof(value));
        checkNumber(value, limits, issues);
      }
    }
  }
};
template<> struct MessageTraits<interfaces::Objects> {
  static void inspect(const interfaces::Objects& m, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectFrame(m.header.frame_id, limits, issues);
    NumericInspector<interfaces::Objects>::inspect(m, limits, issues);
    for (const auto& object : m.objects) {
      inspectQuaternion(object.bbox_pose.orientation, issues);
    }
  }
};
}}  // namespace node_pipeline::detail
