#pragma once

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

namespace morai_localization {

bool isFinite(const geometry_msgs::Vector3& vector);

geometry_msgs::Quaternion normalizeQuaternion(
    const geometry_msgs::Quaternion& quaternion,
    double minimum_norm = 1.0e-6);

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion);

geometry_msgs::Quaternion quaternionFromYaw(double yaw);

double normalizeAngle(double angle);

}  // namespace morai_localization
