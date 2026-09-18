#include "morai_localization/orientation/imu_orientation.hpp"

#include <cmath>
#include <stdexcept>

namespace morai_localization {

bool isFinite(const geometry_msgs::Vector3& vector) {
  return std::isfinite(vector.x) && std::isfinite(vector.y) &&
         std::isfinite(vector.z);
}

geometry_msgs::Quaternion normalizeQuaternion(
    const geometry_msgs::Quaternion& quaternion, double minimum_norm) {
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
      !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w)) {
    throw std::invalid_argument("quaternion contains a non-finite value");
  }
  if (!std::isfinite(minimum_norm) || minimum_norm <= 0.0) {
    throw std::invalid_argument(
        "minimum quaternion norm must be finite and positive");
  }

  const double squared_norm =
      quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (squared_norm < minimum_norm * minimum_norm) {
    throw std::invalid_argument("quaternion norm is too small");
  }

  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  geometry_msgs::Quaternion normalized;
  normalized.x = quaternion.x * inverse_norm;
  normalized.y = quaternion.y * inverse_norm;
  normalized.z = quaternion.z * inverse_norm;
  normalized.w = quaternion.w * inverse_norm;
  return normalized;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  const geometry_msgs::Quaternion normalized =
      normalizeQuaternion(quaternion);
  const double numerator =
      2.0 * (normalized.w * normalized.z +
             normalized.x * normalized.y);
  const double denominator =
      1.0 - 2.0 * (normalized.y * normalized.y +
                   normalized.z * normalized.z);
  return std::atan2(numerator, denominator);
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  if (!std::isfinite(yaw)) {
    throw std::invalid_argument("yaw must be finite");
  }

  geometry_msgs::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

double normalizeAngle(double angle) {
  if (!std::isfinite(angle)) {
    throw std::invalid_argument("angle must be finite");
  }
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace morai_localization
