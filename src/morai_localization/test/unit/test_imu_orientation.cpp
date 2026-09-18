#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "morai_localization/orientation/imu_orientation.hpp"

namespace morai_localization {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(ImuOrientation, NormalizesQuaternion) {
  geometry_msgs::Quaternion input;
  input.z = std::sqrt(2.0);
  input.w = std::sqrt(2.0);

  const geometry_msgs::Quaternion output = normalizeQuaternion(input);
  EXPECT_NEAR(output.z, std::sqrt(0.5), 1.0e-12);
  EXPECT_NEAR(output.w, std::sqrt(0.5), 1.0e-12);
}

TEST(ImuOrientation, RejectsInvalidQuaternion) {
  geometry_msgs::Quaternion zero;
  EXPECT_THROW(normalizeQuaternion(zero), std::invalid_argument);

  geometry_msgs::Quaternion non_finite;
  non_finite.w = 1.0;
  non_finite.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(normalizeQuaternion(non_finite), std::invalid_argument);
}

TEST(ImuOrientation, ExtractsPlanarYaw) {
  EXPECT_NEAR(yawFromQuaternion(quaternionFromYaw(kPi / 2.0)),
              kPi / 2.0, 1.0e-12);
  EXPECT_NEAR(yawFromQuaternion(quaternionFromYaw(-kPi / 3.0)),
              -kPi / 3.0, 1.0e-12);
}

TEST(ImuOrientation, WrapsAnglesToPiRange) {
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), kPi, 1.0e-12);
  EXPECT_NEAR(normalizeAngle(-3.0 * kPi), -kPi, 1.0e-12);
}

TEST(ImuOrientation, ChecksVectorFiniteness) {
  geometry_msgs::Vector3 vector;
  EXPECT_TRUE(isFinite(vector));
  vector.y = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(isFinite(vector));
}

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
