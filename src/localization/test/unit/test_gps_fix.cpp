#include <localization/validators/gps_fix.hpp>
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

namespace {
bool missing(const sensor_msgs::NavSatFix& fix) {
  std::vector<node_pipeline::Kind> issues;
  localization::GpsFix()(fix, node_pipeline::HealthLimits{}, issues);
  return std::find(issues.begin(), issues.end(), node_pipeline::Kind::Missing) != issues.end();
}

sensor_msgs::NavSatFix fix(double latitude, double longitude, double altitude) {
  sensor_msgs::NavSatFix message;
  message.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  message.latitude = latitude;
  message.longitude = longitude;
  message.altitude = altitude;
  return message;
}
}  // namespace

TEST(GpsFix, ValidFixIsNotMissing) { EXPECT_FALSE(missing(fix(37.24, 126.77, 28.0))); }

TEST(GpsFix, NoFixIsMissing) {
  auto message = fix(37.24, 126.77, 28.0);
  message.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
  EXPECT_TRUE(missing(message));
}

TEST(GpsFix, ZeroLatitudeLongitudeIsMissingWithAltitude) {
  EXPECT_TRUE(missing(fix(0, 0, 0)));
  EXPECT_TRUE(missing(fix(0, 0, 28.0)));
}
