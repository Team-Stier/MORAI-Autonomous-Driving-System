#pragma once
#include <node_pipeline/sdk.hpp>
#include <sensor_msgs/NavSatFix.h>

namespace localization { namespace validators {
struct FullEkf {
  void operator()(const node_pipeline::InputView& input, node_pipeline::ProfileHealth& health) const {
    if (!input.present("gps")) return;
    const auto& gps = input.get<sensor_msgs::NavSatFix>("gps");
    auto& issues = health.issues("gps");
    if (gps.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX ||
        (gps.latitude == 0 && gps.longitude == 0 && gps.altitude == 0))
      issues.push_back(node_pipeline::Kind::Missing);
    if (gps.latitude < -80 || gps.latitude > 84 || gps.longitude < -180 || gps.longitude > 180)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
};
}}  // namespace localization::validators
