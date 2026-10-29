#pragma once
#include <node_pipeline/extension.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <vector>

namespace localization {
// 공통 검사 이후 GPS 고유 조건을 검사한다. NO_FIX와 위경도 0,0은 고도와 무관하게 측위 누락이다.
struct GpsFix {
  void operator()(const sensor_msgs::NavSatFix& msg, const node_pipeline::HealthLimits&,
                  std::vector<node_pipeline::Kind>& issues) const {
    if (msg.status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX ||
        (msg.latitude == 0 && msg.longitude == 0))
      issues.push_back(node_pipeline::Kind::Missing);
    if (msg.latitude < -80 || msg.latitude > 84 || msg.longitude < -180 || msg.longitude > 180)
      issues.push_back(node_pipeline::Kind::OutRangePositive);
  }
};
}  // namespace localization
