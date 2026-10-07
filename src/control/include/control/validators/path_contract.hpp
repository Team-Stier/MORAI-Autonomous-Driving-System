#pragma once
#include <control/types.hpp>
#include <node_pipeline/sdk.hpp>
#include <cmath>
namespace control { namespace validators {
struct PathContract {
  void operator()(const node_pipeline::InputView& inputs, node_pipeline::ProfileHealth& health) const {
    if (!inputs.present("path")) return;
    const auto& path = inputs.get<interfaces::Path>("path");
    auto& issues = health.issues("path");
    if (path.points.size() < 2) issues.push_back(node_pipeline::Kind::Missing);
    bool distinct = false;
    for (std::size_t i = 0; i < path.points.size(); ++i) {
      if (path.points[i].velocity < 0) issues.push_back(node_pipeline::Kind::OutRangeNegative);
      if (i > 0) {
        const auto& point = path.points[i].pose.pose.position;
        const auto& previous = path.points[i-1].pose.pose.position;
        distinct = distinct || std::hypot(point.x-previous.x, point.y-previous.y) > 1e-6;
      }
    }
    if (!distinct) issues.push_back(node_pipeline::Kind::Missing);
  }
};
}}
