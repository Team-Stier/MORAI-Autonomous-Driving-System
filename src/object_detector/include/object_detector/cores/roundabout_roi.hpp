#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <tuple>
#include <utility>
#include <vector>

namespace object_detector {
struct RoundaboutRoiParameters {
  lanelet::LaneletMapPtr map;
  double range_m = 50;
  double boundary_margin_m = 0.5;
  double z_min_m = 0.2;
  double z_max_m = 2;
};

class RoundaboutRoi {
 public:
  explicit RoundaboutRoi(RoundaboutRoiParameters parameters) : parameters_(std::move(parameters)) {
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    for (const auto& lane : parameters_.map->laneletLayer) {
      if (!rules->canPass(lane) && !rules->canPass(lane.invert())) continue;
      Region region;
      const auto left = lane.leftBound2d().basicLineString();
      const auto right = lane.rightBound2d().basicLineString();
      region.polygon.assign(left.begin(), left.end());
      region.polygon.insert(region.polygon.end(), right.rbegin(), right.rend());
      region.minimum = region.maximum = region.polygon.front();
      for (const auto& p : region.polygon) {
        region.minimum = region.minimum.cwiseMin(p);
        region.maximum = region.maximum.cwiseMax(p);
      }
      regions_.push_back(std::move(region));
    }
  }

  std::tuple<sensor_msgs::PointCloud2, interfaces::CompetitionVehicleStatus, tf2_msgs::TFMessage> process(
      const sensor_msgs::PointCloud2& cloud, const interfaces::CompetitionVehicleStatus& competition,
      const tf2_msgs::TFMessage& tf) {
    const auto& transform = std::find_if(tf.transforms.begin(), tf.transforms.end(),
        [](const geometry_msgs::TransformStamped& value) {
          return value.child_frame_id == "base_link_level";
        })->transform;
    const auto& q = transform.rotation;
    const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    const Point ego(transform.translation.x, transform.translation.y);
    const double range_squared = parameters_.range_m * parameters_.range_m;
    const double map_range = parameters_.range_m + std::sqrt(2.0)*parameters_.boundary_margin_m;
    std::vector<const Region*> nearby;
    for (const auto& region : regions_) {
      const Point nearest = ego.cwiseMax(region.minimum).cwiseMin(region.maximum);
      if ((nearest-ego).squaredNorm() <= map_range*map_range) nearby.push_back(&region);
    }
    const double cr = std::cos(competition.roll_rad), sr = std::sin(competition.roll_rad);
    const double cp = std::cos(competition.pitch_rad), sp = std::sin(competition.pitch_rad);
    const auto offset = [&](const char* name) {
      return std::find_if(cloud.fields.begin(), cloud.fields.end(),
          [&](const sensor_msgs::PointField& field) { return field.name == name; })->offset;
    };
    const std::uint32_t offsets[] = {offset("x"), offset("y"), offset("z")};
    auto output = cloud;
    std::uint32_t retained = 0;
    for (std::uint32_t row = 0; row < cloud.height; ++row) {
      for (std::uint32_t col = 0; col < cloud.width; ++col) {
        const auto* record = cloud.data.data() + static_cast<std::size_t>(row)*cloud.row_step +
                                               static_cast<std::size_t>(col)*cloud.point_step;
        float xyz[3];
        for (unsigned axis = 0; axis < 3; ++axis)
          std::memcpy(&xyz[axis], record+offsets[axis], sizeof(float));
        if (xyz[2] < parameters_.z_min_m || xyz[2] > parameters_.z_max_m ||
            double(xyz[0])*xyz[0]+double(xyz[1])*xyz[1] > range_squared) continue;
        const double bx = cp*xyz[0]-sp*xyz[2], bz = sp*xyz[0]+cp*xyz[2];
        const double by = cr*xyz[1]+sr*bz, body_z = -sr*xyz[1]+cr*bz;
        if (bx >= -0.7899996042251587 && bx <= 3.845008373260498 &&
            by >= -1.0804176330566406 && by <= 1.080417275428772 &&
            body_z >= -0.3684999644756317 && body_z <= 1.2755463123321533) continue;
        const Point point = ego + Point(cy*xyz[0]-sy*xyz[1], sy*xyz[0]+cy*xyz[1]);
        if (!inRoad(point, nearby)) continue;
        // Test the 3x3 footprint against the road union, not each lane separately.
        bool within_road = true;
        for (int dx = -1; dx <= 1 && within_road; ++dx)
          for (int dy = -1; dy <= 1 && within_road; ++dy)
            if (dx != 0 || dy != 0)
              within_road = inRoad(point+Point(dx*parameters_.boundary_margin_m,
                                               dy*parameters_.boundary_margin_m), nearby);
        if (!within_road) continue;
        std::memcpy(output.data.data()+static_cast<std::size_t>(retained)*cloud.point_step,
                    record, cloud.point_step);
        ++retained;
      }
    }
    output.height = 1;
    output.width = retained;
    output.row_step = retained*output.point_step;
    output.data.resize(output.row_step);
    return std::make_tuple(std::move(output), competition, tf);
  }

 private:
  using Point = lanelet::BasicPoint2d;
  struct Region {
    std::vector<Point> polygon;
    Point minimum, maximum;
  };

  static bool inRoad(const Point& point, const std::vector<const Region*>& regions) {
    for (const auto* region : regions)
      if (!(point.array() < region->minimum.array()).any() &&
          !(point.array() > region->maximum.array()).any() && contains(region->polygon, point)) return true;
    return false;
  }

  static bool contains(const std::vector<Point>& polygon, const Point& point) {
    bool inside = false;
    for (std::size_t i = 0, j = polygon.size()-1; i < polygon.size(); j = i++) {
      const auto& a = polygon[j];
      const auto& b = polygon[i];
      const Point delta = b-a;
      const double length_squared = delta.squaredNorm();
      const double ratio = length_squared == 0 ? 0 :
          std::max(0.0, std::min(1.0, (point-a).dot(delta)/length_squared));
      if ((point-a-ratio*delta).squaredNorm() <= 1e-16) return true;
      if ((a.y() > point.y()) != (b.y() > point.y()) &&
          point.x() < a.x()+(point.y()-a.y())*delta.x()/delta.y()) inside = !inside;
    }
    return inside;
  }

  RoundaboutRoiParameters parameters_;
  std::vector<Region> regions_;
};
}  // namespace object_detector
