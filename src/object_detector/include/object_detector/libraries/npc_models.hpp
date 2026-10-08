#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace object_detector {
// NPC 모델 하나다. 표면점과 collider는 모델 로컬 ROS 축(m)을 공유한다.
struct ModelTemplate {
  std::string label;
  pcl::PointCloud<pcl::PointXYZ> points;
  // 검증된 물리 collider 외곽일 때만 true다.
  bool collider_available = false;
  geometry_msgs::Pose collider_pose;
  geometry_msgs::Vector3 collider_size;
  // 표면·물리 형상이 같은 다른 자산의 의미 라벨이다. 없으면 빈 문자열이다.
  std::string semantic_label;
};

// scripts/convert_npc_models.py가 만든 little-endian NPCM v1 자산을 해석한다. 시작 시 로더만 호출한다.
class NpcModelReader {
 public:
  explicit NpcModelReader(const std::string& bytes) : bytes_(bytes) {}

  std::vector<ModelTemplate> read() {
    if (take(4) != "NPCM" || u32() != 1) throw std::invalid_argument("models_file is not an NPCM v1 asset");
    const std::uint32_t model_count = u32();
    const std::uint32_t point_count = u32();
    struct Entry { ModelTemplate model; std::uint32_t begin, count; };
    std::vector<Entry> entries;
    entries.reserve(model_count);
    for (std::uint32_t i = 0; i < model_count; ++i) {
      Entry entry;
      entry.model.label = take(u16());
      entry.model.semantic_label = take(u16());
      entry.begin = u32();
      entry.count = u32();
      entry.model.collider_available = take(1)[0] != 0;
      auto& pose = entry.model.collider_pose;
      pose.position.x = f64();
      pose.position.y = f64();
      pose.position.z = f64();
      pose.orientation.x = f64();
      pose.orientation.y = f64();
      pose.orientation.z = f64();
      pose.orientation.w = f64();
      entry.model.collider_size.x = f64();
      entry.model.collider_size.y = f64();
      entry.model.collider_size.z = f64();
      if (static_cast<std::uint64_t>(entry.begin) + entry.count > point_count)
        throw std::invalid_argument("models_file point range exceeds data: " + entry.model.label);
      entries.push_back(std::move(entry));
    }
    const std::size_t points_offset = offset_;
    if (bytes_.size() - points_offset != static_cast<std::size_t>(point_count) * 3 * sizeof(float))
      throw std::invalid_argument("models_file point data size mismatch");
    std::vector<ModelTemplate> models;
    models.reserve(entries.size());
    for (auto& entry : entries) {
      entry.model.points.reserve(entry.count);
      for (std::uint32_t i = entry.begin; i < entry.begin + entry.count; ++i) {
        float xyz[3];
        std::memcpy(xyz, bytes_.data() + points_offset + static_cast<std::size_t>(i) * sizeof(xyz), sizeof(xyz));
        entry.model.points.emplace_back(xyz[0], xyz[1], xyz[2]);
      }
      models.push_back(std::move(entry.model));
    }
    return models;
  }

 private:
  std::string take(std::size_t size) {
    if (bytes_.size() - offset_ < size) throw std::invalid_argument("models_file is truncated");
    std::string value = bytes_.substr(offset_, size);
    offset_ += size;
    return value;
  }
  template<class T> T scalar() {
    const auto raw = take(sizeof(T));
    T value;
    std::memcpy(&value, raw.data(), sizeof(T));
    return value;
  }
  std::uint16_t u16() { return scalar<std::uint16_t>(); }
  std::uint32_t u32() { return scalar<std::uint32_t>(); }
  double f64() { return scalar<double>(); }

  const std::string& bytes_;
  std::size_t offset_ = 0;
};
}  // namespace object_detector
