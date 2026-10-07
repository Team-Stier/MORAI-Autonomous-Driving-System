#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace lidar_leveler {
class RollPitchLeveler {
 public:
  sensor_msgs::PointCloud2 process(const sensor_msgs::PointCloud2& cloud,
                                  const interfaces::CompetitionVehicleStatus& competition,
                                  const tf2_msgs::TFMessage& tf_static) {
    // #LEVELING
    // 이 Core는 차량 차체가 roll/pitch할 때 센서에서 본 지면과 물체가 함께
    // 기울어지는 효과를 제거하여 ROI·DBSCAN·OBB를 수평 좌표에서 계산하게 한다.
    // 실제 도로 자체의 경사까지 평평하게 만드는 지면 추정 알고리즘은 아니다.
    // 입력은 velodyne 센서 좌표의 점군이며, 먼저 /tf_static의 장착 변환으로
    // base_link의 차량 뒷차축 중심 기준 좌표로 옮긴다. 장착 회전과 위치 offset은
    // 여기서 한 번만 적용되므로 이후 단계에서 (1.5, 0, 1.403) 등을 다시 더하면 안 된다.
    // 그 다음 해당 점군 처리에 Pipeline이 전달한 CompetitionVehicleStatus의
    // roll_rad·pitch_rad로 base_link 좌표를 base_link_level 좌표로 표현한다.
    // Pipeline은 점군 시각에 대응하는 자세를 조립하므로 Core가 최신 자세를
    // 별도로 읽거나 다른 시점의 값을 대신 사용하지 않는다.
    // 결과 frame은 차체와 함께 기울어지는 base_link가 아니다. 같은 뒷차축 원점과
    // 차량 yaw를 공유하면서 roll/pitch가 0인 수평 perception 좌표계다.
    // +x는 수평 heading 방향, +y는 좌측, +z는 중력 반대 방향이며 이후 ROI,
    // VoxelGrid, DBSCAN, OBB와 /objects는 이 좌표를 그대로 사용한다.
    // downstream이 차체나 map 좌표를 필요로 하면 자신의 입력 경계에서 timestamp에
    // 맞는 명시적인 변환을 해야 한다. frame 이름만 바꾸면 자세가 중복 반영될 수 있다.
    const auto mount = std::find_if(tf_static.transforms.begin(), tf_static.transforms.end(),
      [&](const geometry_msgs::TransformStamped& transform) {
        return transform.header.frame_id == "base_link" && transform.child_frame_id == cloud.header.frame_id;
      });
    const auto& transform = tf_static.transforms.at(mount - tf_static.transforms.begin()).transform;
    const auto& rotation = transform.rotation;
    const auto& translation = transform.translation;
    const tf2::Transform mounting_tf(tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
      tf2::Vector3(translation.x, translation.y, translation.z));

    // #LEVELING_ROTATION
    // 차량 자세의 convention은 R_world_body = Rz(yaw) * Ry(pitch) * Rx(roll)이다.
    // yaw만 유지하는 수평 frame은 R_world_level = Rz(yaw)이므로 body -> level은
    // R_level_body = R_world_level^T * R_world_body = Ry(pitch) * Rx(roll)이다.
    // 즉 차량 좌표의 점을 수평 frame으로 옮길 때 rad 단위 roll/pitch의 입력 부호를
    // 그대로 유지하여 setRPY(roll, pitch, 0)에 넣는다. 점에는 roll 다음 pitch가 적용된다.
    // 이는 frame 변환 방향을 유도한 결과이지, 기울기를 제거한다는 말만 보고
    // 단순히 -roll/-pitch를 대입하는 규칙이 아니다. 센서가 관측한 world 점은
    // 이미 차량 자세의 역회전을 포함하므로 이 body -> level 회전이 기울기를 상쇄한다.
    // yaw는 두 frame에 공통이어서 소거되며 새 yaw 회전을 적용하지 않는다.
    // 최종 식은 p_level = Ry(pitch) * Rx(roll) * (R_mount * p_raw + t_mount)다.
    // 부호와 곱 순서는 ±roll/±pitch·동시 회전·world-ground synthetic test로 검증된다.
    // 의미를 확인하지 않고 부호나 순서를 바꾸면 수평 지면이 다시 기울거나
    // 장착 이동이 잘못 회전하여 ROI와 객체 중심·orientation에 오차가 생긴다.
    tf2::Quaternion leveling_rotation;
    leveling_rotation.setRPY(competition.roll_rad, competition.pitch_rad, 0.0);
    leveling_rotation.normalize();

    auto output = cloud;
    const auto fieldOffset = [&](const char* name) {
      const auto field = std::find_if(output.fields.begin(), output.fields.end(),
        [&](const sensor_msgs::PointField& value) { return value.name == name; });
      return output.fields.at(field - output.fields.begin()).offset;
    };
    const std::array<std::uint32_t, 3> offsets = {{fieldOffset("x"), fieldOffset("y"), fieldOffset("z")}};
    // XYZ are native FLOAT32. row_step includes row padding; point_step includes point padding.
    for (std::size_t row = 0; row < output.height; ++row) {
      for (std::size_t column = 0; column < output.width; ++column) {
        auto* point_data = output.data.data() + row * output.row_step + column * output.point_step;
        std::array<float, 3> coordinates;
        for (std::size_t axis = 0; axis < coordinates.size(); ++axis)
          std::memcpy(&coordinates[axis], point_data + offsets[axis], sizeof(float));
        const auto point_base = mounting_tf * tf2::Vector3(coordinates[0], coordinates[1], coordinates[2]);
        const auto point_level = tf2::quatRotate(leveling_rotation, point_base);
        coordinates = {{static_cast<float>(point_level.x()), static_cast<float>(point_level.y()),
          static_cast<float>(point_level.z())}};
        for (std::size_t axis = 0; axis < coordinates.size(); ++axis)
          std::memcpy(point_data + offsets[axis], &coordinates[axis], sizeof(float));
      }
    }
    return output;
  }
};
}  // namespace lidar_leveler
