#pragma once
#include <node_pipeline/types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <string>

namespace lidar_leveler {
// #FRAME_CONTRACT
// #BASE_LINK_LEVEL
// production 입력 payload는 velodyne -> base_link 장착 변환과 차량 roll/pitch
// 수평화를 이미 마친 점군이다. 이 HeaderInjector는 추가 회전을 하지 않고
// 최종 발행 시 원본 점군 stamp와 실제 계산 좌표계의 이름만 한 번 주입한다.
// base_link_level의 원점은 base_link와 같은 차량 뒷차축 중심이고, +x는 수평
// 차량 heading 방향, +y는 좌측, +z는 중력 반대 방향이다. 차량 yaw를 따르며
// roll과 pitch는 0이다. 기존 base_link는 차체와 함께 roll/pitch하는 frame이다.
// 두 frame은 zero roll/pitch에서만 축이 일치하므로 서로의 별칭이 아니다.
// 따라서 수평 좌표값에 base_link라는 이름만 붙이면 안 된다. 그러면 downstream의
// TF 변환이나 장애물 계산에서 roll/pitch가 중복 반영될 수 있다.
// production YAML frame은 반드시 base_link_level이며 /velodyne_points/leveled가
// 이 계약으로 발행된다. 소비 모듈은 다른 frame이 필요하면 stamp에 맞게 실제
// 좌표를 변환해야 한다. 별도 PassThrough 비교 설정만 mounting-only 결과에 맞춰
// Core 매핑과 frame을 함께 base_link로 지정하며 production과 혼용하지 않는다.
struct HeaderInjector {
  std::string frame = "base_link_level";
  void injectHeader(sensor_msgs::PointCloud2& message, const node_pipeline::Context<3>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = frame;
  }
};
}  // namespace lidar_leveler
