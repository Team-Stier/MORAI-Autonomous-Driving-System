#pragma once
#include <node_pipeline/types.hpp>
#include <interfaces/Objects.h>

namespace object_detector {
// #BASE_LINK_LEVEL
// 입력 cloud와 출력 Objects는 roll/pitch가 제거된 동일한 perception frame이다.
// 이 상수는 ROS 입력 검사와 헤더 주입에 함께 사용되어 계산 frame과 발행 이름이
// 어긋나지 않게 한다. 차체와 함께 기울어지는 base_link로 임의 치환하면 안 된다.
constexpr const char* kPerceptionFrame = "base_link_level";
// #FRAME_CONTRACT
// DBSCAN과 OBB 결과는 실제 계산한 base_link_level 좌표 그대로 발행한다.
// HeaderInjector는 원본 LiDAR stamp를 유지하며 /objects.header.frame_id를
// base_link_level로 지정한다. 좌표 회전·이동은 여기서 수행하지 않는다.
// interfaces::Object의 position, bbox_pose.position, bbox_pose.orientation,
// 유효한 velocity와 acceleration은 모두 Objects.header.frame_id를 기준으로
// 해석한다. orientation은 bbox local 축을 이 header frame으로 표현하는 회전이며
// bbox_size는 박스 자체 local x/y/z 축의 전체 길이이므로 frame 회전과 별개다.
// 실제 좌표를 변환하지 않고 header만 base_link로 바꾸면 이름과 수치가 달라진다.
// downstream은 frame_id를 확인하고 다른 frame이 필요하면 자신의 입력 경계에서
// 메시지 시각에 맞는 명시적인 변환을 수행해야 한다.
//
// #NO_LEVEL_TO_BASE
// 초기 설계에서는 base_link_level에서 OBB를 계산한 뒤 검출 결과를 기존
// base_link로 역변환하여 /objects를 발행하는 방식을 검토했다.
// 하지만 정확한 역변환을 하려면 해당 LiDAR frame을 수평화할 때 사용했던
// 동일 timestamp의 roll/pitch 변환을 검출 결과 생성까지 함께 보존해야 한다.
// DBSCAN/OBB가 끝난 시점에 최신 CompetitionVehicleStatus를 다시 읽으면
// 차량 이동 중 자세가 변하여 수평화에 사용한 자세와 다를 수 있다.
// 이 경우 두 회전이 정확히 상쇄되지 않아 객체 중심과 orientation에 오차가 생긴다.
// 불필요한 시간 동기화 복잡성과 좌표 오차 가능성을 perception 내부에 추가하지
// 않기 위해 ROI·VoxelGrid·DBSCAN·OBB와 /objects를 모두 base_link_level에 유지한다.
// 다른 모듈이 base_link 또는 map 좌표가 필요하면 해당 모듈 입력 경계에서
// timestamp-aware TF 변환을 수행해야 한다. object_detector 내부에서는 최신
// roll/pitch를 임의로 다시 읽어 객체를 역회전하지 않는다.
struct HeaderInjector {
  void injectHeader(interfaces::Objects& message, const node_pipeline::Context<1>& context) const {
    message.header.stamp.fromNSec(context.stamp_ns);
    message.header.frame_id = kPerceptionFrame;
  }
};
}  // namespace object_detector
