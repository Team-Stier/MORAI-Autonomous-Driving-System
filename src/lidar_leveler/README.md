# lidar_leveler

`motion_leveler.hpp`와 `acquisition_leveler.hpp` Core가 센서 장착 TF와 차량 자세 이력을 적용해 점군을 수평화한다.
노드 구성은 `config/lidar_leveler/`, 확장점은 `include/lidar_leveler/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `lidar_leveler_node` |
| 구독 | `/velodyne_points`(frame `velodyne`), `/vehicle/competition_status`, `/tf_static` |
| 발행 | `/velodyne_points/leveled` (`base_link_level`, 입력 점군 stamp) |
| 실행 | `/velodyne_points` 입력마다 |
| Stage | `level`: `process(cloud, competition, tf_static)` → 수평화한 cloud |

- `base_link_level`은 뒷차축 중심과 차량 yaw를 유지하고 roll·pitch가 0인 좌표계다. 도로 경사 자체는 보존한다.
- S11~S13은 `acquisition_leveler.hpp`, 나머지 섹션은 `motion_leveler.hpp`를 선택한다.
- 두 Core는 점별 `time`과 차량 자세·속도 이력으로 스캔 내 회전·이동을 보정하고 차량 자체 반사점을 제거한다. `motion_leveler.hpp`는 도로 평면 잔차를 비교해 이동 보정 점군과 기준 시각의 강체 보정 점군 중 하나를 선택한다.
- Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. cloud의 Stale·ReceiveTimeout·Skew는 허용 issue다.
- `/tf_static`은 latched 입력이므로 stamp·수신 timeout·skew를 검사하지 않는다.
- 출력 cloud는 입력 cloud의 시간 메타데이터를 상속한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
