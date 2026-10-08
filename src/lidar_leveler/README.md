# lidar_leveler

`pass_through.hpp` Core가 센서 장착 TF로 점군을 `base_link`로 변환한다.
노드 구성은 `config/lidar_leveler/`, 확장점은 `include/lidar_leveler/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `lidar_leveler_node` |
| 구독 | `/velodyne_points`(frame `velodyne`), `/vehicle/competition_status`, `/tf_static` |
| 발행 | `/velodyne_points/leveled` (`base_link`, 입력 점군 stamp) |
| 실행 | `/velodyne_points` 입력마다 |
| Stage | `level`: `pass_through.hpp`의 `process(cloud, tf_static)` → 변환한 cloud |
| 등록 Core | `roll_pitch_leveler.hpp`의 `process(cloud, competition, tf_static)` → 장착 TF와 roll·pitch로 수평화한 cloud |
| 등록 Core | `roll_pitch_roi_leveler.hpp`의 `process(cloud, competition, tf_static)` → 수평화 후 `z_min_m`~`z_max_m` 점만 남긴 1행 cloud |
| 등록 Core | `pass_through_roi.hpp`의 `process(cloud, tf_static)` → 변환 후 `z_min_m`~`z_max_m` 점만 남긴 1행 cloud |

- section을 구독하지 않으므로 UNKNOWN=0에 매핑한 `pass_through.hpp`를 사용한다.
- Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. cloud의 Stale·ReceiveTimeout·Skew는 허용 issue다.
- `/tf_static`은 latched 입력이므로 stamp·수신 timeout·skew를 검사하지 않는다.
- 출력 cloud는 입력 cloud의 시간 메타데이터를 상속한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
