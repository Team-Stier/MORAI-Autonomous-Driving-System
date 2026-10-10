# velocity_planner

map 경로에 속도를 주입하고 신호등 정지를 적용한 뒤 base_link로 변환한다.
노드 구성은 `config/velocity_planner/`, 확장점은 `include/velocity_planner/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `velocity_planner_node` |
| 구독 | `/local_path`, `/vehicle/competition_status`, `/traffic_light`, `/tf`, `/section_annotation` |
| 발행 | `/local_path/velocity_injected` (`base_link`, 입력 경로 stamp; 각 point pose header 포함) |
| 실행 | `/local_path` 입력마다 |

| Stage | 입력 | 출력 |
| --- | --- | --- |
| `inject` | path·light·competition·tf | 속도 주입한 map path·light·competition·tf |
| `cutoff` | path·light·competition·tf | 정지 프로파일을 적용한 map path·tf |
| `transform` | path·tf | base_link path |

- `inject`는 섹션 11·12에서 `curvature_velocity100.hpp`, 나머지에서 `curvature_velocity60.hpp`를 사용한다. 속도가 NaN인 point에만 각각 100/3.6, 60/3.6 m/s 상한 안에서 곡률 기반 속도와 가속도 0 m/s²를 채운다. 모든 `inject` Core는 0을 포함한 기존 속도와 해당 point의 가속도를 유지한다.
- `cutoff`는 현재 체크포인트 CSV 기준 섹션 3·4·5에서 `stop_line_cutoff_straight.hpp`, 7에서 `stop_line_cutoff_left.hpp`, 나머지에서 `stop_line_pass_through.hpp`를 사용한다.
- 두 정지 Core는 신호 매핑만 다르다. 직진용은 GREEN 포함, 좌회전용은 LEFT 포함 상태를 진행 신호로 사용한다. 진행 신호 없이 YELLOW가 포함되면 제동거리를 판정하고, 나머지 인식 상태는 정지한다. UNKNOWN·OFF는 새 정지를 요구하지 않으며 기존 정지를 해제하지 않는다.
- 정지선은 OSM의 `type=stop_line` way 중 현재 경로가 최초 접촉하는 선이다. 경로 첫 점은 차량의 경로 투영점이다.
- `stop_line_state.yaml`의 `x_m`만큼 정지선 앞을 목표점으로 잡는다. 목표점 이후 속도는 0, 목표점 이전 `y_m` 구간은 시작점 속도에서 0까지 경로 거리에 비례해 감소한다. 목표점까지 거리가 `y_m` 미만이면 전체 속도를 0으로 채운다.
- 노란불은 `signed_velocity_mps² / (2 × z_mps2)`가 목표점까지의 경로 거리 이하이면 정지한다.
- 정지 상태는 두 Core의 Bank에서 공유한다. 해당 방향의 진행 신호가 서로 다른 영상 stamp로 연속 `n_frames`회 관측되면 해제한다. UNKNOWN·OFF·정지 신호는 확인 횟수를 초기화하며, Core의 진행 신호 매핑이 바뀌어도 확인 횟수를 초기화한다.
- `stop_line_pass_through.hpp`는 기존 정지 상태를 유지하고 확인 횟수를 초기화한다. 정지 해제는 해당 방향의 정지 Core에서 수행한다.
- `transform`의 `tf.hpp`는 map→base_link 역변환을 모든 pose에 적용하고 점 순서·속도·가속도를 유지한다. Validator가 필요한 TF와 pose 계약을 검사한다.
- 출력 path와 각 point의 stamp는 입력 값을 상속한다. 최종 frame 헤더는 Pipeline이 주입한다.
- `inject`의 모든 Core는 Policy `missing_light_unknown.hpp`로 Missing light에 `TrafficLight::UNKNOWN` (`frame_id=camera_7_optical_frame`, `stamp=0`)을 채운다. 다른 입력의 Missing과 값·stamp·frame 오류 및 이미 수신한 light의 Stale·ReceiveTimeout은 보정하지 않고 거부한다.
- `cutoff`·`transform`의 Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. path·tf의 Stale·ReceiveTimeout·Skew 및 light의 Skew는 허용 issue다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
