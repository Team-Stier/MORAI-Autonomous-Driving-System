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

- `curvature_velocity100.hpp`와 `curvature_velocity60.hpp`는 속도가 NaN인 point에만 각각 100/3.6, 60/3.6 m/s 상한 안에서 곡률 기반 속도와 가속도 0 m/s²를 채운다. 모든 `inject` Core는 0을 포함한 기존 속도와 해당 point의 가속도를 유지한다.
- 두 정지 Core는 신호 매핑만 다르다. 직진용은 GREEN 포함, 좌회전용은 LEFT 포함 상태를 진행 신호로 사용한다. 진행 신호 없이 YELLOW가 포함되면 제동거리를 판정하고, 나머지 인식 상태는 정지한다. UNKNOWN·OFF는 새 정지를 요구하지 않으며 진행 신호 확인 횟수를 초기화한다.
- 정지선은 OSM의 `type=stop_line` way를 양 끝에서 `stop_line_extension_m`만큼 연장한 검출선 중 현재 경로가 최초 접촉하는 선이다. 지도 원본은 유지한다. 경로 첫 점은 차량의 경로 투영점이다.
- `stop_line_state.yaml`의 `x_m`만큼 정지선 앞을 뒷차축 기준 목표점으로 잡으며, 좌회전은 `stop_line_cutoff_left.yaml`의 `extra_stop_offset_m`(1.5 m)를 추가한다. 목표점 이전 `y_m` 범위에서는 남은 거리 `d = v²/(2 × z_mps2) + v × stop_response_time_sec`를 만족하는 속도 상한과 기존 속도 중 작은 값을 적용한다. 목표점까지 거리가 `stop_position_tolerance_m` 이하인 점과 목표점 이후의 속도는 0이다. 남은 거리가 `y_m`보다 작다는 이유로 경로 전체를 즉시 정지시키지 않는다.
- 시간 경로가 짧아져 정지선이 검출 범위에서 사라지면 기존 정지선의 지도 위치와 남은 거리를 유지한다. 해당 정지선을 통과한 뒤에는 그 선의 정지 상태를 해제한다. 진행 방향과 정지선 위치를 확인할 수 없는 회피 경로에서는 정지를 유지한다.
- 노란불은 `signed_velocity_mps² / (2 × z_mps2)`가 목표점까지의 경로 거리 이하이거나, `yellow_stop_deceleration_mps2`가 양수이고 `v²/(2 × yellow_stop_deceleration_mps2) + v × stop_response_time_sec`가 정지선까지의 경로 거리에서 `front_axle_m`과 `yellow_stop_margin_m`을 뺀 값 이하이면 정지한다. 이 계수는 `integrated_stop_straight.hpp`·`integrated_stop_left.hpp`·`integrated_stop_dynamic.hpp`의 `<core>.yaml`에 두며 0은 비활성이다.
- `integrated_stop_straight.hpp`·`integrated_stop_left.hpp`·`integrated_stop_dynamic.hpp`는 정지 상태가 아니고 최초 접촉 정지선이 있으며 신호가 UNKNOWN·OFF이고 그 정지선에서 진행 신호를 관측하지 않았으면, 정지선까지 `blind_detection_distance_m` 이내의 점에 목표점까지 남은 거리가 `blind_detection_distance_m`에서 목표점 오프셋을 뺀 값일 때의 위 정지 속도 상한을, 그 앞의 점에 `blind_deceleration_mps2` 감속으로 그 상한에 도달하는 속도 상한을 적용한다. 정지선 이후의 점은 제한하지 않는다. 진행 신호 관측은 최초 접촉 정지선이 바뀌거나 아래 점프 초기화가 일어날 때까지 유지한다. 계수는 `config/velocity_planner/parameters/<core>.yaml`이며 `blind_detection_distance_m: 0`은 비활성이다.
- 정지 상태와 시작 시각은 세 cutoff Core의 Bank에서 공유한다. 해당 방향의 진행 신호가 서로 다른 영상 stamp로 연속 `n_frames`회 관측되거나, 경로 stamp 기준 정지 시작 후 `stop_timeout_sec`(기본 100초)가 지나면 해제한다. 정지 신호·진행 신호 매핑 변경은 확인 횟수를 초기화한다.
- `integrated_stop_pass.hpp`·`integrated_stop_straight.hpp`·`integrated_stop_left.hpp`·`integrated_stop_dynamic.hpp`·`stop_line_pass_through.hpp`는 직전 호출 대비 경로 첫 점 이동이 `jump_distance_m + jump_speed_mps × 경과 시간(최대 2초)`을 넘거나 경로 stamp가 되돌아가면 정지 상태·해제 정지선·진행 신호 확인 횟수·마지막 확인 영상 stamp·교차로 잔여 거리·진행 신호 관측을 초기화한다.
- 시간 만료로 해제한 정지선은 경로의 최초 접촉 정지선이 달라지거나 접촉이 사라지거나 위 점프 초기화가 일어날 때까지 재정지하지 않는다.
- `stop_line_pass_through.hpp`는 진행 신호 확인 횟수를 초기화하고 시간 만료나 위 점프 초기화 전까지 기존 정지 상태를 유지한다.
- `transform`의 `tf.hpp`는 map→base_link 역변환을 `points`와 `lateral_reference`의 모든 pose에 적용하고 점 순서·속도·가속도를 유지한다. Validator가 필요한 TF와 pose 계약을 검사한다.
- 출력 path와 각 point의 stamp는 입력 값을 상속한다. 최종 frame 헤더는 Pipeline이 주입한다.
- `inject`의 모든 Core는 Policy `missing_light_unknown.hpp`로 Missing light에 `TrafficLight::UNKNOWN` (`frame_id=camera_7_optical_frame`, `stamp=0`)을 채운다. 다른 입력의 Missing과 값·stamp·frame 오류 및 이미 수신한 light의 Stale·ReceiveTimeout은 보정하지 않고 거부한다.
- `cutoff`·`transform`의 Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. path·tf의 Stale·ReceiveTimeout·Skew 및 light의 Skew는 허용 issue다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

통합 계획기의 `time_from_start`는 속도·가속도가 바뀌지 않은 경로에서 유지한다. `integrated_stop_straight.hpp`·`integrated_stop_left.hpp`·`integrated_stop_pass.hpp`가 교차로 감속·신호 정지로 속도나 가속도를 변경하면 거리·속도에 맞춰 시간을 다시 계산하고 최초 정지점에서 대기한다. 시간 계획 구간 뒤의 횡방향 미리보기는 `lateral_reference`로 유지한다. 제어기는 시간 경로의 v(t)와 공간 경로의 v(s)를 구분해 추종한다.
