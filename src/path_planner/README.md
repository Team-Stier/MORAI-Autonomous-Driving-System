# path_planner

Checkpoint → GlobalPath → Prediction → LocalPath의 네 Stage 파이프라인이다.
노드 구성은 `config/path_planner/`, 확장점은 `include/path_planner/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `path_planner_node` |
| 구독 | `/localization/ego`, `/vehicle/competition_status`, `/objects`, `/tf`, `/sensors/imu/data`, `/section_annotation` |
| 발행 | `/local_path` (`map`, 입력 조립 기준 stamp; 각 point pose header 포함) |
| 실행 | 30 Hz 주기 |

`launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

## Stage 입출력

| Stage | 입력 | 출력 |
| --- | --- | --- |
| `checkpoint` | ego·competition·objects·tf·imu | ego·competition·objects·tf·imu(패스스루), checkpoints(`path_planner::Checkpoints`) |
| `global` | ego·competition·objects·tf·imu·checkpoints | ego·competition·objects·tf·imu(패스스루), route(`path_planner::Route`) |
| `prediction` | ego·competition·objects·tf·imu·route | ego·competition·objects·tf·imu·route(패스스루), predictions(`path_planner::PredictedObjects`) |
| `local` | ego·competition·objects·tf·imu·route·predictions | path(`interfaces/Path`) |

Stage별 Core 선택은 `config/path_planner/selection.yaml`에서 관리한다.

`path_planner::Checkpoints`는 `std::vector<Checkpoint>`, `path_planner::Route`는 `lanelet::Optional<lanelet::routing::LaneletPath>`, `path_planner::PredictedObjects`는 `std::array<interfaces::Objects, 10>`이며 `libraries/types.hpp`에 정의한다.
패스스루 출력은 입력의 시간 메타데이터를 상속하고, checkpoints·route·predictions는 새 계산값이므로 시간 메타데이터가 없다.

## 체크포인트 Stage

체크포인트 Stage는 구독 입력 다섯 개를 그대로 반환하고, 현재 섹션 k(0~16)를 기준으로 `checkpoints.csv`에서 아직 경유하지 않은 체크포인트 k+1~16을 순서대로 담은 배열을 함께 반환한다.
섹션 0은 체크포인트 1~16 전체, 섹션 16은 빈 배열이다.
남은 체크포인트 선택·저장과 입력 전달은 각 Core 파일 안에서 수행한다. 체크포인트 파일은 `config/path_planner/parameters/checkpoint_k.yaml`의 `checkpoints_file`로 지정한다.

## GlobalPath

Bank `shared_map.hpp`는 `config/path_planner/parameters/shared_map.yaml`의 `map_file` 지도를 노드당 한 번 읽어 GlobalPath와 등록된 대체 Core LocalPath에 같은 인스턴스로 주입한다. 로더는 `checkpoints_file`의 모든 lanelet_id가 지도에 있는지 시작 시 확인한다.

`global_path.hpp`는 ego로 현재 lanelet을 찾고, 전달받은 체크포인트 중 마지막을 목적지, 나머지를 경유지로 `shortestPathVia`를 계산한다. 반환 경로 앞에는 차선 변경을 제외한 현재 lanelet의 predecessor 중 첫 번째를 포함한다. 체크포인트가 비어 있으면 predecessor와 현재 lanelet을 반환하며, predecessor가 없으면 현재 lanelet부터 반환한다.
현재 lanelet이나 체크포인트 lanelet_id 목록이 바뀔 때만 경로를 다시 계산한다. 경로 중심선은 `/visualizer/global_path`로 발행한다.

## Prediction·LocalPath

`/objects`와 예측 객체의 위치·자세·절대 속도·가속도는 map이다. `core_prediction.hpp`는 map 관측을 직접 사용해 차량은 관측 속도·가속도로 구한 등속 선회율을, 나머지는 등속 직선 모델로 예측하고 map 결과를 반환한다. 예측 간격은 `config/path_planner/parameters/core_prediction.yaml`의 `step_sec`로 지정하며 0.5초로 고정한다.
`npc_history_prediction.hpp`는 map 위치 이력으로 운동을 추정한다. 객체 좌표 변환을 위한 ego 이력은 사용하지 않는다.
`core_path_planner.hpp`는 route 중심선·차선 경계와 map 객체 예측, ego 운동 상태·IMU 가속도로 DP/QP 기반 경로를 계산한다. 차량 치수·계획 길이·도로/장애물 여유·곡률·가감속 한도는 `config/path_planner/parameters/core_path_planner.yaml`에 둔다. `frenet_path.hpp`, `frenet_speed_path.hpp`, `roundabout_merge.hpp`도 map 객체·예측을 직접 사용한다.
`route_present.hpp`와 CorePathPlanner의 `core_path_inputs.hpp`는 경로 탐색 결과가 없는 route를 Missing으로 기록하며, 이때 local Stage는 Core를 호출하지 않는다.
`core_path_inputs.hpp`는 관측·예측 속도의 availability와 유한 값을 요구한다.

모든 Core는 Policy `remove_invalid.hpp`로 값·stamp·frame 오류 입력을 제거한다. 허용 issue는 각 `registrations/*.yaml`에 선언한 입력·issue에 한정한다.
`prediction_motion_inputs.hpp`는 관측 벡터로 운동을 예측하는 Core에 속도·가속도의 availability와 유한 값을 요구한다.
`prediction_pass_through.hpp`는 관측 Objects를 10개 예측 시점에 그대로 복사하는 등록된 대체 Core이며, `prediction_core.hpp`는 등속 선회 예측 대체 Core다. `local_path.hpp`는 route 중심선을 ego 투영점부터 잘라 경로를 만드는 등록된 대체 Core이며, 경로 길이는 `config/path_planner/parameters/local_path.yaml`의 `local_path_length_m`로 지정한다.

## 예측 배열 계약

Prediction Core의 반환과 LocalPath Core의 입력은 **Objects 메시지를 원소로 갖는 길이 10의 외부 배열**이다. `Objects.objects` 안의 Object 목록을 10개로 제한하는 뜻이 아니다.
각 `predictions[i].objects`에는 해당 미래 시점의 전체 객체 목록이 들어간다.

- `[0]`은 예측에 사용한 `/objects` 시각에서 0.5초 뒤, `[9]`는 5.0초 뒤의 객체 상태다. 목표 시각은 `objects.header.stamp + 0.5 × (i + 1)`초다.
- 모든 원소의 header는 예측에 사용한 원본 `/objects`와 같다. `stamp`에 미래 시각이나 계산 완료 시각을 넣지 않는다.
- 배열 인덱스는 예측 목표 시각을 나타내고 header는 예측에 사용한 관측의 기준 시각·좌표계를 나타낸다. 객체 상태는 원본 `/objects`와 같은 map 좌표 기준으로 표현한다.
- 외부 배열의 길이는 항상 10이다. 객체가 없으면 각 원소의 `objects` 목록은 비어 있다.


작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

새 워크스페이스의 일반·정적장애물·톨게이트 구간은 링크에서 이식한 DP/QP로 횡방향 경로를 탐색하고 속도·가속도·시간을 계획한다. `Path.lateral_reference`에는 선택된 최적화 경로의 전체 공간 형상을 전달하고 `Path.points`에는 시간 계획 구간을 전달한다. 회전교차로(S8~S10·S16)와 고주로(S11~S13)는 기존 수현 경로·미션 판단을 유지한다. GT 객체는 비교에만 사용한다.


## 터널 정지·판단·회피

S15의 local Stage는 `tunnel_stop_avoid.hpp`를 선택한다.
`process(ego, competition, objects, route, stamp)`가 기존 `/local_path` 계약의 map 경로를 반환한다.
미관측 주행 목표는25km/h다. 장애물 인식 후 분류·선행 정지를 요구하지 않고8km/h 회피 경로를 생성한다.
현재 주행 속도의 조향·횡가속도 한도, 차량 전체 경계·도로 영역·기억한 점유와 관측한 운동의 예측 점유를 검사한다.
회피 경로가 없으면 전체 목표속도0인 정지 경로를 반환하며, 실제 정지 확인 후 남아 있는 장애물 점유에 대해 회피 경로를 다시 탐색한다.
관측 소실은 장애물 점유를 해제하지 않는다.
소실 후 가까운 위치에서 새 ID로 재검출된 정적 후보는 기존 이력을 이어간다.
관측한 운동은 회피 경로의 충돌 제약에 반영한다. 차량 통과 폭 밖의 새 관측2회와 이탈 확인 시간·여유를 만족하면 정지 여부와 무관하게 차로 중심 복귀를 우선한다.
복귀 후보는 현재 위치·방향·곡률에서 연결하고 다른 모든 장애물 점유를 검사한다. 미관측만으로 점유를 해제하지 않는다.

주행 중 새 관측으로 점유 범위를 갱신하고, 기존 경로가 충돌하면 현재 위치·접선·곡률에 연결되는 경로를 재계획한다.
안전한 갱신 경로를 생성할 수 없을 때 정지하며, 기존 회피 형상은 `lateral_reference`에 유지한다.
관측 흔들림 범위는 최근 이력으로 계산하고 관측한 물체 크기와 별도로 반영한다.

S15는 Core 선택 범위이며, 장애물 판단은 차량 기준점 또는 전방 끝이 지도상의 실제 터널 벽 영역에 진입한 뒤 활성화한다.
진입 전에는 장애물을 정지 타깃으로 저장하지 않고 접근 경로를 반환한다.
터널 영역과 bbox가 겹치는 물체만 후보로 받고, 차량 통과 경로와 전체 차체 경계의 충돌 여부로 정지 대상을 선택한다.
터널 영역 계수는 `tunnel_stop_avoid.yaml`에 지도 좌표로 지정하며 Localization 모드 입력은 사용하지 않는다.

회피 경로는 장애물까지 남은 거리로 후보 길이를 미리 제한하지 않고, 현재 상태에서 생성한 전체 후보의 차체 충돌·도로·조향 한도를 검사한다.
복귀 중 갱신 경로는 현재 위치·방향·곡률에서 차로 중심으로 연결한다.
