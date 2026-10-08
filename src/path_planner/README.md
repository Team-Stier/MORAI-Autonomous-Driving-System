# path_planner

Checkpoint → GlobalPath → Prediction → LocalPath의 네 Stage 파이프라인이다.
노드 구성은 `config/path_planner/`, 확장점은 `include/path_planner/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `path_planner_node` |
| 구독 | `/localization/ego`, `/vehicle/competition_status`, `/objects`, `/tf`, `/section_annotation` |
| 발행 | `/local_path` (`map`, 입력 조립 기준 stamp; 각 point pose header 포함) |
| 실행 | 20 Hz 주기 |

`launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

## Stage 입출력

| Stage | 입력 | 출력 | 등록 Core |
| --- | --- | --- | --- |
| `checkpoint` | ego·competition·objects·tf | ego·competition·objects·tf(패스스루), checkpoints(`path_planner::Checkpoints`) | `checkpoint_00.hpp`~`checkpoint_16.hpp` |
| `global` | ego·competition·objects·tf·checkpoints | route(`path_planner::Route`), ego(패스스루), objects(패스스루) | `global_path.hpp` |
| `prediction` | route·ego·objects | predictions(`path_planner::PredictedObjects`), route(패스스루), ego(패스스루) | `prediction_pass_through.hpp` |
| `local` | predictions·route·ego | path(`interfaces/Path`) | `local_path.hpp` |

`path_planner::Checkpoints`는 `std::vector<Checkpoint>`, `path_planner::Route`는 `lanelet::Optional<lanelet::routing::LaneletPath>`, `path_planner::PredictedObjects`는 `std::array<interfaces::Objects, 10>`이며 `libraries/types.hpp`에 정의한다.
패스스루 출력은 입력의 시간 메타데이터를 상속하고, checkpoints·route·predictions는 새 계산값이므로 시간 메타데이터가 없다.

## 체크포인트 Stage

섹션 k(0~16)는 `checkpoint_k.hpp`에 매핑한다. `checkpoint_k.hpp`는 구독 입력 네 개를 그대로 반환하고, `checkpoints.csv`에서 아직 경유하지 않은 체크포인트 k+1~16을 순서대로 담은 배열을 함께 반환한다.
섹션 0은 체크포인트 1~16 전체, 섹션 16은 빈 배열이다. `/section_annotation`이 바뀌면 Core가 교체되어 global Stage에 전달되는 체크포인트가 바뀐다.
공통 계산은 `libraries/checkpoint_route.hpp`에 두고, 각 Core는 자기 섹션 번호만 갖는다. 체크포인트 파일은 `config/path_planner/parameters/checkpoint_k.yaml`의 `checkpoints_file`로 지정한다.

## GlobalPath

Bank `shared_map.hpp`는 `config/path_planner/parameters/shared_map.yaml`의 `map_file` 지도를 노드당 한 번 읽어 GlobalPath와 LocalPath에 같은 인스턴스로 주입한다. 로더는 `checkpoints_file`의 모든 lanelet_id가 지도에 있는지 시작 시 확인한다.

`global_path.hpp`는 ego로 현재 lanelet을 찾고, 전달받은 체크포인트 중 마지막을 목적지, 나머지를 경유지로 `shortestPathVia`를 계산한다. 체크포인트가 비어 있으면 현재 lanelet만 경로로 반환한다.
현재 lanelet이나 체크포인트 lanelet_id 목록이 바뀔 때만 경로를 다시 계산한다. 경로 중심선은 `/visualizer/global_path`로 발행한다.

## Prediction·LocalPath

`prediction_pass_through.hpp`의 `process(objects, route, ego)`는 관측 Objects를 10개 예측 시점에 그대로 복사하고 route·ego를 넘긴다. 위치·속도·가속도를 외삽하지 않는다.
`local_path.hpp`의 `process(ego, route, predictions)`는 route 중심선을 ego 투영점부터 `local_path_length_m`만큼 잘라 경로를 만든다. 지도는 `shared_map.hpp`에서 받고, 경로 길이는 `config/path_planner/parameters/local_path.yaml`에서 읽는다.
Validator `route_present.hpp`는 경로 탐색 결과가 없는 route를 Missing으로 기록하며, 이때 local Stage는 Core를 호출하지 않는다.

모든 Core는 Policy `remove_invalid.hpp`로 값·stamp·frame 오류 입력을 제거하며, 구독 입력에서 이어진 값의 Stale·ReceiveTimeout·Skew는 허용 issue다.
`cores/prediction_core.hpp`는 등속 선회 예측 Core로 보존하며 등록하지 않았다.

| 추가 등록 Core | Stage | 계산 |
| --- | --- | --- |
| `core_path_planner.hpp` | `local` | route 중심선·좌우 경계 corridor, ego, ego pose로 map에 옮긴 예측으로 `stier::PathGenerationCore` 경로를 계산해 map pose 경로로 반환. 결과가 없으면 빈 경로. 정지선 미사용. 파라미터 `config/path_planner/parameters/core_path_planner.yaml` |
| `core_prediction.hpp` | `prediction` | 관측 `base_link` Objects를 `stier::PredictionCore` 등속 모델로 0.5~5.0초 전파. `velocity_available=false`는 정지 |
| `ego_compensated_prediction.hpp` | `prediction` | ego 자세·각속도로 관측 시각 자세를 복원해 map 평면 등속 선회 예측. 위치·bbox·속도·가속도는 관측 `base_link` 기준 |

## 예측 배열 계약

Prediction Core의 반환과 LocalPath Core의 입력은 **Objects 메시지를 원소로 갖는 길이 10의 외부 배열**이다. `Objects.objects` 안의 Object 목록을 10개로 제한하는 뜻이 아니다.
각 `predictions[i].objects`에는 해당 미래 시점의 전체 객체 목록이 들어간다.

- `[0]`은 예측에 사용한 `/objects` 시각에서 0.5초 뒤, `[9]`는 5.0초 뒤의 객체 상태다. 목표 시각은 `objects.header.stamp + 0.5 × (i + 1)`초다.
- 모든 원소의 header는 예측에 사용한 원본 `/objects`와 같다. `stamp`에 미래 시각이나 계산 완료 시각을 넣지 않는다.
- 배열 인덱스는 예측 목표 시각을 나타내고 header는 예측에 사용한 관측의 기준 시각·좌표계를 나타낸다. 객체 상태는 원본 `/objects`의 `base_link` 좌표 기준으로 표현한다.
- 외부 배열의 길이는 항상 10이다. 객체가 없으면 각 원소의 `objects` 목록은 비어 있다.


작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
