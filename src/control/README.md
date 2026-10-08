# control

`path_tracking.hpp` Core로 경로 추종 제어 명령을 만든다. 입력에 문제가 있으면 `safe_stop.hpp` Core로 교체해 안전 정지 명령을 발행한다.
노드 구성은 `config/control/`, 확장점은 `include/control/`에 둔다.

## 입출력

| 구분 | 토픽 | 타입 | 계약 |
| --- | --- | --- | --- |
| 구독 | `/local_path/velocity_injected` | `interfaces/Path` | base_link 경로·주입된 목표속도·가속도 |
| 구독 | `/localization/ego` | `nav_msgs/Odometry` | map pose, base_link 횡속도·yaw rate; 공분산 미사용 |
| 구독 | `/vehicle/competition_status` | `interfaces/CompetitionVehicleStatus` | base_link 전방 속도 `velocity_x_mps` |
| 구독 | `/section_annotation` | `interfaces/SectionAnnotation` | section 선택 |
| 발행 | `/control_cmd` | `morai_msgs/CtrlCmd` | 페달 [0,1], 최대 조향각으로 나눈 조향 [−1,1], longlCmdType=1 |

`process(path, ego, competition)`는 실제 입력 메시지를 개별 인자로 받고 완성한
CtrlCmd를 반환한다. Core는 ROS 통신·입력 검사·fallback·발행을 수행하지 않는다.
계산 상태는 Core가 소유하며 입력 오류 시 자동 reset하지 않는다.

차량 기준점에 가장 가까운 경로점의 `velocity`를 종방향 목표속도로 사용한다.
전진 목표속도만 지원한다. 경로 형상은 재계획하지 않는다. 목표속도 계획은
Velocity Planner, 최종 속도 제한과 Bridge 전달은 Speed Filter가 수행한다.
기존 MPC의 모델 속도 제약과 페달·조향 제약은 계산 라이브러리에 유지한다.

## 실행과 설정

`launch.sh`는 ROS·워크스페이스 환경을 준비하고 이 노드만 실행한다.
전체 bringup은 루트 `run.sh`를 사용한다. `/local_path/velocity_injected` 입력마다 계산한다.
계산 파라미터는 `config/control/parameters/path_tracking.yaml`에서 설정하고 노드를 재시작해 반영한다.
기본 횡·종방향은 MPC이며 횡방향은 Hybrid·Stanley·Pure Pursuit, 종방향은 PID로 선택할 수 있다.
계산 라이브러리는 `include/control/libraries/tracking/`과 `src/libraries/`에 둔다.
계산 간격은 경로 stamp 차이이며 첫 호출은 `nominal_dt_sec`를 사용하고,
적분 간격은 `minimum_control_dt_sec`~`maximum_control_dt_sec`로 제한한다.

| Core | 입력 | 선택 |
| --- | --- | --- |
| `path_tracking.hpp` | path·ego·competition | UNKNOWN=0과 섹션 1~16 |
| `safe_stop.hpp` | 없음 | path·ego·competition 중 하나가 Missing이면 health 매핑으로 교체 |

공통 검사층이 frame·NaN/Inf·stamp·stale·수신 timeout·입력 시차를, Validator `tracking_inputs.hpp`가 경로점 수·형상·quaternion·point frame·목표속도 범위와 ego `child_frame_id`를 검사한다.
목표속도 범위는 `config/control/node.yaml`의 path `range`다.
Policy `remove_all.hpp`는 issue가 있는 입력을 제거해 Missing으로 만들고, 이어서 `safe_stop.hpp`가 선택된다.
`safe_stop.hpp`는 accel=0·steering=0·설정 brake(`config/control/parameters/safe_stop.yaml`)의 CtrlCmd를 반환한다.
CtrlCmd에는 Header가 없으므로 헤더를 주입하지 않는다. 출력 검사와 payload 변경은 수행하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
`docs/`와 bag 분석 도구는 기존 데모의 설계·측정 기록을 보존한다.
