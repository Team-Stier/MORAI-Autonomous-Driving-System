# control

새 워크스페이스는 `integrated_mpc.hpp` Core로 경로 추종 제어 명령을 만든다. 회전교차로 접근·진입·탈출(S8~S10)과 도착 전 회전교차로(S16)는 기존 `morai_mpc.hpp`와 공간 기준 경로 추종을 유지한다.
노드 구성은 `config/control/`, 확장점은 `include/control/`에 둔다.

## 입출력

| 구분 | 토픽 | 타입 | 계약 |
| --- | --- | --- | --- |
| 구독 | `/local_path/velocity_injected` | `interfaces/Path` | base_link 경로·주입된 목표속도·가속도 |
| 구독 | `/localization/ego` | `nav_msgs/Odometry` | map pose, base_link 횡속도·yaw rate; 공분산 미사용 |
| 구독 | `/vehicle/competition_status` | `interfaces/CompetitionVehicleStatus` | base_link 전방 속도 `velocity_x_mps` |
| 구독 | `/localization/status` | interfaces/LocalizationStatus | 실제 추정 모드, map frame |
| 구독 | `/section_annotation` | `interfaces/SectionAnnotation` | section 선택 |
| 발행 | `/control_cmd` | `morai_msgs/CtrlCmd` | 페달 [0,1], 최대 조향각으로 나눈 조향 [−1,1], longlCmdType=1 |

`integrated_mpc.hpp`의 `process(path, ego, competition, localization_status, stamp)`는 실제 입력 메시지를 개별 인자로 받고 완성한
CtrlCmd를 반환한다. Core는 ROS 통신·입력 검사·fallback·발행을 수행하지 않는다.
계산 상태는 Core가 소유하며 입력 오류 시 자동 reset하지 않는다.

횡방향은 `Path.lateral_reference`의 전체 공간 기준 경로를 사용한다. 비어 있으면 `points`의 형상을 사용한다. 종방향은 `points`의 속도·가속도·시간을 사용한다.

시간 경로는 경로 stamp와 `time_from_start`를 기준으로 미래 속도 v(t)·가속도 a(t)를 추종한다.
모든 시간이 0인 공간 경로는 차량 기준점에 가장 가까운 경로점의 `velocity`를 현재 목표로 사용하고,
누적 거리와 point 속도로 예측 이동 위치의 목표속도 v(s)를 보간한다.
앞쪽 감속·정지 속도를 같은 예측 구간에 반영하며, 최초 속도 0 지점을 넘는
예측 이동 거리에는 `stop_position_weight` 비용을 적용한다.
전진 목표속도만 지원한다. 경로 형상은 재계획하지 않는다. 경로와 기본 속도 프로파일은
Path Planner가 계획하고, Velocity Planner가 신호등 감속·정지를 반영한다.
최종 속도 제한과 Bridge 전달은 Speed Filter가 수행한다.
MPC의 모델 속도 제약과 페달·조향 제약은 Core 계산에 적용한다.

## 실행과 설정

`launch.sh`는 ROS·워크스페이스 환경을 준비하고 이 노드만 실행한다.
전체 bringup은 루트 `run.sh`를 사용한다. 30 Hz 주기로 조립한 입력으로 계산·발행한다.
통합 Core의 파라미터는 `config/control/parameters/integrated_mpc.yaml`에서 설정하고 노드를 재시작해 반영한다.
횡방향은 qpOASES 기반 MPC, 종방향은 OSQP 기반 MPC를 사용한다.
통합 Core의 계산 간격은 조립 기준 stamp 차이이며 첫 호출은 `lateral.ctrl_period`를 사용하고,
적분 간격은 `minimum_control_dt_sec`~`maximum_control_dt_sec`로 제한한다.
등록 후보 `path_tracking.hpp`는 `parameters/path_tracking.yaml`로 MPC·Hybrid·Stanley·Pure Pursuit 및 PID를 설정한다.
`autoware_universe.hpp`도 등록 후보이며 `parameters/autoware_universe.yaml`을 사용한다.

| Core | 입력 |
| --- | --- |
| `integrated_mpc.hpp` | path·ego·competition·localization_status·stamp |
| `morai_mpc.hpp` | path·ego·competition·localization_status |
| `path_tracking.hpp` | path·ego·competition |
| `autoware_universe.hpp` | path·ego·competition·imu |
| `safe_stop.hpp` | 없음 |

공통 검사층이 frame·NaN/Inf·stamp·stale·수신 timeout·입력 시차를, 통합 Core의 Validator `integrated_mpc_inputs.hpp`가 경로점 수·형상·quaternion·point frame·목표속도 범위와 ego `child_frame_id`를 검사한다. 시간 경로는 첫 시각 0과 이후 시각의 단조 증가를 요구하며, 속도·가속도가 모두 0인 시간 경로의 정지 대기를 허용한다.
목표속도 범위는 `config/control/node.yaml`의 path `range`다.
Policy `remove_all.hpp`는 issue가 있는 입력을 제거해 Missing으로 만든다. 필수 입력이 Missing이면 Core 호출과 발행을 거부한다.
`safe_stop.hpp`는 accel=0·steering=0·설정 brake(`config/control/parameters/safe_stop.yaml`)의 CtrlCmd를 반환한다.
CtrlCmd에는 Header가 없으므로 헤더를 주입하지 않는다. 출력 검사와 payload 변경은 수행하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
`docs/`와 bag 분석 도구는 기존 데모의 설계·측정 기록을 보존한다.

기존 횡방향 MPC와 차량 보정값은 유지한다. 통합 MPC는 장시간 선택되지 않았다가 다시 사용될 때 현재 실측 조향으로 예측·필터·입력 이력을 초기화한다.

## 추정 모드에 따른 속도 상한

선택된 `integrated_mpc.hpp`와 `morai_mpc.hpp`는 `IMU_LIDAR` 동안 종방향 목표와
미래 속도 참조를 `tunnel_speed_mps`(25 km/h) 이하로 제한한다. 더 낮은 속도와 정지는 유지한다.
시간 참조가 터널 속도 상한 이상이면 속도를 상한 이하로 낮추고 양의 가속도는 0으로 제한한다.
`GPS_FUSED`에서는 기존 속도 계획을 사용하며 GPS 원본을 다시 판정하지 않는다.
상태의 UNKNOWN·frame·stamp·만료·수신 timeout은 공통 검사와 기존 Policy로 처리한다.
필수 상태 입력이 없으면 해당 Core 호출·명령 발행을 거부한다.

`integrated_mpc.hpp`는 기존 동적 구간 또는 `IMU_LIDAR` 모드에서 경로의 모든 목표속도가0이면 accel=0·brake=1을 발행한다. 양의 목표속도를 포함한 감속·회피 경로는 MPC 제어를 사용한다.
