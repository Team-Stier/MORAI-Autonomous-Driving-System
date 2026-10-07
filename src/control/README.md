# control

`node_pipeline` 라이브러리에 등록하는 `autoware_universe.hpp` Core의 제어 계약이다.
횡방향은 Autoware Universe의 조향 지연을 포함한 kinematics MPC·OSQP,
종방향은 Universe PID를 사용한다. 원본 커밋과 이식 변경은 [NOTICE](NOTICE)에 기록한다.
UNKNOWN을 포함한 모든 section(0~16)에 같은 지속 Core를 등록한다.

## 입출력

| 구분 | 토픽 | 타입 | 계약 |
| --- | --- | --- | --- |
| 구독 | `/local_path/velocity_injected` | `interfaces/Path` | base_link 경로·주입된 목표속도·가속도 |
| 구독 | `/localization/ego` | `nav_msgs/Odometry` | map pose와 경사 보상용 자세; 공분산 미사용 |
| 구독 | `/vehicle/competition_status` | `interfaces/CompetitionVehicleStatus` | base_link 전방 속도 `velocity_x_mps`, 실제 조향각 `steering_angle_rad` |
| 구독 | `/sensors/imu/data` | `sensor_msgs/Imu` | imu_link의 3축 선가속도; 공분산 미사용 |
| 구독 | `/section_annotation` | `interfaces/SectionAnnotation` | section 선택 |
| 발행 | `/control_cmd` | `morai_msgs/CtrlCmd` | 페달 [0,1], 최대 조향각으로 나눈 조향 [−1,1], longlCmdType=1 |

`process(path, ego, competition, imu)`는 실제 입력 메시지를 개별 인자로 받고 완성한
CtrlCmd를 반환한다. Core는 ROS 통신·입력 검사·fallback·발행을 수행하지 않는다.
계산 상태는 Core가 소유하며 입력 오류 시 자동 reset하지 않는다.

`base_link` 경로는 입력 ego pose로 Core 안에서 `map`으로 변환한다.
Universe PID의 목표점 계산으로 경로의 목표속도·가속도를 추종한다.
전진 목표속도만 지원한다. 경로 형상은 재계획하지 않는다. 목표속도 계획은
Velocity Planner, 최종 속도 제한과 Bridge 전달은 Speed Filter가 수행한다.
현재 두 Velocity Planner Core의 목표 가속도는 0 m/s²이며 속도 오차에 대한 PID 계산을 수행한다.

IMU 원본 메시지는 Core 안에서 사용한다. 현재 IMU 장착 RPY가 0이므로
`linear_acceleration.x`를 전방 가속도로 사용한다. 3축 벡터의 크기를 종방향 값으로 사용하지 않는다.
MPC의 조향 수렴 상태는 다음 호출의 종방향 PID에 전달한다.
Competition의 실측 조향각은 좌회전 음수이므로 Core에서 부호를 반전해 Universe의 좌회전 양수 규약으로 변환한다.

Core는 계산한 조향각을 차량 최대 조향각으로 나누고 [−1,1]로 제한한다.
양의 명령 가속도는 가속 페달, 음의 명령 가속도는 제동 페달로 변환한다.
변환 계수와 페달 상한은 Core 설정으로 주입한다.

## 실행과 설정

`launch.sh`는 ROS·워크스페이스 환경을 준비하고 이 노드만 실행한다.
전체 bringup은 루트 `run.sh`를 사용한다. 경로 콜백마다 계산하며 ROS 큐는 1이다.
`node.yaml`은 입력·Stage·선택 매핑·최종 발행을 선언하고 `registrations/*.yaml`은 Core별 입력·출력·Validator·로더를 연결한다.
Universe와 PathTracking Core를 등록하며 현재 모든 section은 Universe를 선택한다.
`config/control.yaml`의 `universe`·`tracking` 계산 파라미터는 `parameters/`의 읽기 전용 로더가 생성자에 주입한다. 기존 실행 설정과 `stage1` 선택 매핑도 공통 실행기가 읽는다.
파라미터를 바꾸면 노드를 재시작한다.
빌드에는 Eigen·fmt·tf2_geometry_msgs가 필요하며 CMake가 고정된 OSQP 0.6.3 소스를 받아 빌드한다.
계산 라이브러리는 `include/control/universe/`와 `src/universe/`에 둔다.
공통 실행기는 지속 Core 객체를 소유하고 선택된 Core의 모든 선언 입력을 검사한 뒤 native `process()`를 호출한다.
경로 고유 검사는 `validators/path_contract.hpp`, 공통 값·시간 검사는 Pipeline이 담당한다.
Core의 새 CtrlCmd는 시각 메타데이터 없이 전달하며, 최종 발행 시 CtrlCmd에는 주입할 Header가 없다.
MPC의 제어 주기는 `nominal_dt_sec`다. PID의 계산 간격은 경로 stamp 차이이며
첫 호출은 `nominal_dt_sec`를 사용하고 적분 간격은 `minimum_control_dt_sec`~`maximum_control_dt_sec`로 제한한다.

Pipeline은 frame·NaN/Inf·경로점 수·형상·목표속도 하한과 stamp·stale·수신 timeout·입력 시차를 검사한다.
목표속도는 음수만 거부하고 0을 허용하며 상한 검사는 두지 않는다.
IMU의 stale 최대 나이와 수신 timeout은 각각 0.06초로 설정한다.
허용되지 않은 입력 issue가 남거나 네 필수 입력 중 하나라도 없으면 Universe Core 호출과 발행을 생략한다.
Health 검사 한도를 초과하면 이번 처리와 발행을 종료한다.
계산 중 예외가 발생하면 재시도 없이 노드를 종료한다.
출력 검사와 payload 변경은 수행하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
`docs/`와 bag 분석 도구는 기존 데모의 설계·측정 기록을 보존한다.
