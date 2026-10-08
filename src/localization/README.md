# Localization

50 Hz GPS·관성항법 EKF로 `/localization/ego`를 발행한다.
노드 구성은 `config/localization/`, 확장점은 `include/localization/`에 둔다.

## 입출력

| 구분 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| 구독 | `/sensors/gps/fix` | sensor_msgs/NavSatFix | WGS84→UTM 52N→map XY, 고도 변화 관측 |
| 구독 | `/sensors/imu/data` | sensor_msgs/Imu | 선가속도, 상태 예측 시각 |
| 구독 | `/vehicle/competition_status` | interfaces/CompetitionVehicleStatus | GT roll·pitch·yaw, velocity_x_mps, 3축 angular_velocity_*_radps |
| 구독 | `/velodyne_points/leveled` | sensor_msgs/PointCloud2 | TunnelWall 터널 벽 관측, FullEkf 인자·수용 판정에 포함하지 않음 |
| 구독 | `/section_annotation` | interfaces/SectionAnnotation | section 선택 |
| 발행 | `/localization/ego` | nav_msgs/Odometry | map 위치·자세, base_link 3축 선속도·각속도, IMU 추정 시각 |

section 15는 `tunnel_wall.hpp`, UNKNOWN을 포함한 나머지 section은 `full_ekf.hpp`에 매핑한다. 선택한 Core의 등록 입력에 issue가 하나도 없을 때만 Core를 호출한다.
주기 조립 기준은 최신 IMU(`reference_input: imu`)이며 발행 stamp도 그 IMU 시각이다. 점군은 조립 기준 시각이나 자세·위치의 대체 입력으로 사용하지 않는다.

## Core와 확장점

| 파일 | 내용 |
| --- | --- |
| `cores/full_ekf.hpp` | FullEkf 계산과 EkfParameters |
| `cores/tunnel_wall.hpp` | TunnelWall 계산과 TunnelWallParameters |
| `banks/state_bank.hpp` | 두 Core가 공유하는 필터 상태·공분산·초기화 여부·고도 원점·입력별 마지막 반영 stamp |
| `validators/gps_fix.hpp` | NO_FIX·0,0,0 좌표를 Missing, 위경도 범위 밖을 OutRangePositive로 기록 |
| `validators/timed_cloud.hpp` | 점군이 little-endian이 아니거나 행·데이터 경계를 넘거나 단일 FLOAT32 x·y·z·time 필드가 없으면 Missing으로 기록 |
| `policies/remove_all.hpp` | issue가 있는 입력을 모두 제거 |
| `parameters/full_ekf.hpp` | EKF 계수 로더 |
| `parameters/tunnel_wall.hpp` | 터널 계수 로더 |
| `libraries/projection.hpp` | WGS84→map 계산 |
| `config/localization/parameters/full_ekf.yaml` | 임시 process/measurement noise와 heading 오프셋 |
| `config/localization/parameters/tunnel_wall.yaml` | 터널 원점·축·벽 위치, 벽 점 선별 범위와 noise |

`process(gps, imu, competition_status)`와 `process(imu, competition_status, lidar)`는 구독한 실제 메시지를 개별 인자로 받고 `nav_msgs::Odometry`의 pose·twist payload와 child_frame_id를 완성해 반환한다. 출력 header는 Pipeline이 frame_id=map, IMU 추정 시각으로 주입한다.
공개 자세·3축 각속도는 competition_status의 GT를 사용하고 3축 선속도는 EKF로 추정한다. 누락 시 입력 대체를 하지 않는다.
frame·NaN/Inf·stamp·stale·수신 timeout·세 입력 간 시차는 공통 검사층이, GPS fix와 좌표 범위·점군 필드 배치는 Validator가 검사한다.

## 임시 EKF

상태는 map 축 기준 `[x, y, 상대 고도, vx, vy, vz]`다. GT 자세로 회전한 IMU의 수평 가속도로 예측하며 GPS x·y·상대 고도와 GT 전방 속도 관측으로 보정한다.
상대 고도는 GPS 장착 높이를 자세로 보정한 첫 고도를 원점으로 사용한다. 수직은 등속 모델로 예측한다.
공개 3축 선속도는 상태 속도를 GT 자세로 base_link에 회전해 계산한다. `child_frame_id=base_link`다.
`pose.covariance`와 `twist.covariance`는 미제공이며 기본값 0 배열로 발행하고 구독자는 사용하지 않는다.
동일 stamp의 IMU 예측·GPS 및 GT 관측을 중복 융합하지 않는다. 상태와 공분산은 StateBank에 두어 호출과 Core 전환 사이에 유지한다.
공개 quaternion은 GT roll·pitch·yaw로 계산하며 UTM 격자 heading 오프셋을 적용한다. `position.z=0`이다.
GPS는 EPSG:32652에서 `(302595.0, 4124145.0)`을 차감한다. GPS 장착 높이 1.2 m를 GT 자세로 회전해 안테나 위치에서 차량 기준점으로 XYZ를 보정한다.
입력별 stale·수신 timeout은 `config/localization/node.yaml`의 `inputs`에서 지정한다.

## 터널 벽 보정

TunnelWall은 StateBank 상태를 이어받아 FullEkf와 같은 IMU 예측을 수행한다. GPS는 사용하지 않는다.
GT 전방 속도와 차체 횡속도 0 관측으로 속도만 보정한다.
점마다 `time` 필드와 GT 각속도·추정 속도로 스캔 중 움직임을 보정해 map 축으로 옮긴다.
터널 구간 안의 좌우 벽 점 거리 합의값으로 터널 법선 방향 위치를 보정하며, 터널 축 방향 위치와 고도는 보정하지 않는다.
터널 원점·축 방향·벽 위치는 map 좌표 측량값이다. 공개 자세·각속도·`position.z`·`child_frame_id`는 FullEkf와 같다.

`launch.sh`는 이 패키지의 노드만 시작한다. 저장소 전체 bringup의 진입점은 상위 README의 `run.sh`다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
