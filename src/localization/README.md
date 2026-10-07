# Localization

`node_pipeline`의 공통 main·실행기를 사용하는 실행 패키지다. 50 Hz GPS·관성항법 EKF와 터널 벽면 관측으로 `/localization/ego`를 발행한다.

## 입출력

| 구분 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| 구독 | `/sensors/gps/fix` | sensor_msgs/NavSatFix | WGS84→UTM 52N→map XY, 고도 변화 관측 |
| 구독 | `/sensors/imu/data` | sensor_msgs/Imu | 선가속도, 상태 예측 시각 |
| 구독 | `/vehicle/competition_status` | interfaces/CompetitionVehicleStatus | GT 자세·전방 속도·3축 각속도 |
| 구독 | `/velodyne_points/leveled` | sensor_msgs/PointCloud2 | 터널 벽면 관측 |
| 구독 | `/section_annotation` | interfaces/SectionAnnotation | Selector의 section 메타데이터 |
| 발행 | `/localization/ego` | nav_msgs/Odometry | map 위치·자세, base_link 3축 선속도·각속도 |

섹션 15는 `tunnel_wall.hpp`, UNKNOWN과 나머지 섹션은 `full_ekf.hpp`를 선택한다. 하나의 Stage에서 선택한 Core의 입력을 검사하고 보정한다. 모든 process 인자는 필수이며 모든 issue가 없어야 호출한다.
큐·조립·검사 반복·선택·필수 입력 확인·Core 호출·헤더 주입·발행은 공통 라이브러리가 담당한다. ROS 구독·발행 큐는 1이다.
주기 조립은 전체 입력 큐의 최신 stamp를 기준으로 한다. 추정 계산과 최종 Odometry stamp는 조립된 IMU의 시각을 사용한다. TunnelWall은 스캔·점별 시각으로 IMU 시각에 정렬하므로 LiDAR의 skew 검사는 적용하지 않는다. LiDAR의 stale·수신 timeout은 각각 0.3초다.

## Core 계약

- FullEkf: `process(gps, imu, competition_status) → nav_msgs::Odometry`.
- TunnelWall: `process(imu, competition_status, lidar) → nav_msgs::Odometry`.
- 두 Core는 해당 패키지의 Bank Core인 StateBank를 참조한다. 공통 실행기가 Bank와 지속 Core 객체를 소유한다. TunnelWall은 FullEkf가 초기화한 위치·속도·시각을 이어받으며 선택 전환 때 재생성하거나 reset하지 않는다.
- Core는 pose·twist payload와 `child_frame_id=base_link`를 완성한다. 입력 검사·누락 감지·대체·fallback·발행 헤더 설정·발행은 구현하지 않는다.
- 새 출력의 시간 메타데이터는 비워 둔다. 공통 ROS 경계가 마지막 Odometry의 header에 `frame_id=map`과 조립된 IMU stamp만 주입하고 한 번 발행한다. 출력은 검사하지 않는다.

Core별 domain Validator는 GPS fix·0 좌표·UTM 좌표 범위 또는 LiDAR의 little-endian FLOAT32 x·y·z·time 필드와 버퍼 경계를 검사한다. NaN/Inf·frame·stamp·stale·수신 timeout·skew는 공통 검사다. 전용 RemoveInvalid Policy는 issue가 있는 입력을 Missing으로 만든다. 이미 Missing인 입력은 다시 보정하지 않는다.

## 임시 EKF

상태는 map 축 기준 `[x, y, 상대 고도, vx, vy, vz]`다. GT 자세로 회전한 IMU의 수평 가속도로 예측하며 GPS x·y·상대 고도와 GT 전방 속도 관측으로 보정한다. 수직은 등속 모델이다.
GPS는 EPSG:32652에서 `(302595.0, 4124145.0)`을 차감한다. GPS 장착 높이를 GT 자세로 회전해 차량 기준점으로 보정하고 첫 고도를 상대 고도의 원점으로 사용한다.
공개 자세·각속도는 차량 상태의 GT를 사용하고 선속도는 EKF 상태를 base_link로 회전해 계산한다. quaternion에는 UTM 격자 heading 오프셋을 적용하며 `position.z=0`이다.
`pose.covariance`·`twist.covariance`는 기본값 0 배열이며 구독자는 사용하지 않는다. 동일 stamp의 IMU 예측·GPS·GT 관측을 중복 융합하지 않는다. 계산 상태·공분산은 호출 사이에 유지한다.

## 등록과 설정

`node.yaml`은 입력·Stage·출력·Bank를 선언한다. `registrations/*.yaml`은 Core의 순서 있는 입출력·Validator·Policy·전용 Params Loader를 연결한다. 모든 노드가 공통 main·실행기를 사용하고 빌드는 타입 연결만 생성한다.
Core별 Params 구조체와 계산은 `include/localization/cores/`, readonly Loader는 `parameters/`, domain 검사는 `validators/`, 입력 보정은 `policies/`에 둔다.
`config/localization.yaml`은 실행·입력 한도·선택 매핑과 `ekf`·`tunnel_wall` 계수를 제공한다. Loader가 자기 namespace의 설정을 읽어 생성자에 주입한다.
launch는 이 패키지의 노드만 시작한다. 전체 bringup 진입점은 상위 README의 `run.sh`다.
