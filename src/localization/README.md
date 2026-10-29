# Localization

50 Hz로 `/localization/ego`와 `/localization/status`를 발행한다. 주기 조립과 출력 stamp의 기준은 최신 IMU다.
노드 구성은 `config/localization/`, 확장점은 `include/localization/`에 둔다.

## 선택과 입출력

| 구간 | Core | 위치·자세 계산 |
| --- | --- | --- |
| 0–14, 16 | `full_ekf.hpp` | GPS·IMU·차량 상태를 융합한 위치 추정 |
| 15 | `tunnel_wall.hpp` | 기존 공유 상태에서 IMU·GT 속도와 수평화 LiDAR 벽 관측으로 추정 |

GPS의 Missing·Stale·ReceiveTimeout 동안에는 기존 `tunnel_wall.hpp`로 GPS에서 초기화한 공유 위치·속도 추정을 이어가고, GPS가 복구되면 해당 구간의 기본 Core로 복귀한다. 터널 출구의 섹션 전환만으로 GPS 추정을 강제하지 않는다.

| 구분 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| 구독 | `/sensors/gps/fix` | sensor_msgs/NavSatFix | WGS84→UTM 52N→map XY, 절대 고도 |
| 구독 | `/sensors/imu/data` | sensor_msgs/Imu | 일반 구간 yaw·yaw rate, 터널 가속도, 출력 기준 시각 |
| 구독 | `/vehicle/competition_status` | interfaces/CompetitionVehicleStatus | 일반 구간 전방 속도, 터널 GT 자세·속도·각속도 |
| 구독 | `/velodyne_points/leveled` | sensor_msgs/PointCloud2 | `base_link_level` 점군, 터널 벽 관측 |
| 구독 | `/section_annotation` | interfaces/SectionAnnotation | Core 선택 |
| 발행 | `/localization/ego` | nav_msgs/Odometry | map pose, base_link twist, IMU 기준 stamp |
| 발행 | `/localization/status` | interfaces/LocalizationStatus | 같은 계산 회차의 추정 모드, ego와 같은 stamp |

첫 Stage의 실제 메시지를 `process()`의 개별 인자로 받고 Odometry payload와
`child_frame_id=base_link`와 추정 모드의 tuple을 반환한다. Pipeline이 출력 header와 발행을 담당한다.
입력 검사·누락 제거는 공통 검사층과 연결한 Validator·Policy가 담당한다.
일반 구간은 GPS·IMU·Status를, 15번은 IMU·Status·LiDAR를 필수 입력으로 사용한다.
입력 시간·frame·skew 한도는 기존 `node.yaml`을 유지한다.

## MoraiReference

원본은 `Team-Stier/MOLIT2026-stier1`의 커밋
`d71cadc7f1bb040ae026f95d0a19292d16ca8978`에 있는
`morai_localization/estimation/position_estimator.hpp`와 `localization_fusion_node.cpp`다.
새 Core는 위치 보정 계산과 정상 입력에서의 yaw·속도 계산을 이식한다.
원본 노드의 ROS callback·구독·발행·입력 대체는 이식하지 않는다.

GPS는 EPSG:32652에서 `(302595.0, 4124145.0)`을 차감한다.
IMU quaternion을 정규화한 것과 같은 yaw를 구하고 `-1.34715°`의 격자 보정을 적용한다.
roll·pitch는 0이며 전방 속도는 Status의 `velocity_x_mps`, yaw rate는 IMU의
`angular_velocity.z`다. 다른 twist 성분은 0이다. IMU 가속도는 위치 예측에 사용하지 않는다.
일반 구간에서 GT heading으로 대체하지 않으며, quaternion·yaw rate 검사는
`morai_reference_inputs.hpp`가 담당한다.

새 GPS마다 이전 보정 위치에서 이전·현재 전방 속도와 yaw의 map 속도를 사다리꼴 적분한다.
원본과 같이 `alpha = 1 - exp(-dt / 0.30)`으로 GPS 잔차를 반영한다.
정지 부근(`|speed| < 0.1 m/s`), GPS 간격이 `(0, 0.25]초` 밖인 경우,
예측 잔차가 0.75 m를 넘는 경우에는 해당 GPS 위치에서 위치 이력을 시작한다.
동일 GPS stamp는 반복 보정하지 않는다. 시간상 입력 결합은 현재 Pipeline 규약을 따른다.

현재 출력 시각 계약에 맞춰 보정된 GPS 시각 위치를 현재 속도·yaw로 IMU 시각까지
선형 예측한다. 따라서 GPS 수신 때만 발행하는 원본 노드와 발행 시점·중간 위치가 다르다.
XY는 원본처럼 장착 높이로 보정하지 않는다. Z는 현재 지도·터널 상태 계약에 맞춰
`GPS 절대 고도 - gps_height_m(1.2 m)`를 사용하며 원본의 고정 Z=0과 다르다.
계수는 `parameters/morai_reference.yaml`에 둔다.

## 터널 연결

MoraiReference는 기존 `EkfState` Bank의 map 위치·속도·IMU/GPS/Status 처리 stamp와
초기화 상태를 갱신한다. 터널 Core와 Bank의 구현·설정은 변경하지 않는다.
최초 초기화와 터널에서 일반 구간으로 복귀할 때 Bank 공분산은 기본 단위행렬로 초기화한다.
일반 구간 계산은 EKF가 아니며 이 공분산은 위치 추정 정확도의 측정값이 아니다.
공개 pose/twist covariance는 기존과 같이 기본 0 배열이며 구독자는 사용하지 않는다.

15번에서는 기존 GT roll·pitch·yaw와 3축 각속도, 추정 3축 선속도를 사용한다.
벽 관측에는 수평화 점군을 사용하고 점별 시간 보상을 유지한다.
터널을 나온 뒤에는 이전 일반 구간의 위치 보정 이력을 재사용하지 않고 현재 GPS로 시작한다.
일반 구간과 터널 사이에서 자세 표현과 위치 보정 방식이 바뀐다.

## 기존 FullEkf

`full_ekf.hpp`를 직접 선택하면 GPS·IMU·Status로 map 축의
`[x, y, 절대 고도, vx, vy, vz]`를 추정한다. GT 자세로 회전한 수평 가속도로 예측하고
GPS 위치와 GT 전방 속도로 보정한다. GPS 장착 높이는 GT 자세로 회전해 XYZ에서 차감한다.
공개 자세·각속도는 GT, 선속도는 추정 상태를 base_link로 회전한 값이다.
동일 stamp의 관측을 중복 융합하지 않는다.

`launch.sh`는 이 패키지의 노드만 시작한다. 전체 bringup은 상위 README를 따른다.
작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

## 추정 모드

`full_ekf.hpp`는 `GPS_FUSED`, `tunnel_wall.hpp`는 `IMU_LIDAR`를 반환한다.
공통 실행기는 두 출력에 같은 추정 기준 stamp와 `map` frame을 주입한다.
Core 계산이 거부되거나 실패한 회차에는 새 위치·모드를 발행하지 않는다.
`IMU_LIDAR`는 추정 방식이며 물리적인 터널 위치를 뜻하지 않는다.
