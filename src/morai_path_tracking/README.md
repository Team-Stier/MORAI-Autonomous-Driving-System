# MORAI 경로 추종 제어기

`morai_path_tracking`은 MORAI `25.S4.MolitComp03` 환경에서 전역경로를 추종하는
ROS1 Noetic 패키지다. 기본 설정은 다음과 같다.

- 횡방향: 기본 `MPC`, 비교용 Pure Pursuit+Stanley `hybrid`
- 종방향: 곡률·차선 여유·헤딩 오차 기반 속도 계획 + MPC(비교용 PID)
- 속도 피드백: `/vehicle/competition_status`의 `velocity_x_mps`
- 운전자 설정 속도 단위: `km/h`
- 직선 목표속도: `59 km/h`
- 독립 안전제동 시작: `59.5 km/h`
- 대회 상한: `60 km/h` 초과 금지
- 제어 주기: `30 Hz`
- 횡방향 MPC 상한: `59 km/h` (절대 속도 상한은 별도 `60 km/h`)

제어 수식, IMM, 전체 파라미터, rosbag 분석 및 증상별 튜닝 근거는
[상세 한글 안내서](docs/CONTROLLER_THEORY_AND_TUNING_KO.md)에 정리돼 있다.
MPC 이관 범위, 비용함수와 시험 절차는
[MPC 통합 안내서](docs/MPC_INTEGRATION_KO.md)를 참고한다.

> **차선 안전 가정**
>
> 현재 제어기는 도색 차선을 직접 인식하지 않는다. 전역경로 중심에서 좌우
> `1.50 m`, 총 `3.00 m` 폭의 가상 회랑을 사용한다. 따라서 “바퀴 접촉 없음”은
> 이 가정과 기록된 경로에 대한 결과이며, 실제 폭이 달라지는 차선에 대한 절대
> 보증이 아니다. 실제 차선 폭 또는 차선 인식 결과를 연결하기 전에는 이 값을
> 임의로 넓히지 않는다.

## 빠른 실행

### 1. 빌드

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2
source devel/setup.bash
```

OSQP 외부 솔버 의존성:

```bash
sudo apt install ros-noetic-osqp-vendor ros-noetic-osqp
```

### 2. MORAI 포트 확인

| 데이터 | 방향 | 포트 | ROS 토픽 |
| --- | --- | ---: | --- |
| Competition Vehicle Status | MORAI → ROS | `9094` | `/vehicle/competition_status` |
| Ego Ctrl Cmd | ROS → MORAI | `9093` | `/morai_udp_bridge/control_cmd` |

확인 사항:

- Competition Vehicle Status는 MORAI에서 ROS 방향으로 설정한다.
- Ego Ctrl Cmd는 ROS에서 MORAI 방향으로 설정한다.
- 자율 UDP sender와 수동 `vehicle_control` sender를 동시에 실행하지 않는다.
- 기본 기어는 `4`(Drive)이며 `gear_command_enabled` 기본값은 `false`다.

### 3. 전체 자율 스택 실행

MORAI를 `Manual-Keyboard` 상태로 둔 채 실행한다.

```bash
roslaunch morai_bringup molit_2026_autonomous.launch
```

다음 입력이 정상인지 확인한 뒤 MORAI를 `AV-ExternalCtrl`로 바꾼다.

```bash
rostopic echo -n 1 /vehicle/competition_status
rostopic echo -n 1 /localization/ego
rostopic echo -n 1 /local_path
rostopic echo -n 1 /control/controller_status
rostopic echo -n 1 /morai_udp_bridge/control_cmd
```

정상 제어 상태는 대략 다음과 같다.

```text
active: true
state: ACTIVE
lateral_controller: mpc
```

`accel`과 `brake`가 동시에 양수이면 안 된다. 입력이 누락되거나 0.25초 이상
지연되면 자동으로 다른 속도원으로 바꾸지 않고 안전제동 상태로 전환한다.

### 4. 제어기만 실행

센서 브리지, localization, path manager 및 Competition Status receiver가 이미
실행 중일 때만 사용한다.

```bash
roslaunch morai_path_tracking path_tracking.launch
```

후보 설정을 시험할 때는 기본 YAML을 덮어쓰기보다 별도 파일을 전달한다.

```bash
roslaunch morai_path_tracking path_tracking.launch \
  config:=/tmp/molit_2026_path_tracking_candidate.yaml
```

## 제어 구조

```text
/local_path + /localization/ego + /vehicle/competition_status
  -> 입력 좌표계·시간 검증
  -> 네 바퀴 가상 회랑 여유 계산
  -> 곡률·회랑·헤딩 기반 목표속도 계산
  -> config 선택: MPC 또는 Pure Pursuit+Stanley IMM Hybrid
  -> 조향 변화율 제한 + config 선택 종방향 MPC/PID
  -> /morai_udp_bridge/control_cmd
```

### 횡방향 제어

- **MPC(기본)**: 차량 기준 경로에 비선형 bicycle rollout을 적용한다.
  외부 `OSQP`가 기본이며 자체 `projected_gradient`와 `coordinate_search`는
  명시적인 비교·비상 폴백용으로 선택할 수 있다.
  조향률 출력을 실제 제어 주기로 적분해 절대 조향각을 만든다.
- 곡률 기반 가중치 전환과 yaw-rate 직접 감쇠 파라미터는 구현되어 있지만,
  실차형 시뮬레이션에서 직선 조향 반전을 키운 회귀가 확인되어 기본값에서는
  비활성화한다. 현재는 전 구간에서 동일한 MPC 비용 가중치를 사용한다.
- **Pure Pursuit**: 속도와 곡률에 따라 lookahead distance를 바꾸고 전방 추종점을
  향하는 기하학적 조향을 계산한다.
- **Stanley**: 전륜 투영점의 횡오차와 헤딩 오차에 곡률 feedforward 및 yaw-rate
  damping을 더한다.
- **Hybrid**: IMM 두 모델 확률을 기본 가중치로 사용한다. 후보 부호 충돌, 큰
  횡오차, 헤딩 지연, 차선 여유 부족에서는 연속적인 안전 복구 가중치를 적용한다.
- Hybrid 내부의 보정 Pure Pursuit 후보에는 Stanley 횡오차 feedback 일부를 더해
  완만한 곡선에서 바깥으로 흐른 뒤 안쪽으로 급히 복귀하는 현상을 줄인다.
- 최종 조향에는 곡률별 변화율 제한을 한 번만 적용한다.

### 종방향 제어

- 전방 곡률로 횡가속도 제한 속도를 계산한다.
- 다음 곡선까지 남은 거리와 허용 감속도로 현재 허용속도를 계산한다.
- 바퀴 회랑 여유와 헤딩 오차가 작으면 최종 목표속도를 추가로 낮춘다.
- 목표속도에는 저역통과 필터와 가감속 변화율 제한을 적용한다.
- 기본 종방향 MPC는 Competition Vehicle Status의 x축 속도와 필터링한 가속도를
  상태로 사용해 속도오차, 가속도, jerk, 페달 크기·변화량을 동시에 최소화한다.
- 종방향도 외부 `osqp`가 기본이며 자체 `projected_gradient`와
  `coordinate_search`를 선택할 수 있다. 실패 시 설정에 따라 기존 PID로 폴백한다.
- `59.5 km/h`부터 비용함수와 별개인 강제 제동을 적용하고, 설정 가능한 절대
  상한 자체도 `60 km/h`를 넘지 못하게 시작 시 검사한다.

## ROS 인터페이스

| 방향 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| 입력 | `/local_path` | `nav_msgs/Path` | map 좌표계 local path |
| 입력 | `/localization/ego` | `geometry_msgs/PoseStamped` | map pose |
| 입력 | `/vehicle/competition_status` | `interfaces/CompetitionVehicleStatus` | mode, gear, x속도(m/s), yaw rate |
| 출력 | `/morai_udp_bridge/control_cmd` | `morai_msgs/CtrlCmd` | accel, brake, steering |
| 출력 | `/control/controller_status` | `morai_path_tracking/ControllerStatus` | 제어기 진단값 |
| 출력 | `/control/lookahead_point` | `geometry_msgs/PointStamped` | Pure Pursuit 추종점 |
| 출력 | `/control/stanley_projection_point` | `geometry_msgs/PointStamped` | Stanley 전륜 투영점 |
| 출력 | `/control/mpc_projection_point` | `geometry_msgs/PointStamped` | MPC 최근접 경로 투영점 |

Competition Status UDP payload의 원본 x속도는 `km/h`이며 브리지에서 3.6으로
나눠 `velocity_x_mps`로 발행한다. 제어기 설정의 목표속도는 `km/h`, 내부 계산과
상태 토픽은 `m/s`다.

## 설정

단일 기본 설정 파일:

```text
config/controllers/molit_2026_path_tracking.yaml
```

자주 확인하는 파라미터는 다음과 같다.

| 목적 | 파라미터 | 기본값 |
| --- | --- | ---: |
| 제어기 선택 | `lateral_controller` | `mpc` |
| 횡/종 MPC 솔버 | `mpc_solver` / `longitudinal_mpc_solver` | `osqp` |
| 종방향 선택 | `longitudinal_controller` | `mpc` |
| MPC 속도 상한 | `mpc_test_speed_limit_kph` | `59.0` |
| 직선 목표속도 | `target_speed_kph` | `59.0` |
| 최소 곡선속도 | `minimum_curve_speed_kph` | `15.0` |
| 독립 제동 경계 | `hard_brake_activation_speed_kph` | `59.5` |
| 절대 속도 상한 | `maximum_speed_kph` | `60.0` |
| 가상 차선 반폭 | `lane_half_width_m` | `1.50` |
| 차량 폭 | `vehicle_width_m` | `1.892` |
| 축간거리 | `wheelbase_m` | `3.0` |
| LD 기본값 | `lookahead_base_m` | `4.0` |
| LD 범위 | `lookahead_min_m` / `lookahead_max_m` | `4.0` / `16.0` |
| Stanley 횡오차 gain | `stanley_gain` | `2.0` |
| Hybrid PP 횡오차 보정 | `hybrid_pure_pursuit_cross_track_correction_gain` | `0.60` |
| 최대 횡가속도 | `maximum_lateral_acceleration_mps2` | `1.9` |
| 속도 PID | `speed_kp` / `speed_ki` / `speed_kd` | `0.18` / `0.02` / `0.0` |

공통 파라미터와 선택된 모드의 파라미터는 시작할 때 타입과 범위를 검사한다. 값을
바꾼 뒤에는 제어기 노드를 재시작해야 한다. Hybrid 전체 파라미터와 수식은
[상세 안내서의 파라미터 장](docs/CONTROLLER_THEORY_AND_TUNING_KO.md#13-전체-기본-파라미터)을
참고한다.

## 시각화와 상태 확인

```bash
roslaunch morai_bringup path_lidar.launch
```

RViz 없이 marker 노드만 실행하려면 다음을 사용한다.

```bash
roslaunch morai_bringup path_lidar.launch start_rviz:=false
```

`ControllerStatus`에서 우선 볼 값:

- 추종 상태: `active`, `state`, `cross_track_error_m`, `heading_error_rad`
- MPC: `mpc_solver_success`, `mpc_solver_time_ms`, `mpc_solver_cost`, `mpc_steering_rate_rad_per_sec`
- 종방향 MPC: `longitudinal_mpc_solver_success`, `longitudinal_mpc_solver_time_ms`,
  `longitudinal_mpc_predicted_maximum_speed_mps`, `longitudinal_mpc_hard_speed_guard_active`
- Hybrid 두 후보: `pure_pursuit_steering_angle_rad`, `stanley_steering_angle_rad`
- 실제 혼합: `hybrid_effective_*_weight`
- 곡률 속도: `raw_target_speed_mps`, `filtered_target_speed_mps`
- 최종 속도: `target_speed_mps`, `measured_velocity_x_mps`
- 회랑 안전: `wheel_minimum_clearance_m`, `lane_clearance_speed_limit_mps`
- 종방향: `longitudinal_state`, `speed_error_mps`, `accel`, `brake`

`target_speed_mps`는 곡률 planner 뒤에 회랑과 헤딩 제한까지 적용한 값이다. 안전
제한이 동작하면 `target_speed_mps < filtered_target_speed_mps`가 정상이다.

## 검증

### 자동 테스트

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_morai_path_tracking -j2
catkin_test_results build/test_results/morai_path_tracking --verbose
```

### rosbag 분석

```bash
rosrun morai_path_tracking analyze_tracking_bag.py \
  run_01=/absolute/path/to/기록.bag \
  --output-dir <분석_폴더>
```

2026-08-01 기록 중 한 회에서는 최대 `58.29 km/h`, 가상 회랑 접촉 표본 0회,
CTE RMS `0.1134 m`가 측정됐다. 다만 최소 가상 회랑 여유가 약 `0.0125 m`였고
단일 run이므로 실제 차선 안전의 보증으로 사용하면 안 된다. 원본 bag과 자동 생성
그래프는 저장소에 올리지 않으며, 결과 해석과 재검증 순서는
[상세 안내서](docs/CONTROLLER_THEORY_AND_TUNING_KO.md#15-rosbag-기록과-반복-검증)를
따른다.

## 디렉터리

```text
morai_path_tracking/
├── config/controllers/       # 운영 YAML
├── docs/                     # 한글 상세 설명서
├── include/.../common/       # 공통 시간·안전 도구
├── include/.../controllers/  # 횡·종방향 제어기 인터페이스
├── include/.../planning/     # 곡률 속도·바퀴 회랑 계획
├── launch/controllers/       # 제어기 launch
├── msg/                      # ControllerStatus
├── scripts/analysis/         # rosbag 분석기
├── src/controllers/lateral/  # Pure Pursuit, Stanley, IMM Hybrid
│   └── mpc/                  # MPC 경로·최적화·ROS 어댑터
├── src/controllers/longitudinal/
├── src/nodes/
├── src/planning/
├── test/unit/
└── test/integration/
```

## 문제 해결

- `WAITING_FOR_COMPETITION_VEHICLE_STATUS`: MORAI Status 목적지 포트 `9094`, 브리지
  실행 여부, `/vehicle/competition_status` 수신을 확인한다.
- 조금 가다 멈춤: `/local_path`, ego, Competition Status의 timestamp와
  timeout 상태를 함께 확인한다.
- 속도가 3.6배 차이남: MORAI packet 원본 `km/h`와 ROS 메시지 `m/s` 변환 위치를
  확인한다.
- 움직이지 않음: MORAI가 `AV-ExternalCtrl`인지, 기어가 Drive인지, sender가
  정확히 하나인지 확인한다.
- 조향 방향이 반대임: path/ego frame, yaw 부호, 차량 기준 좌표축을 먼저
  확인한다. gain 부호를 임의로 뒤집지 않는다.

자세한 수식·전체 파라미터·증상별 조정 순서는
[이론·튜닝 상세 안내서](docs/CONTROLLER_THEORY_AND_TUNING_KO.md)를 참고한다.

## Ego 입력과 횡속도

위치·자세는 `/localization/ego` (`geometry_msgs/PoseStamped`), 전방 속도와 yaw rate는 `/vehicle/competition_status`에서 받는다.
트랙커는 연속 ego 위치의 변화량을 원래 header.stamp 간격으로 나누고 현재 ego yaw로 차량 좌표계에 회전해 횡속도를 계산한다.

```text
vx_map = (x_now - x_previous) / dt
vy_map = (y_now - y_previous) / dt
v_y = -sin(yaw) * vx_map + cos(yaw) * vy_map
```

첫 표본에서는 횡속도를 계산하지 않고 두 번째 표본을 기다린다. 같은 stamp는 이전 추정값을 유지한다.
시각 역행·ego timeout 초과 간격·비유한 결과는 추정치를 무효화하고 기존 안전제동 경로를 사용한다.
다음 정상 간격의 두 표본에서 계산을 재개한다. IMU를 직접 구독하거나 가속도를 적분하지 않는다.
