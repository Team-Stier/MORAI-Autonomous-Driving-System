# MPC 제어기 통합 및 시험 안내

기존 `morai_path_tracking` 데모의 설계·측정 기록이다. 현재 실행 계약은 [control README](../README.md)를 따른다.
## 목적과 현재 상태

홈 디렉터리의 `morai_mpc_sandbox`에서 ROS 주행에 필요한 MPC 핵심만
`morai_path_tracking`에 이관했다. 샌드박스의 독립 테스트는 다음과 같이 통과했다.

- MPC 핵심: 173 checks, 0 failures
- readiness: 2942 checks, 0 failures
- validation: 43 checks, 0 failures

이후 횡방향 솔버 선택과 종방향 MPC를 추가했다. 기본 목표는 `59 km/h`, 독립
강제 제동 경계는 `59.5 km/h`, 설정 가능한 절대 상한은 `60 km/h`다. 고속 설정은
자동 시험을 통과한 초기값이며 실제 트랙 성능은 rosbag으로 별도 확인해야 한다.

## 모드 선택

설정 파일은 `config/controllers/molit_2026_path_tracking.yaml`이다.

```yaml
# MPC (기본)
lateral_controller: mpc

# 기존 Stanley + Pure Pursuit IMM Hybrid
lateral_controller: hybrid

# 종방향 MPC(기본) 또는 기존 PID
longitudinal_controller: mpc
# longitudinal_controller: pid

# 두 MPC에서 각각 선택 가능
mpc_solver: projected_gradient
longitudinal_mpc_solver: projected_gradient
# 자체 대안: projected_gradient 또는 coordinate_search
```

변경 뒤 제어기 노드를 재시작한다. `pure_pursuit`, `stanley` 단독 값은 기존 회귀
시험을 위해 코드에서만 유지하며 운영 선택은 `mpc`와 `hybrid` 두 가지다.

## 이관한 파일과 제외한 파일

연산부와 ROS 연결 계층을 기능별 하위 폴더에 배치했다.

```text
include/morai_path_tracking/controllers/lateral/mpc/
├── types.hpp
├── path.hpp
├── optimizer.hpp
├── mpc_controller.hpp
└── mpc_lateral_controller.hpp   # ROS 출력 계약 어댑터

src/controllers/lateral/mpc/
├── path.cpp
├── optimizer.cpp
├── mpc_controller.cpp
└── mpc_lateral_controller.cpp
```

독립 시뮬레이터 앱, validation runner, Python 도구, `build*`, `results`, checkpoint,
CSV와 테스트 산출물은 ROS 런타임에 필요하지 않아 가져오지 않았다. MPC rollout이
자체적으로 bicycle 식을 계산하므로 독립 시뮬레이터용 `vehicle_model`도 제외했다.

## 좌표계와 출력 계약

- `/local_path`와 odometry는 `map` 좌표계다.
- 노드는 경로점을 차량 원점 기준 `base_link`로 변환한다.
- 차량 좌표는 x 전방, y 좌측, yaw와 조향은 좌회전 양수다.
- MPC 차량 상태는 매 주기 `(x, y, yaw) = (0, 0, 0)`으로 두고 변환된 경로를 쓴다.
- 속도는 `/vehicle/competition_status.velocity_x_mps`를 사용한다.
- MPC 연산부 출력은 조향률 `u = d(delta)/dt` [rad/s]다.
- MORAI 명령은 절대 조향각이므로 어댑터가 실제 제어 주기 `dt_actual`로 적분한다.

```text
delta_cmd = clamp(delta_previous + u_mpc * dt_actual,
                  -delta_max, +delta_max)
```

기본 한계는 조향각 ±40 deg, 조향률 ±60 deg/s다. 입력·경로·solver 결과가
유효하지 않으면 기존 노드 정책대로 정상 명령을 내지 않고 안전제동한다.

## MPC 비용함수

각 예측 단계에서 횡오차, 헤딩 오차, 기준 곡률 조향과의 차이, 조향률, 이전
조향률과의 변화량을 최소화한다.

```text
J = sum(
      w_y * e_y^2
    + w_psi * e_psi^2
    + w_delta * (delta - atan(L * kappa_ref))^2
    + w_u * u^2
    + w_du * (u - u_previous)^2
    )
    + w_y_terminal * e_y_terminal^2
    + w_psi_terminal * e_psi_terminal^2
```

기본 최적화기는 ROS Noetic의 외부 `OSQP`다. 비선형 rollout 비용을 유한차분으로
국소 이차화하고, 조향률 한계와 누적 조향각 한계를 선형 부등식으로 구성한 뒤
OSQP가 QP를 푼다. 이 과정을 최대 3회의 SQP로 반복한다. 자체 projected gradient와
coordinate search는 명시적으로 설정했을 때만 사용한다. actuator delay와 1차
시정수는 공통 rollout에 포함한다.

```bash
sudo apt install ros-noetic-osqp-vendor ros-noetic-osqp
```

## 종방향 MPC

상태는 속도와 필터링한 가속도 `x=[v,a]`, 입력은 signed pedal `u`다. `u>0`은
가속, `u<0`은 제동이며 두 페달을 동시에 출력하지 않는다.

```text
a_req = k_accel*u                 (u >= 0)
a_req = k_brake*u                 (u < 0)
a[k+1] = a[k] + alpha*(a_req-a[k])
v[k+1] = max(0, v[k] + a[k+1]*dt)

J = sum(w_v*(v_ref-v)^2 + w_a*a^2 + w_j*jerk^2
        + w_u*u^2 + w_du*(u-u_prev)^2)
    + w_terminal*(v_ref-v_N)^2
```

입력 크기·변화율, 최대 가감속과 jerk를 rollout마다 투영한다. 예측 또는 측정
속도가 상한에 접근하면 최적화 결과와 무관하게 최소 제동을 명령하는 독립 가드를
적용한다. 솔버 실패 시 `longitudinal_mpc_fallback_to_pid: true`이면 PID로 폴백하고
상태 토픽에 이를 표시한다.

## 기본 파라미터 묶음

| 항목 | 기본값 | 의미 |
| --- | ---: | --- |
| `mpc_test_speed_limit_kph` | 59.0 | 횡방향 MPC 모드 속도 상한 |
| `mpc_solver` | osqp | 외부 횡방향 QP 솔버 |
| `mpc_prediction_time_sec` | 1.2 | 기본 예측 시간 |
| `mpc_minimum/maximum_prediction_time_sec` | 1.2 / 1.8 | 가변 예측 범위 |
| `mpc_minimum_preview_distance_m` | 8.0 | 최소 전방 예측 거리 |
| `mpc_resample_distance_m` | 0.5 | 경로 재표본 간격 |
| `mpc_optimizer_iterations` | 10 | 최대 솔버 반복 |
| `mpc_actuator_delay_sec` | 0.10 | 모델 조향 지연 |
| `mpc_actuator_time_constant_sec` | 0.15 | 조향 1차 응답 시정수 |
| `mpc_weight_lateral` | 12.0 | 횡오차 가중치 |
| `mpc_weight_heading` | 7.0 | 헤딩 오차 가중치 |
| `mpc_weight_steering_rate_change` | 4.8 | 조향 변화 진동 억제 |
| `mpc_adaptation_start/full_curvature_m_inv` | 0.003 / 0.020 | 직선/곡선 가중치 전환 범위 |
| `mpc_curve_steering_weight_multiplier` | 1.0 | 곡률 적응 비활성 기본값 |
| `mpc_curve_rate_change_weight_multiplier` | 1.0 | 곡률 적응 비활성 기본값 |
| `mpc_straight_rate_change_weight_multiplier` | 1.0 | 직선 적응 비활성 기본값 |
| `mpc_yaw_rate_damping_gain` | 0.0 | 직접 yaw 감쇠 비활성 기본값 |
| `longitudinal_mpc_horizon_steps` | 30 | 0.1 s 간격, 3초 예측 |
| `longitudinal_mpc_weight_speed` | 8.0 | 목표속도 추종 |
| `longitudinal_mpc_weight_jerk` | 1.2 | 급격한 가감속 억제 |
| `maximum_speed_kph` | 60.0 | 설정 가능한 절대 상한 |

속도를 올리기 전에는 실제 rosbag에서 solver 계산시간이 30 Hz 주기(33.3 ms)보다
충분히 짧은지, 최대 네 바퀴 회랑 여유가 양수인지 먼저 확인한다. 한 번에 여러
가중치를 바꾸지 않는다.

## 관측 토픽

```bash
rostopic echo /control/controller_status
rostopic echo /control/mpc_projection_point
```

`ControllerStatus`의 MPC 필드는 다음과 같다.

- `mpc_solver_success`, `mpc_solver_iterations`
- `mpc_solver_time_ms`, `mpc_solver_cost`
- `mpc_steering_rate_rad_per_sec`
- `mpc_projection_point_base`
- 공통 `cross_track_error_m`, `heading_error_rad`, `steering_angle_rad`
- `longitudinal_mpc_solver_success`, `longitudinal_mpc_fallback_active`
- `longitudinal_mpc_solver_time_ms`, `longitudinal_mpc_solver_cost`
- `longitudinal_mpc_estimated_acceleration_mps2`
- `longitudinal_mpc_predicted_maximum_speed_mps`
- `longitudinal_mpc_hard_speed_guard_active`

## 검증 명령

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=ON -j2
source devel/setup.bash
catkin_make run_tests_morai_path_tracking -DCATKIN_ENABLE_TESTING=ON -j2
catkin_test_results build/test_results/morai_path_tracking --verbose
```

실제 MORAI 시험은 `roslaunch morai_bringup molit_2026_autonomous.launch` 실행 뒤
입력 토픽과 `ACTIVE` 상태를 확인하고 사용자가 MORAI에서 AV 모드로 전환한다.
AV 전환 전에는 정지 상태에서 상태 토픽의 두 솔버 성공 여부와 강제 제동 경계를
확인한다. 실제 고속 주행 결과는 rosbag의 최대속도, solver 시간, 횡오차와 네 바퀴
회랑 최소 여유를 함께 남겨야 한다.
