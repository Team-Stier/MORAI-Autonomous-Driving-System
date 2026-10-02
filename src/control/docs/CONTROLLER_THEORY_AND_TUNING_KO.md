# MORAI 경로 추종 제어기 이론·파라미터·튜닝 상세 안내서

기존 `morai_path_tracking` 데모의 설계·측정 기록이다. 현재 실행 계약은 [control README](../README.md)를 따른다.
> 이 문서는 기존 Pure Pursuit/Stanley/IMM Hybrid의 상세 기준서다. 현재 기본
> 모드는 MPC이며, MPC 비용함수·파라미터·검증 절차는
> [MPC 통합 및 시험 안내](MPC_INTEGRATION_KO.md)를 함께 참고한다.

> 빠른 실행과 운영 점검은 패키지 루트의 `README.md`에서 먼저 확인한다. 이 문서는
> 제어 수식, 전체 파라미터, 검증 방법과 증상별 튜닝 근거를 보존한 상세 참고서다.

`morai_path_tracking`은 MORAI `25.S4.MolitComp03` 대회 환경에서 전역경로를
따라가기 위한 ROS1 Noetic 제어 패키지다. 이 패키지는 다음 기능을 한 노드에서
수행한다.

- `Pure Pursuit`, `Stanley`, 또는 두 제어기를 결합한 `IMM hybrid` 횡방향 제어
- 경로 곡률, 네 바퀴의 가정 차선 여유, 헤딩 오차를 이용한 목표속도 계획
- `CompetitionVehicleStatus.velocity_x_mps`를 피드백으로 쓰는 종방향 MPC와 PID 폴백
- 60 km/h 제한을 넘기지 않기 위한 59.5 km/h 독립 제동 경계
- Pure Pursuit 추종점과 Stanley 투영점, 제어기 내부 상태의 ROS 토픽 발행
- 입력 누락·지연·좌표계 오류·계산 오류 시 즉시 안전제동

기본 직선 목표속도는 **59 km/h**, 설정 파일의
운전자용 속도 단위는 **km/h**다. 계산과 ROS 상태 메시지 안의 속도 단위는
**m/s**다.

> **중요한 안전 범위**
> 이 제어기가 계산하는 차선 여유는 실제 도색 차선이나 카메라 인식 차선을 직접
> 사용하는 값이 아니다. 전역경로 중심에서 좌우 `1.50 m`, 즉 총 `3.00 m` 폭의
> 가상 주행 회랑을 가정한 결과다. 따라서 아래의 “바퀴 접촉 0회”는 이 가정과
> 기록된 전역경로에 대한 검증 결과이지, 폭이 계속 달라지는 실제 차선에 대한
> 수학적 절대 보증은 아니다. 실제 차선 폭 또는 차선 인식 결과를 연결하지 않은
> 상태에서 그보다 강한 보증을 주장하면 안 된다.

---

## 1. 처음 실행하는 사람을 위한 빠른 순서

### 1.1 빌드

```bash
cd "$HOME/catkin_ws"
source /opt/ros/noetic/setup.bash
catkin_make -j2
source "$HOME/catkin_ws/devel/setup.bash"
```

새 터미널을 열 때마다 다음 두 줄을 다시 실행한다.

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
```

### 1.2 MORAI 설정 확인

- 시뮬레이터 버전: `25.S4.MolitComp03`
- Competition Vehicle Status: MORAI에서 ROS 방향, Destination Port `9094`
- Ego Ctrl Cmd: ROS에서 MORAI 방향, Destination Port `9093`
- Competition 상태 토픽: `/vehicle/competition_status`
- 제어 명령 토픽: `/morai_udp_bridge/control_cmd`
- 기본 기어: `4`, 즉 Drive
- 기본 `gear_command_enabled`: `false`
- 자율 UDP sender와 수동 `vehicle_control` sender를 동시에 실행하지 않는다.

센서 preset과 시나리오 파일 위치는 다음과 같다.

| 종류 | 워크스페이스 파일 |
| --- | --- |
| 센서 preset | `src/ioniq5_description/config/morai_presets/0725demo.json` |
| 시험 시나리오 | `src/morai_bringup/config/morai_scenarios/R_KR_PR_K-city_2025/2026_molit_path_start_empty.json` |
| 전역경로 | `src/morai_path_manager/map/R-KR_PG_K-City_2025/2026_molit_comp_global_path.txt` |

### 1.3 전체 자율 스택 실행

MORAI를 `Manual-Keyboard` 상태로 둔 뒤 실행한다.

```bash
roslaunch morai_bringup molit_2026_autonomous.launch
```

이 launch는 기본값 그대로 다음을 모두 실행한다.

- 센서 UDP 브리지
- LiDAR 및 Velodyne 처리
- 차량 TF와 모델
- localization
- global/local path manager
- Competition Vehicle Status `9094` receiver
- 이 패키지의 경로 추종 제어기
- Ego Ctrl Cmd `9093` sender

입력과 제어 상태가 정상인지 확인한 뒤 MORAI를 `AV-ExternalCtrl`로 바꾼다.

```bash
rostopic echo -n 1 /vehicle/competition_status
rostopic echo -n 1 /localization/ego
rostopic echo -n 1 /local_path
rostopic echo -n 1 /control/controller_status
rostopic echo -n 1 /morai_udp_bridge/control_cmd
```

정상 제어 상태는 다음과 같다.

```text
active: true
state: ACTIVE
lateral_controller: hybrid
```

`accel`과 `brake`는 동시에 양수가 아니어야 하며 모든 속도·조향 값은 유한해야
한다.

### 1.4 시각화

```bash
roslaunch morai_bringup path_lidar.launch
```

렌더링 부담 때문에 RViz를 열지 않고 marker 노드만 실행하려면 다음과 같이 한다.

```bash
roslaunch morai_bringup path_lidar.launch start_rviz:=false
```

### 1.5 제어기만 따로 실행

센서, localization, path manager와 Competition Status receiver가 이미 실행 중인
경우에만 사용한다.

```bash
roslaunch morai_path_tracking path_tracking.launch
```

후보 YAML을 적용하려면 원본을 직접 계속 덮어쓰기보다 별도 파일을 전달한다.

```bash
roslaunch morai_path_tracking path_tracking.launch \
  config:=/tmp/molit_2026_path_tracking_candidate.yaml
```

UDP 송신기는 별도 터미널에서 하나만 실행한다.

```bash
roslaunch morai_udp_bridge control_sender.launch
```

---

## 2. 전체 데이터 흐름

```text
MORAI GPS/IMU
  -> morai_udp_bridge
  -> localization
  -> /localization/ego

전역경로
  -> morai_path_manager
  -> /global_path, /local_path

MORAI Competition Vehicle Status UDP :9094
  -> competition_vehicle_status_receiver_node
  -> /vehicle/competition_status

/local_path + /localization/ego + /vehicle/competition_status
  -> path_tracking_controller_node (기본 30 Hz)
     1) 좌표 변환과 입력 검증
     2) 네 바퀴 회랑 여유 계산
     3) 곡률 목표속도 계산
     4) Pure Pursuit / Stanley / hybrid 조향 계산
     5) 차선 여유·헤딩 오차 속도 제한
     6) 종방향 PID
  -> /morai_udp_bridge/control_cmd
  -> control_sender_node (기본 50 Hz)
  -> MORAI Ego Ctrl Cmd UDP :9093
```

속도 PID는 `/vehicle/competition_status`의 `velocity_x_mps`를 사용한다.
반드시 `/vehicle/competition_status.velocity_x_mps`만 사용하며, 이 입력이
0.25초 이상 끊기면 자동 fallback하지 않고 안전제동으로 전환한다.

---

## 3. 좌표계, 부호, 단위

### 3.1 차량 좌표계

`base_link`는 뒷차축 중앙이다.

- `+x`: 차량 전방
- `+y`: 차량 좌측
- `+z`: 위쪽
- 양의 yaw: 반시계 방향, 즉 좌회전
- 양의 조향각: 좌회전
- 축간거리 `L = 3.0 m`
- 차량 외측 폭 가정 `W = 1.892 m`

Stanley가 사용하는 전륜 중앙은 `base_link` 기준 `(L, 0) = (3.0, 0)`이다.

### 3.2 map 점을 차량 좌표로 변환

차량의 map 좌표를 `(x_v, y_v)`, yaw를 `ψ`, 경로점을 `(x_m, y_m)`라 하면 먼저
상대 위치를 계산한다.

```text
dx = x_m - x_v
dy = y_m - y_v
```

그 뒤 map 좌표를 `base_link` 좌표로 회전한다.

```text
x_body =  cos(ψ) dx + sin(ψ) dy
y_body = -sin(ψ) dx + cos(ψ) dy
```

따라서 `y_body > 0`인 경로점은 차량의 왼쪽에 있다. 이 부호 계약이 Pure
Pursuit, Stanley, wheel corridor, 시각화에서 모두 같아야 한다.

### 3.3 속도 단위

YAML에서 사람이 직접 설정하는 다음 값은 `km/h`다.

- `target_speed_kph`
- `minimum_curve_speed_kph`
- `lane_clearance_recovery_speed_kph`
- `heading_error_recovery_speed_kph`
- `hybrid_transition_reference_speed_kph`
- `speed_coast_overspeed_kph`
- `speed_brake_overspeed_kph`
- `hard_brake_activation_speed_kph`

노드는 시작할 때 다음 식으로 `m/s`로 바꾼다.

```text
v_mps = v_kph / 3.6
```

`ControllerStatus`의 속도 필드는 이름이 `_mps`로 끝나며 모두 `m/s`다.

---

## 4. Pure Pursuit

Pure Pursuit는 차량 뒤차축 중심에서 전방 경로의 한 점을 바라보는 기하학적
제어기다. 현재 단독 `pure_pursuit` 연산부의 기본 알고리즘은 유지되어 있다.
Hybrid에서만 Stanley 횡오차 항 일부를 후보 조향에 추가한다.

### 4.1 가변 lookahead distance

속도에 따른 기본 LD는 다음과 같다.

```text
Ld_speed = Ld_base + K_v |v|
```

근거리 preview 곡률 `κ_LD`가 커지면 LD를 줄인다.

```text
Ld_raw = Ld_speed / (1 + K_κ κ_LD)
Ld = clamp(Ld_raw, Ld_min, Ld_max)
```

현재 기본값은 다음과 같다.

```text
Ld_base = 4.0 m
K_v     = 0.75 s
K_κ     = 8.0 m
Ld_min  = 4.0 m
Ld_max  = 16.0 m
```

예를 들어 직선에서 58 km/h는 약 16.11 m/s이므로 곡률 보정 전 LD는
`4 + 0.75 × 16.11 = 16.08 m`이고, 최종값은 최대 제한에 의해 `16 m`다.
곡률 계산에는 전체 45 m 속도 preview가 아니라 차량 앞 8 m의
`lookahead_curvature_m_inv`만 사용한다.

### 4.2 추종점 선택

차량 원점을 중심으로 반지름 `Ld`인 원을 만들고, local path의 각 선분과 이
원의 첫 전방 교점을 찾는다. 교점은 반드시 `x_body > 0`이어야 한다. 교점이
없으면 유효한 전방 점 중 가장 먼 점을 fallback으로 사용한다. 그 점도
`minimum_target_distance_m`보다 가까우면 제어를 거부하고 안전상태로 간다.

### 4.3 조향식

선택한 추종점을 차량 좌표계에서 `(x_t, y_t)`라 하면 Pure Pursuit 조향은
다음과 같다.

```text
δ_PP = atan2(2 L y_t, x_t² + y_t²)
```

`L`은 축간거리다. 결과는 `±40 deg`로 제한한다.

### 4.4 Hybrid 안의 횡오차 보정 Pure Pursuit

Pure Pursuit만 쓰면 완만한 곡선이나 직선에서 일정한 횡편향을 직접 제거하기
어렵다. Hybrid는 Stanley의 CTE 피드백 성분만 일부 더한다.

```text
δ_PP,corr = sat(δ_PP + K_PP,CTE · δ_Stanley,CTE)
```

현재 `K_PP,CTE = 0.60`이다. 이 보정은 `lateral_controller: hybrid`일 때만
적용되며 standalone Pure Pursuit 수식은 바뀌지 않는다.

---

## 5. Stanley

Stanley는 전륜 중앙과 경로 사이의 횡오차, 차량과 경로 사이의 헤딩 오차를
직접 보정한다. 여기에 곡률 선행조향과 yaw-rate 감쇠를 추가했다.

### 5.1 전륜 투영점과 횡오차

1. 전륜 중앙 `(L, 0)`을 local path의 모든 유효 선분에 투영한다.
2. 거리가 가장 가까운 선분 투영점을 Stanley 기준점으로 고른다.
3. 투영점 앞뒤 총 `stanley_heading_window_m` 길이의 chord로 경로 방향을
   계산한다.
4. 경로 방향의 좌측 법선과 `전륜 -> 투영점` 벡터의 내적으로 signed CTE를
   계산한다.

경로 heading을 한 개의 짧은 선분 방향으로 쓰지 않고 기본 4 m chord로 계산하는
이유는 촘촘하거나 불균일한 path 점에서 heading이 튀는 현상을 줄이기 위해서다.

차량 좌표계에서 차량 heading은 0이므로 다음이 곧 heading error다.

```text
e_ψ = wrap(path_heading)
```

### 5.2 부호가 있는 기준 곡률

투영점부터 전방 8 m 구간의 시작점, 중간점, 끝점 세 개를 사용한다. 세 점을
`p1, p2, p3`라 할 때 signed curvature는 다음과 같다.

```text
κ_ref = 2 · cross(p2 - p1, p3 - p1)
        ---------------------------------
        |p2-p1| |p3-p2| |p3-p1|
```

`κ_ref > 0`은 좌곡선, `κ_ref < 0`은 우곡선이다. 기준 yaw rate는 다음과 같다.

```text
r_ref = v κ_ref
e_r = r_measured - r_ref
```

### 5.3 Stanley 조향의 네 성분

```text
δ_ff      = K_ff atan(L κ_ref)
δ_heading = K_h e_ψ
δ_CTE     = atan2(K_s e_CTE, max(|v|, v_min) + v_soft)

K_yaw,applied = K_yaw + K_yaw,nl |r_measured|
δ_yaw     = -K_yaw,applied e_r

δ_Stanley,req = δ_ff + δ_heading + δ_CTE + δ_yaw
```

- `δ_ff`: 커브에 필요한 정상상태 조향을 오차가 커지기 전에 공급한다.
- `δ_heading`: 차량 방향을 경로 방향으로 정렬한다.
- `δ_CTE`: 경로 중심으로 직접 복귀시킨다. 속도가 높을수록 분모가 커져 과민한
  조향을 줄인다.
- `δ_yaw`: 실제 회전이 기준보다 빠르면 반대 조향을 더해 진동과 커브 탈출
  과조향을 줄인다.

Standalone Stanley에서는 `δ_Stanley,req`를 `±40 deg`로 제한한 뒤 기본
`60 deg/s` 조향 변화율 제한을 적용한다. Hybrid에서는 Stanley의 요청 조향을
IMM 후보로 사용하고, 두 후보를 혼합한 뒤 Hybrid 공용 변화율 제한을 한 번 더
적용한다.

---

## 6. IMM 두 모델 필터

### 6.1 IMM이 하는 일

IMM(Interacting Multiple Model)은 “직선이면 Pure Pursuit, 곡선이면 Stanley”처럼
딱 잘라 전환하는 스위치가 아니다. Pure Pursuit 후보와 Stanley 후보를 각각
입력으로 갖는 두 차량 모델이 현재 측정된 sideslip과 yaw rate를 얼마나 잘
설명하는지 계산하고, 그 결과를 연속 확률로 만든다.

```text
μ_PP + μ_Stanley = 1
δ_nominal = μ_PP δ_PP,corr + μ_Stanley δ_Stanley,req
```

실제 최종 조향에는 뒤에서 설명하는 기하학적 안전 guard가 확률을 보정하므로
`IMM probability`와 `effective weight`는 서로 다를 수 있다.

### 6.2 상태와 측정

IMM 상태는 다음 두 값이다.

```text
x = [β, r]ᵀ
β: 차량 무게중심의 sideslip angle [rad]
r: yaw rate [rad/s]
```

트랙커가 ego 위치 변화로 계산한 뒤차축 기준 횡속도를 무게중심 속도로 옮긴다.

```text
v_y,CG = v_y,rear + l_r r
β_measured = atan2(v_y,CG, max(v_x, v_model,min))
z = [β_measured, r_measured]ᵀ
```

### 6.3 선형 bicycle model

```text
x_dot = A x + B δ
```

`m`은 질량, `I_z`는 yaw 관성, `C_f/C_r`은 전·후륜 cornering stiffness,
`l_f/l_r`은 무게중심에서 전·후차축까지 거리, `v`는 최소 모델 속도가 적용된
종방향 속도다.

```text
A00 = -2(C_f + C_r) / (m v)
A01 = -1 - 2(C_f l_f - C_r l_r) / (m v²)
A10 = -2(C_f l_f - C_r l_r) / I_z
A11 = -2(C_f l_f² + C_r l_r²) / (I_z v)

B0 = 2 C_f / (m v)
B1 = 2 C_f l_f / I_z
```

30 Hz 제어 주기에서는 Euler 방식으로 이산화한다.

```text
F = I + A Δt
x(k+1|k) = F x(k|k) + B Δt δ
```

두 branch의 `A`, `B`, 측정은 같고 조향 입력만 다르다.

- PP branch 입력: `δ_PP,corr`
- Stanley branch 입력: `δ_Stanley,req`

### 6.4 Markov 전이와 속도 보정

기본 전이행렬의 행은 이전 모델, 열은 다음 모델이다.

```text
             목적 PP   목적 Stanley
출발 PP        0.90        0.10
출발 Stanley   0.95        0.05
```

속도가 60 km/h에 가까워질수록 `hybrid_transition_speed_gain = 0.1`까지
Stanley 목적 확률에 연속 가산한다. 고속에서 직접 오차 피드백을 완전히 잃지
않도록 하는 prior이며, 측정 likelihood가 최종 확률을 다시 갱신한다.

### 6.5 Kalman innovation과 확률

각 branch는 `H = I`인 Kalman update를 수행한다.

```text
ν_j = z - x_pred,j
S_j = P_pred,j + R
d_j² = ν_jᵀ S_j⁻¹ ν_j
log L_j = -0.5 (d_j² + ln det(S_j))
```

Mahalanobis innovation이 작은 모델일수록 likelihood가 커진다. 전이 prior와
likelihood를 곱해 정규화한 뒤 Stanley 확률을 `[0.15, 0.90]`으로 제한하고
`μ_PP = 1 - μ_Stanley`로 둔다. 확률 제한은 한 모델이 영구적으로 사라지는 것을
막는다.

---

## 7. 최종 Hybrid 방식

Hybrid 계산 순서는 중요하다. 아래 보정은 순차적으로 적용된다.

### 7.1 후보 생성

```text
후보 1 = 횡오차 보정 Pure Pursuit δ_PP,corr
후보 2 = Stanley 요청 조향 δ_Stanley,req
```

두 후보로 IMM을 갱신하고 `μ_PP`, `μ_Stanley`를 얻는다.

### 7.2 후보 부호 충돌 guard

두 후보의 부호가 반대이고 다음 중 하나가 참이면 순간적인 IMM 오선택으로
판단한다.

```text
κ_preview >= 0.015 1/m
또는 |e_CTE| >= 0.45 m
```

현재 production 설정에서는 Pure Pursuit 후보를 선택한다.

```text
w_PP = 1, w_Stanley = 0
```

곡률 부호를 따르는 Stanley를 강제로 선택하는 실험 로직도 코드에 남아 있지만,
`hybrid_curve_preview_stanley_minimum_weight = 0.0`이므로 기본값에서는 완전히
비활성이다. 좁은 차선 실주행에서 기존 궤적보다 악화되어 채택하지 않았다.

### 7.3 같은 방향의 큰 CTE 복귀

후보 부호가 같고, 보정 PP가 CTE를 줄이는 방향이며 Stanley보다 강할 때 동작한다.
기본 full scale `0.55 m`의 절반인 `0.275 m`부터 PP 최소 비중을 smoothstep으로
올린다.

```text
u = clamp((|e_CTE| - 0.275) / (0.55 - 0.275), 0, 1)
S(u) = u²(3 - 2u)
```

단, 큰 heading error에서는 전륜 기준 CTE가 과장될 수 있다. `15 deg`부터
`17.5 deg`까지 PP 강제복귀 비중을 최대 `30%` 억제한다. 최대 억제 시 최종 PP
비중 상한은 `70%`다.

### 7.4 곡률 preview Stanley 최소 비중

두 후보가 같은 방향이고 Stanley가 더 강한 커브 진입 조향을 요구할 때,
`0.02~0.06 1/m`에서 Stanley 최소 비중을 올릴 수 있다. 그러나 현재 최소 비중
설정이 `0.0`이므로 계산 경로만 존재하고 실질적으로 비활성이다.

### 7.5 heading-lag Stanley 복구

다음 조건을 모두 만족하면 헤딩 지연 복구를 위해 Stanley 비중을 높인다.

- 두 후보 조향 방향이 같다.
- Stanley가 보정 PP보다 강하다.
- `e_heading × δ_Stanley > 0`, 즉 Stanley가 헤딩 오차를 줄이는 방향이다.

`|e_heading| = 8~14 deg`에서 smoothstep으로 증가하며, 최대 Stanley 최소 비중은
`0.55`다.

```text
u_h = clamp((|e_heading| - 8 deg) / (14 deg - 8 deg), 0, 1)
w_Stanley,min = 0.55 · S(u_h)
```

이 로직은 커브 전체에서 무조건 Stanley를 강제하지 않고 실제 헤딩 지연이 생긴
순간에만 동작한다.

### 7.6 차선 여유 기반 복귀

가정한 바퀴 최소 여유가 `0.18 m` 아래로 내려가면 urgency를 올리고, 두 후보 중
실제로 CTE를 줄이는 방향의 더 강한 후보에 최소 비중을 준다. `0.05 m`에서
urgency가 1이 된다. 이 단계에도 큰 heading error 억제를 적용한다.

### 7.7 최종 혼합

```text
δ_request = w_PP δ_PP,corr + w_Stanley δ_Stanley,req
w_PP + w_Stanley = 1
```

그 뒤 `±40 deg` 물리 한계와 곡률별 조향 변화율 제한을 적용한다.

### 7.8 곡률별 조향 변화율

저곡률 구간의 좌우 진동을 줄이면서 급커브 응답을 유지하기 위해 최대 조향률을
곡률에 따라 `45 deg/s`에서 `60 deg/s`로 바꾼다.

```text
u_r = clamp(κ_preview / 0.015, 0, 1)
rate = 45 + S(u_r) (60 - 45)  [deg/s]
```

이전 조향과 새 조향의 부호가 같고 조향 절대값을 줄이는 커브 탈출 상황에는
복귀 배수를 적용한다. 저곡률에서는 배수 1에 가깝고, `0.015 1/m` 이상에서는
최대 `2.0`이다. 반대 방향 전환이나 커브 진입 조향률을 임의로 두 배로 만들지는
않는다.

---

## 8. 네 바퀴 차선 회랑 계산

### 8.1 바퀴 외측 대표점

차량 폭의 절반을 `W/2`라 하면 네 바퀴의 바깥 한계를 다음 네 점으로 근사한다.

```text
뒤 왼쪽  : (0, +W/2)
뒤 오른쪽: (0, -W/2)
앞 왼쪽  : (L, +W/2)
앞 오른쪽: (L, -W/2)
```

현재 `W = 1.892 m`, `L = 3.0 m`다. 각 점을 local path 선분에 투영하여 signed
offset `d_i`를 계산한다.

```text
d_outer = max(|d_1|, |d_2|, |d_3|, |d_4|)
clearance = lane_half_width - d_outer
```

- `clearance > 0`: 가정 회랑 안쪽
- `clearance = 0`: 바퀴 외측 대표점이 가정 차선선에 닿음
- `clearance < 0`: 가정 차선선을 넘음

### 8.2 왜 총 3.0 m를 가정하는가

제공된 MGeo JSON의 link 폭은 모두 3.5 m로 균일해 사용자가 관찰한 좁고
들쭉날쭉한 도색 차선을 설명하지 못했다. 실주행과 bag 분석에서 2.6, 2.8,
2.9 m 가정은 타이트한 구간에서 겹침이 발생했고, 현재는 가장 좁게 양의 여유를
확인한 총 3.0 m를 사용한다.

이 값은 실제 차선센서가 아니다. 실제 폭이 3.0 m보다 좁으면 현재 계산상 양의
여유가 실제 양의 여유를 뜻하지 않는다.

### 8.3 여유 기반 속도 제한

```text
u_c = clamp((0.18 - clearance) / (0.18 - 0.05), 0, 1)
urgency = S(u_c)
v_lane = 58 - urgency (58 - 10)  [km/h]
```

최종 곡률 목표속도와 `v_lane` 중 작은 값을 사용한다. 여유가 충분하면 속도를
전혀 제한하지 않고, 가정 여유가 5 cm 이하이면 10 km/h까지 낮춘다.

---

## 9. 곡률 기반 속도 계획

### 9.1 곡률 profile

차량 최근접 local path 점부터 경로를 2 m 간격으로 다시 샘플링한다. 연속 세 점의
외접원 곡률을 계산하며 속도 계획은 부호 없는 곡률을 사용한다.

```text
κ = 2 |cross(p2-p1, p3-p1)|
    -------------------------
    |p2-p1| |p3-p2| |p3-p1|
```

`κ <= 0.001 1/m`는 직선으로 취급한다. 최대 45 m 전방을 검사한다.

### 9.2 곡선 지점 제한속도

허용 횡가속도만 사용한 제한은 다음과 같다.

```text
v_lat = sqrt(a_y,max / κ)
```

고곡률에서 조금 더 보수적으로 만들기 위한 연속 감속 gain을 적용한다.

```text
v_curve,raw = v_lat / (1 + K_reduce κ)
v_curve = clamp(v_curve,raw, v_curve,min, v_configured)
```

현재 값은 다음과 같다.

```text
a_y,max = 1.9 m/s²
K_reduce = 4.0 m
v_curve,min = 15 km/h
v_configured = 58 km/h
```

곡선 지점 자체에서의 이론 속도는 다음과 같다.

| 반경 | 곡률 | 계산 결과 |
| ---: | ---: | ---: |
| 100 m | 0.010 1/m | 약 47.7 km/h |
| 50 m | 0.020 1/m | 약 32.5 km/h |
| 25 m | 0.040 1/m | 약 21.4 km/h |
| 15 m | 0.0667 1/m | 약 15.2 km/h |
| 10 m | 0.100 1/m | 15 km/h 하한 적용 |

### 9.3 곡선까지 거리를 고려한 현재 허용속도

곡선 제한점이 `d` m 앞에 있으면 지금 당장 곡선 지점 속도까지 내리지 않는다.
가정 감속도 `a_approach`를 이용한다.

```text
v_now = sqrt(v_curve² + 2 a_approach d)
```

45 m preview 안 모든 후보의 `v_now` 중 최솟값을 raw 목표속도로 고른다.
`curve_approach_deceleration_mps2 = 1.0`은 직접 brake 명령이 아니라 “앞으로 이
정도의 감속이 가능하다”는 계획 가정이다.

- 값을 높이면 감속 시작이 늦어진다.
- 값을 낮추면 감속 시작이 빨라진다.

### 9.4 목표속도 필터와 변화율

raw 목표속도에는 1차 LPF를 적용한다.

```text
α = 1 - exp(-Δt / τ)
v_filtered ← v_filtered + α(v_raw - v_filtered)
```

현재 `τ = 0.35 s`다. 그 뒤 최종 목표속도의 상승과 하강 변화율을 제한한다.

- 일반 가속: 최대 `2.0 m/s²`
- preview 안에 제한 곡선이 남아 있는 동안 재가속: 최대 `0.2 m/s²`
- 감속: 최대 `5.0 m/s²`

S커브 사이의 짧은 직선에서 목표속도가 급상승했다가 바로 다시 감속하는 현상을
막기 위해 곡선이 남아 있을 때 재가속 제한을 별도로 둔다.

### 9.5 헤딩 오차 속도 제한

곡률 계획과 차선 여유 제한 뒤에도 heading error가 커지면 추가로 감속한다.

```text
u_h = clamp((|e_heading| - 14 deg) / (20 deg - 14 deg), 0, 1)
v_heading = v_current - S(u_h)(v_current - 10 km/h)
```

정렬된 커브는 제한하지 않고, CTE보다 먼저 커지는 S커브 헤딩 지연이 실제로
나타날 때만 10 km/h까지 연속 감속한다.

Standalone `pure_pursuit`는 heading error를 별도로 계산하지 않으므로 이 제한의
urgency가 0이다. Heading-error 속도 제한은 Stanley 또는 Hybrid에서 의미가 있다.

최종 PID 목표는 다음 최솟값이다.

```text
v_target = min(v_curvature_filtered_and_slew,
               v_lane_clearance,
               v_heading_error)
```

---

## 10. 종방향 PID와 부드러운 가감속

### 10.1 기본 제어식

```text
e = v_target - v_measured
d_measurement = (v_measured(k) - v_measured(k-1)) / Δt

u = K_ff max(v_target, 0)
    + K_p e
    + K_i integral(e)
    - K_d d_measurement
```

미분항은 목표속도 변화에 대한 derivative kick을 피하기 위해 오차가 아니라
측정속도에 적용한다. Competition Status 표본 간격에서 미분 펄스가 생길 수 있어
현재 `K_d = 0`이다.

### 10.2 상태별 동작

- `ACCEL`: signed command가 양수이며 `accel`만 발행
- `COAST`: 두 페달 모두 0
- `BRAKE`: signed command가 음수이며 `brake`만 발행
- `HARD_SPEED_BRAKE`: 실제 속도가 59 km/h 이상일 때 독립 제동

목표보다 `0.2 km/h` 이상 빠르지만 `1.8 km/h` 미만 빠른 구간에서는 정상
브레이크 대신 coast를 사용한다. `1.8 km/h` 이상 초과할 때 정상 PID 제동을
허용한다.

### 10.3 적분기와 anti-windup

- 속도 오차 `±0.10 m/s` 안에서는 P/D 오차를 0으로 본다.
- deadband 또는 coast 중에는 적분값을 초당 `0.5`씩 0 방향으로 푼다.
- 출력 saturation을 더 악화시키는 방향의 적분은 반영하지 않는다.
- 적분 절대값은 `1.0`으로 제한한다.

### 10.4 가속·브레이크 반복 방지

PID 출력은 먼저 하나의 signed effort로 표현한다.

```text
effort = accel - brake
```

정상 제어에서는 이 effort의 변화율을 초당 `2.0`으로 제한한다. 30 Hz에서는 한
주기당 약 `0.067`만 변한다. 이전 effort와 새 effort의 부호가 바로 뒤집히면 해당
주기에는 0을 출력하여 coast를 통과한다. 따라서 accel과 brake를 동시에 내거나
한 주기 만에 정반대 페달로 바꾸지 않는다.

입력 오류의 `safe_brake_command`와 59.5 km/h hard guard는 안전을 위해 정상 변화율
제한보다 우선한다.

### 10.5 60 km/h 제한

- 설정 목표: `59.0 km/h`
- 독립 제동 시작: `59.5 km/h`
- hard guard 최소 brake: `0.25`
- 대회 절대 제한: `60 km/h`

59.5 km/h 이상이면 계산된 정상 effort보다 적어도 `brake=0.25`가 되도록 강제한다.
다만 이 소프트웨어 guard도 통신 지연과 시뮬레이터 동역학을 없애지는 못하므로
60 km/h 미만이라는 결론은 매 변경 후 rosbag 실제 속도로 다시 확인해야 한다.

---

## 11. 입력 동기화와 안전상태

### 11.1 입력 계약

`/local_path`와 `/localization/ego`는 message filter로 source stamp를
동기화한다. 현재 `maximum_input_skew_sec = 0.0`이므로 두 메시지는 동일한
localization generation stamp를 가져야 한다. 새 path 하나만 먼저 들어온 짧은
구간에는 직전 정상 pair를 timeout까지 유지한다.

다음 조건이면 정상 제어를 중지한다.

- path 또는 ego pair가 없음
- Competition Vehicle Status가 없음
- 입력 수신 또는 source stamp age가 0.25초 초과
- frame이 `map`, `base_link` 계약과 다름
- stamp가 0이거나 미래/비유한 값
- pose, quaternion, 속도, yaw rate 또는 path 점이 비유한 값
- 제어 주기 `dt`가 `[0.005, 0.10] s` 밖
- 유효한 PP target, Stanley projection 또는 Hybrid 결과가 없음
- wheel corridor, 속도 제한, PID 계산이 유효하지 않음

### 11.2 안전 출력

안전상태 진입 시 PID 적분기, 속도 planner, IMM, 이전 조향을 모두 reset하고 다음을
즉시 발행한다.

```text
accel = 0.0
brake = 0.5
steering = 0.0
```

정확한 원인은 `/control/controller_status.state`에서 확인한다. 예:

- `WAITING_FOR_COMPETITION_VEHICLE_STATUS`
- `STALE_COMPETITION_VEHICLE_STATUS`
- `STALE_SYNCHRONIZED_PATH_ODOMETRY`
- `INVALID_PATH_ODOMETRY_FRAME_OR_STAMP`
- `INVALID_CONTROL_DT`
- `INVALID_HYBRID_CONTROL`
- `INVALID_PID_OUTPUT`

---

## 12. ROS 인터페이스

### 12.1 입력과 출력 토픽

| 방향 | 토픽 | 타입 | 핵심 내용 |
| --- | --- | --- | --- |
| 입력 | `/local_path` | `nav_msgs/Path` | map frame local path |
| 입력 | `/localization/ego` | `nav_msgs/Odometry` | map pose |
| 입력 | `/vehicle/competition_status` | `interfaces/CompetitionVehicleStatus` | control mode, gear, 차량 x속도 m/s |
| 출력 | `/morai_udp_bridge/control_cmd` | `morai_msgs/CtrlCmd` | accel/brake `[0,1]`, steering rad |
| 출력 | `/control/controller_status` | `morai_path_tracking/ControllerStatus` | 제어기 내부 진단값 |
| 출력 | `/control/lookahead_point` | `geometry_msgs/PointStamped` | PP LD target; Stanley 단독은 전륜 투영점 |
| 출력 | `/control/stanley_projection_point` | `geometry_msgs/PointStamped` | Stanley 전륜 투영점 |

### 12.2 CompetitionVehicleStatus

```text
std_msgs/Header header
uint8 control_mode
uint8 gear
float32 velocity_x_mps
```

현재 브리지는 MORAI `CompetitioninfoPublisher`의 152-byte payload를 해석한다.
payload의 x속도 원본은 km/h이며 receiver에서 3.6으로 나눈 뒤
`velocity_x_mps`로 발행한다. 확인된 모드는 `1=Manual`, `2=External control`이며
기어 `4=Drive`다. 이 enum은 시뮬레이터 버전이 바뀌면 실제 packet으로 다시
확인한다.

### 12.3 ControllerStatus 읽는 법

필드가 많으므로 목적별로 묶어서 본다.

| 목적 | 주요 필드 |
| --- | --- |
| 활성/오류 | `active`, `state`, `lateral_controller` |
| Stanley 오차 | `cross_track_error_m`, `heading_error_rad` |
| Stanley 각 성분 | `curvature_feedforward_steering_rad`, `heading_feedback_steering_rad`, `cross_track_feedback_steering_rad`, `yaw_rate_damping_steering_rad` |
| 회전 상태 | `reference_curvature_m_inv`, `reference_yaw_rate_radps`, `measured_yaw_rate_radps`, `yaw_rate_error_radps` |
| 후보 조향 | `pure_pursuit_steering_angle_rad`, `hybrid_corrected_pure_pursuit_steering_angle_rad`, `stanley_steering_angle_rad`, `requested_steering_angle_rad` |
| IMM | `hybrid_pure_pursuit_probability`, `hybrid_stanley_probability`, 두 innovation norm |
| 최종 가중치 | `hybrid_effective_pure_pursuit_weight`, `hybrid_effective_stanley_weight` |
| Hybrid guard | `hybrid_candidate_conflict_*`, `hybrid_cross_track_recovery_*`, `hybrid_lane_clearance_recovery_active`, `hybrid_heading_lag_stanley_recovery_*` |
| 조향률 | `hybrid_applied_maximum_steering_rate_rad_per_sec` |
| 곡률 속도 | `preview_curvature_m_inv`, `speed_limiting_curve_distance_m`, `curvature_speed_limit_mps`, `raw_target_speed_mps`, `filtered_target_speed_mps`, `target_speed_mps` |
| 차선 가정 | `lane_half_width_m`, `vehicle_width_m`, `wheel_outer_offset_m`, `wheel_minimum_clearance_m`, `lane_clearance_speed_limit_mps` |
| 헤딩 감속 | `heading_error_speed_limit_urgency`, `heading_error_speed_limit_mps` |
| 종방향 | `measured_velocity_x_mps`, `speed_error_mps`, `speed_overshoot_mps`, `longitudinal_state`, `accel`, `brake` |
| 시각화 | `lookahead_distance_m`, `lookahead_point_base`, `stanley_projection_point_base` |

`hybrid_*_probability`는 IMM의 원래 확률이고 `hybrid_effective_*_weight`는 모든
guard를 적용한 실제 혼합 가중치다. 튜닝할 때 이 둘을 혼동하면 안 된다.

속도 필드도 계산 단계가 다르다. `raw_target_speed_mps`와
`filtered_target_speed_mps`는 곡률 planner 결과이고, `target_speed_mps`는 그
뒤 차선 여유와 heading-error 제한까지 적용한 최종 PID 입력이다. 따라서 안전
제한이 동작하면 `target_speed_mps < filtered_target_speed_mps`가 정상적으로
발생할 수 있다.

---

## 13. 전체 기본 파라미터

단일 원본은 다음 YAML이다.

```text
config/controllers/molit_2026_path_tracking.yaml
```

모든 private parameter는 필수이며 시작 시 타입과 범위를 검사한다. YAML을 바꾼
뒤에는 제어기 노드를 재시작해야 한다.

### 13.1 인터페이스와 시간

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `local_path_topic` | `/local_path` | local path 입력 |
| `ego_topic` | `/localization/ego` | pose 입력 |
| `vehicle_status_topic` | `/vehicle/competition_status` | Competition 속도 입력 |
| `command_topic` | `/morai_udp_bridge/control_cmd` | actuator 출력 |
| `controller_status_topic` | `/control/controller_status` | 상세 상태 출력 |
| `lookahead_point_topic` | `/control/lookahead_point` | PP 추종점 출력 |
| `stanley_projection_point_topic` | `/control/stanley_projection_point` | Stanley 투영점 출력 |
| `expected_frame_id` | `map` | path/ego frame |
| `expected_velocity_frame_id` | `base_link` | 속도와 차량 기준 frame |
| `control_rate_hz` | `30.0` | WallTimer 제어 주기 |
| `path_timeout_sec` | `0.25` | path timeout |
| `ego_timeout_sec` | `0.25` | ego timeout |
| `vehicle_status_timeout_sec` | `0.25` | Competition Status timeout |
| `maximum_input_skew_sec` | `0.0` | path/odom stamp 최대 차이 |
| `input_sync_queue_size` | `10` | 동기화 queue |
| `minimum_control_dt_sec` | `0.005` | 허용 제어 dt 하한 |
| `maximum_control_dt_sec` | `0.10` | 허용 제어 dt 상한 |
| `safe_brake_command` | `0.50` | 안전상태 brake |

### 13.2 차량, 차선, 헤딩 안전

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `wheelbase_m` | `3.0` | 뒤차축에서 앞차축 거리 |
| `vehicle_width_m` | `1.892` | 바퀴 외측 폭 가정 |
| `lane_half_width_m` | `1.50` | 경로 중심에서 한쪽 차선선까지 거리 |
| `lane_clearance_recovery_start_m` | `0.18` | 차선 복귀·감속 시작 여유 |
| `lane_clearance_recovery_full_m` | `0.05` | urgency 1이 되는 여유 |
| `lane_clearance_recovery_speed_kph` | `10.0` | 여유 부족 시 최소 속도 |
| `heading_error_speed_limit_start_deg` | `14.0` | 헤딩 감속 시작 |
| `heading_error_speed_limit_full_deg` | `20.0` | 헤딩 감속 urgency 1 |
| `heading_error_recovery_speed_kph` | `10.0` | 큰 헤딩 오차 최소 속도 |

### 13.3 Pure Pursuit

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `lookahead_base_m` | `4.0` | 정지 LD 기여분 |
| `lookahead_speed_gain_sec` | `0.75` | 속도에 따른 LD gain |
| `lookahead_curvature_gain_m` | `8.0` | 곡률에 따른 LD 감소 gain |
| `lookahead_min_m` | `4.0` | 최소 LD |
| `lookahead_max_m` | `16.0` | 최대 LD |
| `minimum_target_distance_m` | `0.5` | fallback target 최소 거리 |
| `maximum_steering_angle_deg` | `40.0` | 공용 물리 조향 한계 |
| `lateral_controller` | `hybrid` | `pure_pursuit`, `stanley`, `hybrid` |

### 13.4 Stanley

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `stanley_gain` | `2.0` | CTE 보정 gain |
| `stanley_softening_speed_mps` | `2.0` | 저속 CTE 분모 완화 |
| `stanley_minimum_control_speed_mps` | `1.0` | 계산 최소 속도 |
| `stanley_heading_window_m` | `4.0` | heading chord 전체 길이 |
| `stanley_heading_error_gain` | `0.6` | heading feedback gain |
| `stanley_curvature_feedforward_gain` | `1.0` | 곡률 선행조향 gain |
| `stanley_curvature_preview_distance_m` | `8.0` | signed curvature 계산 거리 |
| `stanley_yaw_rate_damping_gain_sec` | `0.1` | 선형 yaw-rate 감쇠 gain |
| `stanley_yaw_rate_damping_nonlinear_gain_sec2` | `0.4` | 회전량 비례 추가 감쇠 gain |
| `stanley_maximum_steering_rate_deg_per_sec` | `60.0` | standalone Stanley 조향률 |

### 13.5 IMM 모델

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `hybrid_mass_kg` | `2000.0` | bicycle model 질량 |
| `hybrid_yaw_inertia_kgm2` | `4000.0` | yaw 관성 |
| `hybrid_front_cornering_stiffness_n_per_rad` | `60000.0` | 전륜 cornering stiffness |
| `hybrid_rear_cornering_stiffness_n_per_rad` | `60000.0` | 후륜 cornering stiffness |
| `hybrid_front_axle_to_cg_m` | `1.5` | CG-전차축 거리 |
| `hybrid_rear_axle_to_cg_m` | `1.5` | CG-후차축 거리 |
| `hybrid_process_noise_sideslip` | `0.1` | β process noise |
| `hybrid_process_noise_yaw_rate` | `0.01` | yaw-rate process noise |
| `hybrid_measurement_noise_sideslip` | `0.001` | β measurement noise |
| `hybrid_measurement_noise_yaw_rate` | `0.001` | yaw-rate measurement noise |
| `hybrid_initial_covariance_sideslip` | `0.1` | β 초기 covariance |
| `hybrid_initial_covariance_yaw_rate` | `0.01` | yaw-rate 초기 covariance |
| `hybrid_initial_pure_pursuit_probability` | `0.8` | 초기 PP 확률 |
| `hybrid_initial_stanley_probability` | `0.2` | 초기 Stanley 확률 |
| `hybrid_stanley_probability_min` | `0.15` | Stanley 확률 하한 |
| `hybrid_stanley_probability_max` | `0.90` | Stanley 확률 상한 |
| `hybrid_transition_pure_pursuit_to_pure_pursuit` | `0.9` | PP→PP 전이 |
| `hybrid_transition_pure_pursuit_to_stanley` | `0.1` | PP→Stanley 전이 |
| `hybrid_transition_stanley_to_pure_pursuit` | `0.95` | Stanley→PP 전이 |
| `hybrid_transition_stanley_to_stanley` | `0.05` | Stanley→Stanley 전이 |
| `hybrid_transition_speed_gain` | `0.1` | 고속 Stanley prior 증가량 |
| `hybrid_transition_reference_speed_kph` | `60.0` | 속도 보정 기준 |
| `hybrid_minimum_model_speed_mps` | `1.0` | 모델 분모 최소 속도 |

위 동역학 값 중 MORAI IONIQ 5에서 직접 식별되지 않은 값은 초기 모델값이다.
실차 상수처럼 간주하지 말고 rosbag innovation을 근거로 조정해야 한다.

### 13.6 Hybrid 보정과 조향률

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `hybrid_pure_pursuit_cross_track_correction_gain` | `0.60` | PP 후보에 더할 Stanley CTE 비율 |
| `hybrid_curve_preview_stanley_weight_start_m_inv` | `0.02` | 곡률 Stanley 보정 시작 |
| `hybrid_curve_preview_stanley_weight_full_m_inv` | `0.06` | 곡률 Stanley 보정 완료 |
| `hybrid_curve_preview_stanley_minimum_weight` | `0.0` | production 비활성 |
| `hybrid_heading_lag_stanley_weight_start_deg` | `8.0` | 헤딩 지연 복구 시작 |
| `hybrid_heading_lag_stanley_weight_full_deg` | `14.0` | 헤딩 지연 복구 완료 |
| `hybrid_heading_lag_stanley_minimum_weight` | `0.55` | 최대 Stanley 최소 비중 |
| `hybrid_candidate_conflict_curvature_threshold_m_inv` | `0.015` | 후보 충돌 곡률 임계 |
| `hybrid_candidate_conflict_cross_track_threshold_m` | `0.45` | 후보 충돌 CTE 임계 |
| `hybrid_cross_track_recovery_full_scale_m` | `0.55` | PP CTE 복귀 full scale |
| `hybrid_cross_track_recovery_heading_error_suppression_start_deg` | `15.0` | PP 복귀 억제 시작 |
| `hybrid_cross_track_recovery_heading_error_suppression_full_deg` | `17.5` | PP 복귀 억제 완료 |
| `hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio` | `0.30` | 최대 억제 비율 |
| `hybrid_maximum_steering_rate_deg_per_sec` | `60.0` | 고곡률 최대 조향률 |
| `hybrid_low_curvature_maximum_steering_rate_deg_per_sec` | `45.0` | 저곡률 최대 조향률 |
| `hybrid_full_steering_rate_curvature_m_inv` | `0.015` | 60 deg/s가 되는 곡률 |
| `hybrid_steering_return_rate_multiplier` | `2.0` | 고곡률 커브 탈출 복귀 배수 |

### 13.7 속도 planner

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `target_speed_kph` | `59.0` | 직선 최고 목표속도 |
| `minimum_curve_speed_kph` | `15.0` | 곡률 제한 하한 |
| `maximum_lateral_acceleration_mps2` | `1.9` | 허용 횡가속도 |
| `curvature_speed_reduction_gain_m` | `0.0` | 물리 횡가속도 식 외 추가 감속 비활성 |
| `curvature_preview_distance_m` | `45.0` | 속도 곡률 preview |
| `lookahead_curvature_preview_distance_m` | `8.0` | LD용 근거리 곡률 preview |
| `curvature_sample_spacing_m` | `2.0` | 곡률 재샘플 간격 |
| `curve_approach_deceleration_mps2` | `1.0` | 곡선 접근 감속 가정 |
| `curvature_epsilon_m_inv` | `0.001` | 직선 판정 임계 |
| `target_speed_acceleration_limit_mps2` | `2.0` | 일반 목표속도 상승률 |
| `curve_target_speed_acceleration_limit_mps2` | `0.2` | 곡선 preview 중 재가속률 |
| `target_speed_deceleration_limit_mps2` | `5.0` | 목표속도 하강률 |
| `target_speed_filter_time_constant_sec` | `0.35` | raw target LPF 시정수 |
| `speed_filter_time_constant_sec` | `0.0` | 측정속도 LPF, 0은 비활성 |

### 13.8 종방향 PID(비교 및 MPC 실패 폴백)

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `speed_kp` | `0.18` | 비례 gain |
| `speed_ki` | `0.02` | 적분 gain |
| `speed_kd` | `0.0` | 측정속도 미분 gain |
| `speed_integral_limit` | `1.0` | 적분 절대 한계 |
| `speed_integral_unwind_rate_per_sec` | `0.5` | deadband/coast 적분 해제율 |
| `speed_error_deadband_mps` | `0.10` | 속도 오차 deadband |
| `speed_accel_feedforward_gain_per_mps` | `0.008` | 목표속도당 accel feedforward |
| `speed_coast_overspeed_kph` | `0.2` | coast 시작 초과속도 |
| `speed_brake_overspeed_kph` | `1.8` | 정상 brake 허용 초과속도 |
| `hard_brake_activation_speed_kph` | `59.5` | 독립 속도 guard |
| `minimum_hard_brake_command` | `0.25` | guard 최소 brake |
| `maximum_accel_command` | `0.40` | 정상 accel 상한 |
| `maximum_brake_command` | `0.60` | brake 상한 |
| `longitudinal_command_rate_limit_per_sec` | `2.0` | signed effort 변화율 |

---

## 14. 시각화에서 봐야 할 것

`path_lidar.launch`의 marker는 다음 의미다.

| 표시 | 의미 |
| --- | --- |
| 청록색 선 | global path |
| 주황색 선 | local path |
| 빨간 원점/화살표 | 뒤차축 중심 `base_link`와 차량 heading |
| 노란 점과 분홍 선 | 전역경로 최근접점과 위치 오차 |
| 연두색 큰 구 | `/control/lookahead_point`, Hybrid에서는 PP LD target |
| 하늘색 구 | `/control/stanley_projection_point`, 전륜의 경로 투영점 |

Hybrid에서 두 점을 동시에 봐야 한다. PP target은 차량 전방에 있고 Stanley
projection은 앞차축 근처 경로 위에 있어야 정상이다. 좌우가 뒤집히거나 차량
뒤쪽으로 튀면 다음을 먼저 확인한다.

- `base_link`의 +x가 전방인가
- map yaw 부호가 ROS ENU 계약과 맞는가
- `steering_sign`이 현재 MORAI 차량과 맞는가
- local path의 pose 순서가 진행방향인가
- path와 ego stamp가 같은 generation인가

수치 모니터링 예시는 다음과 같다.

```bash
rostopic echo /control/controller_status
rostopic hz /vehicle/competition_status
rostopic hz /local_path
rostopic hz /morai_udp_bridge/control_cmd
```

속도만 km/h로 빠르게 보고 싶으면 다음처럼 확인할 수 있다.

```bash
rostopic echo -p /control/controller_status | \
  awk -F, 'NR==1 {print; next} {print $0}'
```

CSV 열 위치는 메시지 변경에 따라 달라질 수 있으므로 정식 분석에는 다음 절의
bag 분석기를 사용한다.

---

## 15. rosbag 기록과 반복 검증

### 15.1 반드시 기록할 토픽

`/global_path`를 빼면 사후 네 바퀴 궤적과 차선 여유를 재계산할 수 없다.

```bash
mkdir -p "$HOME/controller_bags/$(date +%F)"

rosbag record --duration=230 \
  -O "$HOME/controller_bags/$(date +%F)/candidate_01.bag" \
  /control/controller_status \
  /localization/ego \
  /local_path \
  /global_path \
  /morai_udp_bridge/control_cmd \
  /vehicle/competition_status
```

같은 후보를 여러 번 기록할 때는 `candidate_01`, `candidate_02`처럼 마지막에 두
자리 run 번호를 붙인다. 각 run은 같은 초기 pose와 같은 scenario에서 시작한다.

### 15.2 위치가 이상할 때

차량이 경로에서 크게 벗어나면 그 상태의 주행을 계속 성능 데이터로 쓰지 않는다.

1. 자율 launch를 완전히 종료한다.
2. MORAI를 `Manual-Keyboard`로 전환한다.
3. Simulator 창에서 `i`를 눌러 초기 위치로 되돌린다.
4. 초기 pose와 경로 정렬을 확인한다.
5. 자율 launch를 새로 시작한다.
6. `/control/controller_status`가 `ACTIVE`인지 확인한다.
7. 그 뒤 `AV-ExternalCtrl`로 전환한다.

초기화 뒤 기존 PID 적분, IMM 확률, 조향 상태를 이어 쓰지 않도록 제어기 노드를
반드시 새로 시작한다.

### 15.3 분석기 실행

인자는 ROS remap 문법이 아니라 `LABEL=/absolute/path.bag` 형식이다.

```bash
rosrun morai_path_tracking analyze_tracking_bag.py \
  baseline=/absolute/path/baseline_01.bag \
  candidate=/absolute/path/candidate_01.bag \
  --lane-half-width-m 1.5 \
  --vehicle-width-m 1.892 \
  --wheelbase-m 3.0 \
  --output-dir /absolute/path/report
```

주요 결과물은 다음과 같다.

- `metrics.yaml`, `metrics.csv`: run별 상세 수치
- `aggregate_metrics.yaml`, `aggregate_metrics.csv`: 후보별 요약
- `trajectory_comparison.png`: 전역 궤적 비교
- `tracking_timeseries.png`: CTE와 heading error
- `steering_timeseries.png`: 조향과 조향률
- `longitudinal_timeseries.png`: 속도, 목표속도, accel/brake
- `hybrid_weights_timeseries.png`: IMM 확률과 최종 가중치
- `bag_manifest.yaml`: 입력 bag 경로와 SHA256

### 15.4 합격 판단 순서

빠른 후보가 반드시 좋은 후보는 아니다. 다음 순서로 판단한다.

1. 비유한 값 또는 inactive sample이 없는가
2. 최대 실제 속도가 60 km/h 미만인가
3. 3.0 m 가정에서 wheel contact sample이 0인가
4. 최소 바퀴 여유가 반복 run에서도 양수인가
5. CTE max, p95, RMS가 모두 악화되지 않았는가
6. 고곡률 구간 CTE와 속도가 함께 개선됐는가
7. steering rate RMS/p95가 진동을 숨기지 않는가
8. 직선 brake와 direct pedal reversal이 증가하지 않았는가
9. 최소 5회, 경계 후보는 10회 반복했는가

한 번 접촉이 없었다는 이유만으로 안전하다고 결론 내리지 않는다.

---

## 16. 현재 설정의 실주행 검증 결과와 한계

2026-08-01 로컬에서 기록한 최종 `heading_recovery` bag 한 회의 결과는 다음과
같다. 원본 bag과 자동 생성 그래프·CSV는 용량 때문에 Git에서 제외한다.

```text
bag: docs/path_tracking_tuning/2026-08-01/
     fast_heading_recovery_run/fast_heading_recovery_01.bag
duration: 225.93 s
maximum speed: 58.29 km/h
wheel line contact samples: 0
minimum wheel clearance: 0.0125 m
CTE RMS: 0.1134 m
CTE p95: 0.2261 m
curved CTE p95: 0.2499 m
steering-rate RMS: 21.74 deg/s
steering-rate p95: 50.58 deg/s
```

동일 분석에서 이전 baseline은 다음과 같았다.

```text
maximum speed: 58.33 km/h
minimum wheel clearance: 0.0987 m
CTE RMS: 0.1241 m
CTE p95: 0.2639 m
curved CTE p95: 0.3030 m
steering-rate RMS: 24.77 deg/s
steering-rate p95: 59.91 deg/s
```

즉 최종 후보는 전체/곡선 CTE와 조향률 통계는 개선됐지만, 가정 차선 최소 여유가
약 **1.25 cm**까지 줄었다. 따라서 이 결과만으로 고곡률 속도를 일괄 상향하거나
“실제 차선을 절대 밟지 않는다”고 결론 내릴 수 없다. 또한 최종 run의 direct
pedal reversal은 43회로 baseline 18회보다 많고 run 길이도 서로 다르므로,
동일 길이 반복 run에서 종방향 전환 횟수를 다시 비교해야 한다.

작업 PC의 상세 결과는 다음 로컬 폴더에 있다. 저장소를 새로 clone하면 이 폴더는
포함되지 않는다.

```text
docs/path_tracking_tuning/2026-08-01/final_heading_recovery_comparison/
```

현재 수치에서 고곡률 속도를 더 올리려면 전 구간 `minimum_curve_speed_kph`를
올리는 방식보다 다음 조건을 모두 만족하는 안정된 커브에서만 제한적으로 올리는
방법이 안전하다.

- 바퀴 최소 여유가 충분함
- CTE와 heading error가 작고 감소 중임
- yaw-rate error가 작음
- 일정 시간 이상 상태가 유지됨
- 다음 타이트 곡선까지 감속 거리가 충분함

이 조건부 속도 상향 로직은 현재 production 코드에 구현되어 있지 않다.

---

## 17. 증상별 튜닝 지침

한 번에 한 파라미터 묶음만 바꾸고, 정확한 YAML과 bag을 함께 보관한다.

### 17.1 직선·저곡률에서 좌우로 흔들림

우선 확인할 필드:

```text
cross_track_error_m
heading_error_rad
steering_angle_rad
hybrid_applied_maximum_steering_rate_rad_per_sec
hybrid_effective_*_weight
```

조정 후보:

- `hybrid_low_curvature_maximum_steering_rate_deg_per_sec`를 조금 낮추면 짧은 좌우
  반전을 줄일 수 있지만 복귀가 늦어질 수 있다.
- `stanley_heading_error_gain`을 낮추면 heading feedback 진동은 줄지만 경로
  정렬이 늦어진다.
- `stanley_heading_window_m`을 늘리면 heading이 부드러워지지만 타이트한 S커브
  형상을 늦게 본다.
- `stanley_yaw_rate_damping_*`을 올리면 과회전을 줄이지만 정상 커브 조향까지
  약해질 수 있다.

### 17.2 완만한 곡선에서 바깥으로 흐르다 안쪽으로 툭 침

- `hybrid_pure_pursuit_cross_track_correction_gain`이 너무 작으면 지속 CTE가 남고,
  너무 크면 늦게 강한 복귀가 생긴다.
- `stanley_gain`이 작으면 중앙 복귀가 느리고, 크면 고속 진동이 커진다.
- `hybrid_heading_lag_stanley_*`가 너무 늦으면 커브 진입 heading lag가 쌓이고,
  너무 빠르면 저곡률 조향이 민감해진다.
- PP point, Stanley projection, CTE 부호를 RViz와 status에서 함께 확인한다.

### 17.3 고곡률 구간이 너무 느림

속도 자체를 결정하는 순서를 먼저 구분한다.

```text
curvature_speed_limit_mps
raw_target_speed_mps
filtered_target_speed_mps
lane_clearance_speed_limit_mps
heading_error_speed_limit_mps
target_speed_mps
```

- `curvature_speed_limit_mps`가 낮으면 `maximum_lateral_acceleration_mps2` 또는
  `curvature_speed_reduction_gain_m` 문제다.
- 곡선이 멀리 있는데 raw target이 낮으면 `curve_approach_deceleration_mps2`가
  너무 낮아 일찍 감속하는 것이다.
- lane limit가 낮으면 실제 병목은 바퀴 여유다. 횡제어 정확도를 먼저 개선한다.
- heading limit가 낮으면 실제 병목은 진입 헤딩 지연이다. 속도만 올리면 앞바퀴
  여유가 더 줄어든다.

`minimum_curve_speed_kph` 일괄 상향은 마지막 수단이다. 현재 최종 run의 최소
여유가 1.25 cm이므로 추가 반복검증 없이 올리지 않는다.

### 17.4 가속과 브레이크를 반복함

- `raw_target_speed_mps` 자체가 톱니면 속도 planner 문제다.
- raw는 안정적인데 filtered가 빠르면 `target_speed_filter_time_constant_sec`를
  늘릴 수 있다.
- S커브 사이 재가속이면 `curve_target_speed_acceleration_limit_mps2`를 낮춘다.
- 실제 속도만 튀면 Competition Status 수신 주기와 packet 단위를 확인한다.
- 목표는 안정적인데 pedal만 반복하면 `speed_kp`, feedforward, coast/brake
  threshold와 signed effort rate를 본다.

### 17.5 커브 진입 감속이 너무 빠르거나 늦음

- 너무 빠름: `curve_approach_deceleration_mps2`를 조금 높인다.
- 너무 늦음: 같은 값을 낮춘다.
- `target_speed_deceleration_limit_mps2`가 너무 작으면 planner가 낮은 목표를
  만들었어도 최종 target이 늦게 내려간다.
- LPF 시정수가 너무 크면 감속도 지연된다.

---

## 18. 테스트

### 18.1 패키지 전체 테스트

```bash
cd "$HOME/catkin_ws"
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
catkin_make run_tests_morai_path_tracking -j2
catkin_test_results build/test_results/morai_path_tracking --verbose
```

### 18.2 주요 테스트 범위

- Pure Pursuit 원 교점, 가변 LD, 조향 한계
- Stanley 전륜 투영, CTE/heading 부호, 곡률 feedforward, yaw damping
- IMM 확률 정규화, 전이행렬, innovation
- Hybrid 후보 충돌, CTE/heading/차선 복구, 조향률
- 곡률 속도 planner, LPF, target slew
- wheel corridor와 smoothstep 속도 제한
- PID deadband, coast, anti-windup, hard guard, 페달 상호배제
- path/ego 동기화와 timeout 안전동작
- 실제 ROS 메시지를 이용한 Pure Pursuit/Stanley/Hybrid integration
- rosbag 분석기의 차선 접촉·CTE·조향률·페달 전환 통계

통합 테스트가 전체 병렬 실행에서 timeout으로 한 번 실패하면 곧바로 로직 실패로
단정하지 말고 해당 rostest를 단독 `-j1`로 재실행한다. 단독 재실행도 실패하면
로그와 timestamp 조건을 원인 분석한 뒤 수정한다. 실패 결과를 숨긴 채 전체
통과라고 기록하면 안 된다.

### 18.3 곡률 planner와 최종 안전 제한의 구분

통합 테스트의 2 m 반경 경로를 `lane_half_width_m=1.30 m`, 차량 폭
`1.892 m`, 축간거리 `3.0 m`로 계산하면 다음 값이 나온다.

```text
wheel_outer_offset = 2.022 m
wheel_minimum_clearance = 1.300 - 2.022 = -0.722 m
lane_clearance_speed_limit = 0.833 m/s (3.0 km/h)
```

차선 안전 제한이 최종 `target_speed_mps`를 곡률 planner의
`filtered_target_speed_mps`보다 더 낮추는 것이 정상이다. 테스트는 곡률 profile과
가변 LD를 먼저 검증한 다음, 음수 회랑 여유에서 `lane_clearance_speed_limit_mps`가
`3.0 km/h`로 적용되고 최종 목표속도가 그 제한과 같아지는지를 별도로 검증한다.
따라서 planner의 중간 목표와 차선·헤딩 제한 뒤 최종 PID 목표를 혼동하지 않는다.

2026-08-01에 안전 제한 도입 전의 잘못된 비교식
`target_speed_mps >= filtered_target_speed_mps`를 위 단계별 검증으로 교체했고,
해당 rostest 9개가 모두 통과하는 것을 확인했다. 제어 연산부는 변경하지 않았다.

---

## 19. 디렉터리 구조

```text
morai_path_tracking/
├── config/controllers/
│   └── molit_2026_path_tracking.yaml
├── include/morai_path_tracking/
│   ├── common/
│   ├── controllers/
│   │   ├── lateral/
│   │   └── longitudinal/
│   └── planning/
├── launch/controllers/
│   └── path_tracking.launch
├── msg/
│   └── ControllerStatus.msg
├── scripts/analysis/
│   └── analyze_tracking_bag.py
├── src/
│   ├── common/
│   ├── controllers/
│   │   ├── lateral/
│   │   └── longitudinal/
│   ├── nodes/
│   └── planning/
└── test/
    ├── integration/
    └── unit/
```

연산부를 찾을 때는 다음 파일부터 본다.

| 기능 | 파일 |
| --- | --- |
| 통합 순서와 ROS I/O | `src/nodes/path_tracking_controller_node.cpp` |
| Pure Pursuit | `src/controllers/lateral/pure_pursuit.cpp` |
| Stanley | `src/controllers/lateral/stanley_controller.cpp` |
| Hybrid guard/혼합 | `src/controllers/lateral/hybrid_controller.cpp` |
| IMM | `src/controllers/lateral/imm_two_model_filter.cpp` |
| PID | `src/controllers/longitudinal/pid_controller.cpp` |
| 곡률 속도계획 | `src/planning/curvature_speed_planner.cpp` |
| 네 바퀴 회랑 | `src/planning/wheel_corridor.cpp` |

---

## 20. 자주 생기는 문제

### `WAITING_FOR_COMPETITION_VEHICLE_STATUS`

```bash
ss -lunp | grep ':9094 '
rostopic hz /vehicle/competition_status
```

MORAI Destination Port가 9094인지, Destination IP가 ROS PC인지, receiver가 하나
실행 중인지 확인한다.

### 속도가 3.6배 이상하거나 너무 작음

MORAI packet 원본은 km/h이고 `CompetitionVehicleStatus.velocity_x_mps`는 m/s다.
receiver에서 변환을 한 번만 해야 한다. PID나 YAML loader에서 같은 값을 다시
3.6으로 나누지 않는다.

### 조금 가다가 반복해서 멈춤

`/control/controller_status.state`를 먼저 본다. path/ego/status 중 하나가
0.25초 timeout이 나거나 path/odom stamp가 정확히 맞지 않으면 정상 PID가 아니라
`brake=0.5` 안전상태가 반복될 수 있다.

### 외부제어인데 차량이 움직이지 않음

- MORAI control mode가 실제 External인지 확인한다.
- Competition Status의 `control_mode`, `gear`를 확인한다.
- 기어가 4인지 확인한다.
- control sender가 9093으로 보내는지 확인한다.
- `vehicle_control`과 자율 sender가 동시에 실행 중이지 않은지 확인한다.

### 조향 방향이 반대임

`morai_udp_bridge/config/control/molit_2026_control.yaml`의 `steering_sign`과 차량
좌표계를 확인한다. 현재 기본은 `1.0`이며 임의로 바꾸기 전에 작은 조향 명령으로
좌회전 양의 부호를 확인한다.

### `Cannot load message class`

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
rosmsg show morai_path_tracking/ControllerStatus
rosmsg show interfaces/CompetitionVehicleStatus
```

### 제어기를 멈출 때

제어 launch와 UDP sender를 `Ctrl-C`로 종료한다. 센서/LiDAR 스택을 끌 필요는
없다. 다시 시작하기 전 MORAI가 Manual인지와 9093 sender가 하나뿐인지 확인한다.

---

## 21. 핵심 요약

- 기본 방식은 **횡오차 보정 Pure Pursuit + Stanley + IMM 연속 혼합**이다.
- IMM 확률 뒤에 후보 충돌, CTE, 헤딩 지연, 차선 여유 guard가 순서대로 적용된다.
- 저곡률 조향률은 45 deg/s, 고곡률은 60 deg/s로 연속 변경된다.
- 가정 차선은 전역경로 중심 좌우 1.50 m이며 네 바퀴 외측 대표점을 검사한다.
- 곡률, 바퀴 여유, 헤딩 오차의 세 속도 제한 중 가장 작은 값을 PID 목표로 쓴다.
- 속도 피드백은 UDP 9094의 `CompetitionVehicleStatus.velocity_x_mps`만 사용한다.
- 설정 목표는 58 km/h, 59 km/h부터 독립 제동하며 60 km/h를 넘지 않도록 매
  변경 후 실제 bag으로 검증한다.
- 현재 한 번의 최종 run은 가정 차선 접촉 0회였지만 최소 여유가 1.25 cm이므로
  실제 가변 차선에 대한 절대 보증이나 추가 일괄 속도 상향 근거는 아니다.
- 자율 제어 UDP 9093 sender는 반드시 하나만 실행한다.

## Ego 입력과 횡속도

위치·자세·횡속도·yaw rate는 `/localization/ego` (`nav_msgs/Odometry`), 전방 속도는 `/vehicle/competition_status`에서 받는다.
트랙커는 `pose.pose`의 위치·자세, `twist.twist.linear.y`의 횡속도와
`twist.twist.angular.z`의 yaw rate를 직접 사용한다. `child_frame_id=base_link`를 검사한다.
공분산은 사용하지 않는다. 비유한 값·잘못된 frame·시각·timeout은 기존 안전제동 경로로 처리한다.
