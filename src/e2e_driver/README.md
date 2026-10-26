# e2e_driver: 학생 E2E 모델 폐루프 평가

학습한 학생 모델(와우마요: Qwen3-VL-4B + 이식 expert)이 MORAI를 직접 운전하는 평가 스택이다.
브랜치는 `eval/claude-closedloop-20261025`이고, `origin/main` @79df2c0에서 갈라졌다.

```
MORAI ──UDP──> MORAI-UDP-Bridge-Node ──(카메라 4, GPS, IMU, competition_status)──┐
MORAI ──UDP 2368 (VLP16)──> velodyne_driver ──/velodyne_packets─┬─> vlp16_bev ──/lidar_bev/scan──┤
                                velodyne_pointcloud ─> lidar_leveler ─> localization (tunnel_wall)
                       localization ──/localization/ego (경로 진행거리)───────────┤
                                                                                  v
                     e2e_driver_node ──TCP──> model_server (GPU, chroot 밖 venv)
                            │
                       /control_cmd (morai_msgs/CtrlCmd)
                            v
                       speed_filter ──/morai_udp_bridge/control_cmd──> 브릿지 ──UDP 9093──> MORAI
```

- **직렬 경로**는 MORAI → 브릿지 → 모델 → speed_filter → 브릿지 → MORAI다. LiDAR만 브릿지를 거치지 않고, 수집 때처럼 velodyne_driver가 받는다.
- **곁가지 노드**
  - localization과 section_annotator가 경로 진행거리와 speed_filter의 섹션을 준다.
  - **LiDAR**: 수집 때와 같은 구성이다. velodyne 서브모듈은 수집 빌드와 같은 커밋(29abd0e)이고, 수집 때 `run.sh`와 같은 `roslaunch velodyne_pointcloud VLP16_points.launch`로 띄운다.
    - velodyne_driver가 UDP 2368을 받아 `/velodyne_packets`(76패킷 VelodyneScan)를 낸다.
    - vlp16_bev(모델 BEV)와 velodyne_pointcloud(`/velodyne_points` → lidar_leveler → localization tunnel_wall)가 둘 다 이 토픽을 읽는다. 한 UDP 포트를 두 프로그램이 동시에 받으면 Linux가 패킷을 나눠 주기 때문에 이렇게 한다.
    - 학습 데이터도 bag의 같은 메시지로 만들었으므로, 패킷 묶음과 시각까지 같다.
    - tf_publisher는 lidar_leveler에 장착 TF(`/tf_static`)를 준다.
  - GPS가 끊기면 tunnel_wall로 넘어가는 규칙도 수집 포크(e2cd63a)와 같이 넣었다(`localization/config/localization/selection.yaml`). `origin/main`에는 이 규칙이 없어, GPS가 끊기면 localization이 멈춘다.
  - `run.sh`는 이 체인에 필요한 패키지만 빌드하고 띄운다. path_planner, control, traffic_light는 띄우지 않는다.
- **speed_filter 출발 보조는 껐다**: `stop_tolerance_sec: 1.0e9`(`speed_limit60.yaml`, `no_speed_limit.yaml`). 학습 데이터를 이 보조 없이 모았기 때문이다(수집 포크 1e94244). 0으로 두면 서는 즉시 보조가 걸린다(now − since ≥ 0).

## 구성

| 위치 | 하는 일 |
|---|---|
| `src/vlp16_bev` | velodyne_driver의 `/velodyne_packets`를 받아 학습 코드 `lidar_bev.scan_bev_stream`으로 스캔별 BEV를 만든다(`interfaces/LidarBevScan`) |
| `src/e2e_driver/scripts/e2e_driver_node.py` | 모델 노드. 입력을 버퍼에 모아 10 Hz로 요청을 만들고 모델 서버에 보낸다. 받은 계획을 `/control_cmd`로 50 Hz 발행한다 |
| `src/e2e_driver/server/` | 모델 서버(학습 venv, GPU). 학습 함수로 샘플을 조립하고 백엔드(`hf`, `fake`)로 추론한다 |
| `src/e2e_driver/src/e2e_driver/` | `train_copy.py`(학습 코드 원문 사본), `inputs.py`(프레임 선택·step-0 JPEG), `route.py`(진행거리·nav text), `control.py`(a, κ → 페달·조향), `wire.py`(노드↔서버 전송) |
| `src/e2e_driver/assets/route.json` | 전역 경로, 기동 목록, 속도 예외 구간. 학습 코드에서 `tools/make_route_asset.py`로 뽑았다 |
| `src/e2e_driver/tools/winpc/` | winpc WSL용 빌드·실행 스크립트 |
| `src/e2e_driver/test/` | 학습 코드 사본 검사, 오프라인 일치 시험, 제어 상수 측정, 백엔드 시험 |

## 학습 입력과 맞춘 방법

입력이 학습과 조금이라도 다르면 폐루프 결과가 무의미하다. 그래서 학습 함수를 import하거나 원문 그대로 옮겼다.

| 입력 | 학습 (step 0 + `morai_vla.data`) | 추론 |
|---|---|---|
| 카메라 | bag JPEG → cv2 decode → `INTER_AREA` 축소 → JPEG q92 → PIL decode | 같은 호출을 같은 Noetic rootfs의 cv2 4.2로 한다(`inputs.reencode`). PIL decode는 서버가 `Episode.jpeg`와 같은 코드로 한다 |
| 프레임 선택 | τ 목표 t0, −0.1, −0.2, −0.3 s에 `select_distinct` | 같은 함수(`train_copy.py`), 지터 없음. t0는 요청 시각이다. 그래서 가장 새 프레임도 t0 이전이다(학습 행에는 t0 뒤 0.05 s까지의 프레임도 있었다) |
| 시각 텍스트 | 프레임 τ − t0 | 프레임 수신 시각(브릿지 stamp) − t0 |
| LiDAR | velodyne_driver의 76패킷 스캔. 패킷 시각에서 `_ego` v, yaw rate를 샘플링해 `scan_bev_stream` | 같은 드라이버의 같은 메시지. v는 competition_status `signed_velocity_mps`, r은 competition_status yaw로 학습과 같은 식(아래 `GridYawRate`, 마지막 0.3 s는 추세 외삽)으로 구한다. 서버가 `pack_bev`로 2장을 만든다 |
| ego 이력 | `_ego` τ 격자의 GT v, yaw를 `ego_trajectory`로 적분 | competition_status 속도와 yaw를 t0 − 1.5 … t0에 선형 보간하고, 서버가 같은 `ego_trajectory`로 적분한다 |
| nav text | scorer `KMap.progress`의 path_s(GT base_link 자세)를 `build_index.nav_text`로 | `/localization/ego`(같은 map frame의 EKF)를 `KMap.progress`와 같은 창 추적으로 처리한다. 위치가 끊기면(GPS 음영, 터널) competition_status 주행거리로 진행한다 |
| 토큰화 | `get_preprocess_fn(generation_mode=True)` + `collate_fn` | 같은 함수(`server/assemble.py`) |

- **신호 근거**(run1024b-w1-001 bag)
  - competition_status 속도와 GT 속도의 차이는 중앙값 0.0002 m/s다.
  - competition_status yaw는 GT yaw보다 상수 0.0235 rad 작다. 상대 heading에서는 상쇄된다.
  - **LiDAR yaw rate**: 학습의 `_ego.yaw_rate`는 `np.convolve(np.gradient(yaw, 0.1), np.ones(5) / 5, "same")`(coc_labeler `load_episode`)이다. 0.1 s 격자 GT yaw에 중앙 차분과 5탭 이동평균을 한 값이다.
    - 노드(`vlp16_bev/scan.py` `GridYawRate`)는 competition_status yaw로 같은 계산을 한다.
    - 격자점 하나의 값에는 0.3 s 뒤의 yaw가 필요하다(중앙 차분 ±0.1 s, 5탭 ±0.2 s). 실시간에는 그 0.3 s가 없다.
    - 기본(`grid_trend`)은 최근 0.3 s yaw의 직선 적합으로 그 구간을 채운 뒤 같은 식을 적용한다. `grid`는 지지가 모두 있는 마지막 격자점 값을 유지하며, 회전 시작·끝에서 학습값보다 최대 약 0.3 s 늦다. 지연 없이 하려면 스캔을 0.3 s 늦게 내야 하는데, 그러면 가장 새 스캔이 t0에서 0.15 s 넘게 떨어져 학습 행 조건(`MAX_SCAN_GAP`)을 벗어난다.
    - IMU wz도 GT yaw rate와 상관이 0.99이지만, 학습과 같은 식이 아니라 쓰지 않는다(비교는 아래 일치 시험). competition_status의 `angular_velocity_z_radps`는 부호가 반대다.
- **시간**
  - bag의 모든 stamp는 벽시계다. packet 안의 MORAI 시각도 벽시계 속도로 간다(d/dt = 1.000).
  - 그래서 추론은 벽시계를 τ로 쓴다. 대회는 Time Scale 1을 전제한다.
  - 시뮬레이터가 실시간보다 느리면(RTF < 1) 시각 텍스트와 적분이 그만큼 어긋난다. 평가할 때 RTF를 같이 기록해야 한다.

## 오프라인 일치 시험 (2026-10-26, winpc)

수집 에피소드 run1024b-w1-001(학습 행 1,451개)에서 15행마다 하나씩, 97행을 노드 코드로 재생했다. 같은 행의 `MoraiDataset.sample(rng=None)`과 그 프로세서 배치에 대조했다.

| 항목 | gt 모드 (학습 신호, 96행) | deploy, r = 학습식 `GridYawRate` (기본, 95행) | deploy, r = IMU wz (95행) |
|---|---|---|---|
| 카메라 텐서 16장 | 비트 단위 동일 | 비트 단위 동일 | 비트 단위 동일 |
| 시각 텍스트 값 | 차이 0 | 차이 0 | 차이 0 |
| LiDAR BEV 2장 | 비트 단위 동일 | 4.2 % 행만 동일. 점유 셀 IoU 중앙값 0.970 | 다름. IoU 중앙값 0.985 |
| ego 이력 (r을 쓰지 않음) | 최대 1.4e-14 m | 최대 0.079 m, 중앙값 0.007 m. yaw 최대 0.015 rad | 같음 |
| nav text | 100 % 동일 | 91.6 % 동일. 나머지 8행은 거리 ±1–2 m 차이 | 같음 |
| input_ids / pixel_values | 100 % 동일 / 차이 0 | 91.6 % 동일(nav 차이 때문) / BEV 영상 차이 | 같음 |

- **LiDAR yaw rate 비교** (`tools/winpc/yawcmp.sh`, 같은 96행, 회전 행 = 학습 |yaw rate| > 0.1 rad/s인 28행)

  | r 출처 | BEV IoU 중앙값 | IoU 하위 10 % | 회전 행 IoU 중앙값 | r 오차 평균 / 최대 (rad/s) | 회전 중 r 오차 평균 |
  |---|---|---|---|---|---|
  | 학습식 + 마지막 값 유지 (`grid`) | 0.970 | 0.882 | 0.944 | 0.028 / 0.339 | 0.055 |
  | 학습식 + 추세 외삽 0.3 s (`grid_trend`, **기본**) | 0.980 | 0.914 | 0.960 | 0.014 / 0.180 | 0.026 |
  | 학습식 + 추세 외삽 0.6 s | 0.970 | 0.891 | 0.944 | 0.021 / 0.275 | 0.041 |
  | IMU wz (`imu`) | 0.985 | 0.947 | 0.957 | 0.010 / 0.183 | 0.020 |

  - 추세 외삽: 실시간에 없는 마지막 약 0.3 s의 yaw만, 최근 창(0.3 또는 0.6 s)의 yaw 직선 적합으로 채운다. 그다음 같은 학습식(중앙 차분 + 5탭)을 적용한다.
  - r 오차는 스캔의 패킷마다 잰 |r − 학습 r|(`_ego` yaw_rate를 패킷 시각에 보간한 값)다.
  - ego 이력은 r을 쓰지 않으므로 세 방식 모두 같다.
  - **결정 (10/26 사용자)**: 기본값은 학습식 + 추세 0.3 s(`~yaw_rate_source:=grid_trend`, `~yaw_trend_window_sec:=0.3`, `vlp16_bev/launch/vlp16_bev.launch`)다. 학습과 같은 식을 유지하면서, 실시간에 없는 마지막 0.3 s만 추세로 채운다. 학습식 변형 중 BEV 일치가 가장 높다. 다른 값은 같은 파라미터로 고른다.
- **두 모드**
  - gt 모드는 학습과 같은 신호(`_ego` v, yaw rate, path_s)를 넣어 코드가 같은지 본다.
  - deploy 모드는 대회 입력(competition_status, IMU, 스택 EKF)을 넣어 신호 차이의 크기를 본다.
  - LiDAR 입력은 두 모드 모두 bag의 `/velodyne_packets`(VelodyneScan)다. 스택도 같은 드라이버의 같은 메시지를 읽는다.
- **재생 시각**: bag은 벽시계이고 수집 때 시뮬레이터가 0.71–0.95배였다. 그래서 재생기는 step 0처럼 `_ego` 표로 stamp를 τ로 옮긴다.
- **추가 에피소드**: run1023h-w1-007의 5행도 gt 모드에서 비트 단위로 같았다.
- **재현**: winpc에서 `tools/winpc/parity.sh <episode>`를 돌린다. 내용은 `test/offline_replay.py`(chroot) → `test/offline_compare.py`(venv)다. 결과는 `~/morai-eval/data/parity/<ep>/report.json`에 남는다.

## 터널 구간 localization (2026-10-26)

LiDAR는 수집 때와 같은 velodyne_driver → velodyne_pointcloud → lidar_leveler → localization(tunnel_wall) 경로다.

run1023h-w1-007(CP14 출발)을 `test/bag_loop.sh`로 재생했다. 구간 15(s 1859.6–1996, 음영 + 터널)를 34 s에 지나고, GPS는 10.5–34 s 동안 끊긴다. LiDAR 패킷은 `test/udp_replay.py`가 재생 `/clock`에 맞춰 UDP 2368로 보내고, 드라이버가 받는다.

| 구간 15 | 출력 수 | 최대 공백 | 0.2 s 넘는 공백 합 | 위치 오차 중앙값 / 최대 |
|---|---|---|---|---|
| 우리 localization (3회) | 724–747 | 2.7–3.5 s | 18.9–19.3 s | 2.3–7.1 / 22.7–24.9 m |
| 수집 때 EKF (bag) | 905 | 2.2 s | 16.6 s | 2.5 / 27.1 m |

- **터널을 나온 뒤**: 위치 오차는 최대 5.7–6.9 m에서 줄어든다.
- **bag 자체의 끊김**: 15–20 s에는 bag의 모든 센서가 1.5 s까지 끊긴다. 수집 EKF도 이 구간에서 거의 출력하지 못했다.
- **재생 시각 맞춤**: 처음에는 패킷을 벽시계로 보냈다. 그러면 rosbag과 시작 시각이 매번 달라 점군과 IMU·status가 어긋났고, 그렇게 돌린 8회 중 3회는 EKF가 44–200 m로 발산했다. `/clock`에 맞춘 뒤로는 3회 모두 발산하지 않았다.
- **leveler 통과율**: lidar_leveler는 점군의 절반 정도만 통과시킨다(50 s에 171–183개). bag의 competition_status가 16 Hz라 leveler의 stale 기준 0.06 s에 자주 걸린다. 실주행에서는 98.6 %였다(아래).
- **nav 진행거리**: GPS가 끊긴 동안 모델 노드는 localization 출력을 쓰지 않고, 마지막 GPS 기반 위치에서 competition_status 주행거리로 이어 간다.
  - 위 표처럼 음영 구간 위치는 23–27 m까지 표류하기 때문이다.
  - 시뮬 시계로 맞춘 `test/route_replay.py`, CP14 출발 4개 에피소드의 음영 구간 진행거리 오차:

  | 에피소드 | 모든 위치 사용 (중앙값 / 최대 m) | GPS 끊김 시 주행거리 (중앙값 / 최대 m) |
  |---|---|---|
  | run1023h-w1-007 | 17.3 / 27.9 | 5.0 / 17.9 |
  | run1024a-w1-009 | 1.8 / 9.9 | 3.2 / 9.9 |
  | run1024a-w1-004 | 1.4 / 2.7 | 1.6 / 3.1 |
  | run1023g-w1-005 | 0.5 / 1.3 | 1.0 / 1.5 |

  - 음영 구간의 nav text는 네 에피소드 모두 학습값과 같았다. 이 구간 150 m 안에는 기동이 없다.
  - 에피소드 전체의 nav text 일치는 83–88 %다. 나머지는 거리만 ±1–2 m 다른 경우(10–16 %)와, 한 바퀴 끝·시작이 겹치는 지점에서 잠깐 튄 경우(0.2–2.9 %)다.
- `bag_loop.sh`는 벽시계로 재생한다. 그래서 RTF 0.88 bag에서는 주행거리 적산이 약 13 % 길게 나온다. 진행거리 오차는 그래서 시뮬 시계 재생(`route_replay.py`)으로 쟀다.

## bag 재생 배선 시험 (MORAI 없이, 2026-10-26)

`test/bag_loop.sh`는 chroot 안에서 roscore를 `/use_sim_time`으로 띄운다. 브릿지를 뺀 스택 노드를 띄우고, run1024b-w1-001의 30–90 s 구간을 `rosbag play --clock`으로 재생한다. LiDAR 패킷은 `test/udp_replay.py`가 UDP 2368로 다시 쏜다(이 측정은 velodyne 경로를 되돌리기 전의 것이다). 모델 서버는 가짜 백엔드였고, 전처리는 실제와 같이 돌렸다.

| 항목 | 값 |
|---|---|
| 모델 스텝 | 60 s에 583회 (9.7 Hz) |
| `/control_cmd` → speed_filter 출력 | 59.7 Hz → 58.7 Hz, 지연 중앙값 0 ms |
| 카메라 수신 → `/control_cmd` | 중앙값 132 ms, p95 203 ms |
| 그중 t0의 최신 카메라 나이 | 중앙값 53 ms (bag 카메라가 10 Hz라서) |
| 그중 노드 요청 조립 (step-0 JPEG 재인코딩) | 중앙값 20 ms, p95 40 ms |
| 그중 서버 (전처리 48 ms) | 중앙값 61 ms |

- **카메라 간격 경고**: Cam7 등에서 프레임이 목표 시각과 0.08 s 넘게 떨어진 경고가 난다. bag 카메라가 10 Hz라 생기는 것이다. 실시간 브릿지는 더 빠르다.
- **health check 경고**: localization과 speed_filter가 초당 약 1회 `pipeline Rejected: health check limit`을 낸다. bag의 GPS(15 Hz)가 stale 기준 0.1 s에 걸리는 것으로 본다. speed_filter가 이 때문에 버린 명령은 1.7 %였다.

## MORAI 실주행 시험 (2026-10-26 02:40–02:47, W1)

- **환경**: W1 사본(`C:\morai\multi-03-20261007`, 수집 때 센서·네트워크 프리셋)을 RTX 5080에서 띄웠다. 두 대를 같이 돌리면 CPU가 포화되므로, 원본 시뮬레이터는 시험 동안 내렸다가 다시 올렸다.
- **장면**: `rs2041_001/62`(CP14 출발, 음영 + 터널).
- **측정**: `tools/winpc/probes.sh` → `test/live_probe.py`, `latency_probe.py`. GT는 판정용 Ego status를 측정에만 썼고 스택에는 넣지 않았다.
- **LiDAR yaw rate**: `grid_trend`.

| 시험 | 결과 |
|---|---|
| 가짜 모델, 목표 4 m/s, κ = 0 (70 s) | 차가 움직였다: 110.8 m, 최고 4.6 m/s, control_mode 2, 적용 accel 최대 0.26 |
| 출발 보조 (가짜 모델 목표 0, 53 s 정지) | 적용 accel 최대 0.0, 이동 0 m |
| 작은 학생 체크포인트 `dry/deliver/final` (QAD, HF 백엔드, 40 s) | 계획 29개. 모델 1.25 s/스텝(eager fake-quant)이라 계획 만료(0.5 s)에 걸려 브레이크가 유지됐다 |

- **센서 rate (주행 중)**: competition_status 18.5 Hz, 전방 카메라 16.6 Hz, LiDAR 스캔 7.3 Hz, localization 46 Hz.
- **lidar_leveler 통과율**: 98.6 %다. bag 재생의 약 50 %는 bag의 16 Hz status 탓이었다.
- **localization**: GPS가 61.7 s 끊긴 동안 최대 공백 1.4 s(0.2 s 넘는 공백 합 6.0 s), 위치 오차 중앙값 3.4 m, 최대 22 m(구간 15). nav 진행거리 오차는 중앙값 6.8 m, 최대 25 m이고, nav text는 100 % 맞았다(구간 내 기동 없음).
- **RTF**: 0.83이다(GT 이동거리 / 속도 × 벽시계).
- **지연 (가짜 모델)**: 카메라 수신 → `/control_cmd` 중앙값 265 ms, p95 494 ms다.
  - 구성: t0의 최신 카메라 나이 45 ms, 노드 요청 조립(step-0 JPEG 재인코딩) 98 ms, 서버 120 ms(전처리), `/control_cmd` → speed_filter 0.3 ms, → 브릿지 UDP 송신 17 ms.
  - 모델 스텝은 4.6 Hz였다. 시뮬레이터와 같은 CPU에서 노드 재인코딩과 서버 전처리(PIL, Qwen image processor, 토크나이저)가 느려진다. 10 Hz를 내려면 이 둘을 줄여야 한다.
- **지연 보정 스텝**: 명령은 계획의 t0부터 흐른 시간 + 0.1 s(적용 지연)의 스텝을 읽는다. 그래서 계산 지연이 자동으로 반영된다. 이번처럼 계획이 t0 뒤 약 220 ms에 오면 첫 명령은 스텝 3을 쓴다. 고정 스텝으로 바꾸지 않았다.

## 지연 줄이기 (1dc09fe)

W1 실주행의 4.6 Hz 병목(노드 재인코딩 98 ms, 서버 전처리 약 120 ms)을 다음처럼 줄였다. 학습 함수와 계산 순서는 그대로 두고, 같은 입력의 결과를 다시 쓰거나 일을 겹치게만 했다.

- **노드**: 카메라마다 작업 스레드가 프레임이 들어오는 즉시 step-0 재인코딩을 해 둔다(`Inputs(prefetch=True)`). 요청을 만들 때는 캐시를 읽기만 한다(조립 98 → 1–2 ms).
- **노드 ↔ 서버**: 요청에 `id`를 붙여 응답을 기다리지 않고 보낸다. 동시에 2개까지 보내고(`max_inflight`), 서버가 밀리면 그 틱을 건너뛴다.
- **서버**: 받기, CPU 전처리, GPU 실행을 각각 스레드로 나눠 연속 요청을 겹친다. 이미지별 결과를 내용 해시(blake2b)로 캐시한다.
  - 캐시 대상은 JPEG 디코드, `pack_bev`, 이미지 프로세서 출력(`CachedImageProcessor`)이다. 연속 요청은 카메라 16장 중 12장을 공유한다.
  - 프롬프트 조립과 토큰화는 매번 학습 함수(`get_preprocess_fn`, `collate_fn`)로 한다.
- **서버 프로세스 설정** (`server.sh`): torch 스레드 4개(`OMP_NUM_THREADS=4`)와 힙 전용 malloc(`MALLOC_MMAP_MAX_=0`, `MALLOC_TRIM_THRESHOLD_=1 GiB`)을 쓴다.
  - 전처리 시간의 큰 몫은 35 MB pixel_values 복사 세 번이다(캐시 합치기, 학습 processor, 레시피 collate). 새 mmap 메모리의 페이지 폴트가 복사를 두 배로 느리게 했다.
  - 기본 16 스레드는 시뮬레이터와 CPU를 다퉜다(서버 CPU 787 %).
  - 시뮬레이터 옆 벤치에서 서버 왕복이 75 → 58 ms(p95 118 → 71 ms)로 줄었다.
  - HF 백엔드에서는 전처리가 146 → 82 ms로 줄었고, 모델 시간은 그대로다(513 → 515 ms, GPU를 시뮬레이터와 나눠 씀).
  - 계산은 바뀌지 않는다. 이 설정으로 gt 96행과 연속 41행을 다시 대조해 비트 단위로 같았다.
- **확인**: gt 모드 비트 단위 일치를 표준 96행과 연속 41행(캐시가 맞는 경우)에서 다시 확인했다(`offline_compare.py`). 10/26 두 PC 시험 뒤와 서버 설정 변경 뒤에도 다시 돌렸다.
- **한 PC 실주행** (10/26 04:55, W1 사본 + 스택 + 서버 모두 winpc, 가짜 모델, `rs2041_001/62`, 90 s):

  | | 이전 (W1, 02:40) | 지금 |
  |---|---|---|
  | 모델 스텝 | 4.6 Hz | 9.8 Hz 평균. 간격의 99.1 %가 0.100 s이고, 5 s 창 대부분이 10.0 Hz다 |
  | 카메라 수신 → `/control_cmd` | 265 ms (p95 494) | 114 ms (p95 159) |
  | 노드 조립 | 98 ms | 4 ms |
  | 서버 왕복 | 120 ms | 73 ms (p95 99). 전처리 59 ms, 조립 11 ms |
  | RTF, 전방 카메라, LiDAR | 0.83, 14–17 Hz, 7 Hz | 0.85, 14.9 Hz, 7.2 Hz |

  - 평균이 10 Hz에 못 미치는 것은 0.3 s 넘는 공백 5번(합 3.0 s) 때문이다. 모두 W1 시뮬레이터가 약 0.6 s 멈춘 때다. 공백 직후 카메라 선택이 목표에서 0.4–0.6 s 벗어났고, 같은 때 서버 왕복도 600 ms대로 튀었다.
  - localization은 GPS 음영 65 s 동안 오차 중앙값 1.6 m, 최대 19 m였다. nav text는 100 % 맞았다.

## 두 PC 구성: bc-250 MORAI + winpc 스택 (2026-10-26)

```
bc-250 (MORAI 25.S4 Linux, 192.168.0.15)                 winpc (WSL, RTX 5080)
 Simulator ─UDP─> 127.0.0.1:9291… ─┐                 ┌─> 127.0.0.1:<같은 포트> ─> 브릿지, velodyne_driver, …
                                   lan_tunnel.py sim ══TCP 30011══ lan_tunnel.py stack
 Simulator <─UDP─ 127.0.0.1:9093 <─┘   (유선 LAN)     └─< 127.0.0.1:9093 <─ 브릿지 제어(speed_filter 뒤)
```

- **프리셋**: bc-250에 W1 사본(수집 때)의 기본 프리셋과 장면을 그대로 복사했다. 모든 출력은 127.0.0.1의 수집 때 포트로 나간다.
  - 복사한 파일(md5): `Sensor/25.S4.MolitComp03/SensorInfo_2023_Hyundai_Ioniq5.json`(ee321a03), `Network/…/NetworkInfo_2023_Hyundai_Ioniq5.json`(1d912cbb), `Scenario/R_KR_PR_K-city_2025/{rs2041_001,clcol}.json`과 `EgoNetwork/*_MN.json`.
  - 바꾸기 전 bc-250의 `SaveFile` 전체를 `~/morai-backup-20261026/SaveFile.tgz`에 백업했다.
  - 맵을 다시 읽은 뒤 시뮬레이터 소켓이 W1 포트(9290…, 9094, 9098, 7605 등)로 바뀐 것을 확인했다.
  - 시뮬레이터는 맵을 로드할 때 차량 기본 파일(`NetworkInfo/SensorInfo_<차량>.json`)을 다시 쓴다. 10/26 05:11 맵 로드 뒤 NetworkInfo md5가 59e9736a로 바뀌었다.
  - 더 안전한 방법은 W1 프리셋을 다른 이름으로 두고, 맵 로드 뒤 `msc.py $L load-network <파일>`과 `load-sensors <파일>`로 올리는 것이다. 아직 시험하지 않았다(옵션 형식 미확인).
- **터널** (`tools/lan_tunnel.py`): 시뮬레이터 쪽 루프백의 UDP를 TCP 하나로 묶어 스택 쪽 루프백에 그대로 다시 쏜다. 제어(9093)는 반대 방향으로 보낸다.
  - 프리셋의 목적지 IP를 바꾸지 않아도 된다. WSL은 LAN에서 들어오는 연결을 받지 못하므로, bc-250이 listen하고 WSL이 connect한다.
  - 경로는 WSL NAT → Windows → 유선 LAN이다(Tailscale 아님). 왕복은 0.87 ms다.
  - Ego status(9095)는 측정용 GT 수신기도 받도록 127.255.255.255로 다시 쏜다.
- **MSC** (`tools/msc.py`): bc-250 런처의 `~/msc.py`(status, execute, map)에 pause, play, load-scene을 더한 사본이다.
- **주의**: 두 PC 시험 중에는 winpc에서 시뮬레이터를 띄워 두지 않는다.
  - 맵만 띄운 원본 시뮬레이터가 Windows CPU의 41 %와 5080을 써서, 서버 전처리가 42 → 92 ms로 늘었다.
  - 그 상태에서는 GT 수신값도 섞였다(RTF 1100, 이동거리 633 km로 나옴). 시험 동안 원본 시뮬레이터를 내렸다가 다시 올렸다.

### 결과 (장면 `rs2041_001/62`, 가짜 모델 목표 4 m/s, κ = 0)

모델 스텝은 시뮬레이터가 입력을 주는 동안 **10.0 Hz**다(`/e2e_driver/debug` 1667 스텝).

- 스텝 간격의 99.0 %가 0.100 s다.
- 0.3 s 넘는 공백은 14번(합 17.9 s)이고, 모두 bc-250 시뮬레이터가 멈춘 때다. 그때는 입력 창(2.5 s)이 비어 노드가 요청을 만들지 않는다(`camera front: 1 frames`, `lidar: 3 scans`).
- 공백을 빼면 1652 스텝 / 164.9 s = 10.0 Hz다. 공백을 넣은 평균은 9.1 Hz다.
- 서버가 밀려 건너뛴 틱은 7개다.

| 구간 (두 PC, 가짜 모델) | 중앙값 | p95 |
|---|---|---|
| t0의 최신 카메라 나이 (시뮬 송신 → 브릿지 수신 → 틱 대기) | 50.5 ms | 870 ms* |
| 노드 요청 조립 (재인코딩은 미리 해 둠) | 1.5 ms | 10.4 ms |
| 서버 왕복 | 53.6 ms | 63.6 ms |
| ├ 서버 조립 (JPEG 디코드, BEV, 이력) | 6.0 ms | 9.3 ms |
| ├ 전처리 (프로세서, 토큰화, collate) | 42.2 ms | 49.3 ms |
| └ 모델 (가짜) | 0.4 ms | |
| `/control_cmd` → speed_filter 출력 | 0.23 ms | 10.8 ms |
| speed_filter 출력 → 브릿지 UDP 송신 | 3.4 ms | 19.8 ms |
| 브릿지 → bc-250 (터널, LAN) | 왕복 0.87 ms | |
| **카메라 수신 → `/control_cmd`** | **106 ms** | 918 ms* |

\* p95와 최대값은 bc-250 시뮬레이터가 멈춘 구간(최대 4 s)에서 나온다.

- **HF 백엔드** (작은 학생 체크포인트 `dry/s2/final`, BF16, 10 스텝):
  - 5080 단독으로 오프라인 벤치를 돌리면 모델이 207 ms, 4.7 Hz다. W1과 GPU를 나눠 쓸 때는 509 ms였다.
  - 두 PC 실주행에서는 모델 271 ms, 서버 왕복 498 ms, 카메라 → 명령 553 ms, 약 3.5 Hz다.
  - 이 체크포인트는 출발하지 않았다(최고 0.06 m/s).
  - 10 Hz를 내려면 엔진 백엔드(NVFP4 89 ms 예상)가 필요하다.
- **서버 처리량** (실주행 요청 덤프, 20 Hz로 보냄): 가짜 모델은 20.0 Hz, 왕복 48 ms다.
- **bc-250 시뮬레이터** (이 시험 당시, GPU 100 MHz): RTF 0.67–0.70, 3–10 s마다 0.6–4.5 s 멈춤이 있었다. 아래 '최종 bc-250 설정'에서 해결했다.
- **localization** (GPS 음영 89 s): 위치 오차 중앙값 8.7 m, 최대 50 m, 최대 공백 4.6 s다. W1(중앙값 3.4 m, 최대 22 m)보다 나쁘다. 시뮬레이터 멈춤 때문에 LiDAR가 5 Hz로 줄고 공백이 생긴 탓으로 본다.
- **nav text**: 100 % 맞았다.

### 최종 bc-250 설정과 1시간 결과 (10/26, morai 세션 측정)

- **설정**: 시뮬레이터 `-force-vulkan -force-gfx-mt`, 카메라별 바쁜 대기 스레드를 CPU 10–11에 고정, GPU 거버너 1100 MHz@880 mV.
  - 자세한 내용과 되돌리는 방법은 공유 노트 `bc250-morai-host-20261026.md`에 있다.
  - 시뮬레이터 프로세스 이름은 `Simulator.real`이다(`Simulator.x86_64`는 인자를 붙이는 래퍼).
- **겪은 문제와 결과**:

  | 설정 | RTF | 멈춤 (0.5 s 이상) | 비고 |
  |---|---|---|---|
  | GPU 100 MHz (거버너 없음) | 0.67–0.70 | 3분에 16번 | 위 결과 |
  | 1000 MHz, Vulkan + native gfx jobs | 0.29 | 15분에 117번, 최장 11 s | 카메라가 GPU 메모리를 분당 0.2 GB씩 새게 함 → GTT가 차서 스왑 |
  | OpenGL | – | – | 비동기 리드백이 없어 카메라·LiDAR가 나오지 않음 |
  | 1100 MHz, Vulkan + gfx-mt (최종) | 0.609 (분당 0.58–0.62) | 1시간에 3번, 합 6.1 s | 리셋 없음, 평균 64 W |
  | 1400 MHz | – | – | 2분 만에 박스 리셋 |

- **최종 1시간 결과** (`bc1100h2`, 09:20–10:20, 가짜 모델 경로 추종):
  - 모델 스텝 9.95 Hz, 카메라 → 명령 중앙값 91 ms(p95 144 ms).
  - 카메라는 해상도, JPEG 양자화 표, 4:2:0이 W1 수집 bag과 같다. 시뮬레이션 시간 기준 센서 주기도 W1과 같다.
  - 다른 점은 날씨(하늘의 구름)다. 날씨는 센서가 아니라 장면 환경이다. 수집 때는 에피소드마다 gRPC SetWeather로 정했다. bc-250도 gRPC 7789를 열므로, 맞추려면 주행 시작 때 SetWeather를 보낸다(아직 안 함).
- **스폰 직후 같은 곳에서 멈추는 문제**: 이 장면은 약 16 s 달린 뒤 route_s 약 1859에서 막혀 1분마다 재로딩됐다.
  - 원인은 RTF 0.5–0.6에서 GPS 음영 구간의 localization과 노드 주행거리계가 시뮬레이션 속도를 벽시계 시간으로 적분하는 것이다. 위치가 1/RTF배 앞서 나간다(15 s에 실제 42 m, localization 80 m).
  - 시뮬레이터가 보내는 시각(competition_status source, 카메라 패킷)은 모두 bc-250 시스템 시계라서, 시뮬레이션 시간으로 바꿀 수도 없다.

### 반복 주행과 리셋 (강화학습 대비, 10/26 05:30–06:30, W1으로 도구 확인)

- **15분 주행 부하**: 가짜 모델에 경로 추종 곡률을 준다. 노드의 `fake_follow_route_m`(기본 0, 꺼짐)을 켜면 localization 자세에서 8 m 앞 경로점으로 pure pursuit 곡률을 계산해 요청에 싣는다.
  - 이 값은 가짜 백엔드만 쓴다. 서버가 토큰화 뒤에 붙이므로 모델 배치는 바뀌지 않는다.
  - 장애물·NPC 인지는 없다. 그래서 `STUCK_S`(10 s) 동안 서 있으면 장면을 다시 불러온다.
- **리셋 순서**: pause → load-scene → localization 재시작 → 4 s 대기 → play.
  - localization 필터는 순간이동을 따라가지 못한다. 재시작 없이 시험하면 재로딩 뒤 15 s 동안 36 m, 그 뒤에도 8 m가 어긋났다.
  - 그래서 `localization.launch`에 `respawn`을 켰다. 리셋 때 `rosnode kill /localization_node`로 새 GPS fix에서 다시 시작한다.
  - 나머지 스택은 그대로 둔다. 재로딩 동안에도 노드는 10 Hz로 요청을 보낸다(일시정지 중에도 센서가 나온다).
- **프리셋**: 시뮬레이터는 맵을 로드할 때 차량 기본 파일을 다시 쓴다. 값은 W1과 같았고 md5만 바뀌었다.
  - 그래도 W1 프리셋을 `w1_collect.json`(Network, Sensor)으로 따로 두고, 맵 로드 뒤 MSC로 올린다.
- **W1 결과** (가짜 모델, 같은 PC):

  | | 15분, 막힐 때만 리셋 | 15분, 2분마다 리셋 |
  |---|---|---|
  | 모델 스텝 | 9.99 Hz (간격 99.8 %가 0.100 s, 공백 0) | 9.98 Hz (99.7 %, 공백 0) |
  | 카메라 → 명령 중앙값 (p95) | 122 ms (179) | 120 ms (183) |
  | 서버 왕복 중앙값 (p95) | 75 ms (99) | 73 ms (99) |
  | 주행거리, 리셋 | 4.4 km, 1번 | 3.2 km, 5번 |
  | 리셋 뒤 localization 재동기화 | 0 s | 0–9.4 s (대기 4 s 포함) |
  | 리셋 3 s 뒤 경로 진행거리 | 1854 (스폰 1858) | 1850–1851 (스폰 1847) |
  | RTF, 시뮬레이터 멈춤 (0.5 s 이상) | 0.76, 1번 | 0.68, 0번 |
  | localization 오차 중앙값 / p95 / 최대 | 0.2 / 24 / 242 m | 1.0 / 58 / 237 m |

  - 두 번을 연달아 돌려도 문제가 없었다. 서버 메모리는 1.82 GB 그대로였고, 노드는 225 → 241 MB였다.
  - localization이 가끔 수십–수백 m씩 튄다. 메시지 사이 5 m 넘는 점프가 5분에 561번이었고, section 판정은 15분에 33–38번 바뀌었다. 아래 '남은 문제'에 적었다.

## 제어 변환 (`control.py`)

expert는 64 × (a, κ)를 낸다. a는 종가속도(m/s²), κ는 경로 곡률(1/m, 좌 +)이고 0.1 s씩 유지된다(`UnicycleAccelCurvatureActionSpace`). 시각 t에는 t + 0.1 s의 단계를 읽는다.

- **조향**: δ = atan(L·κ / g), steering = δ / 40°이고 [−1, 1]로 자른다.
  - L = 3.0 m: competition_status `wheelbase_m`.
  - 40°: 적용된 front_steer가 명령 × 40°와 정확히 같고, 0.1 s 늦다. status 각도는 우 +이고, 명령 +1은 좌회전이다.
  - g = 0.77: κ = g·tan δ / L의 적합값이다. 세 에피소드에서 0.782, 0.753, 0.757(R² 0.96–0.97)이 나왔고, control 패키지 MPC의 `steering_prediction.gain`은 0.75다.
- **페달**: 적용 페달에 대한 dv/dτ를 표로 잡고, 측정 속도에서 표를 역으로 읽는다.
  - 가속 표는 저속(4.5 m/s 아래)과 순항(7.5 m/s 위) 두 줄이고, 그 사이는 선형으로 섞는다. 제동 표는 속도와 무관하다. 페달 0은 타력 주행이다.
  - 적용 페달은 명령보다 약 0.1 s 늦고, 가속도는 적용 페달을 지연 없이 따른다.
  - 측정은 `test/fit_vehicle.py`로 했다(run1024b-w1-001, run1024a-w1-002/003, 시뮬 시계 약 460 s).

  | 가속 페달 | 0 | 0.06 | 0.15 | 0.25 | 0.35 | 0.45 | 0.55 | 0.67 |
  |---|---|---|---|---|---|---|---|---|
  | 3–6 m/s (m/s²) | −0.55 | 0.0 | 0.51 | 0.94 | 1.40 | 2.18 | 2.58 | 3.45 |
  | 6–15 m/s (m/s²) | −0.40 | 0.0 | 0.27 | 0.48 | 0.87 | 1.08 | 1.37 | 1.75 |

  | 제동 페달 | 0.06 | 0.15 | 0.25 | 0.38 | 0.52 | 1.0 (외삽) |
  |---|---|---|---|---|---|---|
  | m/s² | −1.1 | −1.95 | −2.6 | −6.1 | −9.1 | −18.8 |

  - control 패키지 MPC의 선형 상수(가속 5.74, 제동 18.76 m/s²/페달)는 이 데이터와 맞지 않아 쓰지 않았다. 선형 적합하면 3.39와 12.3–13.2가 나온다.
- **정지 유지**: 측정 속도와 앞으로 1 s의 계획 속도가 모두 0.3 m/s 아래면 brake를 0.5 이상으로 둔다.
- **계획 만료**: 0.5 s 넘게 새 계획이 없으면 풀브레이크를 낸다. 브릿지도 명령이 0.25 s 끊기면 스스로 풀브레이크를 건다.
- **speed_filter 출발 보조 끔**: `test/start_assist_check.py`는 status 속도 0, `/control_cmd` accel 0을 36 s 보낸다.
  - 원래 설정(30 s)에서는 정지 30.02 s 뒤 accel 0.2가 1 s 나갔다.
  - `1.0e9`로 바꾼 뒤에는 accel을 민 출력이 0개였다.

## 실행 (winpc WSL)

```bash
# Mac: 이 브랜치를 winpc로 복사 (.git, 큰 모델 파일 제외)
rsync -a --delete --exclude .git --exclude /build --exclude /devel --exclude __pycache__ \
  --exclude src/object_detector/models --exclude src/traffic_light/models ./ winpc-wsl:morai-eval/ws/MORAI-Autonomous-Driving-System/
```

```bash
# winpc WSL
cd ~/morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/winpc
./build.sh                                                     # chroot catkin_make, 1분
./server.sh up --backend hf --ckpt <학생 체크포인트 dir>        # 바꿀 것은 이 경로뿐
./stack.sh up                                                  # run.sh: bridge, vlp16_bev, localization, section_annotator, e2e_driver, speed_filter
./chroot.sh probe 'source devel/setup.bash; rosrun e2e_driver latency_probe.py _out:=/data/latency.json'   # 지연 측정, Ctrl-C로 저장
./stack.sh down; ./server.sh down
```

- **시뮬레이터**: W1(`C:\morai\multi-03-20261007`)을 쓴다. 출력은 모두 127.0.0.1로 나가고, UDP-over-TCP 터널로 WSL에 들어온다.
  - 시뮬레이터 시작, 터널, scene 로드는 `morai-parallel-collect` 스킬의 `parallel.sh`를 쓴다(`STACKS=wsl tunnels`, `msc w1 load-scene clcol/62`).
  - 이 스택은 수집 스택(win-1) 대신 WSL 호스트 네트워크에 뜬다. 둘을 동시에 띄우지 않는다.
- **두 PC (기본, `tools/two_pc.sh`, Mac에서 실행)**: `SIM=bc250`이 기본이다. `SIM=w1`이면 winpc의 W1 사본을 쓴다.
  ```bash
  tools/two_pc.sh up        # bc-250 lan_tunnel(sim) + WSL lan_tunnel(stack), 가짜 서버(경로 추종, 7 m/s), 스택
  tools/two_pc.sh prep      # bc-250: 맵 → load-network/load-sensors w1_collect → 장면 rs2041_001/62 (일시정지)
  tools/two_pc.sh drive <이름> 900            # 15분 주행 + 측정 → ~/morai-eval/data/summary_<이름>.json
  EPISODE_S=120 tools/two_pc.sh drive <이름> 900   # 강화학습식: 2분마다 리셋
  tools/two_pc.sh reset     # 리셋 한 번: pause → load-scene → localization 재시작 → play
  tools/two_pc.sh down
  ```
  - 실제 학생 모델은 `tools/two_pc.sh up --backend hf --ckpt <dir>`로 띄운다. 이때는 `FAKE_FOLLOW_ROUTE_M=0`으로 경로 추종 값을 끈다.
  - 시험 중에는 winpc에서 시뮬레이터를 띄우지 않는다(위 '주의').
- **체크포인트**: `StudentVLA.build(init=dir)`로 읽는다. QAD 체크포인트(`modelopt_state.pth`)는 `~/morai-student/pylib`의 nvidia-modelopt를 쓴다.
  - 교사는 `--model-class teacher --vlm Qwen/Qwen3-VL-8B-Instruct`로 띄운다. 다만 BF16 23 GB라 5080에 올라가지 않는다.
- **백엔드 교체**: `server/backends.py`에 `predict(sample) -> {accel, kappa, v0}`를 구현하는 클래스를 추가하면 된다. 이후 NVFP4 엔진을 붙일 자리다.
- **가짜 모델**: `--backend fake --preprocess-config <config.json dir>`는 모델 없이 속도 유지 법칙(기본 4 m/s, κ = 0)으로 계획을 낸다. 전처리와 토큰화는 실제와 똑같이 돌려 지연을 잰다.

## 남은 문제

- **지연**: 가짜 모델 기준 카메라 → 명령은 두 PC에서 106 ms, 한 PC에서 114 ms다. 서버 왕복은 각각 54 ms, 73 ms다. 서버는 CPU 전처리와 GPU 실행을 겹치므로, 스텝 주기는 둘 중 긴 쪽이 정한다.
  - 한 PC에서는 시뮬레이터가 5080을 함께 쓴다. 그래서 GPU 모델 시간이 크게 늘어난다(HF dry/s2: 207 → 515 ms).
  - 엔진 모델(NVFP4 89 ms 예상)이면 10 Hz 안에 들어오지만 여유가 작다.
  - HF 백엔드는 작은 체크포인트로도 207–271 ms라 10 Hz가 안 된다.
- **RTF < 1 (벽시계 적분)**: W1은 0.68–0.85, bc-250 최종 설정은 0.61이다. 스택 전체(localization, 노드 주행거리계, 이력 적분, 시각 텍스트)가 벽시계로 돈다.
  - 그래서 GPS 음영 구간에서 위치와 진행거리가 1/RTF배 앞서 나가고, 모델 입력도 학습 분포(τ)에서 벗어난다.
  - 강화학습이나 정밀 평가에는 RTF를 1 가까이 올리거나, 동기(tick) 모드를 쓰거나, 주행거리 배율을 GPS로 온라인 추정해야 한다. 아직 정하지 않았다.
- **학생 BF16 메모리**: 학생 BF16은 13.4 GiB다. 시뮬레이터와 같은 5080(16 GB)에서는 HF 백엔드로 돌리기 어렵다. 실제 학생은 NVFP4 엔진 백엔드가 필요할 수 있다.
- **RTF**: winpc에서 시뮬레이터와 모델이 함께 돌면 RTF가 1보다 낮을 수 있다. 그러면 시각 텍스트와 이력 적분이 학습 분포에서 벗어난다.
- **localization 점프**: 경로를 계속 달리면 출력이 메시지마다 수십–수백 m씩 오가는 구간이 생긴다(W1 15분 p95 24–58 m, 최대 242 m). bc-250 10분 주행에서는 비정상 값(약 1.9e17 m)도 한 번 나왔다.
  - section 판정이 자주 바뀌는 것으로 보아, full_ekf와 tunnel_wall core가 번갈아 선택되는 되먹임이 의심된다. section은 localization 자세로 정하고, GPS Missing 규칙도 core를 바꾼다.
  - nav 진행거리는 GPS가 있을 때 localization fix를 받으므로 nav text가 틀릴 수 있다. 아직 고치지 않았다.
- **터널·GPS 음영**: localization(full_ekf / tunnel_wall)은 음영 구간에서 수집 때처럼 23–27 m 표류한다. 재생 시각이 어긋났던 시험에서는 수백 m로 발산했고, 발산 뒤에는 GPS가 돌아와도 바로 복귀하지 않았다. nav 진행거리는 GPS가 끊긴 동안에만 주행거리로 버티므로, 발산이 GPS 복귀 뒤까지 이어지면 nav text가 틀린다.
- **한 바퀴 끝**: s ≈ 2184/0 겹침 지점에서 경로 추적이 잠깐(샘플의 약 0.6 %) 엉뚱한 구간으로 튄다. 학습 GT path_s에도 같은 지점의 튐이 있다.
