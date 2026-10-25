# e2e_driver: 학생 E2E 모델 폐루프 평가

학습한 학생 모델(와우마요: Qwen3-VL-4B + 이식 expert)이 MORAI를 직접 운전하는 평가 스택이다.
브랜치는 `eval/claude-closedloop-20261025`이고, `origin/main` @79df2c0에서 갈라졌다.

```
MORAI ──UDP──> MORAI-UDP-Bridge-Node ──(카메라 4, GPS, IMU, competition_status)──┐
MORAI ──UDP 2368 (VLP16)──> vlp16_bev ──/lidar_bev/scan──────────────────────────┤
                       localization ──/localization/ego (경로 진행거리)───────────┤
                                                                                  v
                     e2e_driver_node ──TCP──> model_server (GPU, chroot 밖 venv)
                            │
                       /control_cmd (morai_msgs/CtrlCmd)
                            v
                       speed_filter ──/morai_udp_bridge/control_cmd──> 브릿지 ──UDP 9093──> MORAI
```

- **직렬 경로**는 MORAI → 브릿지 → 모델 → speed_filter → 브릿지 → MORAI다. LiDAR만 브릿지를 거치지 않는다.
- **곁가지 노드**
  - localization과 section_annotator가 경로 진행거리와 speed_filter의 섹션을 준다.
  - `run.sh`는 이 체인에 필요한 패키지만 빌드하고 띄운다. path_planner, control, traffic_light, velodyne은 띄우지 않는다.
- **velodyne**은 서브모듈째 뺐다. 다른 패키지 중 velodyne에 빌드 의존이 있는 것은 없다.
  - lidar_leveler와 localization의 tunnel_wall Core는 `/velodyne_points/leveled`를 구독하지만, 이 스택에서는 둘 다 쓰지 않는다.

## 구성

| 위치 | 하는 일 |
|---|---|
| `src/vlp16_bev` | VLP16 UDP 패킷을 직접 받는다. 76개씩 묶어 스캔 하나를 만들고, 학습 코드 `lidar_bev.scan_bev_stream`으로 스캔별 BEV를 만든다(`interfaces/LidarBevScan`) |
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
| LiDAR | velodyne_driver의 76패킷 스캔. 패킷 시각에서 `_ego` v, yaw rate를 샘플링해 `scan_bev_stream` | 같은 76패킷 묶음과 패킷 수신 시각. v는 competition_status `signed_velocity_mps`, r은 IMU `angular_velocity.z`. 서버가 `pack_bev`로 2장을 만든다 |
| ego 이력 | `_ego` τ 격자의 GT v, yaw를 `ego_trajectory`로 적분 | competition_status 속도와 yaw를 t0 − 1.5 … t0에 선형 보간하고, 서버가 같은 `ego_trajectory`로 적분한다 |
| nav text | scorer `KMap.progress`의 path_s(GT base_link 자세)를 `build_index.nav_text`로 | `/localization/ego`(같은 map frame의 EKF)를 `KMap.progress`와 같은 창 추적으로 처리한다. 위치가 끊기면(GPS 음영, 터널) competition_status 주행거리로 진행한다 |
| 토큰화 | `get_preprocess_fn(generation_mode=True)` + `collate_fn` | 같은 함수(`server/assemble.py`) |

- **신호 근거**(run1024b-w1-001 bag)
  - competition_status 속도와 GT 속도의 차이는 중앙값 0.0002 m/s다.
  - competition_status yaw는 GT yaw보다 상수 0.0235 rad 작다. 상대 heading에서는 상쇄된다.
  - IMU wz는 GT yaw rate와 상관이 0.99이고 부호가 같다. competition_status의 `angular_velocity_z_radps`는 부호가 반대라 쓰지 않는다.
- **시간**
  - bag의 모든 stamp는 벽시계다. packet 안의 MORAI 시각도 벽시계 속도로 간다(d/dt = 1.000).
  - 그래서 추론은 벽시계를 τ로 쓴다. 대회는 Time Scale 1을 전제한다.
  - 시뮬레이터가 실시간보다 느리면(RTF < 1) 시각 텍스트와 적분이 그만큼 어긋난다. 평가할 때 RTF를 같이 기록해야 한다.

## 오프라인 일치 시험 (2026-10-26, winpc)

수집 에피소드 run1024b-w1-001(학습 행 1,451개)에서 15행마다 하나씩, 97행을 노드 코드로 재생했다. 같은 행의 `MoraiDataset.sample(rng=None)`과 그 프로세서 배치에 대조했다.

| 항목 | gt 모드 (학습 신호, 96행) | deploy 모드 (대회 입력, 95행) |
|---|---|---|
| 카메라 텐서 16장 | 비트 단위 동일 | 비트 단위 동일 |
| 시각 텍스트 값 | 차이 0 | 차이 0 |
| LiDAR BEV 2장 | 비트 단위 동일 | 다름. 점유 셀 IoU 중앙값 0.985 |
| ego 이력 | 최대 1.4e-14 m | 최대 0.079 m, 중앙값 0.007 m. yaw 최대 0.015 rad |
| nav text | 100 % 동일 | 91.6 % 동일. 나머지 8행은 거리 ±1–2 m 차이 |
| input_ids / pixel_values | 100 % 동일 / 차이 0 | 91.6 % 동일(nav 차이 때문) / BEV 영상 차이 |

- **두 모드**
  - gt 모드는 학습과 같은 신호(`_ego` v, yaw rate, path_s)를 넣어 코드가 같은지 본다.
  - deploy 모드는 대회 입력(competition_status, IMU, 스택 EKF)을 넣어 신호 차이의 크기를 본다.
- **재생 시각**: bag은 벽시계이고 수집 때 시뮬레이터가 0.71–0.95배였다. 그래서 재생기는 step 0처럼 `_ego` 표로 stamp를 τ로 옮긴다.
- **추가 에피소드**: run1023h-w1-007의 5행도 gt 모드에서 비트 단위로 같았다.
- **재현**: winpc에서 `tools/winpc/parity.sh <episode>`를 돌린다. 내용은 `test/offline_replay.py`(chroot) → `test/offline_compare.py`(venv)다. 결과는 `~/morai-eval/data/parity/<ep>/report.json`에 남는다.

## bag 재생 배선 시험 (MORAI 없이, 2026-10-26)

`test/bag_loop.sh`는 chroot 안에서 roscore를 `/use_sim_time`으로 띄운다. 브릿지를 뺀 스택 노드를 띄우고, run1024b-w1-001의 30–90 s 구간을 `rosbag play --clock`으로 재생한다. LiDAR 패킷은 `test/udp_replay.py`가 UDP 2368로 다시 쏜다. 모델 서버는 가짜 백엔드였고, 전처리는 실제와 같이 돌렸다.

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
- **speed_filter의 출발 보조**: 30 s 넘게 서 있고 accel이 0이면 1 s 동안 accel 0.2, 조향 0을 낸다. 긴 적신호에서 차가 움찔할 수 있다. speed_filter는 손대지 않았다.

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
- **체크포인트**: `StudentVLA.build(init=dir)`로 읽는다. QAD 체크포인트(`modelopt_state.pth`)는 `~/morai-student/pylib`의 nvidia-modelopt를 쓴다.
  - 교사는 `--model-class teacher --vlm Qwen/Qwen3-VL-8B-Instruct`로 띄운다. 다만 BF16 23 GB라 5080에 올라가지 않는다.
- **백엔드 교체**: `server/backends.py`에 `predict(sample) -> {accel, kappa, v0}`를 구현하는 클래스를 추가하면 된다. 이후 NVFP4 엔진을 붙일 자리다.
- **가짜 모델**: `--backend fake --preprocess-config <config.json dir>`는 모델 없이 속도 유지 법칙(기본 4 m/s, κ = 0)으로 계획을 낸다. 전처리와 토큰화는 실제와 똑같이 돌려 지연을 잰다.

## 남은 문제

- **지연**: 모델 서버 전처리(PIL decode + Qwen image processor + 토크나이저)가 CPU에서 요청당 약 50 ms 걸린다. 5080 NVFP4 모델 89 ms와 더하면 10 Hz 예산 100 ms를 넘는다. 과거 프레임 12장의 pixel_values 캐시 등으로 줄여야 한다.
- **학생 BF16 메모리**: 학생 BF16은 13.4 GiB다. 시뮬레이터와 같은 5080(16 GB)에서는 HF 백엔드로 돌리기 어렵다. 실제 학생은 NVFP4 엔진 백엔드가 필요할 수 있다.
- **RTF**: winpc에서 시뮬레이터와 모델이 함께 돌면 RTF가 1보다 낮을 수 있다. 그러면 시각 텍스트와 이력 적분이 학습 분포에서 벗어난다.
- **터널·GPS 음영**: localization이 출력을 멈추고, nav 진행거리는 주행거리 적산으로 버틴다. 이 구간의 오차는 아직 재지 않았다.
