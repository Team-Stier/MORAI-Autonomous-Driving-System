# 폐루프 평가 pod (RunPod, `eval_pod.sh` / `eval_pods.sh`)

WoWmayo 납품본 하나를 MORAI 리눅스 시나리오 여러 개로 동시에 평가한다. 실행은 Mac에서 명령 하나다.

```bash
cd src/e2e_driver/tools/pod
bash eval_pods.sh r2a b2:runs/stageC/<새 납품 폴더> 7 8 9                   # 권장: 시드마다 pod 1대(시뮬 1 + 엔진 1)
bash eval_pod.sh  r2b b2:runs/stageC/<새 납품 폴더> 7 8 9                   # pod 1대에 시뮬 3대 (아래 "한계")
bash eval_pod.sh  r2c /workspace/models/wowmayo_final 7                    # 볼륨에 이미 있는 납품본
```

- 단계: pod 생성(GraphQL, 네트워크 볼륨, `ros:noetic-ros-base-focal`) → 부팅(`setup.sh`) → 코드 동기화(이 checkout) →
  `eval3.sh`(시뮬 N대 + 묶음 엔진 1개 + 스택 N개 + 시드별 `eval_manager.py` 1랩) → 결과 회수 → pod 삭제.
  - 오류, Ctrl-C, 시간 초과(ssh 15분, 설치 25분, 평가 `EVAL_MAX_MIN` 110분)에도 pod를 삭제한다. 볼륨은 남는다.
  - pod 안에서는 idle watchdog도 돈다(GPU 15분 유휴 → 정지).
- `b2:<경로>`로 주면 시뮬레이터가 뜨는 동안 볼륨의 `/workspace/models/<폴더명>`으로 받는다(14 GB에 56초).
- 재고가 없으면 1분마다 다시 시도한다(`RETRY_MIN` 30분). GPU 순서는 5090 → RTX PRO 6000 Server → Workstation이고,
  볼륨 순서는 `VOLUMES`다(기본 EU-CZ-1, 아래 두 볼륨을 다 쓰려면 `VOLUMES="1tra2qsxbl:EU-CZ-1 21z08rmvlx:EUR-IS-1"`).
- 결과(Mac): `~/morai-eval/pod/NAME/`
  - `summary.txt`/`summary.json`: 주행별 랩 결과, RTF, τ 스텝 수와 0.1 s 비율, late 비율, model_ms. 전체 sim-h/wall-h와
    pod 생성부터 주행 시작까지 시간도 들어 있다.
  - `timeline.txt`: 단계별 시각.
  - `wN/`: `NAME_wN.mp4`(1배속, 계기판 없음, 540p, 30 MB 미만), `plans/*.png`(곡선 구간 계획 경로·차선 겹침,
    `test/plan_overlay.py`), `eval_map/`(laps.jsonl, rows), `dbg_wN.csv`, `clk_wN.csv`, 로그.
  - B2 `runs/eval_pod/NAME/`에도 같은 내용이 있다. 원본 프레임은 pod 컨테이너 디스크에만 있다가 pod와 함께 지워진다.
- 평가 기준: 스택은 τ(시뮬레이터 시계)로 돈다. τ에서 관측한 계획은 τ+0.1 s부터 적용하고(`E2E_APPLY_DELAY_S`, 노드
  `apply_delay_s`), 그보다 늦게 온 답은 도착 즉시 적용하며 late로 센다.

## 볼륨 (한 번만 준비, 10/27)

| 볼륨 | 데이터 센터 | 크기 | 비용 |
|---|---|---|---|
| `1tra2qsxbl` wowmayo-eval | EU-CZ-1 | 80 GB (사용 약 45 GB) | 하루 약 $0.19 |
| `21z08rmvlx` wowmayo-eval-is | EUR-IS-1 (재고 대비 사본) | 80 GB | 하루 약 $0.19 |

- 내용: MORAI(`morai/part*.tar` 4.0 GB, bc-250 사본, 파일 896개 sha256 일치), `config.tar`(로그인 prefs. `login.json`은
  넣지 않았다), 저장소와 catkin 빌드, `venv-engine`(cu130, torch 2.14.1, flashinfer 0.7.2.post1), `venv-rl`, `venv-tools`(b2, hf),
  `models/wowmayo_final`(B2 `runs/stageC/deliver`), `models/hf`(Qwen3-VL-4B-Instruct), `cache/`(inductor, triton,
  flashinfer, CUDA. GPU 종류마다 따로 쌓인다), `rootfs/delta.tar`(첫 부팅이 바꾼 루트 파일 2.6 GB).
- 준비 순서: CPU pod(16 vCPU, $0.48/h, 약 50분)에서 `setup.sh` 첫 부팅을 했다(apt, rosdep, catkin 9분, venv 5분).
  - MORAI는 bc-250 → Mac 중계 → pod로 9조각 병렬 전송했다(25분, 연결 하나에 0.3–1 MB/s).
  - 첫 GPU 부팅이 `rootfs/delta.tar`를 만든다. NVIDIA toolkit 파일(드라이버 링크, ICD, ld.so.cache)은 넣지 않는다. 그런 파일이
    섞인 tar로 부팅한 pod에서는 MORAI가 Vulkan 스왑체인에서 죽었다.
  - EUR-IS-1 사본은 CPU pod 두 대 사이에서 rsync로 만들었다(약 215 MB/s).
- 새 패키지가 필요하면 `setup.sh` 목록에 넣고, `rootfs/delta.tar`를 치운 뒤 한 번 부팅한다(CPU pod로 충분).

## 재기동 시간 (캐시가 있는 상태, 측정)

| 단계 | 시간 |
|---|---|
| pod 생성 → ssh | 45–57 s |
| → 설치 끝 (`delta.tar` 10 s + MORAI 3 s) | 63–75 s |
| → 코드 동기화, eval3 시작 | +10 s |
| 시뮬 준비 (로그인은 순서대로, 지도는 병렬) | 1대 64 s, 3대 3분 5초 |
| 엔진 준비 (가중치 읽기 + 양자화 159–216 s, 워밍업) | 5분 10초 |
| **pod 생성 → 주행 시작** | **약 6.5분** (엔진이 임계 경로) |
| 첫 실행(컴파일 캐시 없음) | 엔진만 11–17분 |
| 랩 뒤 정리 (요약, 영상 3개, 그림, B2) | 약 7분 (pod 요금 계속) |

## 비용 (Secure, 10/27)
- 5090 $1.19/h, PRO 6000 Server $2.49/h, Workstation $2.59/h. Community는 네트워크 볼륨을 못 붙여서 쓰지 않는다.
- 랩 하나는 τ 약 700–900 s이고, 1대 RTF 0.5–0.9라 wall 15–30분이다. 여기에 기동 6.5분과 정리 7분이 더해진다.
  - `eval_pods.sh` 3대: 5090이면 회당 약 $2.0–2.6, PRO 6000이면 약 $4.2–5.4.
  - `eval_pod.sh` 1대 3시뮬: 약 $1.0(5090).

## 한계: GPU 한 장에 시뮬 3대 + 엔진 1개는 τ 10 Hz를 못 지킨다
| 구성 | GPU | late | τ 스텝 | model_ms p50 | 서버 왕복 p50 | 결과 |
|---|---|---|---|---|---|---|
| 3대 (t1) | 5090 | 85–87 % | 7.3–7.7 Hz | 142 | 243–246 ms | 2대 시간 제한(복귀 19, 9), 1대 출발 실패 |
| 3대 (t3) | PRO 6000 | 99 % | 3.9–7.8 Hz | 115–252 | — | 2대 출발 실패 |
| 1대 (t5) | PRO 6000 | 12–15 % | 9.86 Hz (0.1 s 99.5 %) | 85 | 124 ms | (아래 t5) |

- 원인: 시뮬 3대가 GPU를 97–99 % 쓰고, 엔진이 그 사이에 끼어 돈다. 요청은 초당 3 × 10 × RTF(≈0.5) ≈ 15개인데
  처리는 초당 약 10개다. CPU는 31코어 중 18코어로 여유가 있다.
- `SIM_FPS`(Unity pref TargetFrameRate)로 시뮬 fps를 묶어 보았으나 RTF가 그대로였다(시뮬레이터가 이 pref를 쓰지 않는다).
- 그래서 권장은 `eval_pods.sh`(GPU마다 시뮬 1대)다. 10/27 winpc 1대 평가(100 ms 초과 16.8 %)와 같은 조건이다.


# RL actor pod (RunPod, RTX 5090 ×1)

Alpamayo-Cosmos `RL_PLAN.md` 3.1의 actor 한 대: MORAI 리눅스 3대, ROS 폐루프 스택 3개, 묶음 추론 엔진 1개, world마다 에피소드 관리자 1개.
- 10/26 RunPod 실측 레시피(side-session-notes `cloud-rl-sim-pod-20261026.md`, `cloud-rl-pod-scripts-20261026/`)를 따른다.
- **유료**다. pod와 네트워크 볼륨은 사용자 승인 뒤에만 만든다(`create_pod.sh`는 보내기 전에 묻는다).

| 파일 | 하는 일 |
|---|---|
| `create_pod.sh volume DC GB` / `pod DC VOLUME_ID` | 네트워크 볼륨과 pod를 만든다(GraphQL `minVcpuCount`, 이미지 `ros:noetic-ros-base-focal`, `NVIDIA_DRIVER_CAPABILITIES=all`). 시작 명령은 `setup.sh`, 그다음 idle watchdog이다 |
| `setup.sh` | 첫 부팅: apt, rosdep, catkin 빌드, venv를 볼륨에 만들고, 바뀐 루트 파일을 `rootfs/delta.tar`로 남긴다. 그 뒤 부팅: delta.tar와 MORAI를 풀고 sshd를 띄운다(약 13초) |
| `worlds.sh up N` | MORAI N대. 런처 없이 직접 띄운다(자동 로그인). world n은 gRPC 7788+n, MSC·UDP +(n−1)×1000, `taskset` 12 CPU, `-force-vulkan -force-gfx-mt -job-worker-count 8`, 카메라 스레드 고정(`pin.sh`) |
| `stacks.sh up N` | ROS 스택 N개. run.sh와 같은 노드 목록이고, master 11310+n, UDP +(n−1)×1000이다. run.sh는 호스트의 ROS를 모두 죽이고 다시 빌드해서 여러 번 쓸 수 없다 |
| `actor.sh up NAME [N]` | worlds → 엔진(`--dump /workspace/dump`) → stacks → world별 `rl_episode.py --host local --direct --world n --objinfo-port 7505+1000(n−1)`. 기록은 learner spool `/workspace/spool/NAME_wn/epK/`(steps.jsonl, 그 world의 req/out 덤프, critic_obs, done.json)이다 |
| `measure.sh SECS` | 시뮬레이터별, 스택별(세션 전체), 엔진, 관리자의 CPU 코어 수와 cgroup 합계, GPU, 메모리 |
| `mk-clone.sh`, `pin.sh`, `mon.sh` | 10/26 스크립트 그대로 |

## 순서
1. 재고 확인: Alpamayo-Cosmos `tools/vast/runpod_stock.sh`. 5090 ×1, vCPU 32 이상, RAM 60 GB 이상인 데이터센터를 고른다.
2. `create_pod.sh volume <DC> 100` → 볼륨을 채운다(아래 목록). 업로드는 같은 DC의 싼 CPU pod에 볼륨을 붙여서 하거나, GPU pod 첫 부팅 때 한다.
3. `create_pod.sh pod <DC> <VOLUME_ID>` → GraphQL로 만든 pod에는 `PUBLIC_KEY`가 들어가지 않는다. `ssh <podHostId>@ssh.runpod.io`(RunPod 프록시)로 들어가 authorized_keys를 넣는다(10/26과 같음).
4. `/workspace/setup.log`에서 `done`을 확인한다. 첫 부팅은 catkin과 venv 때문에 약 15–25분 걸린다(추정).
5. 배선 시험: `ENGINE_ARGS="--backend fake --sims 3 --fake-speed 9" E2E_FAKE_FOLLOW_ROUTE_M=8 actor.sh up plumb 3` → `measure.sh 120` → `actor.sh down`.
6. 실제 엔진: `ENGINE_ARGS="… --sims 3 --rl-sigma S --expert2 --lora-watch F" actor.sh up ppo 3`.

## 네트워크 볼륨 목록 (/workspace)

| 경로 | 내용 | 원본 | 크기 |
|---|---|---|---|
| `morai/morai.tar` | MORAI 25.S4 리눅스. `~/Workspaces/scratches/morai`(시뮬레이터 래퍼 + `Simulator.real`, Bundle, Version의 `V2AccountInfo.byte`, SaveFile의 w1_collect 프리셋·clcol 장면·EgoNetwork) | bc-250 | 4.1 GB |
| `morai/config.tar` | `~/.config`의 `unity3d/MORAI`(로그인 prefs, TimeScale 1, 품질 2)와 `morai-stier/` | bc-250 | 1 MB 미만 |
| `MORAI-Autonomous-Driving-System/` | eval 레포. rl/env와 rl/actor-engine을 합친 것에 서브모듈 3개와 LFS 지도를 더한다. 첫 부팅 뒤 build/devel이 생긴다 | Mac | 0.4 GB + 빌드 약 1 GB |
| `Alpamayo-Cosmos/tools/scorer`, `tools/vast` | 점수기, idle watchdog | Mac | 1 MB 미만 |
| `models/deliver/` | QAD 납품물(`final/` BF16 + `quant_manifest.json`, 엔진이 읽을 때 양자화한다) | winpc | 14 GB(대역 납품물 `synthetic_full` 실측) |
| `models/hf/` | 베이스 VLM Qwen3-VL-4B-Instruct의 HF 캐시(`HF_HOME`, 오프라인) | winpc `~/.cache/huggingface` | 8.3 GB |
| `morai-student/pylib/` | `MORAI_VLA_PYLIB`(QAD 체크포인트용 nvidia-modelopt) | winpc | 54 MB |
| `venv-engine/`, `venv-rl/`, `uv-python/`, `uv-cache/` | 첫 부팅에 만든다. 엔진은 레포의 `engine-requirements.txt`(winpc venv-engine 고정 목록, `--no-deps`)다 | pod | 약 20 GB(winpc venv-engine 10 GB + 캐시) |
| `dump/`, `spool/` | 엔진 덤프(판단마다 req·out)와, 에피소드 관리자가 그것을 옮겨 담은 learner spool이다. learner가 같은 볼륨이나 동기화로 읽는다 | pod | 판단당 req 수백 KB(카메라 JPEG 6장) |
| 합계 | | | **약 48 GB + 기록. 100 GB로 잡는다** |

- `V2AccountInfo.byte`는 대회 계정 로그인 정보다. 볼륨에만 두고 다른 곳(B2 등)에 올리지 않는다.
- 공식 Spawn·WeatherTimeChanger는 7789 포트에만 붙어서 pod에서는 쓰지 않는다. 같은 gRPC 요청을 `--direct`로 보낸다.

## 예상 비용 (RunPod 정가, 10/27 조회. 추정)

| 항목 | 구성 | 시간 | 비용 |
|---|---|---|---|
| 네트워크 볼륨 100 GB | $0.07/GB·월 | 4일 | 약 $0.9 |
| 업로드 | CPU pod(약 $0.1/h) 또는 GPU pod 첫 부팅 중 대기 | 0.5–1 h | $0.1–1.2 |
| 첫 부팅(빌드, venv) | 5090 Secure $1.19/h (Community $0.69) | 0.3–0.4 h | $0.2–0.5 |
| actor 시험(배선 + 실제 엔진 + CPU 측정) | 같음 | 2–4 h | $1.4–4.8 |
| **합계** | | | **약 $2–7** (RL_PLAN 8절 actor 시험 $2–5와 같은 범위) |

- idle watchdog(`tools/vast/idle_watchdog.sh`, `DPH_USD`·`BUDGET_USD`)이 GPU가 15분 놀면 pod를 멈춘다. 볼륨은 남는다.

## CPU 측정 계획 (ROS 스택 3개는 아직 안 쟀다)
- 예산(추정): 시뮬레이터 3 × 4.6코어(스피너 고정) = 14코어, 엔진 전처리 2–4코어, 관리자 3 × 0.2코어다. 스택은 미측정이고 대당 1.5–3코어로 가정한다.
  - 합계는 약 22–27코어다. 그래서 `minVcpuCount`를 32로 둔다.
  - 1×5090 pod는 대부분 vCPU 16개다. 그런 pod에서는 시뮬레이터를 2대로 줄인다.
- 측정 1 (실제 MORAI 15분 시험, 스택 1개): 스택 프로세스의 CPU를 `/proc`로 잰다. 결과는 아래에 적는다.
- 측정 2 (pod 첫 시간): 배선 시험(가짜 엔진, 경로 추종 9 m/s)과 실제 엔진으로 각각 `measure.sh 120`을 2번씩 돌린다. 시뮬레이터별, 스택별, 엔진, cgroup 합계가 나온다.
  - 판정: cgroup 합계가 quota의 85%를 넘거나 시뮬레이터 RTF가 단독 대비 20% 넘게 떨어지면, N을 줄이거나 다음 pod의 `minVcpuCount`를 올린다.
  - RTF는 `rl_episode` 상태의 rho다.
- 측정 결과: (채울 것)
