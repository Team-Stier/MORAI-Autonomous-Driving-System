# RL actor pod (RunPod, RTX 5090 ×1)

Alpamayo-Cosmos `RL_PLAN.md` 3.1의 actor 한 대: MORAI 리눅스 3대, ROS 폐루프 스택 3개, 묶음 추론 엔진 1개, world마다 에피소드 관리자 1개.
- 10/26 RunPod 실측 레시피(side-session-notes `cloud-rl-sim-pod-20261026.md`, `cloud-rl-pod-scripts-20261026/`)를 따른다.
- **유료**다. pod와 네트워크 볼륨은 사용자 승인 뒤에만 만든다(`create_pod.sh`는 보내기 전에 묻는다).

| 파일 | 하는 일 |
|---|---|
| `create_pod.sh volume DC GB` / `pod DC VOLUME_ID` | 네트워크 볼륨과 pod를 만든다(GraphQL `minVcpuCount`, 이미지 `ros:noetic-ros-base-focal`, `NVIDIA_DRIVER_CAPABILITIES=all`). 시작 명령은 `setup.sh`, 그다음 idle watchdog이다 |
| `setup.sh` | 부팅마다 apt 패키지, Xvfb, ko_KR 로캘, sshd를 준비하고 MORAI를 컨테이너 디스크에 푼다. 첫 부팅에만 볼륨에 catkin 빌드와 venv를 만든다 |
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
