# 폐루프 평가 pod (RunPod, `eval_pod.sh` / `eval_pods.sh`)

## 빠른 사용법: 새 5090에서 무작위 장면 주행 영상 받기 (10/28)

### 준비 (Mac, 한 번만)
- `vastai` CLI에 로그인한다.
  - vast 계정 환경 변수에 `B2_APPLICATION_KEY_ID`, `B2_APPLICATION_KEY`가 있어야 한다(`vastai show env-vars`).
  - Mac의 ssh 키가 vast 계정에 등록돼 있어야 한다.
- `~/.morai-secrets/`에 `config.tar`와 `V2AccountInfo.byte`를 둔다(권한 600, 대회 계정). B2에는 올리지 않는다.
- `~/.claude/channels/discord/.env`에 `DISCORD_BOT_TOKEN`을 둔다. 영상과 요약을 디스코드 DM으로 보낼 때 쓴다.
- B2 기동 묶음은 `podimg/v1`(27.1 GB)이다. 코드나 납품본이 바뀌면 볼륨 pod에서 `bundle.py pack v2`를 돌린다.

### 1. 상자 고르기 (바퀴당 비용 = 시간당 요금 × 한 바퀴 wall. 실시간 RTF는 CPU 단일 스레드에 좌우된다)
```bash
vastai search offers 'gpu_name=RTX_5090 num_gpus=1 rentable=true verified=true cpu_ghz>=4.5 inet_down>=900 reliability>0.98 cpu_ram>=30 cpu_cores_effective>=16' -o 'dph+'
```
- 최소 사양: RTX 5090 1장, vCPU 16 이상(cgroup quota 15 이상), RAM 30 GB 이상, 디스크 150 GB 이상.
  - 회선이 1 Gbps 이상이어야 10분 안에 뜬다. 598 Mbps에서는 묶음 받기만 6분이다.
- RunPod: `PROVIDER`를 빼고 `BUNDLE=v1`만 주면 Secure 5090($1.19/h)을 아무 데이터센터에서나 빌린다. B2 키는 RunPod secret에서 온다.
- Brev: 10/28 기준 5090이 없다.

### 2. 명령 한 줄
```bash
cd src/e2e_driver/tools/pod
PROVIDER=vast OFFER="<offer id 1> <offer id 2> <offer id 3>" BUNDLE=v1 LAPS=8 LAP_DELIVER=1 VIEW_FPS=15 EVAL_MAX_MIN=300 \
  bash eval_pod.sh <이름, 예: 20261028-rt8> /workspace/models/wowmayo_final
```
- 시드를 안 주면 run마다 새 무작위 시드를 뽑는다. 같은 장면을 다시 돌려 비교할 때만 끝에 시드를 준다.
  - 범위는 평가 구간 0–99다. 7, 28, 29(이전 run들의 장면)는 빼므로, DAgger(S ≥ 1000)·PPO(S ≥ 2000) 액터 장면과 겹치지 않는다.
  - 재생 확인이 실패해 다시 뽑을 때도 같은 규칙을 따른다.
  - 쓴 시드는 결과 이름 `NAME_s<시드>`(B2 `runs/eval_vps/NAME_s<시드>`), `eval_pod.log`("seeds …"), 바퀴마다 `laps.jsonl`의 `seed`(S·1000+k)와 DM 요약 줄에 남는다.
- `OFFER`에는 후보를 여러 개 준다. 6분(`LOAD_MIN`) 안에 running이 안 되면 그 상자를 지우고 다음 후보로 넘어간다.
  - 10/28 프랑스 호스트는 이미지 내려받기에서 19분 동안 멈춰 있었다.
- 순서: 상자 생성 → B2 묶음 받기(32 MB 구간 48개씩 동시에) → 계정 파일과 코드 → setup → 시뮬·스택·엔진 → 바퀴마다 무작위 장면 → 주행.
  - 바퀴가 끝날 때마다 다음이 나온다:
    - 1배속 폰 영상(20 MB 이하. DM 첨부가 21.3 MB에서 실패했다)
    - 한 줄 요약: 완주·감점·복귀, late, model_ms, 날씨, 터널, 동적 물체, NPC, 재생 확인
    - B2 `runs/eval_vps/<이름>/lap_KK/`와 디스코드 DM. 첨부가 실패하면 B2 7일 링크를 보낸다.
- 옵션:
  - `LAPS`: 바퀴 수(같은 world에서 이어서 돈다).
  - 시드 S: 바퀴 k의 장면은 S·1000+k다. 실시간 평가는 GPU 1장에 world 1대라서 시드를 하나만 준다.
  - `VIEW_FPS`: 영상용 화면 캡처 fps.
  - `EVAL_MAX_MIN`: 생성부터 이만큼 지나면 포기하고 지운다.
  - `REDRAW`(1): 재생 확인에 실패한 장면을 다시 뽑는다.
  - `SYNC=1`: 동기 모드, 아래 "sync". 이때 world 수는 호스트 크기로 자동으로 정한다.
- **자동 삭제:** 마지막 바퀴의 전달과 결과 회수(`~/morai-eval/pod/<이름>/`)가 끝나면 상자를 지운다. 오류, Ctrl-C, 시간 초과 때도 지운다.
  - vast는 `destroy -y`, RunPod은 DELETE를 쓴다.
  - 남기려면 `KEEP=1`이다.
  - 상자 안의 idle watchdog도 GPU가 15분 놀면 상자를 멈춘다.
- 이미 띄운 상자에 다시 붙이기: `ATTACH=<instance id> ATTACH_T0=<생성 epoch> PROVIDER=vast …`

### 새 납품본이 나오면 바로 영상 받기 (`deliver_watch.sh`, 10/28)
```bash
cd src/e2e_driver/tools/pod && nohup bash deliver_watch.sh > ~/morai-eval/deliver_watch.log 2>&1 < /dev/null & disown
nohup caffeinate -i -w $! > /dev/null 2>&1 &         # the Mac stays awake while it waits
```
- R2 미세조정 감시(`Alpamayo-Cosmos/tools/vast/r2ft_autowatch.py`)가 `~/.r2ft-watch.events`에 `STAGE 납품 완료: b2://…/<run>/deliver/final`을 쓰면 다음을 한다:
  - 가장 싼 vast 5090 후보 4개를 고른다(위 조건, $1/h 이하).
  - `eval_pod.sh BUNDLE=v1`을 `b2:<run>/deliver`로 띄운다. `qad_export.py deliver` 출력(`final/` + `quant_manifest.json`)이 와우마요 납품본과 같은 형식이라 변환이 필요 없다.
  - 8바퀴를 돈다. 시드 28은 10/28 rt8과 같은 장면이다.
  - 바퀴마다 "[R2 미세조정 납품본]"으로 시작하는 DM을 보내고, 끝나면 B2 동기화 후 자동 삭제한다.
- 감시를 켜기 전에 적힌 줄은 보지 않는다. 시험은 `EVENTS=<가짜 파일> DRY_RUN=1`로 한다.
- 새 QAD의 층별 NVFP4/FP8 배치가 옛 납품본과 다르면 5090 컴파일 캐시가 맞지 않는다. 그러면 첫 엔진 준비에 컴파일과 autotune으로 10–20분이 더 걸린다. 그동안 world는 멈추고, `EVAL_MAX_MIN` 330분이 이를 덮는다.

### 새 상자 기동 실측 (10/28, vast 58001997: 노르웨이 Ryzen 9 7900X 5.74 GHz, 24 vCPU, 60 GB, 929 Mbps, $0.695/h, `LAPS=1`)
| 단계 | 생성 후 |
|---|---|
| running(이미지 받기) | 0:52 |
| ssh | 0:59 |
| 묶음 27.1 GB 받기 끝(252 s, 108 MB/s. 회선이 한계다) | 5:28 |
| setup 끝 | 5:46 |
| eval3 시작 | 5:56 |
| world 로그인·지도·프리셋과 엔진 준비(가중치 5 s, 캐시 워밍업. 그동안 world는 멈춘다) | 7:14 |
| **첫 주행 step** | **7:15** |
| 1바퀴 끝(시뮬 757 s, wall 15.2분, RTF 0.83) | 22:29 |
| 영상(15 fps 캡처 → 1배속 540p 20 MB 이하)·요약·B2 | 27:21 |
| **디스코드 DM 도착** | **27:27** |
| 결과를 Mac으로 회수하고 상자 삭제 | 28:20 |

- 비용: 1바퀴 상자 약 $0.33.
- 10분이 넘는 경우:
  - 회선이 느린 경우: 598 Mbps 상자는 묶음 받기에 358 s가 걸렸다.
  - 호스트의 이미지 받기가 멈춘 경우: `LOAD_MIN` 뒤 다음 후보로 넘어간다.
- 바퀴가 끝난 뒤 영상까지 약 4.5분이 걸린다(프레임 모으기와 인코딩 두 번). 줄이려면 `VIEW_FPS`를 10으로 내린다.

### 비용 예 (10/28 실측)
| 상자 | 요금 | 한 바퀴 | 비고 |
|---|---|---|---|
| vast 48989799 (7800X3D 5.05 GHz, 16 vCPU, 598 Mbps) | $0.563/h | wall 12.6분(RTF 0.84) → 약 $0.12 | 8바퀴에 약 $1.2 |
| vast 58001997 (7900X 5.74 GHz, 24 vCPU, 929 Mbps) | $0.695/h | wall 15.2분(RTF 0.83) | 기동부터 첫 영상까지 $0.33 |

- 주의: 실시간 평가의 late는 RTF에 따라 달라진다.
  - RTF 0.83–0.84에서는 τ 100 ms가 벽시계 약 120 ms다. 엔진 왕복(p50 약 126 ms: 전처리 28 ms + 모델 78–85 ms)이 그 예산을 넘는 일이 잦아, late가 바퀴에 따라 9–73 %였다.
  - 스택의 시뮬 시계 추정(ρ)이 빗나간 구간은 100 %였다.

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


## sync: MORAI 동기 모드 평가 (`SYNC=1`, 10/28)

### 동작
- 시뮬은 tick 사이에 멈추고, world마다 `eval_manager.py --sync`가 유일한 시계다. 그래서 late는 구조상 0이다.
  - 실제 대회(1배속, 5080에서 100 ms 넘는 스텝 16.8 %)보다 낙관적인 조건이다.
- 한 스텝(TICK_MS 100 또는 50): Ego 상태와 판정 → 스택 단계 → `Tick(1)`.
  - 스택 단계는 `sync_clock_node.py`(sim_clock 자리, 포트 47100+n)를 거친다.
  - e2e 노드 `on_sync`가 τ의 센서를 기다리고, τ+0.1 몫의 계획을 적용하고, 명령을 낸다. 0.1 s 격자에서는 요청을 보낸다.
  - 명령이 speed_filter와 브릿지(500 Hz, 명령 timeout 30 s)를 지나면 `/clock`을 τ+dt로 올린다. tick 동안 받은 센서는 tick이 끝나는 시각으로 찍힌다.
- 1-step 지연 규칙은 그대로다. τ의 프레임으로 만든 계획은 τ+0.1부터 적용한다.
  - overlap(기본): 답을 다음 tick 동안 기다린다.
  - `SERIAL=1`: tick 전에 기다린다. 결과 주행은 같고 wall 시간만 다르다.
- 엔진은 `--gather 0.1`로 최근 2 s 안에 요청한 world들의 요청을 한 묶음으로 모은다. 전처리는 `--prep process`다.
- CPU 배치(`CPU_LAYOUT=shared`, SYNC 기본): world와 스택이 엔진용 8 CPU를 뺀 전부를 같이 쓴다.
  - 모든 world의 카메라 대기 스레드는 CPU 4개에 모은다(`pin.sh SPIN_CPUS`). 그대로 두면 world마다 약 4코어를 태워 pod의 CPU quota를 다 쓴다.
- 엔진이 뜨는 동안에는 world를 SIGSTOP으로 멈춘다. 멈춰 둔 world도 렌더 루프로 대당 약 3.8코어를 쓰기 때문이다.
- Ctrl-C는 tick 사이에서만 받는다. Tick 도중에 끊으면 그 world의 Simulation gRPC 서비스가 영영 멈췄다.
- 조향은 e009536 설정이다. `steer_lookahead_s` 0.3, 속도별 게인이고, 실행 중인 스택 파라미터로도 확인했다.

### 측정 (`sync_bench.sh`, 10/28, RunPod 5090 EUR-IS-1, CPU quota 27.2코어, RAM 61 GB, 와우마요 4.749 bits)
| N × tick | 합계 sim-s/s | $/sim-h (1.19) | world RTF | tick wall | model_ms | VRAM | CPU(cgroup) |
|---|---|---|---|---|---|---|---|
| 1 × 100 | 0.14 | 8.56 | 0.14 | 667 ms | 71 | 13.0 GB | 13.4 |
| 2 × 100 | 0.27 | 4.42 | 0.13 | 717 ms | 77 | 16.2 GB | 15.5 |
| 4 × 100 | 0.52 | 2.28 | 0.13 | 737 ms | 118 | 22.2 GB | 19.4 |
| 4 × 100 serial | 0.41 | 2.94 | 0.10 | 683 + 262 ms 대기 | 94 | 22.3 GB | 19.9 |
| 6 × 100 | 0.76 | 1.57 | 0.13 | 756 ms | 175 | 25.7 GB | 21.4 |
| 6 × 100, spinner 고정 없음 | 0.52 | 2.30 | 0.09 | 1101 ms | 145 | 25.7 GB | quota 포화 |
| 6 × 50 | 0.70 | 1.70 | 0.12 | 2 × 372 ms | 136 | 27.2 GB | 22.5 |
| **7 × 100** | **0.85** | **1.40** | 0.12 | 792 ms | 189 | 29.2 GB | 23.5 |
| 7 × 50 | 0.77 | 1.55 | 0.11 | 2 × 387 ms | 149 | 29.7 GB | 23.4 |
| 8 × 100 | 엔진 OOM | | | | | 8 × 2.5 GB + 엔진 11 GB > 31.4 GB | |

- 병목은 시뮬레이터 tick이다(world 하나에 시뮬 1 s당 wall 약 5.6–7 s, 100 ms든 50 ms든 같다). 추론은 overlap에서 tick 안에 끝난다(plans 대기 0.1 ms).
- world 수에 거의 비례해 늘고, VRAM이 N을 7로 묶는다. 추천: **N=7, 100 ms, overlap.** 50 ms(제어 20 Hz)는 약 10 % 비싸다.
- 호스트별 N(`eval_pod.sh`가 자동으로 정한다): min(⌊(VRAM−8.1)/3.13⌋, ⌊(0.9·quota−2.5)/3.1⌋, ⌊(0.85·RAM−4.5)/4.8⌋) (GiB, 코어).
  - 5090 + quota 27 + 61 GB → 7. 5090 + vCPU 16 pod(quota 약 13) → 3.
  - 최소 사양: 5090(32 GB), CPU quota 24코어 이상, RAM 48 GB 이상, 회선 1 Gbps 이상(10분 기동).

### 어느 5090이든 10분 안에 주행: B2 묶음 기동 (`BUNDLE=TAG`)
- `bundle.py pack v1`: 볼륨 pod의 파일을 약 1 GB 묶음 23개(27.1 GB, 원본 34.9 GB)로 `b2://morai2026-alpamayo/podimg/v1/`에 올린다.
  - 들어가는 것: rootfs delta, 저장소+catkin, venv, sm120 컴파일 캐시, 납품본, VLM 설정·토크나이저, MORAI.
  - 빠지는 것: 대회 계정(`Version/V2AccountInfo.byte`, 로그인 prefs `config.tar`)은 B2에 두지 않는다. 기동 때마다 Mac의 `~/.morai-secrets`에서 ssh로 넣는다. VLM 기본 가중치 8.3 GB도 뺀다(엔진은 납품본에서 모든 가중치를 읽는다).
- 시작 명령: 표준 python 부트스트랩이 B2 키(RunPod secret, 끝의 줄바꿈 제거)로 `bundle.py`를 받는다. 그다음 묶음을 32 MB 구간으로 48개씩 동시에 받아 순서대로 tar에 풀고 `setup.sh`를 돈다.
- 실측(10/28, RunPod EUR-IS-1 5090 pod 안에서 `FETCH_ROOT`로 시험):
  - B2는 연결 하나에 약 17 MB/s다. 16개를 동시에 열면 274 MB/s다.
  - 묶음 받기·풀기는 27.1 GB에 80 s(338 MB/s)였다. 묶음당 미리 읽기를 4로 두면 144 s다.
  - venv 파일 목록이 원본과 같고, 계정 파일은 빠져 있었다.
- 예상 시간표(새 호스트, 아직 처음부터 끝까지 재 보지 않았다):

  | 단계 | 시간 |
  |---|---|
  | pod 생성 → 시작 명령 | 약 1분 |
  | 묶음 받기 | 1.3–2.5분 |
  | setup(delta, sshd) | 약 0.3분 |
  | 계정 파일·코드 | 약 0.2분 |
  | world 7대 병렬(측정 2분)과 엔진 적재 148 s + 캐시 워밍업(측정 5.2분, 그동안 world는 멈춤) | 5분 안팎 |
  | 장면 load·첫 step | 약 0.5분 |
  | **합계** | **약 8–9.5분** |

- `eval_pod.sh`(Mac): `SYNC=1 BUNDLE=v1 VOLUMES= bash eval_pod.sh NAME /workspace/models/wowmayo_final 1 2 3 …`
  - 데이터센터 제한이 없다. 계정 파일을 넣고, 코드를 동기화하고, 호스트 크기로 N을 정한다.
  - 시드를 N개씩 묶어 파도로 돌린다. 재생 확인에 실패한 장면은 새 시드로 한 번 더 뽑는다.

### 실시간 1:1 평가 + 바퀴별 전달 (10/28)
- GPU 1장에 world 1대, free-run 모드다. 10/27 평가와 같은 조건이다: 10 Hz, 계획은 τ+0.1부터 적용하고 늦은 답은 late로 센다. 조향은 e009536이다.
- 같은 world에서 `LAPS`바퀴를 돈다. 시드 S로 장면 S·1000+k를 공급하고, GT 재생 확인을 한다.
- `LAP_DELIVER=1`: 바퀴가 끝날 때마다 pod의 `lap_deliver.sh`가 다음을 한다.
  - 1배속 폰 영상을 24 MB 아래로 만든다.
  - `lap_report.py`로 한 줄 요약을 만든다: 완주·감점·복귀, late %, model_ms p50/p95, 날씨, 터널, 동적 물체, NPC, 재생 확인.
  - B2 `runs/eval_vps/NAME/lap_KK/`에 올리고, 영상의 7일 링크와 함께 `lap_KK.ready`를 남긴다.
  - Mac의 `eval_pod.sh`가 그 영상과 요약을 디스코드 DM으로 보낸다. 첨부가 실패하면 링크를 보낸다.
- 명령(Mac): `PROVIDER=vast OFFER=<id> BUNDLE=v1 LAPS=8 LAP_DELIVER=1 VIEW_FPS=15 EVAL_MAX_MIN=300 bash eval_pod.sh NAME /workspace/models/wowmayo_final` (시드 생략 = 새 무작위)
- 제공자 선택(10/28):
  - 실시간 RTF는 CPU 단일 스레드 속도에 좌우된다.
  - vast의 5 GHz대 Ryzen 5090이 $0.50–0.77/h이고, RunPod 5090(EPYC, $1.19)보다 바퀴당 싸다.
  - Brev에는 5090이 없었다.
- vast 첫 기동 실측: 48989799, 한국, 7800X3D, 16 vCPU, 30 GB, 598 Mbps
  | 단계 | 시각 |
  |---|---|
  | 생성 → ssh | 2분 22초 |
  | 묶음 27.1 GB | 358 s(76 MB/s, 회선 한계) |
  | setup | 8 s |
  | 엔진 | 71 s(가중치 31 s, 볼륨 pod의 148 s보다 빠름) |
  - 그날 실제로는 eval3 변수 충돌(`SYNC`) 때문에 12분을 더 썼다. 고친 뒤 기준으로 생성부터 첫 step까지 약 10분이다.
  - 회선이 1 Gbps 이상이면 약 7–8분이다.

### 시나리오 공급·재생 (`scenario_variants.py`, `eval_manager.py --objinfo-port`)
- 흐름: 시드 S, 바퀴 k마다 S·1000+k → 장면 생성(clcol 기준, 위치 규칙은 생성기 주석) → MSC load(일시정지) → sync on → 재생.
  - 매 step Object Info(Ego 80 m 안)를 기록한다.
  - 바퀴가 끝나면 재생 확인을 `laps.jsonl` `validation`에 쓰고, 요약(`replay`)에 보인다.
- 무작위화 축과 근거: 생성기 docstring(DATA_PLAN §6, DATA_PLAN_R2 §4 A/B, 대회 §2-7-2 날씨·시간).
- 로터리 NPC 차종(10/28 고침): 로터리 spawn point는 위치로 찾는다(clcol UNIQUEID 4, (−111.3, 358.3) 30 m 안). 그 지점을 타입 −1로 두면 MORAI가 spawn마다 차종을 뽑는다.
  - 전에는 기준 장면의 30100004를 그대로 두어서 로터리 차가 모두 같은 차종이었다.
  - `ring_probe.py` 실측(5090):
    - 8종 각각을 지정하면 모두 나온다(40 s에 4–7대).
    - −1은 40–90 s에 7–12대가 나왔고, 치수가 최대 10가지였다. 8종 밖의 MORAI 승용차도 섞인다.
    - 지점을 복사해 다른 차종을 주면 복사본은 하나도 내지 않았다. 그래서 버스(30400004)와 밴(30300002)은 로터리에 나오지 않는다.
- 재생 확인(GT):
  - 동적 물체: Ego가 발동 거리+2 m 안에 들어왔으면 min(2 m, 이동 거리의 30 %) 이상 움직였어야 한다.
  - 정적 물체: 놓은 자리 3 m 안에 있어야 한다.
  - NPC: 로터리(CP8–CP9 ±20 m, 35 m 안)와 고주로(s 1118.7–1741.7, 80 m 안)에 NPC 차량이 있어야 한다. 본 차량 치수 목록도 남긴다.
  - ID 재사용: MORAI가 장면을 다시 불러도 물체 ID를 재사용한다. 그래서 차량은 35 m/s, 보행자·소품은 10 m/s를 넘는 점프에서 궤적을 끊는다.
- Mac 점검:
  - 가짜 rospy, 가짜 시뮬, 가짜 서버로 실제 노드, sync_clock_node, SyncLink를 돌렸다(100/50 ms × overlap/serial, 느린 서버).
  - 생성기 40장면의 분포를 확인했다.
  - 재생 확인기에 가짜 데이터 시험을 했다(움직임, 정지, 미도달, 존재, 누락, ID 재사용 점프, 빈 고주로).


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
