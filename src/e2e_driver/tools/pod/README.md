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
  bash eval_pod.sh <이름, 예: 20261028-rt8> /workspace/models/wowmayo_final <시드, 예: 28>
```
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
- 명령(Mac): `PROVIDER=vast OFFER=<id> BUNDLE=v1 LAPS=8 LAP_DELIVER=1 VIEW_FPS=15 EVAL_MAX_MIN=300 bash eval_pod.sh NAME /workspace/models/wowmayo_final 28`
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

# DAgger 쉐도 (`SHADOW=1`, `shadow.sh`, `dagger_label.py`, 10/28)

와우마요가 운전하는 동안 R2 수집 주행기가 옆에서 계획만 한다. 판단(10 Hz)마다 전문가 라벨 64×(a, κ)를 만든다.

## 구성
- **전문가:** 수집 포크 `MORAI-Autonomous-Driving-System-collect` e2cd63a(R2 이미지 `morai-stack:e2cd63a`)에 수집 설정 `stop_timeout_sec 600`을 얹는다. 여기에 morai-parallel-collect의 GT·R2 노드를 붙인다: Ego status, GT 위치, Object Info 필터, GT AEB, GT yield, TL GT.
  - 포크에서는 계획에 필요한 패키지만 빌드하고 띄운다: tf_publisher, section_annotator, path_planner, velocity_planner, control.
  - `dagger_pod.sh`가 포크 스냅숏(약 65 MB tar)과 GT 노드를 상자에 넣는다. `shadow.sh build`가 상자에서 rosdep(lanelet2. 묶음에 없음)과 빌드를 한다(약 2–3분).
- **안전:** 쉐도는 자기 ROS master(11320+n)에서 벽시계로 돈다(수집 스택과 같다).
  - 그 master에는 브릿지가 없다.
  - 명령은 모두 `/shadow/*`로 돌린다: control `/control_cmd` → `/shadow/control_cmd`, AEB·yield `/morai_udp_bridge/control_cmd` → `/shadow/aeb_cmd`·`/shadow/yield_cmd`.
  - `shadow.sh check n`이 두 master의 실제 명령 토픽 발행자를 적는다.
  - 10/28 실측: 스택 master의 `/control_cmd`는 `/e2e_driver`, `/morai_udp_bridge/control_cmd`는 `/speed_filter_node`뿐이었다. 쉐도 master의 두 토픽은 발행자 0이었다.
- **입력:**
  - Ego status 9095와 TL GT 7509: world에서 바로 받는다(다른 곳에서 쓰지 않는다).
  - Object Info 7505: `udp_tee.py`가 17505(쉐도)와 27505(rl_episode)로 복사한다. 한 포트를 소켓 둘이 받으면 패킷이 나뉜다.
  - `/vehicle/competition_status`, `/sensors/imu/data`: 스택 master에서 `shadow_relay.py`로 옮긴다. 받은 시각(벽시계)으로 다시 찍는다.
- **기록:** `shadow_rec.py` → `$V/shadow/rec_wN/rec_*.jsonl`(벽시계, 시간당 약 0.45 GB, B2에는 gz로 올린다).
  - 계획 `/local_path/velocity_injected`(map으로 되돌림), `/local_path`
  - 명령(control·yield·AEB)
  - GT 자차, GT 물체, 신호, 구간, competition status
- **에피소드 시작:** rl_episode의 `--loc-restart`가 `shadow.sh restart n`도 부른다. 스폰은 순간이동이라 계획기를 새로 띄운다(`/local_path`까지 약 6 s, 일시정지 중).

## 라벨 (`dagger_label.py`, 에피소드마다 `epK/dagger_labels.npz`)
- 형식(러너와 약속): `step` int32 (N,) = steps.jsonl 줄 번호, `ak` float32 (N,64,2) a m/s²·κ 1/m(0.1 s), `valid` bool (N,64), `source` int8 (N,) 0 계획기 / 1 공식 차로.
  - 선택 필드: `signal` int8(t0에 따르는 신호, 0 없음·모름 / 1 적 / 2 황 / 3 녹), `stop_d` float32(앞축에서 다음 판정 정지선까지, m), `intervened` bool(출발 개입 중).
  - `valid=False`: 굴린 라벨이 없을 때, 그리고 채점기(`tools/scorer/label_check.py`)가 fail이나 unrecoverable로 판정할 때. 판정과 이유는 `dagger_debug.npz`에 있다.
  - 파일은 임시 이름으로 쓴 뒤 바꾼다(러너는 생긴 파일만 읽는다).
- **미래 정보 없음 (사용자 10/28):** t0의 계획, 신호, 물체(등속 예측)만 쓴다. 기록된 이후 계획이나 신호로 다시 계획하지 않는다.
- 경로:
  - 기본은 계획기 경로다(source 0). 단, 0.5 s 안의 계획이 차에서 5 m·0.5 rad 안에서 시작해야 한다(path_planner의 `max_initial_*`와 같다).
  - 계획이 없거나 그 범위를 벗어나면 공식 차로 중심선(`assets/route.json`)을 쓴다(source 1).
  - 차가 공식 차로에서 4.5 m 넘게 떨어진 경우와, 차와 30 m 앞 계획이 모두 2 m 넘게 떨어진 경우(계획기가 엉뚱한 차로를 따라감)에도 source 1이다.
    - 예외 1: 공식 차로 위에 선 물체가 있으면 장애물 우회로 보고 계획기를 쓴다(10/28 우회 중 3–4 m).
    - 예외 2: 고주로(구간 11–12)는 frenet_path가 차로를 고르므로 이 기준을 쓰지 않는다(전문가도 route.json 차로에서 6 m까지 떨어져 달렸다).
  - 다음 경우에는 라벨을 비운다: 2 s 안에 계획이 없을 때, 역주행(90° 넘음), 차로에서 15 m 넘게 떨어졌을 때, 시작 1.5 s(이력 없음).
- 굴리기: 차의 GT 자세·속도·조향에서 0.05 s씩 6.4 s.
  - control pp 코어(pp.hpp·pp.yaml 그대로): 룩어헤드 clamp(0.6 v, 2, 15) m, 휠베이스 3.0, 조향 속도 1.047 rad/s, PI 속도 루프. 적분은 정속 상태 값에서 시작한다.
  - speed_filter: 구간 11–12 밖에서 60 km/h를 넘으면 전제동.
  - yield·AEB: 수집 노드 코드(`conflict`/`brake_for`, `danger`)를 그대로 불러 등속으로 옮긴 GT 물체에 적용한다. Object Info 속도는 물체 좌표라 지도 좌표로 돌린다(아래 "알려진 문제").
  - 차량: 운동학 자전거 모델이다. 응답은 수집 주행기의 실제 주행으로 맞췄다: a = 3.56·accel − 14.1·brake − 0.15 − 0.029·v, 0.05 s 지연. pp.yaml 이득(5.74, 18.76)을 쓰면 6.4 s에 +2.5 m/s 빨랐다.
  - **속도:** velocity_planner는 시간 필터를 거친 목표 속도 하나를 경로 모든 점에 넣는다. 계획 길이도 약 50 m다.
    - 그래서 계획 속도가 점마다 같으면, 굴리는 동안 0.05 s마다 velocity_planner의 계산을 다시 한다(`Speeds`):
      - 곡률 미리보기 45 m(횡가속 2.6, 접근 감속 1.7), 시간 필터 0.35 s, 가속 2.0(곡선 1.5)·감속 5.0 제한, 상한 60(구간 11–12는 100) km/h. 시작값은 t0 목표 속도다.
      - 정지선 차단: 정지할 판정 정지선이 50 m 경로 안에 들어오면 목표 0. 정지점은 선 5 m 앞이다.
    - 경로 계획기가 점별 속도를 준 경우(회전교차로 등)는 그 값을 쓴다.
    - 계획이 끝나는 곳부터는 공식 차로를 잇고, 곡률 속도를 적용한다.
  - **신호 (t0 상태 고정):** 판정 정지선은 점수기·채점기와 같다(앞축 기준).
    - 적신호: 최대 제동(8 m/s², 0.2 s 지연)으로 선 전에 설 수 있으면 정지, 못 서면 교차로를 그대로 지난다(`go_unstoppable`).
    - 앞축이 이미 선을 넘었으면 진행한다(`go_past`). 교차 차량·물체는 yield·AEB가 본다.
    - 황신호: velocity_planner처럼 v²/(2·3) ≤ 정지점 거리면 정지.
    - 녹색이어도 t0 계획이 정지 중이면(n_frames 확인 전) 정지. 1 s 뒤 녹색이 될 것을 알고 출발하지 않는다.
- (a, κ): GT 이력 16점(시뮬 시간)에 굴린 속도·헤딩을 이어 `morai_vla.data.ego_trajectory` + `traj_to_action`에 넣는다(학습과 같다). 납품본 config.json의 action_space_cfg를 쓴다.
- 디버그: `epK/dagger_debug.npz`(굴린 궤적, 이유, 플래그). 그림·오차표는 `dagger_eval.py RUN_DIR`(Mac)로 만든다.
- Mac 실행: `uv run --no-project --with torch,numpy,pandas,pillow,hydra-core,einops,scipy,colorlog,pyyaml python dagger_label.py EP… --rec REC --gt <스킬 scripts/wsl> --config <납품본 config.json> --alpamayo <alpamayo f3198c2 src> --jobs 9`.
- 시험: `dagger_label.py --selftest`, `test/dagger_label_check.py`(실제 경로 위 합성 에피소드).

## 실행
```bash
cd src/e2e_driver/tools/pod
OFFER="<vast offer id> ..." bash dagger_pod.sh <이름> 2 1 28     # 정책 에피소드 2, 전문가 직접 주행 1, 시드 28
RANGE="--start-cp 10 --end-cp 12 --past-m 100" ATTACH=<id> ATTACH_NAME=<첫 이름> bash dagger_pod.sh <이름> 1 1 29   # 같은 상자, 고주로만
```
- 상자에서 도는 순서(`dagger_box.sh`):
  1. 쉐도 빌드와 actor 기동(world 1, 엔진 `--dump`, 스택)
  2. 정책 에피소드(`SHADOW=1`, `TAKEOVER=1`, `--red-cap 5 20`)
  2a. 엔진이 내려간 뒤 dagger-engine의 LLM LoRA 교체 확인(`test/lora_llm_check.py`, 결과 `lora_llm_check.json`)
  3. 전문가가 직접 달리는 에피소드(`shadow.sh up 1 drive`, 포크 자체 브릿지·speed_filter)
  4. 라벨 → B2
- **실패하면 복귀 (`rl_episode --returns`, 영상 제공기와 같음):** 에피소드는 전체 랩(START → CP14 + 100 m, 15분)이다.
  - 도로 이탈 2 s, stuck 60 s(적신호 시간 제외), 충돌 뒤 stuck 15 s, CP 건너뜀 10 m가 되면 직전에 지난 CP로 Spawn하고 랩을 이어 간다. 규칙은 eval_manager 복귀와 같고, 복귀 뒤 10 s 동안은 판정하지 않는다.
  - `--return-hook`이 쉐도를 재시작한다.
  - 기록: episodes.jsonl `returns`, steps.jsonl `since_return_s`. 복귀 뒤 1.5 s 행은 이력이 순간이동을 건너므로 라벨 없음이다.
  - 10/28 run e: 랩 하나(400 s)에서 6번 복귀했다(도로 이탈 1, 충돌 뒤 stuck 5. 4번은 고주로 CP11 Spawn 직후 NPC 추돌).
- **Object Info 원본:** `epK/objects.json.gz`. 20 Hz 프레임마다 `[tau, t_wall, [[id, type, x, y, z, heading_deg, length, width, height], ...]]`이고, 랩 하나에 약 1 MB다(PPO critic용).
- **적신호 상한 (`rl_episode --red-cap LO HI`):** 신호 주기는 지도(MGeo)에 고정이라 장면 파일로는 도착 위상만 바꿀 수 있다.
  - 그래서 판정 정지선 40 m 안에서 경로 녹색 없이 서 있은 시간이 [LO, HI]에서 뽑은 값을 넘으면, 그 교차로를 경로 녹색 위상으로 바꾼다(Infrastructure gRPC `SetIntersectionPhase`).
  - 접근, 정지, 출발, 황신호는 그대로 나온다. 바꾼 기록은 episodes.jsonl `red_cap`에 남는다.
- **출발 개입 (`takeover.py`, `TAKEOVER=1`):** 차가 `TAKEOVER_S`(적신호 상한 + 5 s = 25 s) 넘게 서 있고, 쉐도가 "출발"이라고 판단하면 쉐도 control 명령을 실제 명령으로 넘긴다.
  - 쉐도의 "출발": 계획 속도 > 0.5, yield·AEB 1 s 동안 없음. 신호는 따로 보지 않는다(적신호면 velocity_planner가 계획을 0으로 만든다. 신호 검사를 따로 두었더니 좌회전 화살표에서 막혔다: run f CP6 900 s).
  - 3 m/s, 5 s, 또는 출발 판단이 깨지면 정책에게 돌려준다. 다시 개입하려면 새로 N s를 서 있어야 한다.
  - 넘기는 방법: 스택 master의 `/morai_udp_bridge/control_cmd`에 500 Hz로 낸다(speed_filter 20 Hz보다 빠름).
  - 개입 구간은 `rec_w1/takeover.jsonl`에 남고, 라벨 `intervened`가 된다. eval_manager·rl_episode의 stuck 기준은 60 s라 N보다 길다.
- **쉐도 차선 변경 앞당기기 (`shadow_patches/early_lane_change.patch`, 빌드 때 사본에만 적용):**
  - 원인: lanelet2 `shortestPathVia`는 차로를 바꿀 수 있는 마지막 lanelet에서 바꾼다. frenet_path는 후보를 "경로 lanelet 위 진행 정도 → 경로 중심선과의 평균 거리" 순으로 고르므로, 50 m 앞이 그 전환점에 닿을 때에야 바꿨다.
  - 고침: `global_path.hpp`가 경로를 받은 뒤, 같은 방향 평행 차로가 끝까지 이어지는 곧은 구간의 가장 이른 lanelet으로 변경을 당긴다(차선 표시가 허용하는 변경만, 체크포인트 lanelet은 빼지 않음).
- **고주로 차로 변경 (`shadow_patches/frenet_gap.patch`, frenet_path 구간 11–13, 10/28 지시):**
  - 시작: 바꿀 수 있게 되는 첫 지점(위 패치). 전환 길이는 3 s × 속도 이상이다(원래 8–16 m라 23 m/s에서 곡률 속도가 5.7 m/s로 떨어졌다).
  - 틈: 목표 차로의 뒤차(BEV가 보는 20 m 안)는 거리 10 m·TTC 3 s 이상, 앞차(80 m 안)는 10 m·TTC 2 s 이상이어야 바꾼다. 목표 lanelet이 시작되기 전에 있는 뒤차는 그 시작점에서 곧게 뒤로 늘려 본다.
  - 틈이 없으면 차로를 지킨다. 지금 차로가 끝나거나(합류 차로) 그 앞에서 경로로 돌아갈 수 없게 되면(실선), 그 8 m 앞에 서서 기다린다.
  - 차로 변경 후보만 80 m다(3 s × 27 m/s가 들어가게). 후보 순위는 모두 처음 50 m로 매긴다. run f에서는 80 m 끝점이 경로 진행을 더 받아, 경로 차로에 들어간 차가 왼쪽 차로로 되돌아갔다(CP12를 14번 놓침).
  - 움직이는 차는 `shadow_objects.py`가 구간 11–13에서만 계획기에 넘긴다.
  - 쉐도의 구간 판정 반경은 25 m다(원래 10 m). CP11 공식 Spawn이 계획기 CP 점에서 21 m 떨어져, 복귀 뒤 CP12까지 구간이 0이었고 그동안 틈 검사가 꺼져 있었다.
  - **속도 차 규칙 (사용자 10/28):** 목표 차로 흐름(모델 BEV가 보는 뒤 20 m–앞 60 m 차들의 평균 속도, 차가 없으면 차이 0)과 자차 속도 차가 5 m/s 이하일 때만 바꾼다.
    - 흐름이 5 m/s 넘게 느리면 자기 차로에서 2 m/s²로 먼저 줄인다. 빠르면 velocity_planner가 올릴 수 있는 만큼만 올린다(곡률·상한은 넘지 않는다).
    - 지금 차로가 끝나거나 경로를 잃기 3 s 주행 + 8 m 전부터는 강제 합류다: 뒤 20 m가 비고 앞 간격이 10 m 이상이면 바꾼다.
    - 합류를 시작할 때마다 쉐도 rosout(`/root/shadow/n/ros/log/*/rosout.log`)에 `frenet merge: v … flow … n … forced …` 한 줄을 남긴다.
  - "경로를 잃음"은 그 차로에서 경로 lanelet에 다시 닿을 수 없을 때(차로 변경 포함 400 m 안)만이다. 공식 차로가 lanelet2 경로와 떨어져 달리다 다시 붙는 곳(s 1420 → 1591)에서 세우지 않는다.
  - 한계: 틈을 만들려고 가속하지는 않는다(속도는 velocity_planner 그대로). 정지 속도는 path_planner 점 속도로 들어가 velocity_planner의 곡률 제한을 거치지 않는다.
- B2: spool은 `runs/dagger/shadow_debug/spool/`, 기록·결과는 `runs/dagger/shadow_debug/<이름>/`에 둔다.
- **시드 (10/28):** 장면은 `seed × 1000 + k`(k = 에피소드 번호)다.
  - 시드 구간: 0–999 평가(영상 제공기 표준 28000–28007, 29, 7 등. 학습에 넣지 않는다), 1000–1999 DAgger, 2000–2999 PPO.
  - `actor.sh`는 `SEED_BASE`가 꼭 있어야 한다. world n의 시드는 `SEED_BASE + n`이고, spool 이름도 `NAME_s<시드>_wn`이다. 그래서 여러 상자의 B2 경로가 겹치지 않는다. 상자마다, 띄울 때마다 다른 구간을 준다(예: DAgger `1000 + 100 × 라운드 + 10 × 상자`). RL_ARGS에는 `--seed`를 넣지 않는다.
  - `rl_episode`는 출력 폴더에 이미 있는 epK 다음 번호부터 이어 간다. 다시 띄워도 같은 장면을 반복하지 않고, epK를 덮어쓰지 않는다.
  - 디버그(`dagger_box.sh`, `dagger_pod.sh`) 기본 시드는 1990이다. 디버그 기록은 학습 spool에 넣지 않는다.
- 결과는 `~/morai-eval/dagger/<이름>/`로 가져오고(엔진 덤프 제외), 끝나면 상자를 지운다.

## 10/28 디버그 실행 (vast i9-12900K 5090, $0.775/h)
- 기록: 정책 에피소드 3개(4,698판단)와 전문가 직접 주행 2개(2,476판단).
  - 정책: 전체 랩 2개(둘 다 CP7·CP9에서 stuck), 고주로 1개.
  - 전문가: 전체 랩 1개(CP8 회전교차로 진입에서 NPC와 충돌), 고주로 1개.
- 라벨 오차(전문가 직접 주행, 실제로 달린 6.4 s와 비교, t0 정보만): ADE 평균 3.18 m(중앙 1.52), 횡 0.35 m, 종 3.09 m, 3 s ADE 0.84 m.
  - 남은 오차의 대부분은 적신호 정지 중 녹색 전환이다. 미래 정보를 쓰지 않는 한 남는다.
  - 참고로 기록된 이후 계획을 쓰면 1.45 m지만 미래 정보라 쓰지 않는다. 결과: `~/morai-eval/dagger/shadow-20261028c-hw/report/`.
- 채점기: 적신호 행(정책 1,033, 전문가 638)에서 정지선을 넘는 라벨은 0이다. fail은 정책 2.7 %, 전문가 2.2 %다.
  - 차선 회피 가능 위반은 곡선보다 s 250–350(차선 변경·정지선 315.9), s 400–450(4188 교차로), s 1100(고주로 진입)에 몰린다.
  - 룩어헤드를 0.6 → 0.4 s로 줄이면 오히려 나빠졌다(정책 에피소드 하나: 차선 fail 26 → 54, 전체 fail 59 → 140). 그래서 pp.yaml 0.6 s를 유지한다.
- 정책 주행 타이밍(쉐도 붙임): 엔진 왕복 p50 120 ms, 모델 p50 78 ms다. 쉐도가 없던 기준과 같다.
  - CPU는 23코어 중 6.3코어를 썼다(쉐도 1.0).
  - late는 63–99 %다. 이 상자는 RTF가 약 1.0이라 τ 100 ms가 벽시계 100 ms이기 때문이다. 쉐도와는 상관없다.

## 10/28 두 번째 실행 (vast 9950X3D 5090, $0.762/h, 32분 약 $0.41, run d)
- 정책 2랩(5,135판단, CP12·CP8에서 체크포인트 놓침으로 끝남), 전문가 1랩(4188에서 적신호 통과 뒤 stuck).
- 적신호 상한: 208.5 m에서 9.5·10.8 s, 315.9 m에서 8.1 s 대기 뒤 녹색으로 바꿨다. 정책 행 중 적신호 32 %.
- 출발 개입 2번(각 약 1.3 s 만에 3 m/s).
- 채점기(정책): pass 73 %, pass_unavoidable 20 %, fail 4.9 %, unrecoverable 0.3 %. 적신호 행에서 정지선을 넘는 라벨은 0이다.
  - 전문가 쪽 13행은 정지 불가(`go_unstoppable`)·선 넘음(`go_past`) 규칙에 따른 통과다.
- 차선 변경 앞당기기: 고주로에서 쉐도 계획의 차로 이동이 약 60–200 m 앞당겨졌다(그림 `report/lane_changes_plan_highway.png`). 4188 적신호 통과는 그 근처에 차로 이동이 없어 패치와 무관하다(황신호 진행 판단).
- dagger-engine LoRA 교체 확인 통과(교체 0.3–0.45 s, 비트 일치, model_ms p50 78.9).
- late 31 %(RTF 0.8–1.0).

## 10/28 run e (vast 9950X3D 5090, 53분 약 $0.67)
- 정책 전체 랩 1개(복귀 포함, 400 s, 4,041판단): 복귀 6번, 출발 개입 1번, 적신호 상한 2번.
  - 라벨: 빈 행 8.8 %(채점기 fail 218, unrecoverable 2, 복귀 뒤 계획 없음 89, 복귀 뒤 이력 없음 47), source 1 3.9 %, 적신호 440행 중 선 넘는 라벨 0.
- 상자가 전문가 주행 도중 재부팅됐다(호스트 uptime 3분).
  - 시작 명령이 B2 묶음을 다시 풀어 서버 코드가 옛 버전으로 돌아갔다. 코드를 다시 올려 이어 했다.
  - dagger_pod.sh는 이제 ssh 순간 장애를 끝으로 보지 않는다.
- 프레임 미리 전처리(2e4a2bb), boot2와 같은 구성:
  - late 0.11 %(boot2 70 %), server_ms p50 102.6 / p99 134.9, prep 3.5 ms(25 ms).
  - test_frame_stream 600개 동일. model_server 시험은 `dump.id` 불일치로 멈췄다.

## 10/28 run f (vast Ryzen 9 7945HX 5090, $0.806/h, 53분 약 $0.72)
- 정책 전체 랩 1개(900 s 시간 제한, 5,015판단): CP6 좌회전(4356, s 609.9) 정지선 26 m 앞에 서서 끝났다.
  - 적신호 상한은 15번 걸렸다(609.9에서 14번). 출발 개입은 7번이었고, 정체 중에는 0번이었다.
  - 원인 1: takeover가 LEFT·RED_LEFT를 정지 신호로 봤다. 신호 검사를 지웠다. 같은 기록으로 개입 판단만 다시 돌리면 정체 중 처음 t 415.7 s에 개입한다(전 규칙은 0번).
  - 원인 2: rl_episode의 stuck은 eval_manager처럼 "적색 점등·직진 녹색 없음" 시간을 뺀다. 그래서 RED_LEFT 시간도 빠져 복귀가 없었다. 평가와 맞추려고 그대로 둔다.
- 전문가 직접 주행(고주로 패치 첫 판): 679 s, 복귀 17번(missed_cp12 14). 원인과 고침은 위 "고주로 차로 변경"에 있다.
  - 고주로 차로 변경 속도: 전(c-hw) s 1620에서 60 → 25 km/h. 후(f) s 1118 51 → 51, 1365 48 → 47, 1477 51 → 43 km/h.
- 회전교차로(구간 9, 차량 yield 끔): 링 행 587개 중 6.4 s 내내 서 있는 라벨 34개(5.8 %), 가장 긴 연속 1.6 s. 채점 pass 494, pass_unavoidable 88, fail 5(피할 수 있는 충돌). 링 안에서 같은 방향 앞차를 따라간 경우는 이 기록에 없었다.
- 고친 패치 확인(Mac 컨테이너, `morai-stack:c931a5b`, 기록 재생, 상자 없음):
  - A(run f 첫 통과 입력 그대로): 경로 차로를 s 1626.5부터 1702까지 지켰다(전: s 1660에서 왼쪽으로 되돌아감).
  - B(왼쪽 차로 14 m/s, 목표 차로 빔): 점선 시작 직후 s 1631.8에서 바꾸기 시작하고 계획 속도 13–14.7 m/s를 지켰다. 106주기 중 7주기는 왼쪽 차로 유지였다(입력이 움직이지 않는 열린 고리).
  - C(B + 목표 차로 6.5 m 뒤 같은 속도 차): 변경을 계속 거절하고, 계획 속도를 s 1701(실선 8 m 앞)에서 0이 되게 낮췄다.
  - D(납품본 1랩 추돌 장면 입력): 모델이 NPC 48 앞으로 차로를 바꾼 334–336 s에 쉐도는 바꾸지 않고 섰다(계획 속도 0).
  - 그림: `~/morai-eval/dagger/report_20261028/highway_cp12_lane_change.png`, `roundabout_entry.png`.

## 쉐도 로컬 재생 (상자 없이, Mac Docker, 10/28)
기록된 주행(쉐도 기록 `rec_w1`)을 로컬 컨테이너 쉐도에 다시 넣어, 쉐도가 그 상태에서 무엇을 계획하는지 본다. 열린 고리라 차는 기록대로 움직인다. 상태별 계획만 읽는다.
```bash
W=<이 worktree>; F=$W/../MORAI-Autonomous-Driving-System-collect; D=~/shadow-replay; mkdir -p $D/gt
COPYFILE_DISABLE=1 tar -C $F -cf $D/shadow_src.tar --no-mac-metadata --no-xattrs --no-fflags --exclude=.git --exclude=./build --exclude=./devel .
cp ~/.claude/skills/morai-parallel-collect/scripts/wsl/*.py $D/gt/
cd $W/src/e2e_driver/tools   # 입력: 기록 rec_w1의 벽시계 구간 T0..T1
uv run --no-project --with numpy python pod/shadow_replay.py make ~/morai-eval/dagger/<run>/rec_w1 <T0> <T1> $D/a.json
docker run -d --name shadow-replay --cpus 6 -v $D:/workspace/shadow -v $W/src/e2e_driver/tools:/opt/e2e_tools:ro morai-stack:c931a5b sleep infinity
docker exec shadow-replay bash -c 'mv /workspaces/MORAI-Autonomous-Driving-System /workspaces/image-ws; V=/workspace bash /opt/e2e_tools/pod/shadow.sh build'   # 약 10분(rosdep), 패치 포함
docker exec -w / shadow-replay bash /opt/e2e_tools/pod/shadow_replay.sh /workspace/shadow/a.json   # 입력 여러 개 가능
uv run --no-project --with numpy python pod/shadow_replay.py plans $D/rec_w1 $D/marks.txt
```
- 패치를 바꾸면 컨테이너 안에서 `$V/shadow/ws`와 `BUILT`을 지우고 다시 `build` 한다.
- 합성 입력도 같은 JSON(`ego [t, x, y, yaw, v]`, `obj [t, [[id, label, x, y, yaw, vx, vy, va, sx, sy], ...]]`, 속도는 물체 좌표)이면 된다. 10/28 고주로 시험 A–D(위 "run f")가 이렇게 만들어졌다.

## 라벨 종방향 오차 분해 (전문가 직접 주행 3회, 3,805판단, t0 정보만, ADE 평균 3.07 m)
분류에만 기록된 미래를 쓰고, 라벨에는 쓰지 않는다.

| 분류 | 판단 비율 | ADE 평균 / 중앙 | 전체 오차 중 |
|---|---|---|---|
| 6.4 s 안에 신호가 바뀜 (t0 원칙상 피할 수 없음) | 21 % | 6.0 / 2.5 m | 41 % |
| yield·AEB 작동 (물체의 미래 행동, 대부분 피할 수 없음) | 22 % | 4.5 / 3.1 m | 33 % |
| 그 밖의 주행 중 (모델 오차, 고칠 수 있음) | 26 % | 2.9 / 1.6 m | 25 % |
| 정지 상태 | 31 % | 0.2 / 0.0 m | 2 % |

- 고칠 수 있는 부분에는 큰 체계적 편향이 없다(6.4 s 속도 편향 −0.85 m/s, 종방향 끝 평균 −1.35 m, 중앙 +1.76 m). 대부분 흩어짐이다.
  - 예: 45 m 너머의 곡률, 계획기의 정지 장애물 반응, 회전교차로 합류 판단.

## 알려진 문제
- **Object Info 차량 속도는 물체 좌표다.** 수집 노드(gt-yield·gt-aeb)는 이를 지도 좌표로 읽는다. 그래서 남북으로 달리는 앞차를 가로지르는 차로 보고 세운다. R2 수집 때도 그랬을 것이다. 라벨은 지도 좌표로 돌려 쓴다. 수집 노드 코드는 그대로 두었다.
- yield가 켜진 판단(앞차 따라가기, 보행자)의 라벨 오차가 가장 크다. 등속 예측과 노드의 켜짐·꺼짐 반복 때문이다.
- 정지 상태에서 녹색으로 바뀌는 순간은 t0 정보만으로는 맞힐 수 없다(라벨은 정지, 실제는 출발). 사용자 결정대로 정지 라벨을 둔다.
