# NPC 점군 → 미래 궤적 ML / 특권 수집 주행기 승계

작성일: 2026-10-15 KST.

## 1. 다음 세션에서 먼저 알아야 할 것

현재까지 설계 논의와 기존 코드의 읽기 검토만 수행했다. 시뮬레이터 조작, 수집 주행, 학습, Core 선택 변경, 구현 및 테스트 실행은 하지 않았다. 이 파일 작성 요청은 구현·실행 승인이 아니다.

사용자의 목표는 제한된 대회 환경에서 NPC의 미래 궤적을 LiDAR 점군으로 직접 예측하는 것이다. 로터리에서는 ego의 진입 → 탈출에 해당하는 짧은 ¼원 관측 구간에서 잘 동작하면 충분하다. 고속주행로도 원래 관심 범위에 포함되지만, 최근 논의는 로터리 수집기를 먼저 구성하는 방향이다.

가장 최근 사용자 제안은 새 주행기를 만들기보다 기존 `footprint_prediction.hpp`와 `roundabout_merge.hpp`를 GT bbox에 연결해 수집용 주행기로 재사용하자는 것이다. 코드 검토 결과 재사용 가능한 기반임을 확인했다. 실제 주행 성능은 미확인이다.

## 2. 사용자가 명시한 요구와 선호

- 범용 자율주행이나 다른 지도까지의 일반화가 목적이 아니다. 실제 사용할 대회 구간에 집중한다.
- 입력은 해당 객체의 map 좌표 클러스터 점군, 타깃은 미래 n개 시점의 실제 GT 위치, 자세로 두고 싶다.
- 관측 방향, 추정 속도, 차선 ID 같은 수작업 피처를 임의로 추가하지 않는다. 사용자는 이전 ML 시도에서 불필요한 일반화와 피처 추가로 실패했다고 설명했다. 해당 실패 원인은 이번 대화에서 직접 조사하지 않았다.
- 데이터가 충분하면 제한된 구간의 위치·관측 형상·NPC 원호 주행 규칙을 학습시키는 방식에 관심이 있다.
- PointNet++보다 큰 모델도 고려한다. 큰 모델로 성공한 뒤 필요하면 증류하는 방안에 열려 있다.
- 추론 목표로 약 5 Hz를 제시했다. 실제 GPU 종류·여유·전체 처리 지연은 확인하지 않았다.
- 대회 본선 규정과 허용된 랜덤화 범위 안에서 데이터를 충분하고 균형 있게 수집해야 한다는 데 명시적으로 찬성했다.
- 균형 수집 대상: ego 대기/즉시 진입, NPC 거리·속도 차이, 감속·정차·재출발, 다른 차량에 의한 부분 가림.
- 수집 주행기는 GT를 쓰는 특권 방식도 허용 가능한 설계로 논의했다. 수집기 구현·실행까지 승인한 것은 아니다.
- 사용자에게 공손하게 응대하고 불필요한 검증·테스트를 줄인다. 독립 작업은 가능한 범위에서 subagent로 병렬 수행한다.

## 3. 센서 문제와 앞선 설명의 정정

사용자가 지적한 핵심은 관측 방향에 따라 클러스터 형상이 바뀌고, 최소 OBB 중심도 움직이므로 중심 차분을 차량 속도로 볼 수 없다는 점이다. 단순 평활화만으로 관측 면 변화에 따른 편향을 제거할 수 없다.

앞선 답변은 처음에 안정적인 객체 위치·속도가 이미 있다는 전제로 경로 예측을 설명했다. 이 전제를 철회하고 센서 문제를 인정했다. 표면 적합 기반 추적기도 대안으로 논의했지만, 현재 유력 방향은 데이터 기반 직접 궤적 예측이다. 표면 추적기를 필수 선행 작업으로 다시 요구하지 않는다.

TF에는 ego 위치·자세가 들어 있다. 현재 Pipeline은 점군 stamp에 가까운 TF 등 메시지를 묶어서 전달하는 설계다. 최신 TF를 잘못 쓰는 문제가 이미 존재한다고 단정하거나, 별도의 ego pose 입력을 무조건 추가하려 하지 않는다. scan 내부 점별 시간 왜곡은 실제 확인될 때 검토할 별도 문제다.

## 4. ML 구성: 제안과 확정 사항을 구분

사용자가 명시한 계약은 순수 map 점군 → 미래 GT 궤적이다. 다음은 이를 유지하는 **에이전트 제안이며, 입력 이력 길이·모델·출력 시점은 아직 최종 확정되지 않았다.**

```text
같은 객체의 시간 순서가 있는 최근 점군
[P(t−0.8), P(t−0.6), P(t−0.4), P(t−0.2), P(t)]
    → 현재 위치 p(t) 및 미래 위치 p(t+Δ), …, p(t+nΔ)
```

- 단일 점군도 위치별 평균 주행을 학습할 수 있으나, 같은 위치·자세의 정차 차량과 이동 차량을 일반적으로 구별할 정보는 부족하다. 여러 프레임으로 시간 정보를 제공하는 방안을 추천했다.
- 고정 간격의 프레임 슬롯으로 순서를 유지하면 입력은 XYZ 점군만으로 구성할 수 있다. 점별 대응이나 별도 추정 속도·관측 방향은 필수가 아니다.
- 여러 점군을 시간 구분 없이 합치거나 프레임마다 중심을 독립적으로 빼서 정렬하면 이동 정보가 손실될 수 있다.
- map 위치는 이 목적에서 유용한 정보다. 구간별 고정 원점 차감·고정 스케일은 가능하지만, 매 프레임 클러스터 중심 정규화나 불필요한 무작위 회전·이동 증강은 피하는 방향을 제안했다.
- 현재 위치도 출력의 t+0 점으로 학습시키면 흔들리는 관측 bbox 중심을 먼저 차량 중심으로 확정할 필요를 줄일 수 있다.
- 후보: PTv3 공간 인코더 + 시간 Transformer + 좌표 회귀 head. 대안: P4Transformer 계열 시공간 점군 인코더.
- PTv3 자체는 궤적 예측기가 아니다. 이 과제에서 PointNet++보다 우수한지, 5 Hz가 가능한지는 미확인이다.
- 작은 모델 하나와 큰 모델 하나를 같은 데이터로 비교하고, 실제 실패가 나오기 전부터 다중 궤적·복잡한 피처를 늘리지 않는 방향을 제안했다.

참고 원문:

- [FutureDet: raw LiDAR에서 미래 객체 위치 직접 예측](https://arxiv.org/abs/2203.16297)
- [PTv3 공식 구현](https://github.com/Pointcept/PointTransformerV3)
- [P4Transformer 공식 구현](https://github.com/hehefan/P4Transformer)

## 5. 수집 주행기 재사용 경로

제안한 연결은 다음과 같다. 아직 적용되지 않았다.

```text
MORAI Object Info 원본
 → map 좌표의 보정된 GT bbox / interfaces::Objects
 → FootprintPrediction
 → RoundaboutMerge + 지정 RDDF
 → 기존 Velocity Planner / Control / Bridge

LiDAR + 실제 Object Info 이력
 → 객체 대응·시간 정렬
 → 입력 점군과 실제 미래 GT 궤적 데이터셋
```

수집 주행에는 예측 Core의 결과를 사용할 수 있지만, **ML 정답은 나중에 실제로 수신한 GT 위치**로 만든다. 예측 Core 결과를 정답으로 쓰지 않는다.

특권 수집기는 정확한 객체 상태로 ego를 움직이는 도구다. 이를 만들 수 있다고 해서 점군에서 상태·궤적을 알아내는 ML이 불필요해지는 것은 아니다. 반면 비특권 주행기가 이미 충분한 성능을 낸다면 별도 ML의 필요성을 재평가한다.

## 6. 확인한 기존 Core의 동작

### FootprintPrediction

- 파일: `src/path_planner/include/path_planner/cores/footprint_prediction.hpp`
- 등록: `src/path_planner/config/path_planner/registrations/footprint_prediction.yaml`
- 입력 `interfaces::Objects`의 map bbox 중심·방향·크기, ID, header.stamp로 계산한다.
- ego/competition/tf/imu/route 인자는 전달하지만 예측 계산에는 사용하지 않는다.
- ID별 최근 1초 bbox 이력에서 body 속도와 yaw rate를 추정한다. 입력 velocity/acceleration 필드는 사용하지 않는다.
- 같은 stamp가 재사용되면 새 시간 관측처럼 누적하지 않고 최근 관측을 대체한다.
- 관측이 0.5초보다 오래 끊기면 이력을 초기화하고, 시간 역행에도 이력을 초기화한다.
- 일정 body 속도·선회율로 +0.5초부터 +5.0초까지 10개 미래 bbox를 만든다.
- 첫 관측만 있으면 정지 예측이고 속도·가속도 availability는 false다.
- GT bbox에는 적합한 출발점이지만 noisy LiDAR bbox의 중심 흔들림을 해결하는 Core는 아니다.
- 최근 운동 평균을 쓰므로 감속·재출발 반응이 지연될 수 있다. 탈출 시 곡률 변화나 미래 제동을 완벽하게 예측하지 않는다.

### RoundaboutMerge

- 파일: `src/path_planner/include/path_planner/cores/roundabout_merge.hpp`
- 등록 입력: `[ego, competition, objects, predictions, stamp]`.
- RDDF에서 ego 앞의 경로를 잘라 경로 형상을 유지한다. route 인자로 경로를 생성하는 Core가 아니다.
- 현재 객체 bbox와 미래 bbox 점유를 사용해 ego의 이동 중 bbox 충돌을 검사한다.
- 가속도·감속도·jerk·횡가속도 제한 안에서 경로상의 속도·가속도 프로파일을 탐색한다.
- 횡방향 우회보다는 대기·감속·가속으로 빈틈에 합류하는 역할이다.
- 가능한 계획이 없으면 경로 속도를 0으로 설정해 정지를 요청한다. 무충돌 보장은 검증되지 않았다.
- 계획 시각과 객체 관측 시각 차이를 반영한다.
- 예측 범위 밖에는 제동 tail을 붙이며 재계획을 전제로 한다.
- 현재 파라미터 파일의 RDDF: `../map/2026_molit_comp_global_path.txt`.
- 목표속도 8 m/s, 경로 길이 40 m 등은 **현재 코드 설정값**이지 대회 NPC 랜덤화 허용 범위가 아니다.

### 현재 선택과 후단 연결

- `src/path_planner/config/path_planner/selection.yaml`에서 섹션 9 prediction은 `npc_history_prediction.hpp`, local은 `roundabout_merge.hpp`다.
- `footprint_prediction.hpp`는 등록되어 있으나 현재 로터리 prediction 선택은 아니다.
- Velocity Planner inject Core는 NaN 속도만 채우고, 이미 주입된 0 포함 유한 속도와 해당 가속도를 보존한다.
- 따라서 RoundaboutMerge의 속도 계획을 후단으로 전달할 수 있는 기존 구조다. 실제 제어·주행은 이번 대화에서 시험하지 않았다.

## 7. Object Info → GT bbox에서 필요한 처리

참고 파일:

- `src/visualizer/scripts/object_info_udp.py`
- `src/visualizer/scripts/udp_receiver.py`
- `src/visualizer/include/visualizer/cores/object_info.hpp`
- `src/visualizer/config/object_info_visualizer/parameters/object_info.yaml`
- `src/interfaces/msg/Object.msg`

현재 수신기는 Object Info의 source timestamp를 읽고 ID·type·position·heading·size를 파싱한다. velocity는 파싱하지 않지만 FootprintPrediction에는 입력 속도 필드가 필요하지 않다.

Object Info의 reported position과 size를 그대로 `interfaces::Object`의 bbox 중심·크기로 복사해서는 안 된다. visualizer에는 다음 변환과 보정 프로파일이 이미 있다.

```text
map bbox 중심 = reported origin + 회전 × local bbox center offset
bbox 크기 = 보정된 실제 bbox 크기
```

Ioniq5/K5/Universe 등의 프로파일과 heading 보정이 있다. 프로파일 주석에는 2026-10-10, 25.S4.MolitComp03 실측이라고 기록되어 있다. 신규·동일 제원 차량 등에서는 프로파일 식별 한계를 확인해야 한다. 해당 모든 랜덤화 차량의 보정이 완료됐다고 가정하지 않는다.

`interfaces/Object.msg`는 position과 bbox_pose.position을 기하학적 bbox 중심으로 요구한다. 정답을 후륜 차축 기준으로 만들지 bbox 중심으로 만들지는 아직 최종 결정되지 않았다. 학습·평가·계획에서 기준을 섞지 않는다.

루트 README는 visualizer 이외 패키지의 `/visualizer/*` 구독을 금지한다. planner가 `/visualizer/object_info`를 직접 구독하도록 연결하지 않는다. 원본 Object Info를 수집용 입력으로 별도 연결하고 보정 로직·정적 계수를 재사용하는 방향을 논의했다. 최종 파일 배치와 입력 계약은 미확정이다.

## 8. 데이터 밸런스와 누수 방지

- 대회 규정·공식 안내로 확인한 랜덤화 항목과 범위만 본선 범위라고 부른다. 비공개 수치를 임의로 채우지 않는다.
- 조건별 실제 발생량을 집계해 부족한 조건을 추가 수집한다. 똑같은 주행을 반복한 프레임 수만으로 충분하다고 판단하지 않는다.
- 내장 주행기 또는 수집기가 항상 오래 기다리면 NPC 감속·상호작용 데이터가 부족할 수 있다.
- ego 진입 시점·속도·대기를 조절하는 회차 관리와 실제 NPC 움직임·가림 집계가 필요하다. 구현은 아직 없다.
- 입력 점군은 실제 운용의 클러스터 추출·연결 결과를 사용하고, GT는 객체 대응과 정답 생성에 사용한다.
- GT bbox로 완벽히 자른 점군·GT ID로 완벽히 연결한 이력만 학습하면 실제 클러스터 분할·합침·연결 오류를 처음 만나게 된다.
- 객체 이력을 연결하는 작업은 남지만, 정확한 차량 bbox·속도를 먼저 추정해야 하는 것은 아니다.
- 동일 객체 ID라도 회차·재생성 구분을 기록한다. 객체가 사라져 미래 GT가 없으면 유효 마스크를 두고 0 좌표로 채우지 않는다.
- 입력창·미래 정답이 겹치므로 train/validation/test는 프레임 랜덤 분할 대신 주행 회차 단위로 나눈다.
- 대상 차량 점군만으로 앞으로 ego가 취할 행동이나 모든 상호작용을 알 수는 없다. 점군 순수 입력 계약을 임의로 바꾸지 말고, 실제 실패 양상과 예측 시간 범위를 보고 대응을 정한다.
- 평가 제안: 보지 않은 회차의 1/2/3초 위치 오차와 상위 꼬리, 충돌 구간 도착 시각 오차, 객체 누락·연결 오류, 전체 관심 객체 처리 지연.
- 5 Hz는 200 ms 주기다. 예측 기준 stamp를 보존하고 결과 나이를 계획에서 반영해야 한다. 실측 속도·지연·허용 오차 기준은 미확정이다.

## 9. MORAI 기본 주행기·공식 자료 검토 범위

내장 주행기는 `AV-MORAI Built-In`이다. 확인한 24.R2 매뉴얼은 기존 Cruise를 Built-In, Auto를 ExternalCtrl에 대응시킨다. 일반 NPC 설명에 ACC·곡률 감속 등이 있으나, 대회 빌드 ego가 원하는 로터리 출구를 선택하고 교차 차량에 양보해 무충돌 주행한다는 근거는 없다. 따라서 현재는 기존 Core를 쓰는 특권 수집기 재사용안이 더 구체적이다.

`morai-docs` 조회는 앞선 대화에서 원격 main 커밋 `5f5e1705c0e4fb57718061970f489dcd948e1d9c`으로 고정했다. 커밋 게시 시각은 2026-10-13 01:29 KST, Discord 수집 경계는 2026-10-13 01:05 KST다. 이 승계 파일 작성 중 최신 원격 재조회는 하지 않았다. 다음 새로운 대회 규정 조사에서는 스킬에 따라 원격 main을 새로 확인한다.

보존 원문 링크:

- [내장/외부 제어 모드, 24.R2](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/manuals/2026-10-03/24.R2/1897c058f2241aa3.html)
- [NPC 내장 주행 설명, 24.R2](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/manuals/2026-10-03/24.R2/9e718d63d251b958.html)
- [로터리 궤적·속도·상호작용 질문](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261007T160608032310Z/1557056211566989373.json#L22) / [10월 7일 MORAI 답변](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261007T160608032310Z/1557056211566989373.json#L47)
- [고속주행로 생성·시작·도착점 질문](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261012T160507159415Z/1558434123692118057.json#L22) / [10월 12일 MORAI 답변](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261012T160507159415Z/1558434123692118057.json#L47)
- [GT 원점·크기 질문](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261002T191916122335Z/1551370595101319189.json#L22) / [9월 21일 MORAI 답변](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261002T191916122335Z/1551370595101319189.json#L47)
- [수집 동기화 질문](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261002T191916122335Z/1547869432402153493.json#L22) / [대회 빌드 Sync 비활성·추가 예정 답변](https://github.com/Team-Stier/morai-stier-codex-plugin/blob/5f5e1705c0e4fb57718061970f489dcd948e1d9c/assets/discord/20261002T191916122335Z/1547869432402153493.json#L168)

Sync 기능 추가 완료는 당시 수집 자료에서 확인되지 않았다. 현재 빌드에서 동기 수집이 이미 가능하다고 가정하지 않는다. 런타임 GT 수신을 본선 주행에서 허용된 입력으로 해석하지도 않는다.

## 10. 다음 세션의 재개 순서와 권한

1. `AGENTS.md`, 루트 README, 관련 패키지 README, 실제 메시지 정의를 다시 읽고 현재 변경 상태를 확인한다.
2. 사용자의 다음 실행 요청 범위를 확인한다. 지금은 설계·코드 검토와 승계 문서 작성까지만 수행했다.
3. 구현을 요청받으면 GT bbox 변환·입력 연결과 FootprintPrediction/ RoundaboutMerge 선택의 필요한 변경 범위를 먼저 확정한다.
4. 기존 Core 재사용을 우선한다. 기본 주행기의 로터리 성능 조사나 새 RL 주행기 개발부터 다시 시작할 필요는 없다.
5. 허용된 랜덤화 범위를 공식 자료에서 확인하고, 회차별 조건·밸런스 집계·기록 형식을 정한다.
6. 승인된 범위에서 작은 GT 기반 수집 주행을 확인한 뒤 반복 수집으로 확대한다. 바로 대량 수집·학습을 시작하지 않는다.
7. 데이터가 준비되면 순수 점군 입력 조건을 유지한 작은 모델과 큰 모델을 비교한다.

기본 역할은 Core 개발자다. 이 대화에서는 리드 역할을 부여받지 않았다. `src/node_pipeline/CORE_REGISTRATION.md`의 범위를 따른다. 기존 다른 Core, node.yaml, launch.sh, 공유 계약 변경 등의 권한을 승계 문서나 일반 구현 요청만으로 확대하지 않는다. README·AGENTS.md·서브모듈도 임의 변경하지 않는다.

이 파일은 대화 승계 메모이며 프로젝트 계약 문서를 대체하지 않는다. 문서·코드 충돌이 발견되면 관련 작업을 보류하고 실제 상위 계약에 맞춘 해결을 사용자와 결정한다.

파일 작성 시작 시 기존 사용자 변경으로 `src/visualizer/test/unit/test_local_path.cpp`가 untracked 상태였다. 이 파일은 건드리지 않았다. 다음 세션에서는 현재 상태를 다시 확인하고 기존 변경을 보존한다.

## 11. 완료·미완료 상태

완료: 사용자 목표와 범위 정리, 기본 주행기 관련 자료 조회, 순수 점군 ML 방향 검토, 기존 footprint/roundabout Core와 속도 전달 구조 읽기 검토, 승계 문서 작성.

미완료: GT 입력 연결, Core 선택 변경, 수집 회차 관리자, 랜덤화 허용 범위 확정, 실제 주행·GT 동기 확인, 데이터셋 생성, GPU/지연 측정, 모델 학습·평가·증류.

원본 대화의 조회용 임시 자료는 사용 후 삭제했다. 보존된 새 수집 데이터나 학습 결과는 없다.
