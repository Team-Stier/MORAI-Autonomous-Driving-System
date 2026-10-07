# path_planner

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. GlobalPath → Prediction → LocalPath의 세 스테이지를 순차 실행한다.

- 구독: `/localization/ego`, `/vehicle/competition_status`, `/objects`, `/tf`, `/section_annotation`
- 발행: `/local_path`
- 실행: 20 Hz 주기. ROS 발행·구독 큐는 1이고 callback은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- UNKNOWN=0과 섹션 1~16에서 `stage1/global_path.hpp`, `stage2/prediction_core.hpp`, `stage3/local_path.hpp`를 선택한다.
- CoreSet은 GlobalPath·PredictionCore·PredictionPassThrough·LocalPath의 지속 객체를 생성·소유한다. 스테이지별 검사·정책·선택·실행 순회·최종 헤더 주입과 발행은 Pipeline이 담당한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

## 스테이지 입출력

```text
msgs = subscribe()
LaneletPath = global_path_core(msgs)
Objects[10] = prediction_core(Objects, CompetitionVehicleStatus, LaneletPath)
result = local_path(msgs, LaneletPath, Objects[10])
publish(result)
```

`msgs`는 조립된 구독 메시지들의 표기다. Core에는 그중 필요한 실제 메시지를 개별 인자로 전달하며 Data·Slot·msgs 래퍼를 넘기지 않는다.

| 스테이지 | 실제 Core 인자 | 계산 반환 |
| --- | --- | --- |
| 1: GlobalPath | ego·원본 SectionAnnotation | `lanelet::Optional<lanelet::routing::LaneletPath>` |
| 2: PredictionCore | 원본 Objects·CompetitionVehicleStatus·LaneletPath | `std::array<interfaces::Objects, 10>` |
| 3: LocalPath | ego·원본 SectionAnnotation·LaneletPath·예측 배열 | `interfaces::Path` payload |

스테이지 1·2의 연결은 기존 입력 슬롯을 유지하고 Core의 계산 반환값을 뒤에 추가한다. 원본 구독 슬롯은 ego·competition·objects·tf·section 순서이고 LaneletPath는 슬롯 5, 예측 배열은 슬롯 6이다.
PredictionCore의 필수 슬롯은 1·2·5다. `prediction_pass_through.hpp`도 원본 Objects·LaneletPath를 받는 별도 후보로 등록해 보존한다.
미수신 슬롯은 Missing을 유지하며 `core_dispatch.hpp`의 각 case가 실제 Core 인자의 존재를 호출 직전에 검사한다.
수정·패스스루 입력의 시간 메타데이터와 스냅샷 context·token은 유지한다. 새 LaneletPath·예측 배열 슬롯의 시간 메타데이터는 비워 두고 다음 스테이지에서 정상으로 취급하며 값 검사는 수행한다.
세 번째 Core의 반환값은 최종 스냅샷으로 감싸고 루프 밖에서 헤더를 한 번 주입하여 한 번 발행한다. 출력은 검사하지 않는다.

## 예측 배열 계약

Prediction Core의 반환과 LocalPath Core의 입력은 **Objects 메시지를 원소로 갖는 길이 10의 외부 배열**이다. `Objects.objects` 안의 Object 목록을 10개로 제한하는 뜻이 아니다.
각 `predictions[i].objects`에는 해당 미래 시점의 전체 객체 목록이 들어간다.

- `[0]`은 예측에 사용한 `/objects` 시각에서 0.5초 뒤, `[9]`는 5.0초 뒤의 객체 상태다. 목표 시각은 `objects.header.stamp + 0.5 × (i + 1)`초다.
- 모든 원소의 header는 예측에 사용한 원본 `/objects`와 같다. `stamp`에 미래 시각이나 계산 완료 시각을 넣지 않는다.
- 배열 인덱스는 예측 목표 시각을 나타내고 header는 예측에 사용한 관측의 기준 시각을 나타낸다. position·bbox는 관측 시각에 고정한 `base_link` 기준이고 velocity·acceleration은 `map` 축 성분이다.
- 외부 배열의 길이는 항상 10이다. 객체가 없으면 각 원소의 `objects` 목록은 비어 있다.
- PredictionCore는 `map` XY 평면에서 일정 속력·일정 각속도 원형 운동을 예측한다. `ω=(vx·ay-vy·ax)/(vx²+vy²)`이며 접선 가속도는 반영하지 않는다. 수직 속도는 일정하게 유지하고 가속도는 관측값을 보존한다.
- `R_map_base=Rz(yaw+yaw_offset)·Ry(pitch)·Rx(roll)`을 CompetitionVehicleStatus의 RPY로 구성한다. map 변위를 관측 시각 자세의 역회전으로 변환해 관측 position에 더하며, bbox 방향에도 같은 map +z 회전을 적용한다. 미래 자차 위치·자세로 기준을 바꾸지 않는다. `stage2/prediction_core/yaw_offset_rad` 기본값은 -0.023509 rad다.
- 상태와 관측의 stamp 차이 동안 body 각속도를 일정하게 두고 `R_obs=R_map_base·exp([ω_body]×·(objects.header.stamp-status.header.stamp))`로 관측 시각 자세를 복원해 변위·bbox 회전에 사용한다.
- `velocity_available=false`이면 해당 객체의 관측 상태를 유지한다. 속도만 있고 `acceleration_available=false`이면 해당 속도로 직선 예측한다. unavailable 벡터는 계산에 사용하지 않으며 존재 플래그는 보존한다.
- PredictionPassThrough는 원본 Objects를 10개 원소에 그대로 복사한다. LocalPath는 예측 배열을 추가 인자로 받고 기존 중심선 경로 계산을 수행한다.

노드는 지도와 체크포인트 파일을 읽고 설정 로더에서 해석해 Core 생성자에 주입한다. GlobalPath는 남은 체크포인트를 순서대로 경유하는 경로를 탐색하며 `/visualizer/global_path`에 중심선을 발행한다.
LocalPath의 경로 길이는 `stage3/local_path/local_path_length_m`으로 설정하며 기본값은 20 m다.
