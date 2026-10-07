# path_planner

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. GlobalPath → Prediction → LocalPath의 세 스테이지를 순차 실행한다.

- 구독: `/localization/ego`, `/vehicle/competition_status`, `/objects`, `/tf`, `/section_annotation`
- 발행: `/local_path`
- 실행: 20 Hz 주기. ROS 발행·구독 큐는 1이고 callback은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- UNKNOWN=0과 섹션 1~16에서 `stage1/global_path.hpp`, `stage2/prediction_pass_through.hpp`, `stage3/local_path.hpp`를 선택한다.
- CoreSet은 세 지속 Core 객체를 생성·소유한다. 스테이지별 검사·정책·선택·실행 순회·최종 헤더 주입과 발행은 Pipeline이 담당한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

## 스테이지 입출력

```text
msgs = subscribe()
LaneletPath = global_path_core(msgs)
Objects[10] = prediction_core(msgs, LaneletPath)
result = local_path(msgs, LaneletPath, Objects[10])
publish(result)
```

`msgs`는 조립된 구독 메시지들의 표기다. Core에는 그중 필요한 실제 메시지를 개별 인자로 전달하며 Data·Slot·msgs 래퍼를 넘기지 않는다.

| 스테이지 | 실제 Core 인자 | 계산 반환 |
| --- | --- | --- |
| 1: GlobalPath | ego·원본 SectionAnnotation | `lanelet::Optional<lanelet::routing::LaneletPath>` |
| 2: PredictionPassThrough | 원본 Objects·LaneletPath | `std::array<interfaces::Objects, 10>` |
| 3: LocalPath | ego·원본 SectionAnnotation·LaneletPath·예측 배열 | `interfaces::Path` payload |

스테이지 1·2의 연결은 기존 입력 슬롯을 유지하고 Core의 계산 반환값을 뒤에 추가한다. 원본 구독 슬롯은 ego·competition·objects·tf·section 순서이고 LaneletPath는 슬롯 5, 예측 배열은 슬롯 6이다.
미수신 슬롯은 Missing을 유지하며 `core_dispatch.hpp`의 각 case가 실제 Core 인자의 존재를 호출 직전에 검사한다.
수정·패스스루 입력의 시간 메타데이터와 스냅샷 context·token은 유지한다. 새 LaneletPath·예측 배열 슬롯의 시간 메타데이터는 비워 두고 다음 스테이지에서 정상으로 취급하며 값 검사는 수행한다.
세 번째 Core의 반환값은 최종 스냅샷으로 감싸고 루프 밖에서 헤더를 한 번 주입하여 한 번 발행한다. 출력은 검사하지 않는다.

## 예측 배열 계약

Prediction Core의 반환과 LocalPath Core의 입력은 **Objects 메시지를 원소로 갖는 길이 10의 외부 배열**이다. `Objects.objects` 안의 Object 목록을 10개로 제한하는 뜻이 아니다.
각 `predictions[i].objects`에는 해당 미래 시점의 전체 객체 목록이 들어간다.

- `[0]`은 예측에 사용한 `/objects` 시각에서 0.5초 뒤, `[9]`는 5.0초 뒤의 객체 상태다. 목표 시각은 `objects.header.stamp + 0.5 × (i + 1)`초다.
- 모든 원소의 header는 예측에 사용한 원본 `/objects`와 같다. `stamp`에 미래 시각이나 계산 완료 시각을 넣지 않는다.
- 배열 인덱스는 예측 목표 시각을 나타내고 header는 예측에 사용한 관측의 기준 시각·좌표계를 나타낸다. 객체 상태는 원본 `/objects`의 `base_link_level` 좌표 기준으로 표현한다.
- 외부 배열의 길이는 항상 10이다. 객체가 없으면 각 원소의 `objects` 목록은 비어 있다.
- 현재 등록된 PredictionPassThrough는 원본 Objects를 10개 원소에 그대로 복사한다. 위치·속도·가속도 등을 외삽하지 않는다. LocalPath는 예측 배열을 추가 인자로 받고 기존 중심선 경로 계산을 수행한다.

노드는 지도와 체크포인트 파일을 읽고 설정 로더에서 해석해 Core 생성자에 주입한다. GlobalPath는 남은 체크포인트를 순서대로 경유하는 경로를 탐색하며 `/visualizer/global_path`에 중심선을 발행한다.
LocalPath의 경로 길이는 `stage3/local_path/local_path_length_m`으로 설정하며 기본값은 20 m다.

## 좌표계 계약

### #DOWNSTREAM_CONTRACT — perception 객체를 그대로 전달

입력 `/objects`와 PredictedObjects의 모든 원소는 `base_link_level`을 기대한다. 이 frame은 기존 차체 고정 `base_link`와 원점·yaw를 공유하지만 차량 roll/pitch가 제거된 수평 perception 좌표계이다. lidar_leveler가 mounting TF를 한 번 적용하고 점군을 수평화하며, ROI·VoxelGrid·DBSCAN·OBB와 `/objects`는 그 좌표를 유지한다.

현재 PredictionPassThrough는 원본 객체와 header를 10개 원소로 그대로 복사하므로 좌표 변환을 추가하지 않는다. LocalPath는 예측 배열의 위치를 실제 장애물 회피 계산에 사용하지 않고 기존 map 중심선 경로를 계산한다. CompetitionVehicleStatus의 `base_link`, ego와 `/local_path`의 `map` 계약은 유지한다.

### #FRAME_CONTRACT — 서로 다른 frame의 좌표를 혼합하지 않기

객체 중심·bbox pose·유효한 속도·가속도는 `Objects.header.frame_id`를 따른다. bbox_size는 박스 자체 local 축의 전체 길이이다. base_link_level is a perception-frame contract. It is not an alias for base_link. Do not change frame_id from base_link_level to base_link without transforming the actual coordinates.

### #NO_LEVEL_TO_BASE — 입력 경계에서 명시적으로 변환

perception 내부에서는 OBB 계산 후 객체를 base_link로 역회전하지 않는다. 정확히 역회전하려면 해당 점군 수평화에 사용한 동일 timestamp의 roll/pitch 변환을 보존·연결해야 하고, 최신 차량 상태를 다시 사용하면 시간 불일치로 중심과 orientation 오차가 생길 수 있기 때문이다. 향후 tracking·prediction·회피·충돌 검사에서 base_link나 map이 필요하면 해당 알고리즘의 입력 경계에서 메시지 timestamp에 맞는 명시적인 TF 변환을 수행한다. 객체가 수평 frame이라는 사실을 무시하고 기존 차체 좌표로 가정하면 안 된다.
