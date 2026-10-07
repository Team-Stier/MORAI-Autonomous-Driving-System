# object_detector

`node_pipeline` 라이브러리를 사용하는 두 Core 순차 실행 패키지다. stage1에는 `Dbscan`, stage2에는 `ObjectsPassThrough`가 등록되어 있다.

- 구독: `/velodyne_points/leveled`, `/section_annotation`
- 발행 연결: `/objects`, `frame_id=base_link_level`
- 실행: /velodyne_points/leveled 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- 처리: `Core1.process(PointCloud2) → Objects → Core2.process(Objects) → Objects`. 중간·최종 메시지는 `interfaces::Objects` 하나씩이다.
- `config/object_detector.yaml`의 `stage1`·`stage2` 아래에 각각 `section_core_map`·`health_core_map`을 둔다. UNKNOWN=0과 섹션 1~16에서 stage1은 `dbscan.hpp`, stage2는 `objects_pass_through.hpp`를 선택한다. 각 Selector는 같은 section과 각 스테이지 현재 입력의 health를 사용한다. 어느 단계든 Core가 선택되지 않으면 NoCore로 종료하며 계산·발행하지 않는다.
- Core 등록은 `include/object_detector/core_set.hpp`의 FirstCoreId·SecondCoreId·CoreSet에서 수행한다. Core는 `include/object_detector/cores/`에 파일 하나씩 작성한다. CoreSet은 기존 Core ID 구조와 Core 객체를 소유한다. `core_dispatch.hpp`에 필수 입력 목록과 단계별 선택 분기·개별 인자 연결을 둔다. Pipeline이 첫 Core에 구독 점군을 전달하고, 반환된 Objects를 두 번째 Core에 전달한다.
- 중간 Objects는 두 번째 스테이지에서 검사·보정·선택하며 헤더 주입·발행하지 않는다. 최종 Objects는 Pipeline이 입력 점군 stamp와 `frame_id=base_link_level`를 주입하여 `/objects`로 발행한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

노드는 공통 스테이지 루프를 두 번 실행한다. 각 스테이지는 독립된 checker·policy·selector와 `stage1`·`stage2` 설정을 사용한다. CoreSet은 실행을 중계하지 않으며 Pipeline의 case가 소유 객체를 직접 호출한다. 기존 입력의 수정·패스스루 슬롯은 시간 메타데이터를 상속하고, 새 계산 슬롯은 이를 비워 두며 다음 검사에서 그 부재는 정상이다. 스냅샷 context·token은 유지하며 마지막 스테이지 후에만 헤더 주입·발행한다.

## 좌표계 계약

### #BASE_LINK_LEVEL — perception 수평 좌표계

차량 차체가 roll/pitch하면 센서 좌표에서 본 지면과 물체도 기울어진다. RollPitchLeveler는 이 차체 기울기를 제거하여 ROI의 높이·범위, clustering 거리와 OBB의 수평 형상을 같은 수평 좌표에서 계산하도록 한다. 실제 도로 자체의 경사까지 평평하게 만드는 지면 추정 알고리즘은 아니다.

기존 `base_link`는 차량 뒷차축 중심을 원점으로 하는 차체 고정 좌표계이며 차량과 함께 roll/pitch한다. `base_link_level`은 같은 원점과 차량 yaw를 공유하지만 roll/pitch는 0이다. +x는 yaw로 정해지는 수평 진행 방향, +y는 좌측, +z는 중력 반대 방향의 수직 위쪽이다. 단위는 m이고 두 frame의 축은 zero roll/pitch에서 일치한다.

**base_link_level is a perception-frame contract. It is not an alias for base_link.**

### #LEVELING — 장착 변환과 자세 수평화

`/tf_static`의 센서 장착 변환으로 `velodyne` 점을 `base_link`로 옮긴 다음 차량 roll/pitch를 적용하여 `base_link_level`로 표현한다. 장착 이동 `(1.5, 0, 1.403)` m와 장착 회전은 이 첫 변환에서 한 번만 적용한다. ROI·DBSCAN·OBB에서 센서 offset이나 차량 기울기를 다시 적용하지 않는다.

### #LEVELING_ROTATION — 회전 방향과 순서

차량 자세 규약이 `R_world_body = Rz(yaw) Ry(pitch) Rx(roll)`이고 수평 frame의 자세가 `R_world_level = Rz(yaw)`이므로, `R_level_body = R_world_levelᵀ R_world_body = Ry(pitch) Rx(roll)`이다. 따라서 rad 단위 `setRPY(roll, pitch, 0)`을 사용한다. 실제 점의 처리식은 `p_level = Ry(pitch) Rx(roll) (R_mount p_raw + t_mount)`이며 mounting → roll → pitch 순서다. yaw 회전은 추가하지 않는다. 부호를 단순히 음수로 바꾸면 평평한 world ground의 기울기를 상쇄하지 못한다.

### #FRAME_CONTRACT — 계산과 발행의 기준

production의 `/velodyne_points/leveled`, ROI, VoxelGrid, DBSCAN, OBB와 `/objects`는 모두 `base_link_level`을 사용한다. OBB yaw는 그 frame의 +x/+y 평면 기준이고, bbox_size는 박스 자체 local 축의 전체 길이이다. 객체 중심·bbox pose·유효한 속도와 가속도는 `Objects.header.frame_id`를 따른다. `/objects.header.stamp`는 입력 점군의 stamp를 유지한다.

**Do not change frame_id from base_link_level to base_link without transforming the actual coordinates.** 좌표값을 바꾸지 않고 이름만 변경하면 TF나 장애물 계산에서 차체 roll/pitch가 중복 반영될 수 있다.

```text
Raw /velodyne_points (velodyne)
        |
        | static sensor mounting transform
        v
vehicle body reference (base_link)
        |
        | vehicle roll/pitch leveling
        v
base_link_level
        |
        v
ROI
        |
        v
VoxelGrid
        |
        v
DBSCAN
        |
        v
OBB
        |
        v
/objects
frame_id = base_link_level
```

### #NO_LEVEL_TO_BASE — 검출 결과의 역변환을 하지 않는 이유

검출 결과는 OBB를 실제 계산한 `base_link_level`에 유지한다. OBB를 `base_link`로 정확히 역변환하려면 해당 LiDAR frame을 수평화할 때 사용한 동일 timestamp의 roll/pitch 변환을 결과 생성까지 함께 보존·연결해야 한다. DBSCAN/OBB가 끝난 시점의 최신 CompetitionVehicleStatus를 다시 읽으면 차량 이동 중 자세가 바뀌어 수평화에 사용한 자세와 다를 수 있다. 그러면 두 회전이 상쇄되지 않아 객체 중심과 orientation에 오차가 생긴다.

이 불필요한 시간 동기화 복잡성과 좌표 오차 가능성을 perception 내부에 추가하지 않으므로 ROI·DBSCAN·OBB와 `/objects`는 실제 계산 좌표계를 그대로 유지한다. object_detector는 최신 roll/pitch를 다시 읽어 Object를 역회전하지 않는다.

### #DOWNSTREAM_CONTRACT — 소비 모듈의 책임

downstream은 `Objects.header.frame_id`를 확인하고 수평 좌표를 차체 고정 `base_link`로 가정하지 않는다. `base_link`나 `map`이 필요한 모듈은 자신의 입력 경계에서 메시지 timestamp에 맞는 명시적인 frame transform을 수행한다. 위치와 bbox orientation, 유효한 벡터 성분을 함께 변환하며 bbox 자체 축의 길이는 유지한다. 현재 path_planner의 PredictionPassThrough는 header와 객체를 그대로 복사하고 LocalPath는 객체 위치를 장애물 회피 계산에 사용하지 않는다. 향후 prediction·tracking·회피·충돌 검사는 서로 다른 frame의 좌표를 그대로 혼합하면 안 된다.
