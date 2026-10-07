# lidar_leveler

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `roll_pitch_leveler.hpp` Core가 센서 장착 TF를 적용한 뒤 차량 roll/pitch를 수평화해 점군을 `base_link_level`로 변환한다.

- 구독: `/velodyne_points`, `/vehicle/competition_status`, `/tf_static`
- 발행 연결: `/velodyne_points/leveled`, `frame_id=base_link_level`
- 실행: /velodyne_points 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/lidar_leveler.yaml`의 `stage1`은 UNKNOWN=0과 섹션 1~16을 `roll_pitch_leveler.hpp`에 매핑한다.
- `CoreSet`은 지속 Core 객체를 소유한다. `core_dispatch.hpp`의 각 case가 필수 입력 인덱스를 선언·확인하고 실제 메시지를 개별 `process()` 인자로 전달한다. `RollPitchLeveler`의 필수 입력은 cloud(0), competition(1), tf_static(2)다. `PassThrough`는 cloud(0), tf_static(2)를 사용하며 비교·fallback용으로 유지한다.
- Pipeline은 단계 목록을 순서대로 실행하며, 단계마다 독립된 HealthChecker·Policy·Selector가 입력 검사·보정·선택을 수행한다. 선택 매핑과 정책 우선순위는 `stage1`에서 읽는다.
- 원본에서 전달·보정한 출력은 입력의 stamp·receipt metadata를 계승한다. 새 반환 변수는 시각 metadata 없이 전달하며, 시각 정보가 없으면 시간 검사를 생략한다.
- 필수 payload가 Missing이면 Core를 호출하지 않는다. 마지막 결과에만 HeaderInjector가 입력 점군 stamp와 YAML `frame` 값을 주입하고 한 번 발행한다. 출력은 검사하지 않는다.

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

production의 YAML `frame`은 `base_link_level`이다. PassThrough runtime 비교·fallback 설정은 Core 매핑을 `pass_through.hpp`, `frame`을 `base_link`로 함께 지정한다. 두 Core를 서로 다른 출력 좌표계로 같은 설정에서 혼용하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
