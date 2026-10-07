# object_detector

`node_pipeline`의 공통 main·실행기를 사용하는 두 Stage 실행 패키지다. 입력 수평화 점군을 ClusterMatch로 검출하고 Tracker로 추적해 `/objects`를 발행한다.

- 구독: `/velodyne_points/leveled`, `/vehicle/competition_status`, `/section_annotation`.
- 실행: 점군 callback마다 직렬 처리한다. ROS 구독·발행 큐는 1이다.
- Stage1: `ClusterMatch.process(PointCloud2, CompetitionVehicleStatus) → tuple<Objects, CompetitionVehicleStatus>`.
- Stage2: `Tracker.process(Objects, CompetitionVehicleStatus) → Objects`.
- section은 Selector의 메타데이터로만 사용한다. UNKNOWN과 섹션 1~16은 Stage1 `cluster_match.hpp`, Stage2 `tracker.hpp`를 선택한다.
- 각 Core의 모든 process 인자는 필수이며 모든 issue가 없어야 호출한다. 입력 검사·Missing 처리·선택·헤더 주입·발행은 Core 밖에서 수행한다.

Stage1은 계산한 Objects와 입력 차량 상태를 정규 반환값으로 전달한다. 원본 점군은 후속 Stage에 전달하지 않는다. Objects의 시간 메타데이터는 비워 두고 차량 상태의 stamp·수신 시각은 등록의 `time_from: competition`으로 상속한다. Stage2는 자기 입력을 검사·보정하고 Tracker를 호출한다. 중간값에 헤더를 주입하거나 발행하지 않는다.
공통 ROS 경계가 최종 Objects의 header에 입력 점군 stamp와 `frame_id=base_link`를 주입해 한 번 발행한다. 출력은 검사하지 않는다.

## 계산과 상태

ClusterMatch는 자차 점군·지면을 제거하고 클러스터별 bbox를 계산한다. 생성자에 주입된 모델 전체를 관측 점군에 정합하며 검증된 물리 collider를 추정 모델 자세로 강체 변환한다. 정합이 불확실하거나 모델이 없으면 `label=unknown`과 관측 bbox를 유지한다. ID·속도·가속도는 미추정이다. 모델·collider의 파일 로더는 등록하지 않는다.

Tracker는 이전 관측·ID·운동 이력을 내부에 유지한다. 차량 상태의 roll·pitch·yaw와 전방 속도로 자차 이동·회전을 보정하고 map 위치 이력의 다항식 적합으로 속도·가속도를 추정한다. 위치·bbox는 현재 관측의 base_link, 속도·가속도는 map 축 성분이다. 관측 2개부터 속도, 설정한 최소 관측 수부터 가속도의 available을 켠다. 가려진 객체는 내부 수명만 유지하고 출력에는 현재 관측 객체만 포함한다.
운동 계산 시각은 결합된 차량 상태의 header.stamp다. 이전 시각과 추적 상태는 호출 사이에 유지하며 선택 전환 때 Core를 재생성하거나 reset하지 않는다.

## 등록과 설정

`node.yaml`은 입력·Stage·출력을 선언하고 `registrations/*.yaml`은 Core의 순서 있는 입출력·domain Validator·Policy·readonly Params Loader를 연결한다. 모든 노드가 공통 main·실행기를 사용하고 빌드는 타입 연결만 생성한다.
ClusterMatch의 domain Validator는 little-endian FLOAT32 x·y·z 필드와 버퍼 경계를 검사한다. NaN/Inf·frame·시각 검사는 공통 검사이며 보정은 공통 RemoveInvalid를 사용한다.
계산과 전용 Params는 `include/object_detector/cores/`, Loader는 `parameters/`, domain 검사는 `validators/`에 둔다. `config/object_detector.yaml`의 실행·입력 한도·매핑과 `stage1/cluster_match`·`stage2/tracker` 계수 주입 경로를 유지한다.
`launch.sh`는 ROS·워크스페이스 환경을 준비하고 이 패키지만 실행한다. 작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
