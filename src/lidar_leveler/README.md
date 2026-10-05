# lidar_leveler

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `pass_through.hpp` Core가 센서 장착 TF로 점군을 `base_link`로 변환한다.

- 구독: `/velodyne_points`, `/vehicle/competition_status`, `/tf_static`
- 발행 연결: `/velodyne_points/leveled`
- 실행: /velodyne_points 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/lidar_leveler.yaml`의 `stage1`은 UNKNOWN=0과 섹션 1~16을 `pass_through.hpp`에 매핑한다.
- `CoreSet`은 지속 Core 객체를 소유한다. `core_dispatch.hpp`의 각 case가 필수 입력 인덱스를 선언·확인하고 실제 메시지를 개별 `process()` 인자로 전달한다. 현재 필수 입력은 cloud(0), tf_static(2)다.
- Pipeline은 단계 목록을 순서대로 실행하며, 단계마다 독립된 HealthChecker·Policy·Selector가 입력 검사·보정·선택을 수행한다. 선택 매핑과 정책 우선순위는 `stage1`에서 읽는다.
- 원본에서 전달·보정한 출력은 입력의 stamp·receipt metadata를 계승한다. 새 반환 변수는 시각 metadata 없이 전달하며, 시각 정보가 없으면 시간 검사를 생략한다.
- 필수 payload가 Missing이면 Core를 호출하지 않는다. 마지막 결과에만 HeaderInjector가 입력 점군 stamp와 `base_link`를 주입하고 한 번 발행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
