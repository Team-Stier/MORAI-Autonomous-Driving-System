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

`base_link_level`은 `base_link`와 같은 차량 뒷차축 중심을 원점으로 가지며 yaw만 따른다. roll/pitch는 0이고, +x는 heading 방향, +y는 좌측, +z는 중력 반대 방향이다. 기존 `base_link`는 차체와 함께 roll/pitch한다.

처리식은 `p_level = Ry(pitch_rad) Rx(roll_rad) (R_mount p_raw + t_mount)`다. static mounting TF를 한 번 적용한 뒤 roll, pitch 순서로 회전하며 `yaw_rad`는 사용하지 않는다.

production의 YAML `frame`은 `base_link_level`이다. PassThrough runtime 비교·fallback 설정은 Core 매핑을 `pass_through.hpp`, `frame`을 `base_link`로 함께 지정한다. 두 Core를 서로 다른 출력 좌표계로 같은 설정에서 혼용하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
