# velocity_planner

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `plain_velocity60.hpp`와 `plain_velocity100.hpp` Core를 등록한다.

- 구독: `/local_path`, `/localization/ego`, `/vehicle/competition_status`, `/traffic_light`, `/tf`, `/section_annotation`
- 발행 연결: `/local_path/velocity_injected`
- 실행: /local_path 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/velocity_planner.yaml`의 `stage1`은 섹션 11·12를 `plain_velocity100.hpp`, UNKNOWN=0과 나머지 섹션을 `plain_velocity60.hpp`에 매핑한다. Core는 경로 형상·점 순서를 유지하고 TF로 `base_link`에 변환한 뒤 각각 100/3.6, 60/3.6 m/s와 가속도 0 m/s²를 채운다.
- `CoreSet`은 지속 Core 객체를 소유한다. `core_dispatch.hpp`의 각 case가 path(0), tf(4)를 필수 입력으로 선언·확인한 뒤 `process(path, tf)`를 호출한다.
- Pipeline은 단계 목록을 순서대로 실행하며, 단계마다 독립된 HealthChecker·Policy·Selector가 입력 검사·보정·선택을 수행한다. 선택 매핑과 정책 우선순위는 `stage1`에서 읽는다.
- 원본에서 전달·보정한 출력은 입력 경로의 stamp·receipt metadata를 계승한다. 새 반환 변수는 시각 metadata 없이 전달하며, 시각 정보가 없으면 시간 검사를 생략한다.
- 필수 payload가 Missing이면 Core를 호출하지 않는다. 마지막 결과에만 HeaderInjector가 입력 경로 stamp와 `base_link`를 주입하고 한 번 발행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

지도 파일은 노드 구성에서 원본 바이트로 읽는다. 현재 등록 Core는 지도 데이터를 사용하지 않는다.
