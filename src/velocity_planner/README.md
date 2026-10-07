# velocity_planner

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `plain_velocity60.hpp`와 `plain_velocity100.hpp` Core를 등록한다.

- 구독: `/local_path`, `/localization/ego`, `/vehicle/competition_status`, `/traffic_light`, `/tf`, `/section_annotation`
- 발행 연결: `/local_path/velocity_injected`
- 실행: /local_path 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `node.yaml`의 `stage1`은 섹션 11·12를 `plain_velocity100.hpp`, UNKNOWN=0과 나머지 섹션을 `plain_velocity60.hpp`에 매핑한다. Core는 경로 형상·점 순서를 유지하고 TF로 `base_link`에 변환한 뒤 각각 100/3.6, 60/3.6 m/s와 가속도 0 m/s²를 채운다.
- `registrations/*.yaml`은 필수 입력 `path`·`tf`와 `process(path, tf)`의 반환 경로를 연결한다. `validators/transform.hpp`가 `map → base_link` TF 존재를 검사한다.
- 공통 실행기는 지속 Core 객체·조립·section/health 선택·검사·Policy·계산·최종 발행을 담당한다. `/section_annotation`은 선택 메타데이터로만 사용한다.
- 경로 출력은 `time_from: path`로 입력 stamp·수신 시각을 계승한다. 공통 인젝터는 `output.header`의 stamp와 `base_link`를 전체 경로 Header에만 주입한다. Core는 각 경로점의 원본 stamp를 보존하고 frame을 `base_link`로 변환한다. 출력은 검사하지 않는다.
- 기존 `config/velocity_planner.yaml`의 실행 설정과 `stage1` 매핑은 공통 실행기가 읽는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
