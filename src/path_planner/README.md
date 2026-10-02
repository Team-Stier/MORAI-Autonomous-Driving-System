# path_planner

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `global_path.hpp`와 `local_path.hpp` Core를 순차 실행한다.

- 구독: `/localization/ego`, `/vehicle/competition_status`, `/objects`, `/tf`, `/section_annotation`
- 발행 연결: `/local_path`
- 실행: 20 Hz 주기. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/path_planner.yaml`은 UNKNOWN=0과 섹션 1~16에서 첫 단계 `global_path.hpp`, 두 번째 단계 `local_path.hpp`를 선택한다.
- `include/path_planner/core_set.hpp`가 두 Core를 소유하고 호출을 연결한다. GlobalPath는 ego와 원본 SectionAnnotation을 받고 두 메시지와 LaneletPath를 반환한다. LocalPath는 이 반환값을 개별 인자로 받아 `/local_path` payload를 생성한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

노드는 지도와 체크포인트 파일을 읽고 설정 로더에서 해석해 Core 생성자에 주입한다. GlobalPath는 남은 체크포인트를 순서대로 경유하는 경로를 탐색하며 `/visualizer/global_path`에 중심선을 발행한다. LocalPath의 경로 길이는 `core2/local_path/local_path_length_m`으로 설정하며 기본값은 20 m다.
