# path_planner

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. Core는 아직 등록하지 않았다.

- 구독: `/localization/ego`, `/vehicle/competition_status`, `/objects`, `/tf`, `/section_annotation`
- 발행 연결: `/local_path`
- 실행: 20 Hz 주기. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/path_planner.yaml`의 Core 매핑은 비어 있다. 라이브러리가 NoCore로 종료하므로 계산 결과를 발행하지 않는다.
- Core 등록은 `include/path_planner/core_set.hpp`의 CoreId·CoreSet TODO에서 수행한다. Core는 `include/path_planner/cores/`에 파일 하나씩 작성하고 실제 메시지를 개별 process 인자로 받는다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

지도 파일은 노드가 읽어 CoreSet에 원본 바이트로 보관한다. 지도 해석과 Core 인자 연결은 Core 등록 시 해당 노드 adapter에서 구체화한다.
