# object_detector

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. Core는 아직 등록하지 않았다.

- 구독: `/velodyne_points/leveled`, `/section_annotation`
- 발행 연결: `/objects`
- 실행: /velodyne_points/leveled 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/object_detector.yaml`의 Core 매핑은 비어 있다. 라이브러리가 NoCore로 종료하므로 계산 결과를 발행하지 않는다.
- Core 등록은 `include/object_detector/core_set.hpp`의 CoreId·CoreSet TODO에서 수행한다. Core는 `include/object_detector/cores/`에 파일 하나씩 작성하고 실제 메시지를 개별 process 인자로 받는다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
