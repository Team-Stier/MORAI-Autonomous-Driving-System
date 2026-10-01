# tf_publisher

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. Core는 아직 등록하지 않았다.

- 구독: `/localization/ego`
- 발행 연결: `/tf`
- 실행: /localization/ego 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/tf_publisher.yaml`의 Core 매핑은 비어 있다. 라이브러리가 NoCore로 종료하므로 계산 결과를 발행하지 않는다.
- Core 등록은 `include/tf_publisher/core_set.hpp`의 CoreId·CoreSet TODO에서 수행한다. Core는 `include/tf_publisher/cores/`에 파일 하나씩 작성하고 실제 메시지를 개별 process 인자로 받는다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

`/tf_static`은 Core와 독립인 장착 설정으로 시작 시 한 번 latched 발행한다. 장착값은 `spec/0918sensors.json`에서 가져왔으며 camera optical 축 변환을 포함한다. 동적 `/tf`는 Core 등록 전 발행하지 않는다.
