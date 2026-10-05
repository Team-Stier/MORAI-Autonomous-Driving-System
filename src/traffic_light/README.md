# traffic_light

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. Core는 아직 등록하지 않았다.

- 구독: `/image/camera_7/compressed`, `/section_annotation`
- 발행 연결: `/traffic_light`
- 실행: 5 Hz에서 최신 미처리 영상 사용. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/traffic_light.yaml`의 `stage1`은 UNKNOWN=0과 섹션 1~16을 빈 문자열(`""`)로 둔다. Core 미등록 항목은 NoCore로 종료하며 계산·발행하지 않는다.
- Core 등록은 `core_set.hpp`의 CoreId·지속 객체와 `core_dispatch.hpp`의 case에 연결한다. Core는 `include/traffic_light/cores/`에 파일 하나씩 작성한다. dispatch가 필수 입력 인덱스를 선언·확인한 뒤 실제 메시지를 개별 `process()` 인자로 전달한다.
- Pipeline은 단계 목록을 순서대로 실행하며, 단계마다 독립된 HealthChecker·Policy·Selector가 입력 검사·보정·선택을 수행한다. 선택 매핑과 정책 우선순위는 `stage1`에서 읽는다.
- 원본에서 전달·보정한 변수는 입력의 stamp·receipt metadata를 계승한다. 새 반환 변수는 시각 metadata 없이 전달하며, 시각 정보가 없으면 시간 검사를 생략한다.
- 필수 payload가 Missing이면 Core를 호출하지 않는다. 마지막 결과에만 HeaderInjector가 영상 stamp와 `camera_7_optical_frame`을 주입하고 한 번 발행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
