# traffic_light

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. Core는 아직 등록하지 않았다.

- 구독: `/image/camera_7/compressed`, `/section_annotation`
- 발행 연결: `/traffic_light`
- 실행: 5 Hz에서 최신 미처리 영상 사용. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `node.yaml`의 `stage1`은 후보 Core와 선택 매핑이 비어 있다. Core가 없으면 NoCore로 종료하며 계산·발행하지 않는다.
- Core는 `include/traffic_light/cores/`에 파일 하나씩 작성하고 `registrations/*.yaml`로 native 입력·출력·Validator·Policy·로더를 연결한다.
- 공통 실행기는 조립·section/health 선택·검사·Policy·계산·최종 발행을 담당한다. `/section_annotation`은 선택 메타데이터로만 사용한다.
- 최종 출력의 `output.header`는 영상 stamp와 `camera_7_optical_frame`을 선언한다. 출력은 검사하지 않는다.
- 기존 `config/traffic_light.yaml`의 실행 설정과 빈 `stage1` 매핑은 공통 실행기가 읽는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
