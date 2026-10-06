# traffic_light

Camera 7의 압축 영상을 YOLO11m ONNX Runtime CUDA로 추론하는 `node_pipeline` 패키지다.

- 구독: `/image/camera_7/compressed`, `/section_annotation`
- 발행 연결: `/traffic_light`
- 실행: 5 Hz에서 최신 미처리 영상 사용. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- 모델과 manifest 경로, 클래스별 confidence, NMS, ambiguity margin, 최근 프레임 투표 수는 `config/traffic_light.yaml`에서 바꾼다.
- 모델 클래스 순서는 `go`, `left_turn`, `prepare_stop`, `stop`이어야 한다. 출력은 각각 GREEN, LEFT, YELLOW, RED이며 판정 불가 시 UNKNOWN이다.
- 현재 UNKNOWN=0과 섹션 1~16 모두 `yolo_traffic_light_core.hpp`로 연결한다.
- Pipeline은 단계 목록을 순서대로 실행하며, 단계마다 독립된 HealthChecker·Policy·Selector가 입력 검사·보정·선택을 수행한다. 선택 매핑과 정책 우선순위는 `stage1`에서 읽는다.
- 원본에서 전달·보정한 변수는 입력의 stamp·receipt metadata를 계승한다. 새 반환 변수는 시각 metadata 없이 전달하며, 시각 정보가 없으면 시간 검사를 생략한다.
- 필수 payload가 Missing이면 Core를 호출하지 않는다. 마지막 결과에만 HeaderInjector가 영상 stamp와 `camera_7_optical_frame`을 주입하고 한 번 발행한다. 출력은 검사하지 않는다.
- 실행: 워크스페이스 빌드 후 `src/traffic_light/launch.sh`. 런처가 ROS, 워크스페이스, ONNX Runtime/CUDA 라이브러리 경로를 준비한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
