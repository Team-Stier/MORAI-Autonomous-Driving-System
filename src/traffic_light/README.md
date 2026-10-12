# traffic_light

camera_7 영상으로 신호등을 인식하는 노드다.
노드 구성은 `config/traffic_light/`, 확장점은 `include/traffic_light/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `traffic_light_node` |
| 구독 | `/image/camera_7/compressed`, `/section_annotation` |
| 발행 | `/traffic_light` (`camera_7_optical_frame`, 판정 영상 stamp) |
| 실행 | 5 Hz 주기, 최신 미처리 영상만 사용 |
| Stage | `recognize`: `process(image)` → TrafficLight |

- Core가 영상을 전처리하고 macOS의 CoreML `CPUAndGPU` 서버에 ONNX 추론 텐서를 전달한다. NMS·신호 판정·시간 투표는 Core가 수행하며 영상 stamp는 Pipeline이 유지한다.
- Mac에서 `./src/traffic_light/start_mac_gpu.sh`로 추론 서버를 시작한다. 서버는 `127.0.0.1:18765`에서 고정 크기 float32 텐서를 받고 `traffic_light_yolo11m.onnx`의 추론 결과만 반환한다.
- `launch.sh`는 Mac Core가 선택됐을 때 서버의 CoreML 실행 설정을 확인한 뒤 노드를 시작한다. 서버 통신 실패는 로그와 UNKNOWN으로 보고하며 CPU 추론으로 전환하지 않는다.
- 같은 영상 묶음은 다시 계산하지 않는다(`require_new_snapshot`).
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
