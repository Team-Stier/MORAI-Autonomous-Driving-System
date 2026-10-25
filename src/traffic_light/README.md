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

- `camera7_frame_gpu` Core가 전체 RGB 640×480 영상을 ResNet-34·ImageNet 기반 모델에 입력하고, 17개 점등 조합 확률 중 최댓값을 `TrafficLight.state`로 반환한다.
- MORAI, 모델 학습과 추론은 모두 로컬 GPU만 사용한다. 추론은 ONNX Runtime WebGPU로 실행하며 CPU 추론 전환을 비활성화한다.
- 판정은 현재 영상만 사용하며 영상 stamp와 출력 헤더는 Pipeline이 유지한다.
- 같은 영상 묶음은 다시 계산하지 않는다(`require_new_snapshot`).
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
