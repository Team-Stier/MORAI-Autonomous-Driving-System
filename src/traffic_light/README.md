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

- 같은 영상 묶음은 다시 계산하지 않는다(`require_new_snapshot`).
- `launch.sh`가 ROS·워크스페이스 환경과 ONNX Runtime 라이브러리 경로를 준비하고 자기 노드만 실행한다.

## YoloTrafficLight

`recognize`의 모든 섹션(UNKNOWN=0, 1~16)은 `yolo_traffic_light.hpp`를 선택한다.

| 항목 | 내용 |
| --- | --- |
| Validator | `decodable_image.hpp`: 영상을 실제로 디코딩하고 실패하면 Missing |
| 모델 | `models/traffic_light_yolo11m.onnx`: YOLO11m Detect, 입력 `[1,3,1280,1280]`, 출력 `[1,8,33600]`. 파라미터 로더가 `asset("model")`로 읽는다. |
| 파라미터 | `config/traffic_light/parameters/yolo_traffic_light.yaml` |
| 의존성 | OpenCV, ONNX Runtime(`ONNXRUNTIME_ROOT`, 기본 `/opt/onnxruntime-gpu/1.19.2`) |

1. 영상을 BGR로 디코딩하고, 비율을 유지해 가운데에 놓고 114로 채운 RGB float32 `[0, 1]` NCHW 입력을 만든다.
2. Core 생성 시 `device`(`cuda`|`cpu`, 기본 `cuda`)로 한 번 만든 세션으로 추론한다. 다른 장치로 전환하지 않는다.
3. 후보마다 최고 점수 클래스를 고르고, 클래스별 `confidence`와 클래스별 NMS(`nms_iou`)를 적용한다.
4. 남은 검출의 클래스가 뜻하는 점등 등의 합집합을 TrafficLight 조합 enum으로 정한다. 검출이 없으면 UNKNOWN이며 OFF는 발행하지 않는다.
5. 최근 `history_size` 프레임(UNKNOWN 포함)에서 `minimum_votes` 이상 득표한 상태를 반환한다. 미달이면 UNKNOWN, 동률이면 가장 최근 상태다.

| 클래스 인덱스 | 이름 | 점등 등 |
| --- | --- | --- |
| 0 | `go` | green |
| 1 | `left_turn` | left |
| 2 | `prepare_stop` | yellow |
| 3 | `stop` | red |

- 클래스 순서와 점등 등은 파라미터 `classes`로 정한다.
- 세션 생성이 실패하면 ROS ERROR를 남기고 이후 UNKNOWN을 반환한다. 추론 실행 오류는 ROS ERROR를 남기고 그 프레임을 UNKNOWN으로 투표한다.
- 모델 입출력이 파라미터의 입력 크기·클래스 수와 맞지 않으면 시작 단계 실패다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
