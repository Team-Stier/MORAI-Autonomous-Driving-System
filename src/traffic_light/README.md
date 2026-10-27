# traffic_light

camera_7 영상으로 신호등을 인식하는 노드다.
노드 구성은 `config/traffic_light/`, 확장점은 `include/traffic_light/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `traffic_light_node` |
| 구독 | `/image/camera_7/compressed`, `/section_annotation` |
| 발행 | `/traffic_light` (`camera_7_optical_frame`, 판정 영상 stamp) |
| 실행 | 10 Hz 주기, 최신 미처리 영상만 사용 |
| Stage | `recognize`: `process(image)` → TrafficLight |

현재 전 구간 `yolo_traffic_light.hpp`로 CUDA YOLO 인식을 실행한다.
velocity_planner는 3·4·5번 섹션에 직진 신호 정지, 7번 섹션에 좌회전 신호 정지를 적용한다.

## Camera7 전체 영상 분류

`camera7_frame_cpu.hpp`는 등록된 비교용 대체 Core이며 기본 선택에서 사용하지 않는다.
원본은 `Team-Stier/MORAI-Autonomous-Driving-System`의 `new_yolo/taeyun` 브랜치,
커밋 `0973c9fbc0892876975ad46b9b31a59d8eda0e67`이다.
Core·로더·등록·계수·모델·provenance·테스트 소스를 원본과 동일하게 이식한다.

camera_7 전체 영상을 RGB float32 `[1,3,480,640]`, 0–1 범위로 변환하고
CPU ONNX Runtime으로 `[1,17]` 상태 확률을 계산한다.
최댓값의 인덱스를 TrafficLight enum으로 반환하며 ROI 검출·NMS·프레임 투표를 거치지 않는다.
`parameters/camera7_frame_cpu.yaml`의 원본 값은 CPU thread 2개,
`minimum_confidence=0.0`이다. 별도 CUDA·Mac 추론 서버는 사용하지 않는다.
모델 입출력 이름·형상·타입은 파라미터 로더가 확인한다.
10 Hz 실행·입력 신선도·출력 stamp 계약은 기존 `node.yaml`을 유지한다.

모델과 학습 이력은 `models/camera7_frame_cpu/`에 보관한다.
provenance의 99.93%는 학습 데이터 재평가 정확도이며 독립 검증 정확도가 아니다.

## Linux CUDA 대체 Core

`yolo_traffic_light.hpp`는 현재 선택된 CUDA YOLO Core다.
Core는 로컬 ONNX Runtime의 CUDA Execution Provider로
`models/traffic_light_yolo11m.onnx`를 추론한다. 별도 macOS 추론 서버는 사용하지 않는다.
모델은 `suhyeon-01004/Suhyeon-MORAI-Autonomous-Driving-System`의
`feature/traffic-light-far-distance-finetune` 커밋 `78e8f698b6d506e30ba227509f9bbddf46655613`에서 가져왔다.
`launch.sh`가 `ONNXRUNTIME_ROOT`(기본 `/opt/onnxruntime-gpu/1.19.2`)의
`lib`와 `cuda/lib`를 라이브러리 검색 경로에 추가한다.
CUDA provider 생성에 실패하면 초기화 실패로 보고하며 Core가 임의로 UNKNOWN을 대체 발행하지 않는다.

Core가 letterbox·RGB 전처리, NMS, 신호 판정과 시간 투표를 수행한다.
입력은 1280×1280이며, 계수는 `parameters/yolo_traffic_light.yaml`에서 읽는다.
최근 5개 판정 중 최소 3표가 모여야 해당 상태를 반환하며 탐지·투표가 불충분하면 UNKNOWN이다.
신호 조합 값은 비트 플래그가 아닌 `interfaces/TrafficLight.msg`의 enum을 따른다.
같은 영상 묶음은 다시 계산하지 않는다(`require_new_snapshot`).

## 다른 Core

`unknown.hpp`는 UNKNOWN을 발행하는 대체 Core다.
`yolo_traffic_light_mac.hpp`는 macOS CoreML `CPUAndGPU` 서버로 추론 텐서를 전달한다.
이를 선택하면 Mac에서 `./src/traffic_light/start_mac_gpu.sh`로 서버를 시작해야 한다.
`launch.sh`는 Mac Core가 선택됐을 때 서버 설정을 확인한다.
Mac 서버 통신 실패는 로그와 UNKNOWN으로 보고하며 CPU 추론으로 전환하지 않는다.

## 제어와 표시

`velocity_planner`는 `/traffic_light`를 받아 기존 구간별 정지선 규칙을 적용한다.
cutoff는 3·4·5번 섹션에서 `stop_line_cutoff_straight.hpp`, 7번에서
`stop_line_cutoff_left.hpp`를 사용하며 나머지는 `stop_line_pass_through.hpp`다.
UNKNOWN·OFF의 정지 동작은 velocity_planner 계약을 따른다.
`visualizer`의 `traffic_light_visualizer_node`는 신호 상태를 Marker로 바꾸며
RViz `visualizer/TrafficSignalOverlay`가 우측 상단에 색상과 상태 문자열을 표시한다.

`launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
