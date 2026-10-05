# Traffic Light YOLO Core 설계

작성일: 2026-10-05  
대상 브랜치: `feature/traffic-light-yolo-core`

## 1. 목적

`camera_7`의 압축 영상을 YOLO11m Detect 모델로 추론해 주행 차선 전방 신호등의 상태를 `interfaces/TrafficLight`로 발행하는 C++ Core를 완성한다.

모델의 학습 클래스와 ROS 출력은 다음처럼 대응한다.

| YOLO 클래스 | `TrafficLight.state` |
| --- | --- |
| `go` | `GREEN` |
| `left_turn` | `LEFT` |
| `prepare_stop` | `YELLOW` |
| `stop` | `RED` |
| 탐지 없음, 낮은 신뢰도, 판정 충돌 | `UNKNOWN` |

직진과 좌회전이 함께 점등된 조합을 별도 학습 클래스로 추가하는 것은 이번 범위가 아니다. 현재 데이터셋의 네 클래스만 사용하며, `OFF` 및 복합 enum은 출력하지 않는다.

## 2. 범위

### 포함

- `/image/camera_7/compressed`의 JPEG 디코딩
- YOLO11m ONNX 전처리, CUDA 추론, 출력 파싱, confidence 필터, NMS
- 검출 결과에서 한 개의 최종 상태 결정
- 최근 프레임 결과를 이용한 설정 가능한 시간 안정화
- ROS enum 변환과 `/traffic_light` payload 생성
- Core 설정 로딩과 유효성 검사
- 기존 1단계 Pipeline에 Core 등록
- 계산, 파싱, 매핑, 안정화 및 등록 계약 테스트
- 학습 산출물 `best.pt`의 ONNX 변환·동등성 확인 절차

### 제외

- 카메라 ROI 선택 또는 crop 기반 관심 영역 제한
- 정지선까지 거리, 경로, 차량 위치와의 결합
- 신호등까지의 거리 추정
- 차량 제어 또는 정지 판단
- 데이터 재수집·라벨 변경·재학습
- 공통 `node_pipeline`, 메시지 정의, 토픽, launch, 실행 주기 변경
- 모델 파일 자체를 일반 Git blob으로 커밋하는 작업

주행 차선 전방 신호등만 정답으로 라벨링한 데이터셋을 모델 선택성의 근거로 사용한다. 옆 방향 및 반대 차선 신호등은 `unknown` 객체가 아니라 배경으로 취급한다.

## 3. 기준 계약

- 입력: `/image/camera_7/compressed`, `sensor_msgs/CompressedImage`
- 출력: `/traffic_light`, `interfaces/TrafficLight`
- 목표 실행: 5 Hz, 최신 미처리 프레임, 중첩 추론 없음
- 입력과 출력 frame: `camera_7_optical_frame`
- 출력 stamp: 실제 판정에 사용한 입력 영상 stamp
- ROS 구독·발행 queue: 1

기존 `ros_node.cpp`와 Pipeline이 위 입출력, 직렬 callback, 주기 실행, 최신 프레임 선택, header 주입을 이미 담당한다. Core는 영상 메시지를 받아 `state` payload만 완성한다.

## 4. 선택한 구조

### 배포 형식

학습에는 Ultralytics의 `best.pt`를 사용하되 C++ 실행 노드는 이를 직접 읽지 않는다. 학습이 끝나면 고정된 Ultralytics 버전에서 ONNX로 내보내며 다음 계약을 사용한다.

- batch: 1
- 입력 크기: 기본 `1280 x 1280`
- 입력 이름과 출력 이름: export 결과에서 검증 후 설정 또는 코드 계약으로 고정
- 입력 형식: RGB, `float32`, NCHW, `[0, 1]`
- 전처리: aspect ratio를 보존하는 letterbox
- 후처리: Core에서 confidence 필터 및 class-aware NMS 수행
- 실행 provider: CUDA 우선, 설정으로 CPU fallback 허용 여부 결정

ONNX와 함께 클래스 순서 및 export 환경을 기록한 manifest를 배포한다. 클래스 인덱스는 `go`, `left_turn`, `prepare_stop`, `stop` 순서로 고정하고, 시작 시 manifest/모델 출력 차원이 이 계약과 맞지 않으면 즉시 실패한다.

### Core 구성

단일 `YoloTrafficLightCore`가 다음 순서로 처리한다.

1. `sensor_msgs::CompressedImage::data`를 OpenCV로 디코딩한다.
2. letterbox, BGR→RGB, 정규화, NCHW 변환을 수행한다.
3. ONNX Runtime session으로 batch 1 추론을 실행한다.
4. YOLO 출력 tensor를 bounding box, class, confidence 후보로 변환한다.
5. 클래스별 confidence threshold와 NMS IoU threshold를 적용한다.
6. 남은 후보 중 최고 confidence 후보를 현재 프레임의 상태로 선택한다.
7. 후보가 없으면 `UNKNOWN`으로 정한다. 서로 다른 클래스의 상위 후보가 설정된 모호성 기준 안에서 경합하면 안전하게 `UNKNOWN`으로 정한다.
8. 최근 프레임 판정 이력을 갱신하고 다수결 결과를 `interfaces::TrafficLight.state`로 반환한다.

Core는 ONNX Runtime session과 최근 판정 deque를 지속 상태로 소유한다. Pipeline의 health, section, stamp, frame 및 발행 상태는 Core로 전달하지 않는다.

### 시간 안정화

기본값은 최근 5프레임, 최소 3표다. 모든 상태는 `UNKNOWN`을 포함해 이력에 기록한다.

- 한 상태가 `minimum_votes` 이상이면 최다 득표 상태를 출력한다.
- 최다 득표가 동률이면 가장 최근 프레임에서 더 나중에 나타난 상태를 선택한다.
- 어떤 상태도 최소 표 수를 충족하지 못하면 `UNKNOWN`을 출력한다.
- `history_size: 1`, `minimum_votes: 1`로 설정하면 안정화를 사실상 끌 수 있다.
- 설정 변경은 재시작 시 반영하며, 런타임 동적 재설정은 지원하지 않는다.

이 방식은 영상 흔들림에 따른 단발성 오검출을 줄이는 대신 기본 설정에서 상태 전환 확정에 최대 약 0.4초가 더 걸릴 수 있다. 검증 결과에 따라 `history_size`를 3~5 범위에서 조정한다.

## 5. 설정

`traffic_light.yaml`에 Core 전용 namespace를 추가한다.

```yaml
yolo_traffic_light:
  model_path: /absolute/path/to/traffic_light_yolo11m.onnx
  manifest_path: /absolute/path/to/traffic_light_yolo11m.json
  input_width: 1280
  input_height: 1280
  confidence:
    go: 0.40
    left_turn: 0.40
    prepare_stop: 0.35
    stop: 0.40
  nms_iou_threshold: 0.45
  ambiguity_margin: 0.05
  history_size: 5
  minimum_votes: 3
  allow_cpu_fallback: false
```

경로가 비어 있거나 파일을 열 수 없음, 크기가 양수가 아님, threshold 범위가 `[0, 1]` 밖임, `minimum_votes > history_size`, 클래스 계약 불일치, CUDA provider를 요구했으나 사용할 수 없음은 시작 오류로 처리한다.

첫 배포에서는 UNKNOWN을 포함한 모든 section ID를 같은 Core에 매핑한다. 이는 section 또는 정지선 정보를 추론 입력으로 사용한다는 의미가 아니며, 네 신호등 구간 밖에서도 노드가 NoCore로 끊기지 않고 모델 결과 또는 `UNKNOWN`을 일관되게 발행하도록 하기 위함이다.

## 6. 변경 파일과 책임

Core 등록 가이드가 허용한 범위에서만 아래 변경을 수행한다.

| 파일 | 변경 내용 |
| --- | --- |
| `include/traffic_light/cores/yolo_traffic_light_core.hpp` | Core 선언, 추론과 지속 이력 |
| `src/yolo_traffic_light_core.cpp` | OpenCV/ONNX Runtime 기반 구현 |
| `include/traffic_light/types.hpp` | Core parameter 구조체 |
| `include/traffic_light/config.hpp` | ROS parameter 로더와 검증 |
| `include/traffic_light/core_set.hpp` | Core ID, include, 소유 객체 등록 |
| `include/traffic_light/core_dispatch.hpp` | 입력 0 존재 검사, `before()`, native 호출, `wrapResult()` 연결 |
| `src/ros_node.cpp` | 이름→ID map과 설정 주입만 추가 |
| `config/traffic_light.yaml` | Core 선택 map과 계수 추가 |
| `CMakeLists.txt`, `package.xml` | OpenCV 및 ONNX Runtime 직접 의존성, Core 소스와 테스트 target 연결 |
| `test/unit/*` | parser, 매핑, 이력, 오류 처리, Pipeline 등록 회귀 테스트 |

`src/traffic_light/launch/traffic_light.launch`, 메시지 정의, 공통 Pipeline은 변경하지 않는다. ONNX Runtime을 현재 ROS/rosdep 환경에서 찾는 표준 방식이 없다면 임의 설치 경로나 전역 linker 설정을 코드에 박지 않고, 재현 가능한 패키지 탐색 방법을 먼저 확정한다.

## 7. 실패 처리

- 모델 또는 설정 오류: 노드 시작 시 예외로 종료해 잘못된 배포를 숨기지 않는다.
- JPEG 디코딩 실패: 해당 프레임 판정을 `UNKNOWN`으로 처리하고 다음 프레임에서 회복한다.
- 탐지 없음 또는 confidence 부족: `UNKNOWN`.
- 상충하는 근접 confidence 후보: `UNKNOWN`.
- 추론 실행 자체의 예외: 기본적으로 예외를 전파해 노드를 종료한다. Pipeline 계약에 없는 내부 재시도 루프를 만들지 않는다.
- stale/missing/frame mismatch: 기존 Pipeline health 계약이 Core 호출 전에 처리한다.

## 8. 검증 계획

### 단위 및 계약 테스트

- letterbox 좌표 역변환이 원 영상 좌표와 일치하는지 확인
- 알려진 tensor fixture로 confidence 필터와 class-aware NMS 확인
- 네 클래스의 ROS enum 매핑 및 unsupported index 거부 확인
- no detection, 낮은 confidence, ambiguity가 `UNKNOWN`인지 확인
- 3프레임 및 5프레임 투표, 동률, 시작 구간, 이력 길이 제한 확인
- 잘못된 설정과 manifest가 시작 시 거부되는지 확인
- dispatch가 Missing 입력에서 Core를 호출하지 않고, 정상 입력에서 한 개의 `TrafficLight`를 반환하는지 확인
- Pipeline 결과가 입력 stamp와 `camera_7_optical_frame`을 유지하는지 확인

### 모델 검증

- 학습/검증에 쓰지 않은 `foggy 15시` holdout에서 클래스별 precision, recall, mAP와 confusion matrix 확인
- 같은 이미지 묶음에서 PyTorch `best.pt`와 ONNX의 class/confidence 결과를 비교
- 작은 원거리 신호등, 역광, 안개, 교차로 주변의 다른 방향 신호등을 별도 실패 사례로 검토

### 통합 검증

- 기존 rosbag의 `camera_7`만 재생해 `/traffic_light`가 목표 5 Hz로 발행되는지 확인
- 출력 stamp가 판정 입력 영상 stamp와 같은지 확인
- frame이 `camera_7_optical_frame`인지 확인
- GPU 사용과 추론 중첩 부재 확인
- RTX 3090에서 p50/p95 처리 시간과 GPU memory를 기록하고 200 ms 예산 충족 여부 확인
- 상태 변화 구간에서 안정화 지연과 단발성 흔들림 억제 효과 확인

## 9. 산출물과 완료 기준

- 재현 가능한 export 명령과 버전 기록
- 검증된 `.onnx`와 manifest의 배포 위치 또는 Git LFS/Release 참조
- C++ Core, 설정, 등록, 테스트
- holdout 및 rosbag 통합 검증 결과

다음 조건을 모두 만족하면 Core 완료로 판단한다.

1. 패키지가 clean 환경에서 컴파일·링크된다.
2. 신규 단위/등록 테스트와 기존 관련 테스트가 통과한다.
3. 네 학습 클래스가 정해진 ROS enum으로 발행된다.
4. no detection 및 애매한 결과는 `UNKNOWN`이다.
5. 출력 header 계약과 5 Hz/latest-frame 계약이 유지된다.
6. RTX 3090에서 추론이 200 ms 주기 안에 안정적으로 끝난다.
7. ROI나 정지선 거리 입력 없이 동작한다.

## 10. 구현 전 확인 사항

- 최종 `best.pt` 학습 완료 여부와 실제 클래스 인덱스
- ONNX export에 사용한 Ultralytics/ONNX opset 버전
- 배포할 모델 및 manifest의 영구 경로
- 대상 PC에서 사용할 ONNX Runtime GPU 패키지의 설치·탐색 방식

이 항목들은 구현 계획에서 구체적인 명령과 테스트 순서로 확정한다.
