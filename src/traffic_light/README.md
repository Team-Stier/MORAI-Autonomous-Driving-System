# traffic_light

camera_7 영상으로 신호등을 인식하는 노드다. Core는 아직 등록하지 않았다.
노드 구성은 `config/traffic_light/`, 확장점은 `include/traffic_light/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `traffic_light_node` |
| 구독 | `/image/camera_7/compressed`, `/section_annotation` |
| 발행 | `/traffic_light` (`camera_7_optical_frame`, 판정 영상 stamp) |
| 실행 | 5 Hz 주기, 최신 미처리 영상만 사용 |
| Stage | `recognize`: `process(image)` → TrafficLight |

- 등록한 Core가 없으므로 모든 섹션에서 NoCore로 종료하며 계산·발행하지 않는다.
- 같은 영상 묶음은 다시 계산하지 않는다(`require_new_snapshot`).
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
