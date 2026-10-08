# object_detector

수평화 점군에서 장애물을 검출·추적하는 두 Stage 노드다. 두 Stage 모두 Core는 아직 등록하지 않았다.
노드 구성은 `config/object_detector/`, 확장점은 `include/object_detector/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `object_detector_node` |
| 구독 | `/velodyne_points/leveled`, `/section_annotation` |
| 발행 | `/objects` (`base_link`, 입력 점군 stamp) |
| 실행 | `/velodyne_points/leveled` 입력마다 |
| Stage | `detect`: `process(cloud)` → objects, `track`: `process(objects)` → objects |

- 등록한 Core가 없으므로 모든 섹션에서 NoCore로 종료하며 계산·발행하지 않는다.
- 중간 objects는 Core가 새로 계산한 값이므로 시간 메타데이터가 없다. 최종 objects에만 헤더를 주입해 발행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
