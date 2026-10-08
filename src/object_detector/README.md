# object_detector

수평화 점군에서 장애물을 검출·추적하는 두 Stage 노드다.
노드 구성은 `config/object_detector/`, 확장점은 `include/object_detector/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `object_detector_node` |
| 구독 | `/velodyne_points/leveled`, `/section_annotation` |
| 발행 | `/objects` (`base_link`, 입력 점군 stamp) |
| 실행 | `/velodyne_points/leveled` 입력마다 |
| Stage | `detect`: `dbscan.hpp`의 `process(cloud)` → objects, `track`: `objects_pass_through.hpp`의 `process(objects)` → objects |

- 모든 섹션에서 `dbscan.hpp`와 `objects_pass_through.hpp`를 선택한다.
- `dbscan.hpp`는 ROI·voxel로 줄인 점군을 DBSCAN으로 군집화하고 군집마다 최소 면적 OBB 객체를 만든다. 계수는 `config/object_detector/parameters/dbscan.yaml`이다.
- `objects_pass_through.hpp`는 검출 객체를 그대로 넘긴다. `id`는 0이고 속도·가속도는 미추정이다.
- Validator `cloud_layout.hpp`는 little-endian, 행·데이터 경계, 단일 FLOAT32 x·y·z 필드를 확인해 어긋나면 cloud를 Missing으로 기록한다. Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거하며, cloud의 Stale·ReceiveTimeout은 허용 issue다.
- `cores/cluster_match.hpp`는 NPC 모델 정합 Core로 보존하며 등록하지 않았다. 모델은 `config/object_detector/parameters/cluster_match.yaml`의 `models_file`로 `models/npc_models.bin`을 읽는다. 이 파일은 `scripts/convert_npc_models.py`로 만들며, 출처·검증값은 `models/npc_models.json`에 있다.
- 중간 objects는 Core가 새로 계산한 값이므로 시간 메타데이터가 없다. 최종 objects에만 헤더를 주입해 발행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
