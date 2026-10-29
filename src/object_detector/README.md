# object_detector

수평화 점군에서 장애물을 ROI 필터링·클러스터링·TF 변환·bbox 피팅·추적하는 다섯 Stage 노드다.
노드 구성은 `config/object_detector/`, 확장점은 `include/object_detector/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `object_detector_node` |
| 구독 | `/velodyne_points/leveled`, `/vehicle/competition_status`, `/tf`, `/section_annotation` |
| 발행 | `/objects` (`map`, 입력 점군 stamp) |
| 실행 | `/velodyne_points/leveled` 입력마다 |
| Stage | `roi`: `process(cloud, competition, tf)` → cloud·competition·tf, `clustering`: `process(cloud, competition, tf)` → clusters·competition·tf, `tf`: `process(clusters, competition, tf)` → clusters·competition·tf, `bbox_fitting`: clusters·competition·tf → objects·competition, `track`: `process(objects, stamp)` → objects |

- `tracker.hpp`와 `objects_pass_through.hpp`는 등록된 대체 Core다.
- `clusters`는 voxel downsampling과 DBSCAN 크기 필터를 통과한 클러스터별 `pcl::PointCloud<pcl::PointXYZ>` 배열이며, `obb.hpp`가 각 클러스터의 최소 면적 OBB를 계산한다.
- ROI·클러스터링 좌표는 `base_link_level`이다. `cluster_tf.hpp`는 `/tf`의 `map → base_link_level` 변환으로 각 클러스터의 XYZ를 map으로 옮긴다. OBB와 추적은 map에서 계산한다.
- `cluster_transform.hpp` Validator는 필요한 변환과 유한 위치·단위 quaternion을 검사한다. TF는 입력 점군 stamp에 가장 가까운 샘플로 조립하며 허용 시차와 신선도는 `node.yaml`로 제한한다. 출력 stamp는 입력 점군 stamp를 유지한다.
- `tracking.hpp`는 map XYZ 위치로 ID·절대 3축 속도를 추정하고 연속 유효 속도의 차분으로 절대 가속도를 추정한다. 최초 관측의 속도와 충분한 속도 이력이 없는 가속도는 unavailable이다. 대체 `tracker.hpp`도 map 관측과 입력 점군 시각을 사용한다.
- `cloud_layout.hpp` Validator가 ROI·클러스터링 단계에서 PointCloud2 필드 구성을 검사한다.
- 중간 objects는 Core가 새로 계산한 값이므로 시간 메타데이터가 없다. 최종 objects에만 헤더를 주입해 발행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

`viewpoint_bbox.hpp`는 `process(clusters, competition, tf)`로 관측 위치와 도로 방향을 사용해 차량 박스를 완성하는 대체 Core다. 관측된 점을 포함하면서 차량 크기 사전값으로 가려진 면을 보완한다. `obb.hpp`는 기존 `process(clusters, competition)` 계약을 유지한다.
