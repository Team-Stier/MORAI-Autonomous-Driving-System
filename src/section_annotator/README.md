# section_annotator

`checkpoint_sections.hpp` Core가 체크포인트 기준 섹션 16개를 판정한다.
노드 구성은 `config/section_annotator/`, 확장점은 `include/section_annotator/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `section_annotator_node` |
| 구독 | `/localization/ego` |
| 발행 | `/section_annotation` (`map`, ego stamp) |
| 실행 | `/localization/ego` 입력마다 |
| Stage | `annotate`: `checkpoint_sections.hpp`의 `process(ego)` → SectionAnnotation |

- `checkpoints.csv`의 0-based 행 인덱스 0~15는 섹션 값 1~16에 대응한다. Core의 초기 저장 섹션은 UNKNOWN=0이다.
- `radius_m`은 모든 체크포인트의 map XY 판정 반지름(m)이며 경계를 포함한다.
- Core는 반지름 내부로 판정되면 저장 섹션을 갱신한다. 여러 반지름에 속하면 가장 가까운 체크포인트, 거리 동률이면 작은 인덱스를 선택한다.
- 어느 반지름에도 속하지 않으면 저장 섹션을 유지한다. 매 `process(ego)` 호출마다 저장 섹션을 반환한다. 첫 반지름 진입 전에는 UNKNOWN을 반환한다.
- 파라미터 로더는 `checkpoints_file` CSV를 정적 자산으로 읽어 유한한 좌표를 가진 16개 행과 반지름을 검사한다. 값은 `config/section_annotator/parameters/checkpoint_sections.yaml`에 둔다.
- section을 구독하지 않으므로 UNKNOWN=0에 매핑한 Core를 사용한다. Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거하며 ego의 Stale·ReceiveTimeout은 허용 issue다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
