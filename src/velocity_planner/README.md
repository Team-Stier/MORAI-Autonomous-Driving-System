# velocity_planner

`plain_velocity60.hpp`와 `plain_velocity100.hpp` Core로 경로에 속도 프로파일을 주입한다.
노드 구성은 `config/velocity_planner/`, 확장점은 `include/velocity_planner/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `velocity_planner_node` |
| 구독 | `/local_path`, `/localization/ego`, `/vehicle/competition_status`, `/traffic_light`, `/tf`, `/section_annotation` |
| 발행 | `/local_path/velocity_injected` (`base_link`, 입력 경로 stamp; 각 point pose header 포함) |
| 실행 | `/local_path` 입력마다 |
| Stage | `inject`: `process(path, tf)` → 속도를 주입한 path |

- 섹션 11·12는 `plain_velocity100.hpp`, UNKNOWN=0과 나머지 섹션은 `plain_velocity60.hpp`를 선택한다.
- Core는 경로 형상·점 순서를 유지하고 TF로 `base_link`에 변환한 뒤 각각 100/3.6, 60/3.6 m/s와 가속도 0 m/s²를 채운다.
- 출력 path는 입력 path의 시간 메타데이터를 상속한다.
- Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. path·tf의 Stale·ReceiveTimeout·Skew는 허용 issue다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
