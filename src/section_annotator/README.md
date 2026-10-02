# section_annotator

`node_pipeline` 라이브러리를 사용하는 실행 패키지다. `checkpoint_sections.hpp` Core가 체크포인트 기준 섹션 16개를 판정한다.

- 구독: `/localization/ego`
- 발행 연결: `/section_annotation`
- 실행: /localization/ego 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `checkpoints.csv`의 0-based 행 인덱스 0~15는 섹션 값 1~16에 대응한다. Core의 초기 저장 섹션은 UNKNOWN=0이다.
- `checkpoint_sections/radius_m`은 모든 체크포인트의 map XY 판정 반지름(m)이며 기본값은 10.0이다. 경계를 포함한다.
- Core는 반지름 내부로 판정되면 저장 섹션을 갱신한다. 여러 반지름에 속하면 가장 가까운 체크포인트, 거리 동률이면 작은 인덱스를 선택한다.
- 어느 반지름에도 속하지 않으면 Core의 저장 섹션을 유지한다. 매 `process(ego)` 호출마다 저장 섹션을 반환하고 Pipeline이 발행한다. 첫 반지름 진입 전에는 UNKNOWN을 반환한다.
- `CoreSet`은 ego를 필수 입력으로 선언하고 지속 Core의 `process(ego)`에 연결한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

설정 로더는 `checkpoints_file`의 CSV를 읽어 유한한 좌표를 가진 16개 행과 설정 반지름을 검사한다. `checkpoints_file`이 없으면 기존 `map_file`과 같은 디렉터리의 `checkpoints.csv`를 사용한다. 체크포인트와 반지름은 Core 생성 시 주입한다. Core는 저장 섹션 payload를 반환하며 HeaderInjector가 ego stamp와 frame_id=map을 주입한다.
