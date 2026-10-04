# speed_filter

`node_pipeline` 라이브러리로 `speed_limit60.hpp`와 `no_speed_limit.hpp` Core를 실행한다.

- 구독: `/control_cmd`, `/vehicle/competition_status`, `/section_annotation`
- 발행 연결: `/morai_udp_bridge/control_cmd`
- 실행: /control_cmd 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `config/speed_filter.yaml`의 `stage1`에서 UNKNOWN=0과 섹션 1~16을 매핑한다. 섹션 10·12는 `no_speed_limit.hpp`, 나머지는 `speed_limit60.hpp`를 선택한다.
- 두 Core는 `process(command, competition)`으로 실제 메시지를 개별 인자로 받고 CtrlCmd를 반환한다. `speed_limit60`·`no_speed_limit` namespace에서 각각 계산 설정을 읽는다.
- `speed_limit60`의 기본 제한은 60 km/h, `no_speed_limit`의 기본 제한은 0으로 비활성이다. 두 Core 모두 설정한 정지 시간 후 출발 명령을 적용하며 계산 상태는 각각 유지한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

CoreSet은 두 지속 Core 객체를 소유한다. 현재 단일 Stage는 자기 Checker·Policy·Selector를 소유한다.
`include/speed_filter/core_dispatch.hpp`의 선택 case가 필수 입력 0·1을 검사하고, 존재할 때만 `before()`와 해당 Core를 호출한다. Missing이면 NoCore로 종료한다.
Stage 사이에서 입력을 계승한 슬롯은 원래 stamp·수신 시각을 유지한다. 새 계산값은 시각 메타데이터가 없어도 정상 입력이며, 없는 시각의 검사만 생략한다.
두 Core의 반환 명령은 입력 command의 시각 메타데이터를 계승한다. Pipeline은 마지막 결과에 HeaderInjector를 한 번 적용하고 발행하며 CtrlCmd에는 주입할 Header가 없다.

노드는 `map_file`을 원본 바이트로 읽는다. 현재 두 Core의 계산 인자로 전달하지 않는다.

CtrlCmd에는 Header가 없으므로 원래 생성 stamp를 만들거나 stale·Skew 검사를 하지 않는다. 수신 timeout은 steady 시각으로 검사한다.
