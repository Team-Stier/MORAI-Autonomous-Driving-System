# speed_filter

`node_pipeline` 라이브러리로 `speed_limit60.hpp`와 `no_speed_limit.hpp` Core를 실행한다.

- 구독: `/control_cmd`, `/vehicle/competition_status`, `/section_annotation`
- 발행 연결: `/morai_udp_bridge/control_cmd`
- 실행: /control_cmd 콜백. ROS 발행·구독 큐는 1, 콜백은 직렬 실행한다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.
- `node.yaml`의 `stage1`에서 UNKNOWN=0과 섹션 1~16을 매핑한다. 섹션 10·12는 `no_speed_limit.hpp`, 나머지는 `speed_limit60.hpp`를 선택한다.
- 두 Core는 `process(command, competition)`으로 실제 메시지를 개별 인자로 받고 CtrlCmd를 반환한다. `parameters/`의 읽기 전용 로더가 `speed_limit60`·`no_speed_limit` 설정을 각각 생성자에 주입한다.
- `speed_limit60`의 기본 제한은 60 km/h, `no_speed_limit`의 기본 제한은 0으로 비활성이다. 두 Core 모두 설정한 정지 시간 후 출발 명령을 적용하며 계산 상태는 각각 유지한다.
- 입력 검사·Missing 처리·선택·헤더 주입은 Core 밖에서 수행한다. 출력은 검사하지 않는다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

`registrations/*.yaml`은 필수 입력 `command`·`competition`과 반환 명령을 연결한다.
공통 실행기는 지속 Core 객체·조립·section/health 선택·검사·Policy·계산·최종 발행을 담당한다. `/section_annotation`은 선택 메타데이터로만 사용한다.
두 Core는 `calculation.hpp`의 계산 라이브러리를 각자 소유하며 다른 Core를 소유·호출하지 않는다.
반환 명령은 `time_from: command`로 입력의 시각 메타데이터를 계승한다. CtrlCmd에는 최종 주입할 Header가 없다.
기존 `config/speed_filter.yaml`의 실행 설정과 `stage1` 매핑은 공통 실행기가 읽는다.

CtrlCmd에는 Header가 없으므로 원래 생성 stamp를 만들거나 stale·Skew 검사를 하지 않는다. 수신 timeout은 steady 시각으로 검사한다.
