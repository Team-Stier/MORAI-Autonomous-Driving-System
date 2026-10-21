# speed_filter

`speed_limit60.hpp`와 `no_speed_limit.hpp` Core로 제어 명령에 최종 속도 제한을 적용한다.
노드 구성은 `config/speed_filter/`, 확장점은 `include/speed_filter/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `speed_filter_node` |
| 구독 | `/control_cmd`, `/vehicle/competition_status`, `/section_annotation` |
| 발행 | `/morai_udp_bridge/control_cmd` |
| 실행 | `/control_cmd` 입력마다, 조립 큐 1 |
| Stage | `limit`: `process(command, competition)` → 제한한 command |

- 두 Core는 계산 라이브러리 `libraries/speed_limiter.hpp`의 SpeedLimiter를 각자 소유하며 계산 상태도 각각 유지한다. 계수는 `config/speed_filter/parameters/<core>.yaml`에서 읽는다.
- `speed_limit60`의 제한은 60 km/h, `no_speed_limit`은 0으로 비활성이다. 두 Core 모두 명시적 제동 요청(`brake > 0`)을 유지한다. 가속·제동 요청이 모두 없는 상태에서 설정한 정지 시간이 지나면 출발 보조를 적용한다.
- CtrlCmd에는 Header가 없으므로 stamp·stale·Skew 검사를 하지 않고 수신 timeout만 steady 시각으로 검사한다. 출력 command는 입력 command의 시간 메타데이터를 상속하며 주입할 헤더가 없다.
- Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. command의 ReceiveTimeout, competition의 Stale·ReceiveTimeout은 허용 issue다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.
