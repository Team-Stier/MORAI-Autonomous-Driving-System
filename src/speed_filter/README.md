# speed_filter

`speed_limit60.hpp`, `speed_guard60.hpp`, `speed_guard60_hold.hpp`, `no_speed_limit.hpp` Core로 제어 명령에 최종 속도 제한을 적용한다.
노드 구성은 `config/speed_filter/`, 확장점은 `include/speed_filter/`에 둔다.

| 항목 | 내용 |
| --- | --- |
| 노드 | `speed_filter_node` |
| 구독 | `/control_cmd`, `/vehicle/competition_status`, `/section_annotation` |
| 발행 | `/morai_udp_bridge/control_cmd` |
| 실행 | `/control_cmd` 입력마다, 조립 큐 1 |
| Stage | `limit`: `process(command, competition)` → 제한한 command |

- 두 Core는 계산 라이브러리 `libraries/speed_limiter.hpp`의 SpeedLimiter를 각자 소유하며 계산 상태도 각각 유지한다. 계수는 `config/speed_filter/parameters/<core>.yaml`에서 읽는다.
- `speed_limit60`의 제한은 60 km/h, `no_speed_limit`은 0으로 비활성이다. 두 Core 모두 설정한 정지 시간 후 출발 명령을 적용한다.
- `speed_guard60`은 SpeedLimiter를 쓰지 않고 Core 안에서 계산한다. 예측 속도(현재 속도 + 양의 가속도 추정 × `prediction_horizon_sec`)가 `accel_taper_start_kph`~`accel_cutoff_kph`이면 accel을 1→0 비율로 줄이고, `brake_start_kph`~`brake_full_kph`이면 accel=0과 `maximum_guard_brake`까지의 brake를 적용한다. 60 km/h 초과 시 전제동과 정지 후 출발 명령은 `speed_limit60`과 같다.
- `speed_guard60_hold.hpp`는 `speed_guard60`과 같은 예측 accel 감소·약제동과 60 km/h 초과 전제동을 적용하고, 정지 후 출발 명령은 적용하지 않는다.
- CtrlCmd에는 Header가 없으므로 stamp·stale·Skew 검사를 하지 않고 수신 timeout만 steady 시각으로 검사한다. 출력 command는 입력 command의 시간 메타데이터를 상속하며 주입할 헤더가 없다.
- Policy `remove_invalid.hpp`는 값·stamp·frame 오류 입력을 제거한다. command의 ReceiveTimeout, competition의 Stale·ReceiveTimeout은 허용 issue다.
- `launch.sh`가 ROS·워크스페이스 환경을 준비하고 자기 노드만 실행한다.

작성 계약은 [루트 README](../../README.md)와 [Pipeline 가이드](../node_pipeline/README.md)를 따른다.

## E-stop

첫 Stage `estop`이 `/estop`(`std_msgs/Bool`, `estop` 패키지)을 받아 true인 동안 명령을 풀브레이크(accel 0, brake 1)로 바꾼 뒤 `limit` Stage로 넘긴다.
`/estop`을 아직 받지 못했으면 `estop_pass.hpp`가 명령을 그대로 넘긴다. `limit` Core의 정지 후 출발 보조(`stop_tolerance_sec` 90 s)는 e-stop 중에도 동작할 수 있다.
