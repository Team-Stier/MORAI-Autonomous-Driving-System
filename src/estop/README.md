# estop

`/local_path/velocity_injected`를 속도 프로파일대로 주행했을 때 `/objects`와 겹치면 `/estop`(`std_msgs/Bool`)에 true를 발행한다.
경로가 `release_after_sec` 동안 계속 비어 있으면 false로 해제한다. `speed_filter`가 이를 구독해 true인 동안 풀브레이크로 명령을 덮어쓴다.

| 입력 | 토픽 | 용도 |
| --- | --- | --- |
| path | `/local_path/velocity_injected` (base_link) | 경로와 속도 프로파일 |
| objects | `/objects` (map) | 객체 bbox·속도 |
| tf | `/tf` | 경로를 map으로 옮길 map→base_link |

판정 관대·엄격도는 `config/estop/parameters/estop_overlap.yaml`의 공간 여유(`longitudinal_margin_m`·`lateral_margin_m`·`object_margin_m`)와
시간 여유(`time_tolerance_sec`·`time_samples`), 검사 범위(`horizon_sec`·`horizon_m`)로 정한다.
