# 경로 관리자

`morai_path_manager`는 검증된 전역경로 파일을 읽어 `nav_msgs/Path`로 발행하고,
현재 위치에 가장 가까운 지점부터 전방 local path를 추출한다. localization
계산, 차량 제어, UDP 통신과 RViz marker는 이 패키지에 포함하지 않는다.

향후 GPS 기반 경로 대신 planner나 E2E 모델 경로를 사용할 때도 경로의 선택과
제어 입력 계약을 이 패키지 경계에서 관리한다.

## 입출력

| 구분 | 토픽 | 타입 | 동작 |
| --- | --- | --- | --- |
| 입력 | `/localization/pose` | `geometry_msgs/PoseStamped` | 최종 localization pose |
| 출력 | `/global_path` | `nav_msgs/Path` | 전체 공식 경로, latched 1회 발행 |
| 출력 | `/local_path` | `nav_msgs/Path` | 현재 위치부터 전방 경로, 위치 입력마다 갱신 |

모든 메시지는 현재 `map` frame을 사용한다. 위치 입력이 없더라도
`/global_path`는 발행되지만 `/local_path`는 발행되지 않는다.

## 현재 local path 계약

기본 모드는 `distance`이고 local path는 현재 위치부터 전방 **100 m**다.

- 현재 공식 경로 간격은 약 `0.5 m`이므로 일반적으로 약 201 pose가 들어간다.
- 목표 거리 이상이 되는 첫 waypoint까지 포함하므로 실제 길이는 100 m보다
  최대 한 waypoint 간격만큼 길 수 있다.
- 경로 파일의 점 간격이 바뀌어도 local path의 전방 거리는 100 m로 유지된다.
- 제어기는 `header.stamp`와 pose 수를 확인하고 오래된 경로를 계속 사용하지
  않아야 한다.

제어팀 인터페이스와 freshness 주의사항은
[`docs/interfaces/CONTROL_PATH_INTERFACE_KO.md`](docs/interfaces/CONTROL_PATH_INTERFACE_KO.md)에 더
자세히 정리되어 있다.

## 설정 파일과 파라미터

기본 설정은 `config/routes/molit_2026_kcity_route_path.yaml`이다.

| 파라미터 | 현재값 | 설명 |
| --- | --- | --- |
| `localization_topic` | `/localization/pose` | 최종 localization 입력 |
| `global_path_topic` | `/global_path` | 전체 경로 출력 |
| `local_path_topic` | `/local_path` | 제어용 부분 경로 출력 |
| `frame_id` | `map` | 경로 좌표 frame |
| `local_path_mode` | `distance` | `point_count` 또는 `distance` |
| `local_path_point_count` | `20` | point-count 모드에서만 사용하는 pose 수 |
| `local_path_length_m` | `100.0` | distance 모드에서 사용할 길이 |
| `max_path_point_spacing_m` | `1.0` | 입력 경로의 허용 최대 점 간격 |
| `search_backward_m` | `20.0` | 이전 index 기준 역방향 연속 탐색 범위 |
| `search_forward_m` | `200.0` | 이전 index 기준 전방 연속 탐색 범위 |
| `reacquire_distance_m` | `10.0` | 연속 탐색을 버리고 전역 재탐색할 거리 |
| `flatten_z` | `true` | 경로 z를 0으로 평탄화 |
| `require_closed_loop` | `true` | 폐루프 경로 강제 검증 |

현재 `distance` 모드에서는 `local_path_length_m`만 local path horizon에
영향을 주며 `local_path_point_count`는 사용하지 않는다. 별도 config에서
`point_count` 모드로 바꾸면 반대로 point count만 추출 기준이 된다.

경로 파일은 launch의 `path_file` 인자로 주입되는 필수 파라미터다.

## 실행

팀 공통 bringup 진입점:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_bringup molit_2026_path_manager.launch
```

패키지 자체를 개발하거나 독립 점검할 때:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_path_manager route_path_publisher.launch
```

다른 경로/config를 사용할 때:

```bash
roslaunch morai_path_manager route_path_publisher.launch \
  config:=/absolute/path/to/route.yaml \
  path_file:=/absolute/path/to/global_path.txt
```

센서, localization, path manager 전체 운영 stack:

```bash
roslaunch morai_bringup molit_2026_stack.launch
```

경로 RViz는 별도 터미널에서 실행:

```bash
roslaunch morai_bringup path.launch
```

## 경로 파일을 바꿀 때

1. 새 경로와 localization config가 같은 map/local 원점을 사용하는지 확인한다.
2. 파일의 점 간격이 `max_path_point_spacing_m` 이내인지 확인한다.
3. 폐루프가 아니면 이유를 검토한 뒤 `require_closed_loop`를 설정한다.
4. 100 m local path가 새 경로에서도 필요한 곡률 preview와 LD보다 긴지
   확인한다.
5. RViz에서 초록색 `START`, 전체 경로, 현재 위치와 local path가 겹치는지
   확인한다.

공식 원본은 `map/R-KR_PG_K-City_2025` 아래에 보존한다. 원본을 직접
덮어쓰기보다는 새 파일과 새 config를 추가해 비교 가능하게 관리한다.

## 시각화

```bash
roslaunch morai_bringup path.launch
```

표시 의미:

- 청록색: 전역경로
- 초록색 구와 `START`: 전역경로 파일의 첫 점
- 주황색: 전방 100 m local path
- 빨간색 구·화살표: 현재 최종 위치와 IMU yaw
- 노란색: 전역경로 최근접 waypoint
- 분홍색 선: 현재 위치와 최근접 waypoint 사이 거리

## 확인 및 테스트

```bash
rostopic echo -n 1 /global_path
rostopic echo -n 1 /local_path
rostopic hz /local_path
rostopic echo -n 1 /local_path/poses

cd ~/catkin_ws
catkin_make run_tests
catkin_test_results build/test_results --verbose
```

`rostopic echo -n 1 /local_path/poses`에서 현재 공식 경로 기준 약 201개 pose가
들어오는지 확인한다. 정확한 개수보다 첫 pose부터 마지막 pose까지의 arc
length가 100 m 이상인지가 기본 계약이다.
