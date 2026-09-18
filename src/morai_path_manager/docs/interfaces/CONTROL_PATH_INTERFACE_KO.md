# 제어팀 경로 인터페이스 인수인계

이 문서는 2026 MOLIT K-City 공식 XY 경로를 사용하는 제어기를 위한 독립적인
인터페이스 명세다. 경로 파일은
`map/R-KR_PG_K-City_2025/2026_molit_comp_global_path.txt`이며, 검증된 경로는
폐루프이고 총 길이는 **2.1846 km**다. 중복 종료점을 제외한 공식 waypoint는
4,391개이고, `/global_path`에는 폐루프를 표현하기 위해 첫 점을 다시 붙여
**4,392 pose**가 들어 있다.

## 토픽 계약

| 토픽 | 타입 | frame | 발행/의미 |
| --- | --- | --- | --- |
| `/global_path` | `nav_msgs/Path` | `map` | 시작 때 한 번 발행하는 **latched** 전체 공식 경로. 4,392 pose, 2.1846 km 폐루프다. 늦게 구독해도 마지막 메시지를 받는다. |
| `/local_path` | `nav_msgs/Path` | `map` | 최종 localization pose를 받을 때마다 발행하는 **non-latched** 전방 경로다. 현재 위치에 가장 가까운 공식 waypoint부터 전방 100 m 이상을 포함하며 폐루프 끝에서는 처음으로 이어진다. 현재 약 0.5 m 간격 경로에서는 일반적으로 약 201 pose다. |
| `/localization/pose` | `geometry_msgs/PoseStamped` | `map` | GPS X/Y와 IMU yaw를 결합한 최종 pose다. route path node는 position만 경로 검색에 사용하고 입력 `header.stamp`를 `/local_path`에 그대로 사용한다. |

`/global_path`와 `/local_path`의 각 `PoseStamped.pose.position`은 공식 경로의
XY 좌표이다. orientation은 인접 경로점으로 계산한 **경로 접선 방향**이며,
측정된 차량 yaw/IMU 자세가 아니다. 따라서 제어기는 orientation을 차량의 실제
자세 센서값으로 취급하거나 자세 추정의 입력으로 사용하면 안 된다.

GPS projector가 만드는 위치는 **GPS 안테나 점**이다. 현재 GPS 안테나는
`base_link`인 rear axle 중심의 수직 위 `(0, 0, 1.2)`에 있으므로 XY는 차량
기준점과 일치한다. 장착 위치가 바뀌면 lever arm 보정을 다시 검토해야 한다.

현재 구성은 noise-off 검증용 direct GPS/IMU localization을 사용한다. GPS X/Y와
IMU yaw를 직접 결합하며 EKF/UKF, dead reckoning과 보간은 없다. GPS blackout,
`NO_FIX`, IMU 중단 또는 두 센서 timestamp 차이가 허용값을 넘으면 새
`/localization/pose`와 `/local_path`가 발행되지 않는다. 특히 `/local_path`는
latched가 아니므로, 현재 구독자가 마지막 메시지를 보더라도 이를 최신 데이터로
간주해서는 안 된다.

제어기는 매 제어 주기마다 `/local_path.header.stamp`의 freshness를 검사해야
한다. GPS blackout에서는 stamp가 오래된 경로를 계속 보일 수 있으므로 타임아웃
시 안전 동작으로 전환해야 한다. 정확한 타임아웃 값은 여기서 정하지 않는다.
제어팀이 자신의 제어 주기와 안전 정책에 맞추어 선택해야 한다.

## 실행과 점검

센서 입력부터 localization과 route path publisher까지 함께 실행하려면 다음을
사용한다.

```bash
roslaunch morai_bringup molit_2026_stack.launch
```

센서, localization, path manager는 서로 다른 bringup launch다. GPS projector
대신 같은 `/localization/pose` 계약을 만족하는 다른 localization을 사용할
때는 `molit_2026_path_manager.launch`만 실행하면 된다.

토픽 타입/발행자와 갱신 상태는 다음처럼 확인한다.

```bash
rostopic info /global_path
rostopic info /local_path
rostopic hz /localization/pose
rostopic hz /local_path
```

`rostopic info /global_path`와 `rostopic info /local_path`에서는 각각
`nav_msgs/Path` 타입과 발행자를 확인한다. `/global_path`의 latch 여부는 이
문서의 계약(시작 시 한 번 발행 후 보존)이며, 늦게 시작한 구독자로
`rostopic echo -n 1 /global_path`를 실행해 수신 여부를 점검할 수 있다.
`/local_path`의 Hz는 최종 localization pose가 유효하게 들어올 때의 갱신률과
같다. GPS 또는 IMU 입력이 끊기면 pose와 local path Hz가 0이 되는 것이 정상이다.

경로 publisher만 독립 실행할 때는 다음 launch를 사용한다.

```bash
roslaunch morai_path_manager route_path_publisher.launch
```

빌드 후에는 현재 terminal에서 workspace를 다시 source한다.

```bash
cd /home/suhyeon/catkin_ws
catkin_make
source devel/setup.bash
```

## route path 파라미터

기본값은 `config/routes/molit_2026_kcity_route_path.yaml`에 있다. `path_file`만은 YAML
항목이 아니라 launch 인자이며, 기본값은 패키지의 공식 경로 파일이다.

| YAML 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `localization_topic` | `/localization/pose` | 구독할 `geometry_msgs/PoseStamped` 최종 pose |
| `global_path_topic` | `/global_path` | latched 전체 `nav_msgs/Path` 출력 토픽 |
| `local_path_topic` | `/local_path` | non-latched 전방 `nav_msgs/Path` 출력 토픽 |
| `frame_id` | `map` | 입력 point가 일치해야 하는 frame 및 두 path의 frame |
| `local_path_mode` | `distance` | `point_count`면 pose 개수, `distance`면 거리로 local path를 추출 |
| `local_path_point_count` | `20` | `point_count` 모드에서만 사용하는 pose 개수 |
| `local_path_length_m` | `100.0` | `distance` 모드에서 nearest waypoint부터 추출할 전방 거리(m) |
| `max_path_point_spacing_m` | `1.0` | 허용하는 인접 공식 경로점의 최대 간격(m); 초과하면 경로 로드를 거부 |
| `search_backward_m` | `20.0` | 직전 nearest index 주위에서 연속 탐색할 뒤쪽 거리(m) |
| `search_forward_m` | `200.0` | 직전 nearest index 주위에서 연속 탐색할 앞쪽 거리(m) |
| `reacquire_distance_m` | `10.0` | 연속 탐색 결과가 이 거리보다 멀면 전역 nearest 탐색으로 재획득하는 기준(m) |
| `flatten_z` | `true` | path pose의 Z를 0으로 평탄화할지 여부 |
| `require_closed_loop` | `true` | 폐루프 경로만 허용할지 여부 |

제어 preview를 조정하려면 재컴파일하지 않고 YAML의
`local_path_length_m`을 바꾸면 된다. 기본 100 m는 제어기의 45 m 첫 주행
곡률 preview와 16 m 최대 LD보다 길고, 실제 감속거리 측정 후 preview를
늘릴 여유가 있다. 추출은 목표 거리를 넘는 첫 waypoint까지
포함하므로 실제 길이는 설정값보다 최대 한 waypoint 간격만큼 길 수 있다.
pose 개수를 고정해야 하는 별도 구성에서만 `local_path_mode: point_count`와
`local_path_point_count`를 사용한다. 별도 설정 파일은 다음처럼 전달한다.

```bash
# 전체 stack
roslaunch morai_bringup molit_2026_stack.launch \
  route_path_config:=/absolute/path/to/route.yaml

# Path manager bringup
roslaunch morai_bringup molit_2026_path_manager.launch \
  config:=/absolute/path/to/route.yaml \
  path_file:=/absolute/path/to/global_path.txt
```

node는 시작 시 파라미터를 읽으므로, 실행 중 변경한 값은 node를 재시작한 뒤
적용된다.

## 문제 해결

### `/local_path`가 나오지 않음

1. `rosnode list | grep route_path_publisher`로 path node가 실행 중인지 확인한다.
2. `rostopic hz /localization/pose`로 GPS+IMU 최종 pose가 실제로 발행되는지
   확인한다. GPS blackout/`NO_FIX` 또는 IMU 중단에는 새 local path가 없다.
3. `rostopic echo -n 1 /localization/pose/header`로 frame이 `map`인지 확인한다.
   비어 있지 않은 다른 frame 입력은 node가 버린다.
4. node 로그에서 경로 파일 로드 실패 또는 파라미터 검증 오류를 확인한다.

### 소스를 바꿨는데 예전 node/문서가 보임

다른 catkin workspace의 prebuilt `devel`을 source한 경우가 많다. 현재 workspace를
다시 빌드하고 반드시 이 workspace의 `devel/setup.bash`를 마지막에 source한다.
특히 `/home/suhyeon/catkin_ws`처럼 이전 workspace의 setup을 남긴 terminal에서는
새 node나 설치된 `docs`가 아닌 오래된 결과가 보일 수 있다.

```bash
cd /home/suhyeon/catkin_ws
catkin_make
source devel/setup.bash
rospack find morai_path_manager
```

마지막 명령의 경로가 현재 workspace의 package를 가리키는지 확인한다.
