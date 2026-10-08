# MORAI-Autonomous-Driving-System

MORAI SIM 기반 자율주행 시스템. ROS 1 Noetic을 사용한다.

## Convention

### 협업과 Core 작성

이 프로젝트는 Strategy 패턴으로 작성하며, 알고리즘 Core와 ROS 실행 Pipeline을 분리해 협업한다.
Pipeline은 공통 실행기로 봉인한다. 노드는 YAML로 구성하고, 기능은 공식 확장점 파일로만 확장한다.

- 각 파트 담당자는 노드를 직접 개발하지 않고 자신의 Core와 가이드가 지정한 파일만 작성한다. 등록 전제 확인을 위한 기존 코드의 읽기는 허용한다.
- Core 개발·등록 전에 [Core 등록 가이드](src/node_pipeline/CORE_REGISTRATION.md)를 읽고 등록 전제 확인, 수정 범위, 검증, 사용자 안내·보고 절차를 따른다. Core 개발자는 가이드가 지정한 파일만 작성·수정한다.
- 허용 범위 밖의 변경은 중단하고, 가이드의 보고·리드 협의·명시적 허가 절차를 따른다.
- Core 개발 요청이나 원하는 동작의 지정만으로 허용 범위 밖의 변경이 승인된 것으로 해석하지 않는다. 알고리즘을 Core 밖의 Policy·Validator·Selector·파라미터 로더·Bank·callback 등에 옮겨 요구사항을 우회 구현하지 않는다.
- 리드 개발자는 노드의 `node.yaml`로 실행 방식·입력 계약·Stage 순서와 출력 계약·발행을 선언한다. 공통 실행기가 등록된 Core를 ROS 입출력에 연결하며, 노드별 실행 코드는 작성하지 않는다.
- Core는 정규 `process()`의 인자와 반환값만으로 Pipeline에 연결되는 알고리즘 컴포넌트다. 계산 라이브러리와 Core가 소유한 내부 계산 상태는 사용할 수 있다. 시각화 목적의 토픽 발행과 로그를 제외한 ROS 통신, 다른 노드·Pipeline·Core 객체 접근, 외부 공유 상태 및 별도 입출력 경로를 만들지 않는다. 같은 패키지에서 선언된 Bank는 예외로 허용한다.
- 시각화 목적의 토픽 발행과 로그만 Core 디커플링 규칙의 예외로 허용한다. 로그는 모든 확장점 파일에서 `<ros/console.h>`의 `ROS_*` 로그 매크로로 남긴다. 시각화 토픽 이름은 `/visualizer/*`로 지정하며, `/visualizer/*` 토픽은 `src/visualizer` 패키지 이외에서 절대 구독하지 않는다.
- Core는 입력이 자신의 입력 계약을 만족한다고 전제하고 계산한다. 입력 유효성 검사, 누락 감지, 입력 대체, fallback 및 처리 성공·실패 상태 반환을 구현하지 않는다. 입력 검사와 처리 정책은 Pipeline의 공통 검사층과 Validator·Policy 확장점이 담당한다. 알고리즘 자체에 필요한 조건 분기와 수치 계산은 수행한다.
- Core가 의존하는 입력 내용 전제(예: 특정 transform의 존재, 점군 필드 구성)는 `node.yaml`의 입력 한도나 연결한 Validator로 선언한다. Core는 실행 중 예외(`throw`·`.at()`·`.value()` 등)로 실패를 알리지 않는다. Core가 자체 라이브러리를 만들어 참조하는 방식으로 이 규칙들을 우회하지 않는다.
- Core 하나당 파일 하나를 둔다. 첫 Core는 실제 구독 메시지를 `process()`의 개별 인자로 받고, 후속 Core는 이전 Core의 반환값을 개별 인자로 받는다. 중간 Core는 다음 Core의 계산에 필요한 하나 이상의 변수를 반환할 수 있으며, 개수·타입·순서는 `node.yaml`의 Stage 출력 계약으로 정한다. 마지막 Core는 목표 발행 메시지 타입의 payload를 완성해 반환하며, 출력 헤더 주입과 발행은 Pipeline이 담당한다.

구체적인 입출력 계약과 작성 방식은 [node_pipeline 가이드](src/node_pipeline/README.md)를 참고한다.

### Core 등록

등록 절차와 설정 주입은 [Core 등록 가이드](src/node_pipeline/CORE_REGISTRATION.md)에 따른다.

Core 등록·해제에 Pipeline C++을 수정하지 않는다. 노드별 YAML은 `src/<package_name>/config/<node>/`,
확장점 파일은 `src/<package_name>/include/<package_name>/` 아래 종류별 디렉터리에 둔다.
각 YAML은 아래 담당 범위만 선언하며 같은 정보를 다른 YAML에 다시 선언하지 않는다.

| 파일 | 담당 | 내용 | 변경 반영 |
|---|---|---|---|
| `config/<node>/node.yaml` | 리드 | 실행 방식, 입력 계약, Stage 순서와 출력 계약, 발행 | 재빌드 |
| `config/<node>/registrations/<core>.yaml` | Core 개발자 | Core의 Stage, `process()` 인자, 확장점 연결 | 재빌드 |
| `config/<node>/selection.yaml` | Core 개발자 | Stage별 `section_core_map`·`health_core_map` | 재시작 |
| `config/<node>/parameters/<core>.yaml` | Core 개발자 | Core 계수 | 재시작 |
| `include/<package_name>/{cores,validators,policies,parameters,banks}/` | Core 개발자 | 확장점 구현 | 재빌드 |
| `include/<package_name>/libraries/`, `src/libraries/` | Core 개발자 | 계산 라이브러리 | 재빌드 |
| `package.xml`, `CMakeLists.txt` | Core 개발자 | 의존성·외부 include root 선언, `np_add_nodes()` 인자 | 재빌드 |
| `models/` | Core 개발자 | 파라미터 로더가 읽는 정적 자산 | 재시작 |

등록 파일 이름이 Core ID·Core 파일·클래스 이름을 정한다. 예를 들어 `full_ekf.yaml`은
Core ID `full_ekf.hpp`, 파일 `cores/full_ekf.hpp`, 클래스 `FullEkf`다.
등록 YAML을 추가하면 Core가 연결되고, 삭제하면 해제된다. 선택 매핑의 값은
확장자를 포함한 Core 파일 이름이다. 이후 `selection.yaml`의 매핑을 변경해
등록한 Core를 바꿔 끼우며 개발한다.

Core 내부에는 입력 valid 처리를 두지 않는다. 등록 YAML의 `inputs`는 `process()` 인자 순서이며
모두 필수 입력이다. `process()` 인자의 개수·타입은 컴파일 시 고정하며, Pipeline은 선언된 입력이
하나라도 Missing이면 해당 Core 호출과 발행을 거부한다. 선언된 입력의 Missing은 허용 issue로
면제하지 않는다. 필수 입력 외의 추가 메시지는 허용한다.
업스트림 메시지 환경에 대응하도록 사용할 입력 조합과 인자 개수별로 Core를 마련하고,
`health_core_map`에서 input별 issue 조건을 해당 Core 파일 이름에 연결한다.
`section_core_map`으로 기본 Core를 선택한다. 선택된 Core의 선언 입력이 모두 존재하고 허용 issue만
남을 때까지, 일치하는 health 매핑으로 Core를 덮어쓰고 선택된 Core의 Policy를 적용해 재검사한다.

입력 불안정에 대한 fallback은 이 Core 교체로 수행한다.
다른 곳, 특히 Core 내부에서 입력 대체나 별도의 fallback을 구현하지 않는다.

### 좌표계

#### 글로벌 좌표계 — `map`

지도에 고정된 좌표계이다. WGS84 / UTM zone 52N (`EPSG:32652`)에서
지도 원점 `(E₀, N₀, Z₀) = (302595.0, 4124145.0, 0.0)`을 차감해 사용한다.

| 항목 | 규약 |
|---|---|
| 원점 | 위 UTM 좌표에 해당하는 지도 기준점 |
| 축 | `+x` UTM 격자 동쪽, `+y` 격자 북쪽, `+z` 위쪽 |
| 단위 | m |
| 적용 대상 | 지도, 체크포인트, `/localization/ego`, `/local_path` |

GPS 위경도는 WGS84에서 UTM 52N으로 투영한 뒤 다음과 같이 변환한다.
격자 북쪽과 진북은 구분한다.

```text
x = Easting  - 302595.0
y = Northing - 4124145.0
```

- `checkpoints.csv`의 `x, y`와 `kcity_lanelet2.osm`의 `local_x, local_y`는 이 좌표계이다.
- OSM의 `lon, lat`은 WGS84 위경도이다. 각 노드의 추가 속성인
  `<tag k="local_x" v="..."/>`, `local_y`, `local_z`에는 지도 원점을 차감한
  `map` 좌표가 저장되어 있다. 이 이름의 `local`은 차량 `base_link`를 뜻하지 않는다.
- 전역 데이터의 Header는 `frame_id=map`으로 사용한다. `/localization/ego`는
  `nav_msgs/Odometry`로 차량 기준점의 위치·자세, 3축 선속도·각속도와 추정 기준 시각을 발행한다.
- 지도·체크포인트의 원본 z는 보존한다. 수직 기준면과 GPS 고도와의 관계는 아직 미확정이다.

#### Quaternion 규약

Pose와 TF의 orientation은 ROS `geometry_msgs/Quaternion`의 **`(x, y, z, w)`** 순서를
사용한다. `w`는 스칼라 성분이고, quaternion 성분 자체에는 각도 단위가 없다.
각도 입력은 rad를 사용하며 외부 deg 값은 변환한다. 진북 기준 센서 heading은
격자 북쪽과의 차이를 보정한 뒤 사용한다.

| 항목 | 규약 |
|---|---|
| 회전 방향 | 오른손 법칙; 양의 yaw는 `+z`에서 내려다볼 때 반시계 방향 |
| 회전 없음 | `(0, 0, 0, 1)` |
| 크기 | 단위 quaternion: `x² + y² + z² + w² = 1` |
| ego orientation | `base_link`의 축을 `map`에서 표현하는 회전 |
| TF 회전 | 자식 frame의 벡터를 부모 frame으로 변환하는 회전 |
| RPY 변환 | rad 단위, 고정축 X→Y→Z 회전; `R = Rz(yaw) Ry(pitch) Rx(roll)` |

#### 로컬 좌표계 — `base_link`

차량에 고정된 오른손 좌표계이다. 차량이 이동·회전하면 함께 이동·회전한다.

| 항목 | 규약 |
|---|---|
| 원점 | **MORAI 차량 좌표계 원점**: 공식 정의상 좌우 뒷바퀴 축의 중점(뒷차축 중심) |
| 축 | `+x` 전방, `+y` 좌측, `+z` 위쪽 |
| 단위 | m |
| 적용 대상 | `/velodyne_points/leveled`, `/objects`, `/local_path/velocity_injected` |


#### 센서 좌표계 및 변환

- **IMU (`imu_link`)**: 장착 RPY가 `(0, 0, 0)°`이면 `imu_link`와 `base_link`의 축 방향이 같다.
- **GPS (`gps_link`)**: 장착 RPY가 `(0, 0, 0)°`이면 `gps_link`와 `base_link`의 축 방향이 같다.
- **LiDAR (`velodyne`)**: 장착 RPY가 `(0, 0, 0)°`이면 `velodyne`와 `base_link`의 축 방향이 같다.
- 센서 원점은 각 센서의 장착 위치이다. 축 방향이 같다면 `base_link`로 변환할 때 장착 위치의 평행이동을 적용한다.


### 차량·제어 계약

| 항목 | 규약 |
|---|---|
| 차량 / 휠베이스 | Hyundai Ioniq5 / 3.0 m |
| 차량 경계 | `base_link` 기준 bbox 사용; 상태 메시지의 `size_*`는 별도 제원 값 |
| 종방향 속도 / 가속도 | m/s / m/s², 전방 +·후방 − |
| 내부 조향각 | rad, 좌회전 +·우회전 −; 최대 조향각 40° (0.698132 rad) |
| `CtrlCmd.steering` | `조향각 / 최대 조향각`인 정규화값 [−1, 1] |
| `CtrlCmd.accel` / `brake` | 페달값 [0, 1]; 둘 다 양수이면 Bridge가 accel=0 처리 |
| Bridge 제어 모드 | UDP Mode 1, D 기어(4) 고정; `velocity`·`acceleration` 필드는 사용하지 않음 |
| Bridge 상태 단위 변환 | km/h → m/s, 조향·RPY deg → rad |

Control은 이 프로젝트의 Bridge 계약에 맞춰 조향각을 정규화해 발행한다.
상세 치수·bbox 8꼭지점·센서 장착 설정 및 검증 근거는 [차량 및 센서 제원](spec/README.md)을 참조한다.

### 데이터 계약

- `/localization/ego`의 `pose.covariance`와
  `twist.covariance`는 미제공으로 정의한다. 두 배열은 기본값 0으로 발행하며,
  구독자는 공분산을 사용하지 않는다. 0을 오차가 없다는 의미로 해석하지 않는다.
- `/localization/ego`는 `nav_msgs/Odometry`를 사용한다. `header.frame_id=map`,
  `child_frame_id=base_link`, `header.stamp`는 추정 기준 IMU 시각이다.
  `pose.pose`는 차량 뒷차축 중심의 map 위치·자세이며 `position.z=0`이다.
  `twist.twist.linear.{x,y,z}`는 EKF로 추정한 base_link 기준 전방·좌측·위쪽
  속도(m/s), `twist.twist.angular.{x,y,z}`는 차량 상태의 GT 각속도(rad/s)다.
  자세는 차량 상태의 GT roll·pitch·yaw에 UTM 격자 heading 보정을 적용한다.
  수직속도는 GPS 고도 변화로 추정하며 절대 고도를 map z로 사용하지 않는다.
- TODO: `/vehicle/competition_status`의 노드별 사용 필드.
- TODO: 센서 시각, 유효성, 입력 만료 및 결과 발행 조건.
- TODO: 구간 미확정 시 초기 동작과 구간 전환 조건.

## 패키지 구조 설계

소스·설정·모델·테스트·패키지 전용 실행 파일은 해당 `src/<package_name>/` 안에 둔다.
내부 패키지 간 데이터는 ROS 토픽·서비스·액션으로 전달하고, 공유 사용자 정의 타입은
`interfaces`에 정의한다. 표준 ROS 및 외부 의존성은 각 패키지에서 선언한다.


### 패키지 구성

```text
src/
├── MORAI-ROS_morai_msgs/       # 외부 메시지
├── MORAI-UDP-Bridge-Node/      # MORAI UDP 입출력
├── velodyne/                  # LiDAR 드라이버
├── interfaces/                # 공유 사용자 정의 타입
├── node_pipeline/             # 공통 실행기·생성기·계약 검사
├── map/                       # 지도 자산 및 체크포인트
├── localization/              # Localization_Node
├── lidar_leveler/             # LiDAR_Leveler_Node
├── object_detector/           # Object_Detector_Node
├── traffic_light/             # Traffic_Light_Node
├── section_annotator/         # Section_Annotator_Node
├── tf_publisher/              # TF_Publisher_Node, Static_TF_Publisher_Node
├── path_planner/              # Path_Planner_Node
├── velocity_planner/          # Velocity_Planner_Node
├── control/                   # Control_Node
└── speed_filter/              # Speed_Filter_Node
```
위 구성에 포함되지 않은 이름의 패키지는 데모를 위한 임시 패키지로, 추후 새로 제작한다.

`MORAI-ROS_morai_msgs`, `MORAI-UDP-Bridge-Node`, `velodyne`는 Git 서브모듈이다.
각 서브모듈은 이 저장소가 기록한 커밋에 고정된다. 서브모듈 원격 브랜치에 새 커밋이
생겨도 부모 저장소의 포인터를 갱신해 커밋하기 전에는 이 저장소를 pull해도 자동으로
따라가지 않는다. 현재 고정 커밋은 아래 패키지 설명을 따른다.

이 프로젝트의 패키지는 서비스 패키지와 실행 패키지 두가지이다.

**서비스 패키지**: 실행 패키지에 라이브러리, 정의 파일들을 제공하는 패키지. 
* interfaces/
* MORAI-ROS-morai_msgs/
* map/
* node_pipeline/

**실행 패키지**: 실행되는 ROS 노드를 갖는 패키지. **실행 패키지 사이에 커플링을 만들지 않는다.**
* localization/
* lidar_leveler/
* ...etc

## ROS 1 architecture

노드는 원, 토픽은 사각형, 파일 입력은 원통으로 표현한다.
아래 연결은 설계안이며, 메시지 타입은 Topics에서 별도로 확정한다.

```mermaid
flowchart TD
    S((("MORAI")))
    B(("MORAI-UDP-Bridge-Node"))
    V(("velodyne"))
    subgraph PL ["perception layer"]
        L(("Localization_Node"))
        LL(("LiDAR_Leveler_Node"))
        ll["/velodyne_points/leveled"]
        O(("Object_Detector_Node"))
        TL(("Traffic_Light_Node"))
    end
    subgraph GM ["global manager"]
        SA(("Section_Annotator_Node"))
        TF(("TF_Publisher_Node"))
        STF(("Static_TF_Publisher_Node"))
    end
    subgraph J ["judgement layer"]
        P(("Path_Planner_Node"))
        p["/local_path"]
        VP(("Velocity_Planner_Node"))
    end
    subgraph CL ["control layer"]
        C(("Control_Node"))
        ctrl["/control_cmd"]
        SF(("Speed_Filter_Node"))
    end

    LET[("lanelet")]
    CHK[("checkpoints")]
    g["/sensors/gps/fix"]
    i["/sensors/imu/data"]
    c["/vehicle/competition_status"]
    e["/localization/ego"]
    l["/velodyne_points"]
    if["/image/front/compressed"]
    il["/image/left/compressed"]
    ir["/image/right/compressed"]
    i7["/image/camera_7/compressed"]
    o["/objects"]
    tl["/traffic_light"]
    sa["/section_annotation"]
    tf["/tf"]
    tfs["/tf_static"]
    pv["/local_path/velocity_injected"]
    ct["/morai_udp_bridge/control_cmd"]

    S ~~~ B
    S ---> V
    S <--> B
    B ---> g
    B ---> i
    B ---> c
    B ---> if
    B ---> il
    B ---> ir
    B ---> i7
    V ---> l ---> LL ---> ll
    e ---> SA ---> sa
    e ---> TF --> tf
    STF --> tfs
    tfs --> LL
    tfs --> L
    sa ---> TL
    i7 ---> TL --> tl
    sa ---> O
    ll ---> O ---> o
    e ---> P
    c ---> P
    sa ---> P
    LET --> P
    CHK --> P
    o ---> P
    tf --> P
    tf --> VP
    e ---> C
    c ---> C
    i ---> C
    P ---> p
    sa --> VP
    e ---> VP
    c ---> VP
    LET --> VP
    tl --> VP
    p ---> VP --> pv --> C
    LET ---> SF
    sa ---> SF
    c ---> SF
    C ---> ctrl ---> SF ---> ct ---> B
    ll ---> L
    c ---> L ---> e
    sa ---> L
    g ---> L
    i ---> L
    c ---> LL
```

### Nodes

| 노드 | 역할 |
|---|---|
| MORAI-ROS_morai_msgs | 커밋 `45c6baf148f2327f4c9fefd48262f25bdfe4b567`에 고정하여 깃 서브모듈 링크. MORAI ROS 메시지 정의. |
| MORAI-UDP-Bridge-Node | 커밋 `dbd2da34abbd13634304ad20803b17be13228007`(현장 점검에서 검증된 저장소)에 고정하여 깃 서브모듈 링크. MORAI UDP 패킷과 ROS 토픽 사이를 번역. 단순 번역 계층으로 독자적인 로직·판단으로 콘텐츠를 훼손·누락하지 않는다. 센서·차량 상태 수신 및 최종 제어 명령 송신. |
| velodyne | 커밋 `29abd0e1361cb7f5eda451d2b51c35eeca45e0d5`에 고정하여 깃 서브모듈 링크. velodyne vlp16의 공식 드라이버. LiDAR 데이터를 `/velodyne_points`로 발행. |
| Localization_Node | gps,imu,LiDAR센서 데이터를 이용하여 차량의 글로벌 위치를 추정하고 /localization/ego로 발행한다.         |
| LiDAR_Leveler_Node | TF_Publisher의 센서 장착 TF로 점군을 `base_link`로 변환해 `/velodyne_points/leveled` 발행. |
| Object_Detector_Node | 수평화 점군을 클러스터링해 `/objects` 발행. |
| Traffic_Light_Node | 목표 5 Hz로 최신 camera_7 영상을 YOLO 추론해 `/traffic_light` 발행. 추론은 중첩하지 않으며 대기 영상은 ROS 구독 큐(`queue_size=1`)에서 관리. |
| Section_Annotator_Node | ego가 맵의 어느 구간에 위치해 있는지 발행. 각 노드는 콜백으로 자신의 모드를 스위치할 수 있다. |
| TF_Publisher_Node | ego 콜백으로 동적 `map → base_link` 발행. |
| Static_TF_Publisher_Node | 센서 장착 변환을 시작 시 한 번 `/tf_static`으로 latched 발행. TF_Publisher_Node와 같은 `tf_publisher` 패키지에 둔다. |
| Path_Planner_Node | ego 위치·자세와 `/vehicle/competition_status`의 차량 상태를 사용해 차선 추종·장애물 회피 경로 생성. |
| Velocity_Planner_Node | 전달받은 경로의 속도 프로파일을 생성하고 TF로 `map`에서 `base_link`로 변환해 발행. 경로 형상을 다시 계획하지 않는다.     |
| Control_Node | 속도 프로파일이 주입된 경로와 ego·차량 상태·IMU를 사용해 제어 명령 생성. |
| Speed_Filter_Node | 제어 명령에 최종 속도 제한을 적용해 Bridge에 전달. 최종 제어 명령. |

### Topics

센서·Bridge 행은 현재 코드·설정 기준이며, 신규 노드 행은 구현할 계약이다.
`timestamp`는 별도 표시가 없으면 `header.stamp`를 뜻한다.
`허용 지연`은 입력 stamp부터 결과 발행까지의 한도다. `수신 timeout`은 기대한 입력이 도착하지 않는 시간 한도이며, `stale 최대 나이`는 stamp 기준으로 메시지를 사용할 수 있는 최대 나이다. 수신 timeout과 stale 최대 나이는 별도 설정값이다.

#### 공통 통신·처리 정책

- 모든 토픽은 **최신값 우선, 대기 깊이 1**을 원칙으로 한다. ROS 2 QoS로 표현하면
  **Best Effort + Keep Last(1)**이다. 현재 ROS 1 Noetic에서는 발행·구독 `queue_size=1`로 설정한다.
  이는 대기 메시지 수 제한이며, TCPROS를 Best Effort 전송으로 바꾸는 설정은 아니다.
- 대기 메시지는 **ROS 구독 큐(`queue_size=1`)**가 관리한다. 새 메시지가 수신되면
  ROS는 큐에서 대기 중인 이전 메시지를 버리고 최신 메시지만 남긴다.
- 다중 입력을 시각으로 결합하는 노드는 각 입력에 제한된 큐를 둘 수 있다. 이 큐는 인접 timestamp
  입력을 조립하기 위한 것이며, 조립된 입력 묶음은 기본적으로 직렬 처리한다. 큐 크기와 최대 시각 차이는 노드별 계약으로 정한다. 별도 큐 대기 한도는 두지 않고, 묶음을 결합한 뒤 사용하기 전 이뤄지는  `stale` 검사에 신선하지 않은 입력에 대한 대응을 맡긴다.
- ROS 구독 큐는 `queue_size=1`로 두고, 그 다음의 timestamp 결합 큐와는 별도로 관리한다. 단일 입력
  콜백은 새 입력을 처리하고 발행한다. 시간 결합 노드는 조립된 새 입력 묶음에 대해서만 발행하며,
  주기 실행 노드는 목표 주기에 최신으로 조립된 유효 입력을 사용한다.
- 입력의 `receive_timeout`은 시작 후 첫 메시지를 기다리는 시간 및 수신 이후의
  경과 시간으로 판정한다. 시간 결합 노드는 선택된 입력의 수신 시각을 기준으로 한다. stamp가 있는 메시지는 별도로 `현재 ROS 시각 - header.stamp`를 계산해
  `stale 최대 나이`와 비교한다. 시간 초과 판정은 독립적인 monotonic clock으로 수행한다.
  미래 stamp나 유효하지 않은 값은 stale과 별개로 잘못된 입력으로 거부한다.
- stale 입력에 대한 정책은 각 패키지에서 결정한다.
- 주기 토픽의 기본 `receive_timeout`과 `stale 최대 나이`는 각각 입력 주기의 3배이며, 설정에서 별도로 조정한다.
  이벤트 기반 입력은 해당 토픽의 timeout을 노드 설정서 조정한다.
- Header가 없는 `/control_cmd` 계열은 원래 생성 시각을 알 수 없으므로 stale검사를 생략한다.
- `/tf_static`은 **latched**로 유지하여 늦게 연결한 구독자에게도 전달한다.
  한 발행자의 정적 변환 전체를 하나의 메시지에 담는다.


| 토픽 | 내용 | 메시지 타입 | frame / 좌표 기준 | timestamp 기준 | 발행 주기 / 트리거 |
| --- | --- | --- | --- | --- | --- |
| `/sensors/gps/fix` | GPS 측위 | `sensor_msgs/NavSatFix` | `gps_link` (위경도 값은 WGS84) | Bridge 수신 ROS 시각 | 유효 GPS 패킷 수신마다 |
| `/sensors/imu/data` | IMU | `sensor_msgs/Imu` | `imu_link` | Bridge 수신 ROS 시각 | 유효 IMU 패킷 수신마다 |
| `/vehicle/competition_status` | 차량 상태 | `interfaces/CompetitionVehicleStatus` | `base_link`; 필드별 기준은 별도 정의 | Bridge 수신 ROS 시각 | 유효 차량 상태 패킷 수신마다 |
| `/image/front/compressed` | 전방 영상 | `sensor_msgs/CompressedImage` | `camera_front_optical_frame` | Bridge 영상 수신 ROS 시각 | 영상 프레임 조립 완료마다 |
| `/image/left/compressed` | 좌측 영상 | `sensor_msgs/CompressedImage` | `camera_left_optical_frame` | Bridge 영상 수신 ROS 시각 | 영상 프레임 조립 완료마다 |
| `/image/right/compressed` | 우측 영상 | `sensor_msgs/CompressedImage` | `camera_right_optical_frame` | Bridge 영상 수신 ROS 시각 | 영상 프레임 조립 완료마다 |
| `/image/camera_7/compressed` | 신호등 영상 | `sensor_msgs/CompressedImage` | `camera_7_optical_frame` | Bridge 영상 수신 ROS 시각 | 영상 프레임 조립 완료마다 |
| `/velodyne_points` | 원본 점군 | `sensor_msgs/PointCloud2` | `velodyne` | 드라이버 scan stamp 유지; 현재 설정은 마지막 패킷 시각 | 10 Hz (100 ms); scan 수신마다 |
| `/velodyne_points/leveled` | 차량 기준 점군 | `sensor_msgs/PointCloud2` | `base_link` | 입력 점군 stamp 유지 | 목표 10 Hz; `/velodyne_points` 콜백마다 |
| `/localization/ego` | 차량 위치·자세·3축 선속도·각속도 | `nav_msgs/Odometry` | pose: `map`, twist: `base_link`; 차량 뒷차축 중심 | 추정 상태가 나타내는 시각; 발행 시각으로 대체하지 않음 | 50 Hz |
| `/objects` | 장애물 검출·추적 객체 목록 | `interfaces/Objects` | `base_link` | 입력 수평화 점군 stamp 유지 | 목표 10 Hz; `/velodyne_points/leveled` 콜백마다 |
| `/traffic_light` | 신호등 인식 결과 | `interfaces/TrafficLight` | `camera_7_optical_frame` | 판정에 사용한 영상 stamp 유지 | 목표 5 Hz (200 ms); 최신 미처리 영상 사용 |
| `/section_annotation` | 현재 구간 정보 | `interfaces/SectionAnnotation` | `map` | 판정에 사용한 ego stamp 유지 | `/localization/ego` 콜백마다 |
| `/tf` | 동적 좌표 변환 | `tf2_msgs/TFMessage` | 각 변환의 `header.frame_id`가 부모, `child_frame_id`가 자식 | ego stamp 유지 | `/localization/ego` 콜백마다 |
| `/tf_static` | 센서 고정 장착 변환 | `tf2_msgs/TFMessage` | `base_link` → 센서 frame (`velodyne`, `imu_link`, `gps_link`, 카메라 optical frame) | 0 (조립 기준 입력 없음) | 시작 시 1회 (latched) |
| `/local_path` | 계획된 경로 | `interfaces/Path` | `map`; 각 point의 pose도 `map` | 입력 조립 기준 stamp(전체 입력 큐의 최대 stamp); 각 point pose stamp도 동일 | 20 Hz 주기 |
| `/local_path/velocity_injected` | 속도 프로파일이 주입된 경로 | `interfaces/Path` | `base_link`; 각 point의 pose도 `base_link` | 입력 경로 및 각 point pose stamp 유지 | `/local_path` 콜백마다 |
| `/control_cmd` | 필터 적용 전 명령 | `morai_msgs/CtrlCmd` | frame 필드 없음; 차량 조향·종방향 명령 | timestamp 필드 없음; 수신자가 수신 시각 기록 | `/local_path/velocity_injected` 콜백마다 |
| `/morai_udp_bridge/control_cmd` | 최종 명령 | `morai_msgs/CtrlCmd` | frame 필드 없음; 차량 조향·종방향 명령 | timestamp 필드 없음; Bridge가 수신 시각 기록 | `/control_cmd` 콜백마다 |



#### 시간 및 frame 계약

`tf_publisher` 패키지가 아래 트리를 소유한다. TF_Publisher_Node가 동적 변환을,
Static_TF_Publisher_Node가 정적 변환을 발행한다. 동적 변환은 ego stamp를 유지하고,
정적 변환은 센서 장착 위치·자세와 camera optical 축 규약을 반영한다.
정적 변환 전체는 시작 시 한 메시지로 latched 발행한다.

```text
map
└── base_link                    # 동적: ego pose
    ├── gps_link                 # 정적: 센서 장착 변환
    ├── imu_link                 # 정적
    ├── velodyne                 # 정적
    ├── camera_front_optical_frame # 정적: optical 축 변환 포함
    ├── camera_left_optical_frame  # 정적: optical 축 변환 포함
    ├── camera_right_optical_frame # 정적: optical 축 변환 포함
    └── camera_7_optical_frame     # 정적: optical 축 변환 포함
```

### External inputs and outputs

- **MORAI**: 센서·차량 상태의 공급자이자 제어 명령 수신자.
- **lanelet**: 경로 계획과 속도 계획·제한에 사용하는 Lanelet2 OSM 데이터다. OSM `lon/lat`은 WGS84이고, 노드의 `local_x/local_y/local_z` 태그는 `map` 좌표다. 포맷·필드·좌표 규약은 [`src/map/kcity_lanelet2.meta.json`](src/map/kcity_lanelet2.meta.json)에 정의한다.
- **checkpoints**: 경로 계획의 경유 지점이다. UTF-8 CSV의 `x,y,z,yaw` 필드와 행 순서로 표현하며, 위치는 `map` 기준 m, `yaw`는 `+x` 동쪽 0°·반시계 양수의 deg다. 포맷·필드·좌표 규약은 [`src/map/checkpoints.meta.json`](src/map/checkpoints.meta.json)에 정의한다.

## Bringup

`./run.sh`가 시스템 전체 bringup의 유일한 진입점이다. 저장소 바깥의 현재 작업
디렉터리와 무관하게 동작해야 하며, 사용자가 별도로 ROS 환경을 `source`하거나
빌드·네트워크 준비 명령을 실행할 필요가 없어야 한다. 실행 스크립트가 불러온 환경은
그 스크립트와 자식 프로세스에 적용된다.

### 실행 규약

- `run.sh`는 저장소 루트를 스크립트 위치에서 찾고 ROS Noetic 환경을 불러온다.
- 필요한 시스템·ROS 의존성을 확인하고, 없으면 `rosdep`/시스템 패키지 관리자로 설치를
  시도한다. 현재 실행 스택이 쓰는 RViz와 compressed image transport 의존성도 포함한다.
  워크스페이스는 `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo`로 빌드하고
  `devel/setup.bash` 환경을 사용한다.
- 새 스택을 시작하기 전에 컴퓨터에서 실행 중인 기존 ROS 프로세스를 모두 정리한다.
  이 동작은 같은 컴퓨터에서 실행 중인 다른 ROS 스택에도 영향을 준다.
- 환경 준비, 의존성 설치, 빌드, 네트워크 설정, 제한 시간 내 readiness 미확인 또는
  개별 패키지 시작이 실패해도 그 실패를 기록하고 다음 실행 단계를 시도한다. 실패를
  성공으로 표시하지 않으며, 전체 bringup 결과와 실패한 단계를 사용자에게 보여준다.
  실행 시도 완료는 주행 준비 완료를 뜻하지 않는다.
- 시작한 프로세스는 `run.sh`가 추적한다. 종료 신호를 받으면 시작한 프로세스들을 함께
  정리한다. 업스트림 토픽이 아직 없더라도 다음 패키지 시작을 막지 않는다.

### 패키지 실행 진입점

공식 Velodyne 드라이버만 예외로 하며, 나머지 모든 실행 패키지는
`src/<package_name>/launch.sh`를 제공한다. 각 `launch.sh`는 어느 작업 디렉터리와
쉘에서 호출해도 ROS 및 워크스페이스 환경을 준비하고 자기 패키지를 한 번에 시작해야
한다. 패키지 내부의 `scripts/*.py` 같은 헬퍼 사용은 허용한다.

`run.sh`는 Velodyne을 제외한 패키지의 `roslaunch`, 노드 실행 파일, 패키지 내부 launch
파일을 직접 호출하지 않는다. 해당 패키지의 `launch.sh`만 호출한다. Velodyne은 공식
드라이버의 제공 launch 파일을 사용한다.

### 실행 순서

`run.sh`는 아래 순서로 실행 패키지의 `launch.sh`를 호출한다. Velodyne은 공식 드라이버
예외로, 제공된 launch 파일을 호출한다. 개별 패키지 시작에 실패해도 다음 패키지의 시작을
시도한다.

| 순서 | 실행 패키지 |
|---:|---|
| 1 | `MORAI-UDP-Bridge-Node` |
| 2 | `velodyne` (공식 드라이버 예외) |
| 3 | `tf_publisher` |
| 4 | `lidar_leveler` |
| 5 | `localization` |
| 6 | `section_annotator` |
| 7 | `object_detector` |
| 8 | `traffic_light` |
| 9 | `path_planner` |
| 10 | `velocity_planner` |
| 11 | `control` |
| 12 | `speed_filter` |
