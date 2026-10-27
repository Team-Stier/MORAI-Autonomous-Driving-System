# 차량 및 센서 제원

## 모델 및 치수

대회 빌드의 `hyundai_ioniq5.vehicle` 자산 기준이다. 실행 중 Dynamics 오버라이드는 별도이다.

| 항목 | 값 |
|---|---|
| 차량 | Hyundai Ioniq5 (`hyundai_ioniq5.vehicle`) |
| bbox 길이 × 폭 × 높이 | 4.635008 × 2.160835 × 1.644046 m |
| 휠베이스 | 3.000000 m (앞·뒤 바퀴 Transform의 간격) |
| 후방 오버행 (bbox 기준) | 0.790000 m |
| 전방 오버행 (bbox 기준) | 0.845008 m |
| 모델 최대 조향각 설정 | 40° = 0.698132 rad (`steering.maxSteerAngle`) |

## 속도·조향·가감속

| 항목 | 단위·부호·범위 |
|---|---|
| 내부 종방향 속도 | m/s, 차량 전방 + / 후방 − |
| 내부 종방향 가속도 | m/s², 전방 + / 후방 −; 후진 중 감속은 양수일 수 있음 |
| 내부 조향각 | rad, 좌회전 + / 우회전 −; 모델 설정 기준 ±0.698132 rad |
| 이 프로젝트의 `CtrlCmd.steering` | 무차원 정규화값 `조향각 / 최대 조향각`, 범위 [−1, 1]; 기존 Controller는 부호 반전 없이 송신 |
| `CtrlCmd.accel` | 무차원 가속 페달, [0, 1]; 음수로 후진을 지시하지 않음 |
| `CtrlCmd.brake` | 무차원 제동 페달, [0, 1] |
| Bridge 동시 페달 처리 | accel·brake가 모두 양수이면 accel=0으로 처리 |
| Bridge 송신 모드 | Mode 1, D 기어(4) 고정; 후진 기어 지시하지 않음    |
| `CtrlCmd.velocity/acceleration` | 현재 Bridge는 읽지 않음; 속도·가속도 직접 제어 미지원 |
| 차량 상태 속도 | UDP km/h를 Bridge가 m/s로 변환 |
| 차량 상태 조향·RPY | UDP deg를 Bridge가 rad로 변환 |

**현재 Bridge의 `CtrlCmd.steering`은 MORAI 표준 ROS의 rad 계약과 다르다.**
UDP의 정규화 조향값을 그대로 전송하므로 Control이 정규화해서 넣어야 한다.
[UDP 공식 정의](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/udp-1)와
[Bridge 코드](../src/MORAI-UDP-Bridge-Node/src/protocol.cpp)를 기준으로 한다.
좌·우 응답과 실행 중 최대 조향각은 이번 작업에서 주행 검증하지 않았다.

기존 `src/morai_path_tracking` 설정의 운용 제한은 최고속도 **60 km/h (16.666667 m/s)**,
종방향 가속도 **+2.5 / −5.0 m/s²**, 가속 페달 상한 **0.70**이다.
이는 [기존 제어기 설정](../src/morai_path_tracking/config/controllers/molit_2026_path_tracking.yaml)의
값이며 차량의 물리적 최고속도·최대 가감속 성능이나 신규 제어기의 확정 제한값이 아니다.
차량의 실제 최고속도·최대 가감속은 해당 자산 필드만으로 확정하지 않는다.


## 차량 bbox — `base_link` 기준

대회 빌드 `v.S4.251001.MolitComp03`의 `hyundai_ioniq5.vehicle`에서 추출한
**Ioniq5 모델의 bbox**이다. 실행 중 gRPC bbox와 대조했으며 8꼭지점 성분의 최대 차이는
0.443 mm였다. 단위는 m, 아래 표는 소수점 6자리로 표시한다.

| 꼭지점 | 위치 | `(x, y, z)` |
|---|---|---|
| 0 | 앞·좌·하 | `(3.845008, 1.080417, -0.368500)` |
| 1 | 앞·우·하 | `(3.845008, -1.080418, -0.368500)` |
| 2 | 뒤·우·하 | `(-0.790000, -1.080418, -0.368500)` |
| 3 | 뒤·좌·하 | `(-0.790000, 1.080417, -0.368500)` |
| 4 | 앞·좌·상 | `(3.845008, 1.080417, 1.275546)` |
| 5 | 앞·우·상 | `(3.845008, -1.080418, 1.275546)` |
| 6 | 뒤·우·상 | `(-0.790000, -1.080418, 1.275546)` |
| 7 | 뒤·좌·상 | `(-0.790000, 1.080417, 1.275546)` |

- 길이 × 폭 × 높이: **4.635008 × 2.160835 × 1.644046 m**.
- 원점 기준 bbox 중심: `(1.527504, -0.000000179, 0.453523) m`.
- 하단 z는 **-0.368500 m**이며 원점은 지면이 아니다. 앞바퀴 축은 원점에서 전방 3 m이다.
- 근거: 모델의 `BoxColl`·`BoxCollTrigger` 경계값 및 부모 Transform 전체가 동일 회전·이동 없음,
  배율 1임을 확인했다. Unity `(x,y,z)`를 `base_link (z,-x,y)`로 변환했다.
- **런타임 대조 (2026-09-29)**: `GetActorState`의 bbox 크기는
  **4.635008 × 2.160835 × 1.644046 m**로 위 자산 값과 일치했다.
  이 빌드의 gRPC `extent` 순서는 **폭/2, 길이/2, 높이/2**이다.
- 상태 패킷의 **4.635 × 1.892 × 2.434 m**는 `GetVehicleSpec`의 차량 제원과 일치하며,
  collider bbox와는 별개이다. `size_*`로 위 꼭지점을 재구성하지 않는다.
  런타임 휠베이스 3.000 m, 전방/후방 오버행 0.845/0.790 m, 최대 조향각 40°도 확인했다.
- **원점 대조**: 상태 위치는 모델의 실제 Transform 원점을 발행한다.
  런타임 bbox 중심을 이 원점 기준으로 역변환한 값은 자산 경계값과 1 mm 이내로 일치했다.
  `base_link`는 모델의 기준 뒤차축 중앙이며, 서스펜션 운동 중 바퀴 메시의 순간 중심을 따라 이동하는 좌표계는 아니다.
- 대회 패킷의 위치 영역은 모두 0이다. 위치 대조에는 일반 Ego 패킷과 gRPC를 사용했다.
  자세 역변환에는 이 빌드의 yaw 오프셋을 반영했다. 측정값·변환 근거·원본 패킷은
  [ioniq5_bbox.meta.json](ioniq5_bbox.meta.json)에 기록한다.

## 센서 장착 설정

센서별 장착 위치·RPY와 출력 설정은 [0918sensors.json](0918sensors.json)을 참조한다.
장착 위치는 `base_link` 기준 m, 장착 RPY는 deg이다.

## LiDAR 출력 주기

LiDAR 설정 주기는 **10 Hz (100 ms)**이다.
[0918sensors.json](0918sensors.json)의 `Lidar3DList[0].lc.sensorPeriod`는 약 0.1 s이며,
[run.sh](../run.sh)는 RPM 재정의 없이 VLP16 드라이버를 실행한다.
[VLP16_points.launch](../src/velodyne/velodyne_pointcloud/launch/VLP16_points.launch)의
기본값 600 RPM도 10 Hz에 해당한다. 이는 설정 기준이며 수신 주기의 실측값은 아니다.

대회 빌드 **25.S4.MolitComp03**의 VLP-16 (`lidar3DType=0`)은
**5 / 10 / 15 Hz**를 선택할 수 있다. 기본 제공 설정의 최대 선택값은 **15 Hz**이다.
15 Hz로 변경하려면 MORAI Rotation Rate와 ROS 드라이버 `rpm:=900`을 함께 맞춘다.
현재 10 Hz는 저장된 설정 및 드라이버 기본값이며, 최대치나 공식 권장값을 뜻하지 않는다.

확인 근거: 해당 빌드 `Assembly-CSharp.dll`의 `CLidar3D.GetLidar3DFrequencies`
(RVA `0x9afff8`)에서 모델 0이 반환하는 배열을 확인했다.
배열 초기값은 `(5, 10, 15)`이며, DLL SHA-256은
`697e89e113febed8ac63ec328f49d8906c16ef3e7aebfb99c920992917b62eb6`이다.
실제 VLP-16 하드웨어는 [제조사 매뉴얼 §8.3](https://data.ouster.io/downloads/velodyne/user-manual/vlp-16-user-manual-revf.pdf)상
300–1200 RPM (5–20 Hz)을 지원하지만, 이 대회 빌드의 선택 범위와 구분한다.
