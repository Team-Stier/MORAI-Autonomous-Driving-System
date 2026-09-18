# 2026-09-18 MORAI 실행 및 네트워크 점검 메모

## 최종 구성

`run.sh`는 `src/MORAI-UDP-Bridge-Node/config/config.yaml`을 먼저 읽은 뒤
`config/bridge.yaml`로 덮어쓴다. 실제 장비별 IP와 NIC는 반드시
`config/bridge.yaml`에 적는다.

```yaml
network_launch:
  launch_option: 2
  remote_pc:
    bridge_ip: 192.168.0.10
    simulator_ip: 192.168.0.1
  open_ufw_for_remote: true
```

MORAI Windows PC의 센서 송신 목적지는 `192.168.0.10`이다. 주요 포트는 다음과
같다.

- Front/Left/Right camera: `9291`, `9293`, `9295`
- GPS: `9301`
- IMU: `9999`
- Competition status: `9099`
- Control command(MORAI 수신): `9093`

`909`, `999`처럼 1024 미만 포트를 쓰면 일반 사용자 프로세스에서
`bind: Permission denied`가 발생한다.

## 이번 점검에서 해결한 문제

- `yaml` import 실패: Homebrew Python이 먼저 선택되고 있었다. `run.sh`에서
  `/usr/bin`을 우선해 Ubuntu의 `python3-yaml`을 사용하도록 수정했다.
- NIC 자동 감지 실패: `enp5s0`, `eth0` 두 후보가 잡혔다. carrier 개수 대신
  `simulator_ip`의 커널 route에 선택된 물리 유선 NIC를 우선하도록 수정했다.
- 백그라운드 sudo 실패: NIC 설정을 터미널이 붙어 있는 포그라운드에서 먼저
  수행한 뒤 ROS bridge를 분리 실행하도록 수정했다.
- 잘못된 상대 IP: package 설정의 `.1`이 workspace override의 `.8`로 덮여
  있었다. 최종 `simulator_ip`는 `192.168.0.1`이다.
- 센서 수신 IP: 실제 Ubuntu 유선 주소 `.10` 대신 추가 주소 `.20`에 bridge가
  바인딩돼 있었다. 최종 `bridge_ip`는 `192.168.0.10`이다.
- RViz 창: `~/.local/bin/rviz`가 `DISPLAY=:2` VNC 화면으로 강제한다. 로컬
  모니터에는 다음처럼 실행한다.

  ```bash
  DISPLAY=:0 /opt/ros/noetic/bin/rviz
  ```

- 안전 브레이크: `/sensors/imu/data`가 0 Hz이면 localization과 local path가
  생성되지 않는다. 컨트롤러 상태가
  `WAITING_FOR_SYNCHRONIZED_PATH_ODOMETRY`가 되며 `brake=0.5`를 발행한다.
  MORAI IMU 목적지는 `192.168.0.10:9999`여야 한다.

## 유선 NIC 후보가 여러 개일 때

브리지는 먼저 `ip route get <simulator_ip>` 결과의 `dev`를 사용한다. 해당
장치가 물리 유선 NIC가 아니거나 route 조회가 실패할 때만 carrier가 연결된
유선 NIC 목록으로 fallback한다.

```bash
ip -brief link
ip -brief address
ip route get 192.168.0.1
ip neigh show 192.168.0.1
ping -I enp5s0 192.168.0.1
```

판단 기준:

1. `LOWER_UP`이고 carrier가 있는 물리 NIC를 찾는다.
2. `ip route get` 결과의 `dev`가 그 NIC인지 확인한다.
3. 해당 NIC를 지정한 ping이 성공하는지 확인한다.
4. 실제 NIC의 기존 IPv4를 `bridge_ip`로 사용한다.
5. 자동 선택이 틀린 예외 상황에서만 `interface`를 명시하고 재시작한다.

`eth0`라는 이름만 보고 선택하면 안 된다. USB NIC, Docker/VM 장치 또는 다른
유선망일 수 있다. 정상적인 route가 있으면 이름이 달라도 자동으로 올바른
장치를 고른다. 여러 NIC에 같은 대역 route가 잡힌 예외 상황에서만 아래처럼
강제한다.

```yaml
remote_pc:
  interface: enp5s0
```

## 빠른 런타임 확인

```bash
ping -I enp5s0 192.168.0.1
rostopic hz /sensors/gps/fix
rostopic hz /sensors/imu/data
rostopic hz /vehicle/competition_status
rostopic hz /image/front/compressed
rostopic echo -n 1 /morai_udp_bridge/control_cmd
```

정상 순서는 네트워크 ping, 센서 토픽, localization, local path, 제어 명령이다.
앞 단계가 0 Hz이면 뒤 단계의 안전 브레이크를 먼저 해제하려 하지 말고 끊긴
입력을 고친다.

## 기록

- 커밋 메시지: `9-18 검증됨`
- 상위 저장소 커밋: `a149aa2`
- UDP Bridge 서브모듈 커밋: `9ae86da`
- 전체 스냅샷:
  `/home/stier/MORAI-Autonomous-Driving-System-snapshot-20260918-111219.tar.zst`
- 스냅샷 체크섬:
  `/home/stier/MORAI-Autonomous-Driving-System-snapshot-20260918-111219.tar.zst.sha256`
