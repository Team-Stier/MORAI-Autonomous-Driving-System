# morai_mpc

`control::MoraiMpc`는 MOLIT2026-stier1의 횡방향 Autoware AI MPC, 종방향 MPC,
조향 응답 예측을 기존 Control Pipeline의 native 입출력에 맞춰 이식한 Core다.
속도 계획은 포함하지 않는다. 현재는 계산 코드·설정·테스트를 추가한 상태이며,
Core 등록과 실행 선택은 적용하지 않았다.

프로젝트의 [Core 작성 규약](../../../../README.md)과
[Core 등록 가이드](../../../node_pipeline/CORE_REGISTRATION.md)를 따른다.

## 구성과 이식 출처

원본은 [MOLIT2026-stier1](https://github.com/Team-Stier/MOLIT2026-stier1)의
`d71cadc7f1bb040ae026f95d0a19292d16ca8978`이며, 이식 기준 main은
`3c92472920e077797e85fc05ff8b3f2b4e0276df`다.

| 구성 | 이식 내용 | 원본 위치 |
| --- | --- | --- |
| 횡방향 | Autoware AI `kinematics` 모델 + `qpoases_hotstart` | `src/vendor/autoware_ai/mpc_follower` |
| 종방향 | 원본 MPC + 실제 OSQP 계산 구현 | `src/morai_path_tracking/src/controllers/longitudinal/mpc`, 같은 패키지의 `src/common/osqp_sqp_solver.cpp` |
| 조향 예측 | 명령 이력에 대한 지연 + 1차 응답 | `src/morai_path_tracking/scripts/control/steering_prediction.py` |

원본에 적용된 조향 모델의 정확한 이산화, 조향 gain, 곡률별 가중치,
첫 조향 명령 변화 비용, 조향 변화율 제약과 qpOASES hotstart 상태를 유지한다.
원본의 별도 custom 횡방향 MPC, 속도 계획·목표속도 평활화, ROS 통신·디버그
발행, 실험용 시간 정렬과 입력 검사·fallback은 이식 대상에서 제외한다.

## 파일 배치

아래 경로는 `src/control/` 기준이며, 표시한 `morai_mpc` 파일·폴더는 모두 이번
이식으로 추가했다. 기존 파일의 변경은 의존성·빌드·테스트 연결을 위한
`CMakeLists.txt`와 `package.xml`에 한정한다.

```text
include/control/
├── cores/morai_mpc.hpp                 # native Core와 지속 계산 상태
└── tracking/morai_mpc/
    ├── autoware/                      # Autoware 계산 보조 헤더
    ├── vendor/{amathutils,qpoases}/    # 외부 수학 함수·솔버 헤더
    ├── parameters.hpp                 # 설정 타입과 기본값
    ├── ros_parameters.hpp             # Core 외부의 설정 로더
    └── *.hpp                          # 횡·종방향 MPC와 조향 예측 API
src/controllers/morai_mpc/
├── autoware/                          # Autoware 계산 보조 구현
├── vendor/{amathutils,qpoases}/        # 외부 수학 함수·솔버 구현
└── *.cpp                              # 횡·종방향 MPC와 조향 예측 구현
config/morai_mpc.yaml                  # 전용 파라미터
test/unit/test_morai_mpc.cpp            # Core 회귀 테스트
test/data/morai_mpc_reference.csv       # 원본 계산 결과
docs/morai_mpc/                        # 이 문서와 라이선스
```

`vendor/`는 `morai_mpc` 내부에서 `autoware/`와 같은 깊이에 둔다.
별도 vendor catkin 패키지는 만들지 않는다. 계산 라이브러리는
`control_morai_mpc`, qpOASES 공유 라이브러리는 `control_morai_mpc_qpoases`다.
OSQP는 원본과 같은 0.6 API를 사용하며 ROS Noetic의 `osqp_vendor`에 의존한다.
추가 ROS 의존성은 `tf`, `tf2`, `tf2_geometry_msgs`이고 Eigen3가 필요하다.

`autoware/lane.hpp`는 계산에 필요한 pose·twist만 가진 내부 자료구조다.
`autoware_msgs` 패키지나 새로운 ROS 메시지·토픽은 추가하지 않는다.
빌드 산출물은 catkin의 `build/`, `devel/`, `install/`에 두며 소스에 포함하지 않는다.

## 입출력과 계산 방식

[Core](../../include/control/cores/morai_mpc.hpp)의 공개 연결은
`process(path, ego, competition) → morai_msgs::CtrlCmd`다.

| 인자·출력 | 타입 | 사용 방식 |
| --- | --- | --- |
| `path` | `interfaces::Path` | 뒷차축 중심 `base_link` 경로. 원점에 가장 가까운 점의 `velocity`가 종방향 목표속도 |
| `ego` | `nav_msgs::Odometry` | `twist.twist.linear.x`를 횡방향 모델의 현재 속도로 사용. global pose를 경로에 다시 적용하지 않음 |
| `competition` | `interfaces::CompetitionVehicleStatus` | `velocity_x_mps`가 종방향 현재 속도 |
| 반환값 | `morai_msgs::CtrlCmd` | `longlCmdType=1`, 페달 `[0,1]`, 조향각을 Bridge의 고정 40°로 나눈 조향값 |

종방향 MPC는 가까운 점에서 고른 **하나의 목표속도**를 예측 구간 전체에 사용한다.
경로 전체의 미래 속도·가속도 프로파일을 예측 단계별 목표로 넣지 않는다.
횡방향 참조 경로의 속도는 원본 adapter와 같이 모든 점에
`max(0.5, abs(ego.twist.twist.linear.x))`를 넣는다.

조향 예측기는 Core가 최종 제한한 조향 명령을 보관하며 delay·time constant·gain을
반영한다. 그 이후 하위 노드가 바꾼 명령은 현재 입력 계약으로 관측하지 못한다.
Core는 입력 검사·누락 감지·발행을 수행하지 않으며 수치 계산 실패는 예외로 전달한다.
입력 정책과 실행·발행은 Pipeline이 담당한다.

## 파라미터와 시간 기준

설정은 [config/morai_mpc.yaml](../../config/morai_mpc.yaml)의 `morai_mpc` namespace에
독립적으로 둔다. 횡방향은 `lateral`, 종방향은 `longitudinal`, 조향 지연·응답은
`steering_prediction`에 둔다. 조향 모델과 예측기는 같은 delay·time constant·gain을 쓴다.

[loadParameters](../../include/control/tracking/morai_mpc/ros_parameters.hpp)는 해당
namespace의 `ros::NodeHandle`을 받아 모든 필수 설정을 읽고 검증한다.
생성된 `Parameters`를 `MoraiMpc` 생성자에 주입한다. Core 자체는 로더를 사용하지 않는다.

| 항목 | 이식 기본값·기준 |
| --- | --- |
| 횡방향 튜닝 | 원본 `config/controllers/autoware_mpc.yaml` |
| 횡방향 모델 주기 | `1/30 s` |
| 횡방향 예측 | 기본 45단계, 단계 간격 `0.1 s`; 최소 예측 길이 20 m에 따라 최대 90단계 |
| 경로 재샘플링 | 거리 간격 `0.25 m` |
| 종방향 튜닝 | 원본 `molit_2026_path_tracking.yaml` + `autoware_mpc.launch` 운영 override |
| 종방향 제한 | 최대속도·강제 제동 진입 60 km/h, accel 최대 0.70, brake 최대 0.60, 명령 변화율 2.0/s |
| 조향 제한 | 최대 40°, 최종 명령 변화율 60°/s |
| 조향 응답 | delay `1/6 s`, time constant `0.05 s`, gain `0.77` |

종방향 적분과 최종 조향 변화율 제한은 경로 stamp 차이를 `0.005–0.10 s`로 제한해
사용하며 첫 호출은 횡방향 모델 주기를 사용한다. 조향 예측의 시간 기준도 경로 stamp다.
횡방향 모델 주기는 별도로 고정되어 있으며 설정값 변경이 Pipeline 실행 주기를 바꾸지는 않는다.

**예측 horizon 45는 45 m가 아니라 45단계다.** 입력 경로가 45 m여도 속도에 따라
시간 예측 구간의 공간 길이와 다를 수 있다. 경로 길이·입력 주기·지연과 무관하게
같은 추종 성능을 보장하지 않는다.

## 짧은 경로 처리와 방향각 수정

거리 재샘플링 전에 같은 XY 좌표가 연속되는 점은 중복 제거한다. 경로가 재샘플링
간격보다 짧으면 실제 마지막 점을 유지한다. 곡률 계산은 확보한 실제 점 수에 맞게
윈도 크기를 줄여 원본의 unsigned 인덱스 underflow를 막고, 두 점이면 곡률을 0으로 둔다.
원본의 정지 끝점 연장은 실제 적응형 예측 horizon까지 시간을 확보하는 방식으로 유지한다.
추가 공간 경로를 생성하지 않는다.

### 재샘플링 결과가 두 점일 때의 방향각

`enable_yaw_recalculation=true`일 때 방향각은 경로의 XY 형상으로 다시 계산한다.
기존 함수는 앞뒤 점이 있는 중간 점만 계산하고 양 끝점에는 인접 점의 방향각을 복사했다.
점이 정확히 두 개이면 중간 점 계산이 실행되지 않아 입력 pose의 방향각이 남았다.

재현 입력은 `(0,0) → (0.1,0)`의 직선에 두 pose의 방향각을 모두 90°로 준 경우다.
형상에 따른 방향각은 0°인데 기존 계산은 90°를 유지했고, 차량과의 방향 차이를
60° 미만으로 제한하는 최근접 경로 검색에서 모든 후보가 제외되어 계산이 실패했다.

[calcTrajectoryYawFromXY](../../src/controllers/morai_mpc/autoware/mpc_utils.cpp)는
두 점일 때 다음 방향을 구해 두 점에 동일하게 넣는다.

```cpp
const double yaw = std::atan2(y1 - y0, x1 - x0);
```

3점 이상은 기존 윈도 기반 계산을 유지한다. 재계산을 끈 경우에는 이 분기를 사용하지 않는다.
이는 유효 입력의 경로 형상에 대한 수치 계산이며, 입력 대체나 Pipeline 정책 변경이 아니다.
`MoraiMpc.RecalculatesHeadingForTwoRealSamples` 테스트로 해당 실패를 재현한 뒤 수정 후
직선 조향이 0에 가까운지 확인했다. 이 수정이 짧은 경로의 충분한 예측 거리나 주행 성능을
보장하는 것은 아니다.

## 빌드·검증과 실행 적용 상태

ROS Noetic 환경에서 필요한 의존성을 준비한 뒤 워크스페이스 루트에서 실행한다.

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -j2
catkin_make run_tests_control -j2
catkin_test_results build/test_results/control --verbose
```

2026-10-05에 `control`과 직접 의존 패키지를 구성한 격리 Noetic 환경에서 확인한 결과다.

- 라이브러리·기존 control 노드 빌드 및 install 성공. 설치 결과를 사용하는 별도 프로그램의
  컴파일·실행과 YAML 파라미터 로딩 확인.
- [신규 테스트](../../test/unit/test_morai_mpc.cpp) 11개 통과. native 입출력, 상태 유지,
  단일 목표속도 선택, 조향·페달 제약, 45 m 전방 경로, 짧은 경로와 조향 지연 응답 검사.
- [원본 계산 fixture](../../test/data/morai_mpc_reference.csv) 252개 결과 비교 통과.
  원본 소스를 별도 실행 파일로 빌드해 기록한 값이며 이식 코드로 생성한 기대값이 아니다.
- 검사한 전체 181개 테스트 중 기존 실패·오류 19건은 이식 전과 같고 추가 실패는 없음.
  실제 폐루프 주행 성능은 미검증.

원본 비교 fixture의 조건은 다음과 같다. 각 조합마다 제어기 상태를 새로 시작한다.

| 구분 | 비교 조건 |
| --- | --- |
| 횡방향 216개 | 속도 0/5/16 m/s × 곡률 0/+0.015/−0.015 m⁻¹ × offset −0.2/+0.2 m × 연속 12회 |
| 횡방향 경로·명령 | 호 길이 `s=-4…45 m`, 간격 0.5 m. `x=sin(s*k)/k`, `y=offset+(1-cos(s*k))/k`, yaw=`s*k`; 곡률 0이면 직선. 참조속도 `max(0.5,speed)`, 모델 조향 `0.01*sin(step*0.3)`, 이전 조향은 원본의 직전 출력 |
| 종방향 36개 | 목표속도 0/8/16 m/s × 연속 12회. 측정속도 `5+0.03*step`, dt=`1/30 s`, 위 운영 override 적용 |

원본 비교용 생성 프로그램은 개발용 도구이며 런타임 의존성이나 설치 노드에 포함하지 않는다.

현재 `core_set.hpp`, `core_dispatch.hpp`, `ros_node.cpp`, `control.yaml`, launch는
변경하지 않았다. **새 YAML만 로드해도 이 Core로 전환되지 않는다.** 실행 등록 시에는
기존 Path/Odometry/Competition 입력과 파라미터 생성자 주입을 연결해야 한다.

등록 전제에는 `ego.twist.twist.linear.x`의 유효성 검사도 포함된다.
현재 [Odometry Validator](../../include/control/components.hpp)는 `linear.y`와
`angular.z`를 검사하고 `linear.x`는 검사하지 않는다. 이식 단계에서는 Validator를
변경하지 않았으며, 필요한 입력 검사와 등록 전제를 갖추기 전까지 이 Core를 활성화하지 않는다.

## 출처 표기와 라이선스

원본 저작권 헤더를 유지한다. Autoware 계산 보조 코드는
`control::morai_mpc::autoware`, amathutils는 `control::morai_mpc::amathutils`에 둔다.
qpOASES의 변경은 include 경로에 한정하며 공유 라이브러리로 빌드한다.

| 구성 요소 | 원본 저장소가 기록한 upstream revision | 라이선스 |
| --- | --- | --- |
| Autoware AI mpc_follower | `48c67999823d66d297f9fd01dda5584604282135` | Apache-2.0 |
| amathutils_lib | `202e492b88a89b58ca83f66cec089eaa27b440d4` | Apache-2.0 |
| qpOASES | `51d3fbea30142d3acbf40cf7a1c519efc27ea67b` | LGPL-2.1-or-later |

프로젝트 소유 종방향·조향 예측 코드의 MIT 선언은 원본 패키지를 따른다.
[Apache 전문](LICENSE-Apache-2.0.txt)과 qpOASES의
[라이선스 안내](licenses/qpOASES/LICENSE)·[LGPL 전문](licenses/qpOASES/LICENSE.txt)을 보존한다.
미사용 수학 필터, qpOASES 예제·언어 binding·benchmark 자료는 포함하지 않는다.
