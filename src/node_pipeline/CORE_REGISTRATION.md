# Core 등록 가이드

[프로젝트 규칙](../../README.md), [Pipeline 계약](README.md), 대상 패키지 README와 실제 메시지 정의를 따른다.
리드가 `node.yaml`로 제공한 **기존 Stage**에 Core를 등록한다. 첫 번째부터 n번째 Stage까지 같은 절차다.
Core 개발자는 계산과 아래 경로의 파일을 담당하고, 리드는 실행 방식·입력 계약·Stage 구성·출력 계약·발행과 빌드 구성을 제공한다.

**Core 개발자는 아래 경로의 파일만 작성·수정한다.** Pipeline C++·생성 파일·리드 소유 파일은 수정하지 않는다.
경로 안에서도 자신의 Core에 필요한 변경만 적용하고, 기존 객체·설정·사용자 변경은 보존한다.

| 경로 | 내용 |
| --- | --- |
| `include/<package>/cores/<core>.hpp` | Core 클래스·파라미터 구조체 |
| `include/<package>/validators/<name>.hpp` | 도메인 고유 입력 검사 |
| `include/<package>/policies/<name>.hpp` | 입력 보정 |
| `include/<package>/parameters/<core 또는 bank>.hpp` | 파라미터 로더 |
| `include/<package>/banks/<name>.hpp` | 같은 노드의 Core가 공유하는 계산 상태 |
| `include/<package>/libraries/`, `src/libraries/` | 계산 라이브러리 |
| `package.xml` | 의존성, 외부 include root(`<export><node_pipeline include_roots="…"/></export>`) |
| `CMakeLists.txt` | `find_package()`와 `np_add_nodes(INCLUDE_DIRS … LIBRARIES …)` |
| `models/` | 파라미터 로더가 `asset()`으로 읽는 정적 자산 |
| `config/<node>/registrations/<core>.yaml` | Core 등록 |
| `config/<node>/parameters/<core 또는 bank>.yaml` | Core·Bank 계수 |
| `config/<node>/selection.yaml` | 해당 Core의 선택 매핑 |
| `test/unit/test_<core>.cpp` | Core 테스트 |

다른 Core가 함께 연결한 Validator·Policy·Bank·라이브러리를 변경해야 하면 해당 담당자·리드와 먼저 협의한다.

## 시작 전 확인

루트 README·AGENTS.md·Pipeline README·대상 README·메시지 정의와 기존 변경을 읽고 다음을 확인한다.

- 대상 노드의 `node.yaml`에 등록할 Stage가 있다.
- Core에 필요한 입력이 해당 Stage의 입력에 있다. 첫 Stage의 입력은 `inputs`, 이후 Stage의 입력은 이전 Stage의 `outputs`다.
- `process()`의 반환이 해당 Stage `outputs`의 개수·타입·순서와 일치한다. 마지막 Stage는 발행 메시지 하나다.
- Core 설정·정적 자산은 파라미터 로더로 주입할 수 있다.
- Core가 의존하는 입력 내용 전제(특정 transform의 존재, 필드 구성 등)를 `node.yaml` 입력 한도나 연결할 Validator로 선언할 수 있다.

전제가 부족하거나 문서·코드가 충돌하면 아래 보고 절차를 먼저 따른다.
시작할 때 대상 Core·Stage·입출력 계약·작성할 파일을 사용자에게 짧게 안내한다.

## 등록

1. `cores/<core>.hpp`에 Core를 작성한다. 클래스 이름은 파일 이름의 PascalCase이며, 오버로드 없는 `process()` 하나를 둔다.
2. `registrations/<core>.yaml`에 Stage와 `process()` 인자 순서의 입력 이름을 선언한다. 선언한 입력은 모두 필수 입력이다.
3. 필요할 때만 Validator·Policy·Bank를 연결하고 허용 issue를 선언한다. 시각화 발행은 Core가 `node_pipeline::Visualizer<T>` 멤버로 직접 하며 등록하지 않는다.
4. 계수가 있으면 `parameters/<core>.hpp` 로더와 `config/<node>/parameters/<core>.yaml` 값을 함께 둔다.
5. `selection.yaml`의 해당 Stage에서 실제 적용할 section 행만 추가·교체한다. health 조건으로 선택하는 경우에만 health 규칙을 추가·교체한다. 다른 매핑은 보존한다.
6. `test/unit/test_<core>.cpp`를 작성하고 재빌드한다.

```cpp
// include/localization/cores/full_ekf.hpp
#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
namespace localization {
struct FullEkfParameters {
  double acceleration_noise = 1.0;
};
class FullEkf {
 public:
  explicit FullEkf(FullEkfParameters parameters) : parameters_(parameters) {}
  nav_msgs::Odometry process(const sensor_msgs::NavSatFix& gps, const sensor_msgs::Imu& imu,
                             const interfaces::CompetitionVehicleStatus& status);
 private:
  FullEkfParameters parameters_;
};
}  // namespace localization

// include/localization/parameters/full_ekf.hpp
#pragma once
#include <localization/cores/full_ekf.hpp>
#include <node_pipeline/extension.hpp>
#include <cmath>
#include <stdexcept>
namespace localization {
inline FullEkfParameters loadFullEkfParameters(const node_pipeline::ParamReader& params) {
  FullEkfParameters parameters;
  parameters.acceleration_noise = params.number("acceleration_noise");
  if (!std::isfinite(parameters.acceleration_noise) || parameters.acceleration_noise <= 0)
    throw std::invalid_argument("invalid acceleration_noise");
  return parameters;
}
}  // namespace localization
```

```yaml
# config/localization/registrations/full_ekf.yaml
stage: estimate
inputs: [gps, imu, competition]
policy: remove_invalid.hpp

# config/localization/parameters/full_ekf.yaml
acceleration_noise: 0.03

# config/localization/selection.yaml
estimate:
  section_core_map:
    "0": full_ekf.hpp
  health_core_map: []
```

Core 생성자는 연결한 것만 파라미터 값, Bank 참조 순서로 받는다.
등록 YAML·확장점 계약의 상세는 [Pipeline 계약](README.md)을 따른다.

## 해제

등록 YAML을 삭제하고 `selection.yaml`에서 그 Core를 가리키는 항목을 제거한 뒤 재빌드한다.
더 이상 쓰지 않는 Core·로더·값 파일과 그 Core만 연결한 확장점 파일을 삭제한다.

## Core 검증

새 Core의 native 입력 fixture와 계산·상태·입출력 계약을 검사하는 최소 회귀 테스트를 작성한다. 빈 테스트나 등록 코드를 복사한 테스트로 대신하지 않는다.
테스트 본문은 해당 Core의 fixture·호출·기대값 검사로 한정한다. `test/unit/test_*.cpp`·`test_*.py`는 `np_add_nodes()`가 자동 등록한다.

## 허용 범위와 보고

다음은 Core 개발자의 범위 밖이며 필요하다고 판단해도 먼저 적용하지 않는다.

- 공통 Pipeline 라이브러리·생성기·계약 검사와 생성 파일.
- `node.yaml`의 실행 방식·입력·한도·Stage 수·순서·출력 계약·발행.
- `launch.sh`.
- README·AGENTS.md·메시지 정의·서브모듈.
- 다른 Core의 Core·등록·파라미터 파일.

계약 검사를 우회하지 않는다. Core 알고리즘을 Validator·Policy·파라미터 로더·Bank·Selector로 옮겨 우회하지 않는다.

**허용 범위 밖 변경이 필요하면 해당 변경을 중단하고 리드와 상의하며 반드시 명시적 허가를 요구한다.**
입력 전달·타입·검사·선택·헤더·의존성·빌드 등 모든 범위 밖 변경에 같은 절차를 적용한다.
보고에는 다음을 포함한다.

1. 무엇이 불가능한지와 확인한 파일·위치·코드·실제 전달 경로.
2. 제한 또는 충돌 근거인 문서 링크·위치와 실제 지침 문구. 문서·코드 충돌이면 양쪽을 제시하고 높은 권한의 문서에 맞출 해결을 제안한다.
3. 필요한 파일별 diff·사유·직접 필요한 연쇄 변경과 허용 경로만으로 해결할 수 없는 이유.
4. **허용 범위 안에서 개발 가능한 방향과 한계**, 적용·검증한 결과와 미적용 변경.
5. 위 변경에 한정한 리드 협의·명시적 허가 요청.

예: “SectionAnnotation을 사용하려는데 대상 Stage의 입력에 section이 없어 개별 Core 인자로 전달할 수 없다”를 실제 `node.yaml`로 보고한다.
필요한 `node.yaml` 변경과 제한 문구를 제시하고, 범위 안 대안으로 “Core·등록을 작성하고 native 메시지를 직접 주는 계산 테스트까지 진행”을 제안한다.
구독 없음·Stage 입력 없음·실행 중 수신 없음·health/policy 후 Missing을 구분한다.

사용자 결정 전에는 해당 변경과 이에 의존하는 등록을 보류한다. 독립적인 범위 안 작업은 계속할 수 있다.
Core 개발 요청만으로 범위 밖 허가를 추정하지 않는다. 이미 명시적으로 승인된 같은 변경은 다시 묻지 않으며 질문·이의 제기·무응답은 승인으로 해석하지 않는다.
다른 대화의 승인은 현재 작업의 권한으로 가져오지 않는다. 자동 승인 검토가 거절하면 거절된 동작과 이유를 별도로 보고한다.

완료할 때 자신의 변경을 이 경로 범위·적용 문서와 대조하고 대상 Core 테스트와 계약 검사·노드 컴파일·링크를 확인한다.
Core 계산 완료·등록 완료·실제 발행 확인을 구분하고 선택 조건·검증 결과·미확인 항목을 짧게 보고한다. 실행 환경 부족은 사유와 함께 미확인으로 기록한다.
검증을 이유로 다른 노드·Pipeline·환경을 임의 변경하거나 전체 스택을 시작하지 않는다. 통과한 검증은 새 변경·실패가 없으면 반복하지 않는다.
범위 밖 변경을 잘못 적용했다면 변경·영향·현재 상태를 보고하고 자신의 변경을 바로잡는다. 사용자 변경과 겹치는 수정은 먼저 협의한다.
