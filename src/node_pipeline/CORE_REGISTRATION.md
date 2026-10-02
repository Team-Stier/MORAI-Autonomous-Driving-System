# Core 등록 가이드

[프로젝트 작성 규칙](../../README.md), [Pipeline 계약](README.md), 해당 패키지의 README와 실제 메시지 정의를 따른다.
Core 작성·추가·교체·등록 작업에 사용한다. Pipeline 기능 개발이나 노드 입력 계약 변경은 별도 작업이다.
참고 사례는 [path_planner Core 구현](codex://threads/01a0fd81-ba2c-7721-ac33-118a45193317)이다. 다른 대화의 승인·구현 상태를 현재 작업의 권한으로 가져오지 않는다.

목표는 기존 노드 입력으로 계산 가능한 Core를 작성하고, 등록·설정·선택·빌드까지 연결하는 것이다.
입력 전달 등 등록 전제가 갖춰져 있지 않으면, 그 사실과 해결에 필요한 허가를 정확히 보고한다.
예시 diff는 변경 종류를 정의하며 계산 구현을 대신하지 않는다. 실제 등록에는 완성한 Core를 사용한다.
경로는 저장소 루트 기준이다. 각 작업에 해당하는 diff만 적용한다.

## 등록 형태 선택

대상 노드가 이미 제공하는 실행 구조를 먼저 확인한다.

| 기존 노드 구조 | 적용할 등록 diff |
| --- | --- |
| 단일 Core 선택·호출 | 1·2절의 Core·설정, 3~5절의 등록·이름·YAML |
| 두 단계 Core 선택·순차 호출 | 1·2절의 Core·설정, 8절의 단계별 등록·이름·YAML |
| 공통 | 필요한 경우 6·7절의 의존성·Core 전용 테스트 |

[단일 Core 템플릿](templates/)과 [두 Core 순차 템플릿](templates_two_core/)은 가이드 전용이며 실행·빌드 대상이 아니다.
diff는 대상 실행 패키지의 파일에 적용한다. 템플릿 파일을 등록 작업으로 수정하지 않는다.
단일 단계 노드를 두 단계로 바꾸는 작업은 등록 허용 범위에 포함하지 않는다.
두 단계 Selector·설정 namespace·호출 경로가 없으면 필요한 구조 변경을 먼저 보고하고 명시적 허가를 요청한다.

## 수정 범위

Core 파일 외 수정은 아래 diff에 제시된 종류의 변경만 허용한다.
이 가이드에 안내되지 않은 변경은 지양한다.
패키지명·Core명·설정 항목·입력 인자·의존성 이름은 개발 대상에 맞게 적용한다.
표는 대상 파일과 변경 내용을 요약한 것이며, 파일 전체의 수정 권한을 부여하지 않는다.
허용 범위 밖의 변경이 필요하다고 판단하면 해당 변경을 중단하고 리드와 상의하며, 변경할 파일·내용·필요성을 설명해 반드시 명시적 허가를 요청한다.
등록에 필요하다는 판단만으로 허용 범위를 확대하지 않는다. 기존 파일의 버그·계약 불일치도 자동 수정하지 않는다.
이 중단·보고·허가 절차는 모든 허용 범위 밖 변경에 적용한다. 입력 전달은 그중 한 사례일 뿐이다.
계산 구현·설정·등록·검증 중 어느 단계에서 발견했든, 변경의 크기나 필요성과 무관하게 같은 절차를 따른다.
허가 요청 시 변경할 파일·내용·필요성과 함께 허용 범위 안에서 개발 가능한 방향과 그 한계를 제안한다.
명시적 허가 전에는 범위 밖의 변경을 수행하지 않는다.

| 파일 | 변경 내용 | 담당 |
| --- | --- | --- |
| `src/localization/include/localization/cores/gps_only.hpp` | 새 Core 클래스·계산·내부 상태 | Core 개발자 |
| `src/localization/include/localization/types.hpp` | 새 Core 설정 구조체 | Core 개발자 |
| `src/localization/include/localization/config.hpp` | 해당 설정 로딩·검증 함수 | Core 개발자 |
| `src/localization/include/localization/core_set.hpp` | include·단일 또는 단계별 ID·지속 객체·필수 구독 입력·호출 연결; 두 단계는 중간 반환값의 개별 인자 연결까지 | Core 개발자 |
| `src/localization/src/localization_node.cpp` | 단일 또는 단계별 이름→ID 등록·생성자 설정 주입 | Core 개발자 |
| `src/localization/config/localization.yaml` | Core 계수·단일 또는 기존 단계별 namespace의 선택 매핑 | Core 개발자 |
| `src/localization/CMakeLists.txt` | Core에 필요한 의존성 검색·export·헤더 경로·링크와 Core 전용 테스트 target 추가 | Core 개발자 |
| `src/localization/package.xml` | 해당 의존성 선언 추가 | Core 개발자 |
| `src/localization/test/unit/test_gps_only.cpp` | 해당 Core의 계산·계약 회귀 테스트 | Core 개발자 |

Core 등록만으로 공통 Pipeline, Validator, Selector, callback, 토픽, HeaderInjector, launch를 수정하지 않는다.
Core에 새 의존성이 필요하면 6절의 diff 범위에서 해당 패키지의 `CMakeLists.txt`와 `package.xml`에 추가한다.
설정이 없는 Core는 설정 구조체·로더·계수 추가를 생략한다.
각 diff의 추가 코드에 직접 필요한 include도 허용한다. 기존 include·의존성은 중복 추가하지 않는다.
`types.hpp`의 설정 구조체 추가 허가는 `Data` 타입 pack·입출력 메시지 타입 변경을 허용하지 않는다.
YAML은 해당 Core의 계수와 선택 매핑만 변경한다. 실행 주기·trigger·입력 한도·policy 설정은 별도 허가 대상이다.
README·AGENTS.md·메시지 정의·서브모듈 변경은 등록 허용 범위에 포함하지 않는다.
Core 알고리즘을 Policy·Validator·Selector·callback·설정 로더 등에 옮겨 우회하지 않는다.
표의 localization 경로는 단일 Core 예시다. 두 단계 노드는 8절의 경로·diff로 같은 담당 범위를 적용한다.
단계 추가·삭제, Selector 결합 조건 변경, 중간 변수의 검사·보정·헤더 주입·발행 경로 추가는 별도 허가 대상이다.

## 작업 흐름과 사용자 안내

### 시작: 계약과 변경 범위 확인

루트 README·AGENTS.md·Pipeline README·대상 패키지 README·실제 메시지 정의를 읽는다.
대상 패키지의 기존 변경을 확인해 작업 전 상태를 구분하고, 사용자 변경을 보존한다.
문서·코드가 충돌하면 각 출처의 링크·위치와 실제 지침 문구 또는 코드를 함께 인용하고, 어느 내용이 충돌하는지 설명한다.
규칙이 있다는 말이나 링크만으로 설명을 대신하지 않는다. 높은 권한의 문서에 맞출 것을 제안한다.
해당 변경은 사용자 결정 전까지 멈추며, 충돌과 독립적인 작업만 계속한다.

시작 안내는 대상 Core·사용할 메시지·등록 범위를 짧게 말한다. 두 단계는 각 Core의 단계와 중간 입출력 계약도 말한다.
허용된 등록 변경마다 다시 허가를 묻지 않는다.

> `path_planner`의 Core와 설정을 작성하고, 기존 입력으로 전달 가능한지 확인한 뒤 CoreSet·이름 등록·YAML·의존성을 연결하겠습니다.

### 편집 전: 필요한 메시지가 실제로 Core까지 전달되는지 확인

단일 Core 또는 두 단계의 첫 Core가 요구하는 `process()` 인자마다 아래 경로를 확인한다.
구독 목록이나 include만으로 전달 가능하다고 판단하지 않는다.

| 확인 위치 | 확인할 내용 |
| --- | --- |
| `nh.subscribe()`와 callback | 실제 메시지 타입, 연결한 callback, `node.onInput()` 호출 여부 |
| 입력 ID와 `Buffer` 타입 pack | callback의 입력 ID가 메시지 타입·tuple 순서와 대응하는지 |
| `Data<...>` | 실제 타입·인덱스, `data.get<I>().message()`로 해당 메시지를 얻을 수 있는지 |
| `requiredInputs()`·`CoreSet::process()` | 선언한 필수 인덱스와 전달 인자가 일치하는지 |
| 설정·정적 자산 주입 | Core 생성자가 요구하는 값이 기존 설정·자산 공급 경로에서 준비되는지 |
| 기존 HeaderInjector | 출력 타입·stamp·frame 계약을 이미 충족하는지 |

구독 → callback → onInput → Buffer/Assembler → Data → CoreSet → process 인자까지 확인한다.
`Data`의 입력 개수가 N이면 허용되는 인덱스는 0부터 N−1이다. Core별 인자 타입·순서도 실제 선언과 대조한다.
선택된 첫 Core가 사용하는 모든 구독 입력 인덱스를 `requiredInputs()`에 포함한다. 단일 Core는 그 Core가 첫 단계이자 마지막 단계다.
두 단계에서는 첫 Core 반환값 → CoreSet의 값 해체 → 두 번째 Core의 개별 인자까지 이어서 확인한다.
중간 변수는 Data 입력이나 필수 구독 입력 인덱스에 추가하지 않는다.
기존 단계별 Selector·이름 등록·YAML namespace가 있는지, 두 단계가 같은 section·최종 구독 입력 health로 선택되는지도 확인한다.
등록할 첫 Core와 두 번째 Core의 모든 호출 조합에서 중간 변수의 개수·타입·순서가 맞아야 한다.
YAML에서 현재 선택하지 않는 조합이라는 이유로 컴파일되지 않는 호출 연결을 남기지 않는다.

`node.onSection()`은 Selector용 section 저장 경로다. 이 호출만으로 원본 `interfaces::SectionAnnotation`이 Core의 Data 입력에 추가되지 않는다.
이 메시지가 `process()` 인자로 필요하면 실제 `onInput()` 연결·Buffer·Data가 있는지 별도로 확인한다.

전제가 부족하면 등록을 적용하기 전에 차단 보고를 한다. 승인 대기 중인 입력을 가정해 인덱스·이름·YAML만 먼저 연결하지 않는다.
입력 외에도 의존성·정적 자산 공급·헤더·선택·발행 등의 전제가 범위 밖 변경을 요구하면 동일하게 보고한다.
독립적으로 가능한 Core·설정 작성과 계산 검증은 계속할 수 있다. 사용할 수 없는 Core의 활성 매핑은 적용하지 않는다.

### 구현과 등록

Core의 계산·내부 상태를 작성한 뒤 아래 diff로 설정·CoreSet·노드 생성자·YAML·필요한 의존성을 연결한다.
등록 파일에는 소유·인자 전달만 둔다. 선택이 바뀌어도 Core 객체를 재생성하거나 상태를 자동 reset하지 않는다.
지도·체크포인트 등 정적 자산은 기존에 합의된 공급 경로를 사용한다. 새 입출력 경로가 필요하면 별도 허가를 요청한다.

진행 안내에는 확인된 사실과 남은 연결을 말한다. 계산 구현이 끝났다는 이유로 등록이나 발행도 끝났다고 표현하지 않는다.

### 검증과 완료 판단

다음 검증은 별개다. 수행하지 못한 단계는 이유와 함께 미확인으로 보고한다.

| 단계 | 확인할 결과 |
| --- | --- |
| 변경 범위 | 이번에 추가한 모든 변경이 Core 구현 또는 아래 등록 diff의 변경 종류에 해당함 |
| 입력·등록·선택 | 실제 인덱스·타입, 단일 또는 단계별 Core 파일 이름→ID, 생성자, YAML에서 선택되는 Core가 모두 일치함 |
| 계산 | 요구사항을 검증하는 해당 Core의 최소 테스트가 통과함; 두 단계는 중간 반환 계약·인자 연결·최종 payload도 확인함 |
| 노드 빌드 | ROS Noetic 환경에서 해당 노드 target이 컴파일·링크됨 |
| ROS 발행 | 실행 확인을 수행한 경우 실제 선택·Core 호출·출력 토픽·payload·헤더를 관찰함 |

필수 입력이 Missing이거나 section·health 매핑에서 Core가 선택되지 않아 계산이 실행되지 않은 상태를 성공으로 기록하지 않는다.
health 매핑이 일치하지 않으면 기존 section 선택을 유지한다.
두 단계에서는 어느 단계든 선택되지 않거나 첫 Core의 필수 구독 입력이 Missing이면 두 Core 모두 호출하지 않고 헤더 주입·발행도 생략한다.
각 단계의 health 매핑은 최종 구독 입력 health에 적용한다. 중간 변수의 health를 새로 만들거나 검사하지 않는다.
중간 변수에는 헤더를 주입하거나 발행하지 않고, 마지막 Core의 결과에만 기존 HeaderInjector·IO를 적용한다.
이 동작이 기존 노드에서 보장되지 않으면 등록 코드로 우회하지 않고 범위 밖 변경으로 보고한다.
Core 계산 테스트 통과만으로 노드 빌드·실제 발행이 확인됐다고 말하지 않는다.
환경이 없으면 실행한 검증까지만 보고한다. 다른 노드·Pipeline 수정이나 실행 환경 설치로 임의 확장하지 않는다.
`run.sh`는 기존 ROS 프로세스를 종료할 수 있으므로 Core 등록 검증이라는 이유만으로 전체 스택을 시작하지 않는다.

코드를 변경했다면 해당 target 빌드와 관련 테스트를 우선한다. 이미 통과한 검증은 새 변경·실패가 있을 때만 반복한다.
실행 중인 노드에 새 Core가 반영됐다고 보고하려면 새 바이너리로 재시작했는지도 확인한다.

## 1. Core 파일 추가

Core 하나당 파일 하나를 둔다. 아래 클래스에 담당자가 계산 구현을 작성한다.
첫 Core는 실제 구독 메시지를 개별 인자로 받고, 후속 Core는 이전 Core의 반환값을 개별 인자로 받는다.
중간 Core는 다음 Core의 계산에 필요한 하나 이상의 변수를 반환할 수 있으며 개수·타입·순서는 연결된 Core의 계약으로 정한다.
여러 변수는 `std::pair`·`std::tuple` 같은 표준 컨테이너로 묶어 반환하고 CoreSet에서 풀어 전달한다. 두 개로 고정하지 않는다.
마지막 Core는 목표 발행 메시지 타입의 payload를 완성해 반환한다. 단일 Core는 첫 Core이자 마지막 Core다.
출력 메시지의 발행 header는 설정하지 않는다. 별도 계산 결과 래퍼나 성공·실패·준비 상태 코드는 만들지 않는다.
설정은 생성자로 받고 이전 계산 결과·필터 상태는 멤버로 유지한다.
입력 검사·누락 대응·입력 대체·fallback·성공/실패 상태 반환은 구현하지 않는다.
ROS 통신·설정 서버·Pipeline 객체·외부 공유 상태에 접근하지 않는다.

```diff
--- /dev/null
+++ src/localization/include/localization/cores/gps_only.hpp
+#pragma once
+#include <localization/types.hpp>
+#include <sensor_msgs/NavSatFix.h>
+#include <nav_msgs/Odometry.h>
+
+namespace localization {
+class GpsOnly {
+ public:
+  explicit GpsOnly(GpsOnlyParameters parameters) : parameters_(parameters) {}
+  nav_msgs::Odometry process(const sensor_msgs::NavSatFix& gps);
+
+ private:
+  GpsOnlyParameters parameters_;
+};
+}
```

`process()` 정의는 같은 Core 파일에 작성한다.

## 2. 설정 구조체·로더 추가

기존 설정과 로더를 유지하고 새 Core용 항목을 추가한다.

```diff
--- src/localization/include/localization/types.hpp
+++ src/localization/include/localization/types.hpp
 namespace localization {
+struct GpsOnlyParameters {
+  double gps_height_m = 1.2;
+};
```

```diff
--- src/localization/include/localization/config.hpp
+++ src/localization/include/localization/config.hpp
 namespace localization {
+inline GpsOnlyParameters loadGpsOnlyParameters(ros::NodeHandle& params) {
+  GpsOnlyParameters config;
+  params.param("gps_only/gps_height_m", config.gps_height_m, config.gps_height_m);
+  if (!std::isfinite(config.gps_height_m))
+    throw std::invalid_argument("invalid GPS height");
+  return config;
+}
```

Core는 `config.hpp`를 포함하지 않고 설정값만 받는다.

`config.hpp`가 없으면 설정 로더 파일을 만들고 필요한 include를 함께 추가한다.
이미 파일이 있으면 기존 로더를 보존하고 해당 함수와 부족한 include만 추가한다.
로더는 해당 Core 설정을 읽고 검증한다. 입력 메시지 검사나 Core 알고리즘을 구현하지 않는다.

```diff
--- /dev/null
+++ src/localization/include/localization/config.hpp
+#pragma once
+#include <localization/types.hpp>
+#include <ros/node_handle.h>
+#include <cmath>
+#include <stdexcept>
+
+namespace localization {
+inline GpsOnlyParameters loadGpsOnlyParameters(ros::NodeHandle& params) {
+  GpsOnlyParameters config;
+  params.param("gps_only/gps_height_m", config.gps_height_m, config.gps_height_m);
+  if (!std::isfinite(config.gps_height_m))
+    throw std::invalid_argument("invalid GPS height");
+  return config;
+}
+}
```

## 3. CoreSet 등록

다음은 기존 FullEkf를 보존하면서 GpsOnly를 추가하는 경우다.

필수 입력 인덱스는 `Data<...>`의 타입 순서와 일치시킨다.
Core별 인자 수가 다르면 `requiredInputs()`가 동일한 컨테이너 타입을 반환하도록 한다.

```diff
--- src/localization/include/localization/core_set.hpp
+++ src/localization/include/localization/core_set.hpp
 #include <localization/cores/full_ekf.hpp>
+#include <localization/cores/gps_only.hpp>
+#include <vector>
+#include <stdexcept>
-enum class CoreId { FullEkf };
+enum class CoreId { FullEkf, GpsOnly };
 struct CoreSet {
   FullEkf full_ekf;
-  std::array<std::size_t, 3> requiredInputs(CoreId) const { return {{0, 1, 2}}; }
-  nav_msgs::Odometry process(CoreId, const Data& data) {
-    return full_ekf.process(data.get<0>().message(), data.get<1>().message(), data.get<2>().message());
+  GpsOnly gps_only;
+  std::vector<std::size_t> requiredInputs(CoreId id) const {
+    switch (id) {
+      case CoreId::FullEkf: return {0, 1, 2};
+      case CoreId::GpsOnly: return {0};
+    }
+    throw std::invalid_argument("unknown core");
+  }
+  nav_msgs::Odometry process(CoreId id, const Data& data) {
+    switch (id) {
+      case CoreId::FullEkf:
+        return full_ekf.process(data.get<0>().message(), data.get<1>().message(), data.get<2>().message());
+      case CoreId::GpsOnly:
+        return gps_only.process(data.get<0>().message());
+    }
+    throw std::invalid_argument("unknown core");
   }
```

라이브러리는 선택된 Core의 필수 입력을 검사하고 Missing이면 계산·발행하지 않는다.
전체 입력 개수의 일치는 요구하지 않으며 추가 입력은 허용한다.
Selector에 전체 입력 Green 같은 사전 차단 조건을 추가하지 않는다.

### Core가 미등록인 패키지

이미 완성한 `EgoPath::process(const nav_msgs::Odometry&) → interfaces::Path`를 최초 등록하는 예시다.
기존 path_planner의 ego는 Data 인덱스 0에 연결돼 있다. 이 예시는 SectionAnnotation을 인자로 요구하지 않는다.
필요한 인자가 기존 Data에 없는 Core에는 이 예시를 적용하지 않는다.

```diff
--- src/path_planner/include/path_planner/core_set.hpp
+++ src/path_planner/include/path_planner/core_set.hpp
 #include <path_planner/types.hpp>
+#include <path_planner/cores/ego_path.hpp>
+#include <array>
+#include <stdexcept>
-enum class CoreId {};
+enum class CoreId { EgoPath };
 struct CoreSet {
-  std::array<std::size_t, 0> requiredInputs(CoreId) const { return {}; }
-  interfaces::Path process(CoreId, const Data&) {
-    throw std::logic_error("no registered core");
+  EgoPath ego_path;
+  std::array<std::size_t, 1> requiredInputs(CoreId id) const {
+    switch (id) {
+      case CoreId::EgoPath: return {{0}};
+    }
+    throw std::invalid_argument("unknown core");
+  }
+  interfaces::Path process(CoreId id, const Data& data) {
+    switch (id) {
+      case CoreId::EgoPath: return ego_path.process(data.get<0>().message());
+    }
+    throw std::invalid_argument("unknown core");
   }
 };
```

```diff
--- src/path_planner/src/ros_node.cpp
+++ src/path_planner/src/ros_node.cpp
-    std::map<std::string, pkg::CoreId>{}, selector, policy);
+    std::map<std::string, pkg::CoreId>{{"ego_path.hpp", pkg::CoreId::EgoPath}}, selector, policy);
-    Components::Checker(limits, io.readClocks().steady_ns), policy, selector, {}, {}, io);
+    Components::Checker(limits, io.readClocks().steady_ns), policy, selector,
+    pkg::CoreSet{pkg::EgoPath{}}, {}, io);
```

```diff
--- src/path_planner/config/path_planner.yaml
+++ src/path_planner/config/path_planner.yaml
 section_core_map:
-  "0": ""
+  "0": ego_path.hpp
```

대상 선택 조건에 필요한 항목만 등록한다. 다른 Core의 선택·계수는 유지한다.

## 4. 노드의 이름 등록·설정 주입

`loadRosConfig()`에 전달하는 등록 키는 확장자를 포함한 Core 파일 이름이다.
파일 이름은 등록 키이며 실행 중 소스 파일을 읽어들이는 방식이 아니다.

```diff
--- src/localization/src/localization_node.cpp
+++ src/localization/src/localization_node.cpp
 #include <localization/components.hpp>
+#include <localization/config.hpp>
-    std::map<std::string, loc::CoreId>{{"full_ekf.hpp", loc::CoreId::FullEkf}}, mapping, configured_policy);
+    std::map<std::string, loc::CoreId>{{"full_ekf.hpp", loc::CoreId::FullEkf},
+                                     {"gps_only.hpp", loc::CoreId::GpsOnly}}, mapping, configured_policy);
   const auto ekf = loc::loadEkfParameters(params);
+  const auto gps_only = loc::loadGpsOnlyParameters(params);
-    loc::CoreSet{loc::FullEkf(ekf)}, {}, io);
+    loc::CoreSet{loc::FullEkf(ekf), loc::GpsOnly(gps_only)}, {}, io);
```

## 5. YAML 매핑·계수 추가

기존 section 매핑은 유지하고 health 규칙과 새 Core 계수를 추가한다.

```diff
--- src/localization/config/localization.yaml
+++ src/localization/config/localization.yaml
-health_core_map: []
+health_core_map:
+  - when:
+      - {input: imu, issue: Missing}
+      - {input: competition_status, issue: Missing}
+    priority: 100
+    core: gps_only.hpp
+gps_only:
+  gps_height_m: 1.2
```

위 규칙은 IMU와 competition_status가 모두 Missing일 때 일치한다.
`when`은 AND 조건이며 지정하지 않은 input·issue는 일치 조건에 포함하지 않는다.
여러 규칙이 일치하면 가장 높은 priority를 적용한다. 최고 priority 동률은 설정 오류다.
일치하지 않으면 section 선택을 유지한다. policy의 Remove로 제거한 입력은 Missing이 된다.

입력 조합별 Core를 health 매핑으로 연결해 fallback한다.
Core 내부나 별도 처리 경로에 fallback을 구현하지 않는다.

## 6. 의존성 선언

GPS-only 예시는 기존 의존성만 사용한다. 아래는 새 Core에 `std_msgs`와 OpenCV가 필요한 경우의 diff다.
Core 계산에 필요한 의존성만 추가하며 기존 의존성·target·빌드 옵션은 유지한다.
의존성 선언은 구독 입력·토픽·callback 변경을 허용하지 않는다.

```diff
--- src/localization/CMakeLists.txt
+++ src/localization/CMakeLists.txt
-find_package(catkin REQUIRED COMPONENTS nav_msgs node_pipeline roscpp sensor_msgs geometry_msgs interfaces)
+find_package(catkin REQUIRED COMPONENTS nav_msgs node_pipeline roscpp sensor_msgs geometry_msgs interfaces std_msgs)
 find_package(Eigen3 REQUIRED)
+find_package(OpenCV REQUIRED COMPONENTS core)
-catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS nav_msgs node_pipeline roscpp sensor_msgs geometry_msgs interfaces DEPENDS Eigen3)
+catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS nav_msgs node_pipeline roscpp sensor_msgs geometry_msgs interfaces std_msgs DEPENDS Eigen3 OpenCV)
-include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
+include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
 add_dependencies(localization_node ${catkin_EXPORTED_TARGETS})
-target_link_libraries(localization_node ${catkin_LIBRARIES})
+target_link_libraries(localization_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
```

```diff
--- src/localization/package.xml
+++ src/localization/package.xml
   <depend>nav_msgs</depend>
+  <depend>std_msgs</depend>
+  <depend>libopencv-dev</depend>
   <export/>
```

ROS 패키지는 `catkin`의 `COMPONENTS`·`CATKIN_DEPENDS`와 `package.xml`에 선언한다.
기존 `${catkin_INCLUDE_DIRS}`·`${catkin_LIBRARIES}`·`${catkin_EXPORTED_TARGETS}` 연결을 사용한다.
외부 계산 라이브러리는 해당 `find_package`·export·헤더 경로와 필요한 링크를 추가하고
`package.xml`에 rosdep 의존성 키를 선언한다. 헤더 전용 라이브러리는 링크 추가를 생략한다.

## 7. Core 전용 검증 추가

Core의 요구사항·회귀 사례를 검사하는 최소 테스트를 해당 패키지의 `test/unit/`에 둔다.
아래는 발행 헤더를 Core가 주입하지 않는 계약을 검사하는 예시다. 알고리즘 검증을 대신하지 않는다.
새 테스트 target에는 해당 Core 계산에 직접 필요한 의존성만 연결한다. 기존 테스트·target·실행 설정은 보존한다.

```diff
--- /dev/null
+++ src/localization/test/unit/test_gps_only.cpp
+#include <localization/cores/gps_only.hpp>
+#include <gtest/gtest.h>
+
+TEST(GpsOnly, LeavesPublicationHeaderToPipeline) {
+  sensor_msgs::NavSatFix gps;
+  gps.header.stamp.fromSec(1.0);
+  gps.header.frame_id = "gps_link";
+  gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
+  gps.latitude = 37.23;
+  gps.longitude = 126.77;
+  gps.altitude = 10.0;
+  localization::GpsOnly core(localization::GpsOnlyParameters{});
+  const auto output = core.process(gps);
+  EXPECT_TRUE(output.header.stamp.isZero());
+  EXPECT_TRUE(output.header.frame_id.empty());
+}
```

```diff
--- src/localization/CMakeLists.txt
+++ src/localization/CMakeLists.txt
 if(CATKIN_ENABLE_TESTING)
+  catkin_add_gtest(test_gps_only test/unit/test_gps_only.cpp)
+  if(TARGET test_gps_only)
+    target_compile_features(test_gps_only PRIVATE cxx_std_14)
+    add_dependencies(test_gps_only ${catkin_EXPORTED_TARGETS})
+    target_link_libraries(test_gps_only ${catkin_LIBRARIES})
+  endif()
```

새 의존성의 헤더 경로·링크가 필요하면 6절의 의존성 선언과 같은 범위에서 이 테스트 target에도 연결한다.


등록 코드 변경 후 해당 노드를 다시 빌드한다.

## 8. 두 단계 Core 등록

[수정된 CoreSet](templates_two_core/core_set.hpp), [단계별 Selector](templates_two_core/template_components.hpp),
[ROS 이름 등록](templates_two_core/ros_node.cpp), [단계별 YAML](templates_two_core/config/template.yaml)의 연결을 따른다.
대상 노드가 이 구조를 이미 제공해야 한다. 이 절은 Core의 추가·교체·인자 연결을 허용하며 Pipeline·Selector·callback 구조 변경을 허용하지 않는다.

아래 `two_stage`는 예시 패키지명이다. 기존 `FirstCore`·`SecondCore`를 보존하고 첫 단계에 `PrimaryOnlyCore`, 두 번째 단계에 `SumCore`를 추가한다.
예시는 기존 Data의 인덱스 0·1이 모두 `Message`이며, 기존 첫 Core의 반환 타입이 `std::pair<Message, Message>`인 경우다.
각 단계에서 선택될 수 있는 Core는 다음 계약에 맞게 구현을 완료한 뒤 등록한다.

| 단계 | 새 Core 파일 | `process()` 계약 |
| --- | --- | --- |
| 첫 단계 | `src/two_stage/include/two_stage/cores/primary_only.hpp` | `PrimaryOnlyCore<Message, Access>::process(const Message&) → std::pair<Message, Message>` |
| 마지막 단계 | `src/two_stage/include/two_stage/cores/sum.hpp` | `SumCore<Message, Access>::process(const Message&, const Message&) → Message` |

Core 파일 추가·계산·상태는 1절, 필요한 설정 구조체·로더는 2절, 의존성·테스트는 6·7절의 diff 범위를 각 Core에 적용한다.
이 예시는 설정 없는 Core다. 설정이 필요하면 각 Core의 설정을 로드하고 4절처럼 해당 객체의 생성자에 주입한다.

### 8.1. 단계별 ID·객체·필수 입력·호출 연결

`CoreIds`는 기존 Selector가 반환하는 첫 단계 ID·두 번째 단계 ID 묶음이다. 이 구조와 Selector의 결합 조건은 유지한다.
`requiredInputs(CoreIds)`에는 선택된 첫 Core가 사용하는 구독 입력만 선언한다.
첫 Core별 필수 입력 개수가 다르면 3절처럼 같은 컨테이너 타입을 반환한다.

```diff
--- src/two_stage/include/two_stage/core_set.hpp
+++ src/two_stage/include/two_stage/core_set.hpp
 #include <two_stage/cores/first_core.hpp>
 #include <two_stage/cores/second_core.hpp>
+#include <two_stage/cores/primary_only.hpp>
+#include <two_stage/cores/sum.hpp>
-#include <array>
+#include <vector>
 #include <stdexcept>
-enum class FirstCoreId { First };
-enum class SecondCoreId { Second };
+enum class FirstCoreId { First, PrimaryOnly };
+enum class SecondCoreId { Second, Sum };
 struct CoreIds { FirstCoreId first{}; SecondCoreId second{}; };
 template<class Message, class Access> struct CoreSet {
   FirstCore<Message, Access> first;
   SecondCore<Message, Access> second;
-  std::array<std::size_t, 2> requiredInputs(CoreIds ids) const {
+  PrimaryOnlyCore<Message, Access> primary_only;
+  SumCore<Message, Access> sum;
+  std::vector<std::size_t> requiredInputs(CoreIds ids) const {
     switch (ids.first) {
-      case FirstCoreId::First: return {{0, 1}};
+      case FirstCoreId::First: return {0, 1};
+      case FirstCoreId::PrimaryOnly: return {0};
     }
     throw std::invalid_argument("unknown first core");
   }
   Message process(CoreIds ids, const node_pipeline::Data<Message, Message>& snapshot) {
     switch (ids.first) {
       case FirstCoreId::First: {
         const auto data = first.process(
           snapshot.template get<0>().message(), snapshot.template get<1>().message());
         switch (ids.second) {
           case SecondCoreId::Second: return second.process(data.first, data.second);
+          case SecondCoreId::Sum: return sum.process(data.first, data.second);
         }
         throw std::invalid_argument("unknown second core");
       }
+      case FirstCoreId::PrimaryOnly: {
+        const auto data = primary_only.process(snapshot.template get<0>().message());
+        switch (ids.second) {
+          case SecondCoreId::Second: return second.process(data.first, data.second);
+          case SecondCoreId::Sum: return sum.process(data.first, data.second);
+        }
+        throw std::invalid_argument("unknown second core");
+      }
     }
     throw std::invalid_argument("unknown first core");
   }
 };
```

대상 노드의 기존 Data 타입명·namespace에 맞춰 적용한다.
CoreSet에는 ID 분기·객체 소유·인자 전달만 둔다. 계산·입력 검사·fallback을 추가하지 않는다.
Core 객체는 기존 객체와 함께 지속 소유하며 선택될 때마다 생성하지 않는다.

중간 변수가 한 개면 그 값을 다음 Core의 한 인자로 전달한다. 여러 개면 반환 컨테이너에서 각각 꺼낸다.
예를 들어 기존 Core 간 계약이 `std::tuple<Message, double, int>` 반환과 `process(const Message&, double, int)` 입력이면 다음 호출 연결을 적용한다.
반환 개수를 런타임 설정으로 바꾸거나 인자를 묶은 채 다음 Core에 넘기지 않는다. `<tuple>` 등 직접 필요한 include는 함께 추가한다.

```diff
--- src/two_stage/include/two_stage/core_set.hpp
+++ src/two_stage/include/two_stage/core_set.hpp
+#include <tuple>
         const auto data = first.process(
           snapshot.template get<0>().message(), snapshot.template get<1>().message());
         switch (ids.second) {
-          case SecondCoreId::Second: return second.process(data.first, data.second);
+          case SecondCoreId::Second:
+            return second.process(std::get<0>(data), std::get<1>(data), std::get<2>(data));
```

이 diff는 세 변수 계약을 가진 노드의 호출 연결 예시이며 위 두 변수 예시에 그대로 덧붙이지 않는다.
실제 Core의 반환 선언·후속 Core 인자에 맞는 개수·타입·순서로 해당 호출 분기를 모두 연결한다.
범용 실행기나 새 변환·검사 컴포넌트를 만들지 않는다. 구독 입력 경로·Selector·단계 구조 변경이 필요하면 범위 밖 변경으로 보고한다.

### 8.2. 단계별 이름 등록·생성자 주입

기존 `core1_params`·`core2_params`와 각 단계의 `loadRosConfig()` 호출에서 이름→ID 등록을 확장한다.
한 단계의 파일 이름은 그 단계의 ID에만 연결한다. 공통 실행·health·policy 로딩과 단계별 namespace는 유지한다.
예시의 `ex`는 대상 패키지 namespace, `RosAccess`는 기존 메시지 필드 접근 타입이다.

```diff
--- src/two_stage/src/ros_node.cpp
+++ src/two_stage/src/ros_node.cpp
   np::loadRosConfig(nh, core1_params, topics, inputs,
-    std::map<std::string, ex::FirstCoreId>{{"first_core.hpp", ex::FirstCoreId::First}}, selector.core1, mapping_policy);
+    std::map<std::string, ex::FirstCoreId>{{"first_core.hpp", ex::FirstCoreId::First},
+                                         {"primary_only.hpp", ex::FirstCoreId::PrimaryOnly}}, selector.core1, mapping_policy);
   np::loadRosConfig(nh, core2_params, topics, inputs,
-    std::map<std::string, ex::SecondCoreId>{{"second_core.hpp", ex::SecondCoreId::Second}}, selector.core2, mapping_policy);
+    std::map<std::string, ex::SecondCoreId>{{"second_core.hpp", ex::SecondCoreId::Second},
+                                          {"sum.hpp", ex::SecondCoreId::Sum}}, selector.core2, mapping_policy);
   Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
-    Components::Checker(settings.health, io.readClocks().steady_ns), policy, selector, {}, {}, io);
+    Components::Checker(settings.health, io.readClocks().steady_ns), policy, selector,
+    ex::CoreSet<Message, RosAccess>{ex::FirstCore<Message, RosAccess>{},
+                                   ex::SecondCore<Message, RosAccess>{},
+                                   ex::PrimaryOnlyCore<Message, RosAccess>{},
+                                   ex::SumCore<Message, RosAccess>{}}, {}, io);
```

CoreSet 객체 초기화 순서는 실제 멤버 선언 순서와 맞춘다. 기존 Core가 받던 설정·정적 자산은 그대로 보존한다.
설정이 없는 기본 생성 Core만 추가하면 기존 `{}` 초기화를 유지할 수 있다. 설정이 있는 Core는 2·4절의 로딩·생성자 주입 diff를 각 단계 객체에 적용한다.
Selector를 새로 작성하거나 stage별 로더 호출·namespace를 새로 만드는 변경까지 등록 허가로 해석하지 않는다.

### 8.3. 단계별 YAML 매핑

각 단계의 기존 `section_core_map`·`health_core_map`에 그 단계의 등록 파일 이름을 사용한다.
공통 실행·health·policy 설정은 최상위에 유지한다. 새 Core 계수는 기존 Core 설정 로딩 경로에 맞춰 2·5절처럼 추가한다.

```diff
--- src/two_stage/config/two_stage.yaml
+++ src/two_stage/config/two_stage.yaml
 core1:
   section_core_map:
     "0": first_core.hpp
-  health_core_map: []
+  health_core_map:
+    - when:
+        - {input: auxiliary, issue: Missing}
+      priority: 100
+      core: primary_only.hpp
 core2:
   section_core_map:
-    "0": second_core.hpp
+    "0": sum.hpp
   health_core_map: []
```

예시는 auxiliary가 Missing이면 첫 단계를 한 입력 Core로 바꾸고, 두 번째 단계는 SumCore를 선택한다.
각 단계의 health 규칙은 같은 최종 구독 입력 health를 사용하며 5절의 AND·priority·동률 규칙을 따른다.
두 번째 단계도 health 매핑이 필요하면 `core2.health_core_map`에 같은 diff 종류의 규칙을 등록한다.
첫 단계 health 선택이 없으면 기존 section 선택을 유지한다. 어떤 선택이든 첫 Core의 필수 입력 존재 여부는 Pipeline이 검사한다.

### 8.4. 검증·사용자 보고

Core 계산 검증과 실제 대상 노드 빌드로 각 단계의 모든 호출 분기·중간 변수 계약을 확인한다. 템플릿을 테스트 target으로 만들거나 실행하지 않는다.
선택·필수 입력 누락·중간 변수 전달·최종 헤더 주입·발행은 기존 노드 경로로 확인하고, 수행하지 못한 검증은 이유와 함께 미확인으로 보고한다.
완료 보고에는 단계별 Core 파일·선택 조건, 중간 변수의 개수·타입·순서, 실제 수행한 검증을 포함한다.
후속 Core가 요구하는 변수가 전달되지 않거나 단계 구조 변경이 필요하면 다음 절의 동일한 중단·리드 협의·명시적 허가 절차를 따른다.

## 범위 밖 변경이 필요한 경우의 보고

모든 범위 밖 변경을 적용하기 전에 이 절차를 따른다. 기능 구현, 버그 수정, 계약 정합, 빌드·검증 문제 해결도 예외가 아니다.
작업에 그 변경이 필요하다는 사실과 변경 권한이 있다는 판단을 구분한다.
막힌 요청 동작을 먼저 말하고, 현재 전달 경로와 허용 범위를 근거로 설명한다.
“입력이 없다”, “연결이 필요하다”, “최소 변경이다”만으로 설명을 끝내지 않는다.
구독 없음·Core 전달 없음·실행 중 수신 없음·health 검사 후 Missing을 구분한다.
확인한 코드 사실과 추정한 원인도 구분한다.

범위 안 등록과 범위 밖 입력 연결을 묶어서 “가이드 범위 안”이라고 말하지 않는다.
승인 요청에는 현재 미적용 상태와 허가할 구체적인 변경을 함께 제시한다.
이미 승인된 동일 변경을 다시 묻지 않는다. 질문·이의 제기·응답 없음은 승인으로 해석하지 않는다.

보고에는 다음 내용을 포함한다.

| 항목 | 내용 |
| --- | --- |
| 요청 동작과 차단 지점 | 구현해야 하는 동작 중 무엇이 현재 불가능한지 |
| 확인한 사실 | 파일·메서드·타입·인덱스와 실제 전달 경로 |
| 허용 범위 근거 | 적용되는 문서 링크·위치와 실제 제한 문구를 함께 인용 |
| 필요한 변경 | 파일별 수정할 코드·계약·직접 필요한 연쇄 변경 |
| 필요성 | 기존 등록 diff만으로 해결할 수 없는 이유 |
| 범위 안 대안과 한계 | 지금 개발 가능한 결과와 충족하지 못하는 요구사항 |
| 현재 작업 상태 | 작성·적용·검증한 항목과 미적용 항목 |
| 허가 요청 | 위에 한정한 변경의 명시적 허가와 리드 협의 요청 |

### 보고 예시: SectionAnnotation이 Core로 전달되지 않음

> `/section_annotation`에 따라 체크포인트를 제외하는 동작의 노드 등록을 진행할 수 없습니다.
>
> `/section_annotation` 구독은 있습니다. 다만 `src/path_planner/src/ros_node.cpp`의 `onSection()`은 `node.onSection()`으로 Selector용 구간 정보만 저장합니다. 원본 메시지를 `node.onInput()`으로 넣는 연결은 없습니다. `types.hpp`의 Data와 `components.hpp`의 Buffer는 ego·competition·objects·tf 네 타입이며, 유효 인덱스는 0~3입니다. 따라서 `process(ego, section_annotation)`에 필요한 두 번째 메시지를 전달할 수 없습니다.
>
> 이 가이드는 “Core 등록만으로 공통 Pipeline, Validator, Selector, callback, 토픽, HeaderInjector, launch를 수정하지 않는다”고 정합니다. [루트 README](../../README.md)는 “허용 범위 밖의 변경은 중단하고, 가이드의 보고·리드 협의·명시적 허가 절차를 따른다”고 정합니다. `core_set.hpp`의 등록 권한으로 입력 pack과 callback 변경까지 수행할 수는 없습니다.
>
> 필요한 변경은 다음과 같습니다.
>
> - `src/path_planner/include/path_planner/types.hpp`: Data에 원본 SectionAnnotation 타입 추가.
> - `src/path_planner/include/path_planner/components.hpp`: 입력 ID·Buffer 타입 pack·Checker 입력 개수를 같은 순서로 확장.
> - `src/path_planner/src/ros_node.cpp`: 기존 Selector용 전달을 유지하면서 원본 메시지의 `onInput()` 연결, 입력 이름·토픽 등록, 입력별 health 설정 연결, Context 입력 개수 대응. stamp·frame 계약은 유지.
> - `src/path_planner/config/path_planner.yaml`: 새 Core 입력에 필요한 section 입력의 health 한도 설정. 이 변경은 Core 계수·선택 매핑과 별도 허가 대상.
>
> CoreSet은 기존 Data의 메시지를 전달할 수만 있으므로, 이 연결 없이 섹션에 따른 체크포인트 제외를 구현할 수 없습니다. 필요해 보이는 다른 계약 변경이 확인되면 추가 적용 전에 따로 보고하겠습니다.
>
> 허용 범위 안에서는 Core·설정 작성과 원본 메시지를 직접 주는 계산 테스트까지 가능합니다. 이 결과는 노드에서 섹션을 받거나 `/local_path`를 발행한 결과가 아닙니다. 지금은 Core·설정만 작성했고, 사용할 수 없는 입력 인덱스·이름 등록·활성 YAML 매핑은 적용하지 않았습니다.
>
> 리드와 협의하여 위 입력 연결과 직접 필요한 입력 설정 변경을 허가해 주시겠습니까? 허가 전에는 이 연결과 이에 의존하는 활성 등록을 적용하지 않겠습니다.

실제 보고에서는 확인한 파일 위치와 변경 내용을 사용한다. 위 예시의 입력 순서·파일·상태를 다른 패키지에 그대로 복사하지 않는다.
다른 변경 사유를 한꺼번에 승인받으려 묶지 않는다. README·HeaderInjector의 시각 계약 등 별도 변경이 필요하면 각각 이유를 밝힌다.

## 완료 또는 중단 보고

마지막 안내는 실제 도달한 상태부터 말한다. 변경한 Core·등록 파일, 선택 조건, 검증 결과, 남은 제한을 짧게 기록한다.

> Core 작성과 등록을 완료했습니다. 기존 Data의 메시지를 연결하고 해당 YAML 선택 조건에 매핑했습니다. 해당 노드 빌드와 Core 계산 테스트는 통과했습니다. ROS 실행·출력 발행은 수행하지 않아 미확인입니다.

> Core·설정 작성과 계산 테스트까지 완료했습니다. 원본 SectionAnnotation 전달 연결이 허용 범위 밖이므로 노드 등록·활성 매핑은 미적용 상태이며, 위 연결 변경의 명시적 허가를 기다리고 있습니다.

잘못 적용한 등록이나 범위 밖 변경을 발견하면, 위반한 변경·영향·현재 상태를 즉시 보고한다.
기존 사용자 변경과 구분해 자신이 추가한 문제를 바로잡는다. 되돌림에 사용자 후속 변경이 겹치면 먼저 협의한다.
원인이 권한 판단이나 잘못된 편집이었다면 이를 직접 밝힌다. “혼란을 드렸다”는 표현으로 실제 변경의 문제를 대신하지 않는다.
