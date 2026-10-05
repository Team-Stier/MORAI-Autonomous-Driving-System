# Core 등록 가이드

[프로젝트 규칙](../../README.md), [Pipeline 계약](README.md), 대상 패키지 README와 실제 메시지 정의를 따른다.
리드가 제공한 **기존 스테이지**에 Core를 등록한다. 첫 번째부터 n번째까지 같은 절차다.
Core 개발자는 계산과 아래 등록 diff를 담당하고, 리드는 입력 전달·검사·정책·스테이지 구성·헤더·발행 경로를 제공한다.

**Core 파일 밖에서는 아래 diff가 정의하는 변경 종류와 담당 범위만 허용한다.**
파일 전체의 수정 권한을 뜻하지 않는다. 필요한 diff만 적용하고 기존 객체·설정·사용자 변경은 보존한다.
경로·이름·인덱스·타입·인자 수·반환 수·설정값은 실제 기존 계약에 맞게 치환한다.
등록 접점과 입출력 계약이 갖춰져 있으면 이 diff만으로 등록을 완료한다.

## 시작 전 확인

루트 README·AGENTS.md·Pipeline README·대상 README·메시지 정의와 기존 변경을 읽고 다음을 코드에서 확인한다.

- 기존 단계의 enum, CoreSet, dispatch 함수, `loadStageConfig()` 이름 map, YAML namespace가 있다.
- 필요한 입력이 해당 단계의 `Data<Ts...>` 슬롯에 실제 전달되고 기존 검사 계약을 충족한다. 구독이나 `onSection()`만으로 Core 입력 전달을 판단하지 않는다.
- `process()`의 개별 인자 인덱스·타입이 슬롯과 일치한다. 중간 반환의 개수·타입·순서는 다음 단계의 입력 계약을 충족하며 마지막 반환은 목표 발행 메시지 하나다.
- Core 설정·정적 자산은 기존 합의된 공급 경로로 주입할 수 있다. 모든 기존 단계에 호환되는 Core와 선택 매핑이 있어야 최종 발행까지 진행된다.

전제가 부족하거나 문서·코드가 충돌하면 아래 보고 절차를 먼저 따른다. 타입을 지운 슬롯이나 YAML 미선택을 이유로 호환되지 않는 Core를 등록하지 않는다.
시작할 때 대상 Core·기존 단계·입출력 계약·적용할 등록 범위를 사용자에게 짧게 안내한다.

## 공통 등록: Core 파일과 네 등록 지점

| 위치 | Core 개발자의 변경 |
| --- | --- |
| `cores/new_core.hpp` | 개별 인자를 받는 `process()`와 계산·지속 상태 |
| `core_set.hpp` | include·해당 단계 enum 항목·소유 객체·초기화 |
| `core_dispatch.hpp` | 해당 ID의 필수 입력 검사·실제 Core 호출·반환 슬롯 연결 |
| 노드 생성 코드 | 기존 파일 이름→ID map 항목·Core 객체 생성·설정 주입 |
| YAML | 해당 단계의 Core 선택 매핑·해당 Core 계수 |

Core 하나당 파일 하나를 둔다. 첫 Core는 구독 메시지, 후속 Core는 이전 반환 변수를 실제 타입의 개별 인자로 받는다.
중간 Core는 하나의 값 또는 여러 값을 `std::pair`·`std::tuple`로 반환하고, 마지막 Core는 목표 메시지 payload를 완성한다.
Core에 Data·Slot·Health·수신 관리 메타데이터를 넘기지 않는다. 입력 유효성 검사·누락 감지·대체·fallback·성공/실패 상태 반환과 발행 헤더 설정을 넣지 않는다.

아래는 입력 두 개로 새 값을 계산하는 Core의 등록 패턴이다. `StageCoreId`·`processStage()`는 대상 단계의 기존 이름으로, `FirstInput`·`SecondInput`은 이미 전달되는 실제 타입으로 바꾼다.
`EXISTING_IDS`·`EXISTING_MEMBERS`·`EXISTING_NAME_ENTRIES`·`EXISTING_OBJECTS`는 기존 항목 전체를 보존한다는 표기이며 비어 있으면 해당 표기와 불필요한 쉼표를 생략한다.
직접 필요한 include만 추가하고 이미 있는 include는 생략한다.

```diff
--- src/<package>/include/<package>/core_set.hpp
+++ src/<package>/include/<package>/core_set.hpp
+#include <package/cores/new_core.hpp>
-enum class StageCoreId { EXISTING_IDS };
+enum class StageCoreId { EXISTING_IDS, NewCore };
 struct CoreSet {
   EXISTING_MEMBERS
+  NewCore new_core;
 };

--- src/<package>/include/<package>/core_dispatch.hpp
+++ src/<package>/include/<package>/core_dispatch.hpp
+#include <required_native_header>
   switch (id) {
+    case StageCoreId::NewCore:
+      if (!data.hasInputs({0, 1})) return false;
+      before();
+      result = node_pipeline::wrapResult(cores.new_core.process(
+        data.message<FirstInput>(0), data.message<SecondInput>(1)), {});
+      return true;

--- src/<package>/src/<node_source>.cpp
+++ src/<package>/src/<node_source>.cpp
   np::loadStageConfig(stage_params, input_names,
     std::map<std::string, pkg::StageCoreId>{
+      {"new_core.hpp", pkg::StageCoreId::NewCore},
       EXISTING_NAME_ENTRIES
     }, selector, policy);
-    stages, pkg::CoreSet{EXISTING_OBJECTS}, header_injector, io);
+    stages, pkg::CoreSet{EXISTING_OBJECTS, pkg::NewCore{}}, header_injector, io);

--- src/<package>/config/<package>.yaml
+++ src/<package>/config/<package>.yaml
 stageN:
   section_core_map:
-    "0": <existing_core_or_empty>
+    "0": new_core.hpp
```

`hasInputs()` 목록은 실제 Core 인자에 사용한 인덱스와 일치시킨다. 하나라도 Missing이면 false를 반환하고 `before()`·Core 호출을 하지 않는다.
`before()`는 필수 입력 확인 직후 실제 호출 전에 둔다. 추가 슬롯은 제한하지 않는다.
`wrapResult()`의 두 번째 인자는 반환 변수 순서에 맞춘 원본 입력 인덱스 목록이다. 수정·패스스루 값은 대응 인덱스, 새 계산값은 `-1`을 쓴다. 전부 새 값이면 `{}` 또는 인자 생략을 사용한다.
기존 값은 원본 시간 메타데이터를 상속하고 새 값은 비워 둔다. 반환을 다음 스냅샷으로 교체하는 실행은 Pipeline이 담당한다.

기존 dispatch가 빈 stub이면 같은 함수에 인자 이름과 switch 틀을 추가한 뒤 위 case를 넣는다. 함수 타입·기존 unknown-ID throw는 유지한다.

```diff
--- src/<package>/include/<package>/core_dispatch.hpp
+++ src/<package>/include/<package>/core_dispatch.hpp
-inline bool processStage(CoreSet&, StageCoreId, const node_pipeline::StageData&,
-                         node_pipeline::CoreResult&, const node_pipeline::BeforeCore&) {
+inline bool processStage(CoreSet& cores, StageCoreId id, const node_pipeline::StageData& data,
+                         node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
+  switch (id) {
+    // 위 NewCore case를 여기에 추가한다.
+  }
```

CoreSet이 aggregate면 멤버 선언 순서대로 기존 객체와 새 객체를 모든 생성 위치에 주입한다.
기존 `{}` 생성도 기존 Core의 기본값을 유지한 명시적 객체 목록으로 바꿀 수 있다. 명시적 생성자가 있으면 새 멤버의 인자·초기화만 확장한다.

```diff
--- src/<package>/include/<package>/core_set.hpp
+++ src/<package>/include/<package>/core_set.hpp
+#include <utility>
-  CoreSet(ExistingCore existing) : existing(std::move(existing)) {}
+  CoreSet(ExistingCore existing, NewCore new_core)
+      : existing(std::move(existing)), new_core(std::move(new_core)) {}
```

이름 map과 YAML 값은 확장자를 포함한 같은 Core 파일 이름을 쓴다. 실제 적용할 section 행만 추가·교체하고 다른 매핑은 보존한다.
health 조건으로 선택하는 경우에만 해당 단계의 기존 입력 이름·검사 대상에 맞는 규칙을 추가·교체한다.

```diff
--- src/<package>/config/<package>.yaml
+++ src/<package>/config/<package>.yaml
 stageN:
-  health_core_map: []
+  health_core_map:
+    - when:
+        - {input: first, issue: Stale}
+      priority: 100
+      core: new_core.hpp
```

`when`은 AND 조건이며 최고 priority의 일치 규칙을 선택한다. 최고 priority 동률은 설정 오류다. 필수 입력이 Missing인 Core를 선택해도 호출할 수 없다.

## 필요한 경우만: 설정·의존성·테스트

### 설정이 있는 Core

해당 Core의 설정 구조체·로더·계수만 추가하고 생성자에 값으로 주입한다. Core는 로더 헤더를 include하지 않는다.
설정 헤더는 기존 `types.hpp` 또는 기존 Core 설정 헤더를 사용한다. `config.hpp`가 없으면 아래 추가 내용으로 생성할 수 있다.
`#pragma once`·namespace 틀은 새 로더 파일에만 만들고, 기존 파일에서는 기존 namespace에 함수만 추가한다.

```diff
--- src/<package>/include/<package>/types.hpp
+++ src/<package>/include/<package>/types.hpp
+#include <required_settings_header>
 namespace package {
+struct NewCoreParameters {
+  double coefficient = 1.0;
+};

--- src/<package>/include/<package>/config.hpp
+++ src/<package>/include/<package>/config.hpp
+#pragma once
+#include <package/types.hpp>
+#include <ros/node_handle.h>
+#include <cmath>
+#include <stdexcept>
+namespace package {
+inline NewCoreParameters loadNewCoreParameters(ros::NodeHandle& params) {
+  NewCoreParameters config;
+  params.param("new_core/coefficient", config.coefficient, config.coefficient);
+  if (!std::isfinite(config.coefficient)) throw std::invalid_argument("invalid coefficient");
+  return config;
+}
+}

--- src/<package>/src/<node_source>.cpp
+++ src/<package>/src/<node_source>.cpp
+#include <package/config.hpp>
+  const auto new_parameters = pkg::loadNewCoreParameters(params);
-    stages, pkg::CoreSet{EXISTING_OBJECTS, pkg::NewCore{}}, header_injector, io);
+    stages, pkg::CoreSet{EXISTING_OBJECTS, pkg::NewCore(new_parameters)}, header_injector, io);

--- src/<package>/config/<package>.yaml
+++ src/<package>/config/<package>.yaml
+new_core:
+  coefficient: 1.0
```

설정 필드·로더는 해당 Core의 실제 계수와 설정 검증에 맞춘다. 입력 메시지 검사·계산·새 자산 공급 경로는 추가하지 않는다.

### 새 의존성이 있는 Core

새 Core에 직접 필요한 의존성만 선언하고 대상 노드·Core 테스트에 include·생성 순서·링크를 연결한다. 기존 선언은 중복하지 않는다.
아래 의존성 이름과 target은 실제 사용하는 것으로 바꾼다. 헤더 전용 의존성은 링크를 생략하고 공개 헤더에 필요한 것만 export한다.

```diff
--- src/<package>/CMakeLists.txt
+++ src/<package>/CMakeLists.txt
-find_package(catkin REQUIRED COMPONENTS EXISTING_ROS_DEPENDENCIES)
+find_package(catkin REQUIRED COMPONENTS EXISTING_ROS_DEPENDENCIES new_ros_dependency)
+find_package(NewLibrary REQUIRED)
-catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS EXISTING_ROS_DEPENDENCIES DEPENDS EXISTING_LIBRARIES)
+catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS EXISTING_ROS_DEPENDENCIES new_ros_dependency DEPENDS EXISTING_LIBRARIES NewLibrary)
+target_include_directories(affected_target PRIVATE ${catkin_INCLUDE_DIRS} ${NewLibrary_INCLUDE_DIRS})
+add_dependencies(affected_target ${catkin_EXPORTED_TARGETS})
-target_link_libraries(affected_target EXISTING_LINK_LIBRARIES)
+target_link_libraries(affected_target EXISTING_LINK_LIBRARIES ${catkin_LIBRARIES} ${NewLibrary_LIBRARIES})

--- src/<package>/package.xml
+++ src/<package>/package.xml
+  <depend>new_ros_dependency</depend>
+  <depend>new_library_rosdep_key</depend>
```

ROS 의존성·외부 계산 라이브러리 중 필요한 부분만 적용한다. `find_package()`는 export·target 연결보다 앞에 둔다.
기존 target의 `target_link_libraries()`가 `PRIVATE` 등을 사용하면 같은 signature를 유지한다.

### Core 검증

새 Core의 native 입력 fixture와 계산·상태·입출력 계약을 검사하는 최소 회귀 테스트를 작성한다. 빈 테스트나 등록 코드를 복사한 테스트로 대신하지 않는다.
신규 테스트 파일의 본문은 해당 Core의 fixture·호출·기대값 검사로 한정한다.

```diff
--- /dev/null
+++ src/<package>/test/unit/test_new_core.cpp
+#include <package/cores/new_core.hpp>
+#include <required_native_header>
+#include <required_test_fixture_header>
+#include <gtest/gtest.h>
+// 해당 Core의 native fixture와 계산·상태·입출력 계약 테스트를 작성한다.

--- src/<package>/CMakeLists.txt
+++ src/<package>/CMakeLists.txt
+if(CATKIN_ENABLE_TESTING)
+  catkin_add_gtest(test_new_core test/unit/test_new_core.cpp)
+  if(TARGET test_new_core)
+    target_compile_features(test_new_core PRIVATE cxx_std_14)
+    target_include_directories(test_new_core PRIVATE ${catkin_INCLUDE_DIRS})
+    add_dependencies(test_new_core ${catkin_EXPORTED_TARGETS})
+    target_link_libraries(test_new_core ${catkin_LIBRARIES} gtest_main)
+  endif()
+endif()

--- src/<package>/test/unit/<existing_registration_test>.cpp
+++ src/<package>/test/unit/<existing_registration_test>.cpp
+#include <package/cores/new_core.hpp>
+#include <required_native_header>
+#include <required_test_fixture_header>
+  NewCoreParameters new_parameters{};
-  CoreSet cores{EXISTING_OBJECTS};
+  CoreSet cores{EXISTING_OBJECTS, NewCore(new_parameters)};
+// 새 case의 Missing 거부·native 호출·반환 개수/타입·원본 슬롯 대응 검사를 추가한다.
```

설정이 없으면 설정 선언 없이 `NewCore{}`를 쓴다. 기존 등록 테스트의 Node 생성에서도 CoreSet 인자만 같은 방식으로 갱신한다.
새 case의 검사 추가는 기존 fixture·기대 동작을 바꾸는 권한이 아니다. 필요한 외부 라이브러리는 위 의존성 diff를 테스트 target에도 적용한다.
기존 `CATKIN_ENABLE_TESTING` 블록이 있으면 target 부분만 추가한다.

## 허용 범위와 보고

등록에 따른 범위 밖 변경은 지양한다. 다음은 등록 diff의 대상 밖이며 필요하다고 판단해도 먼저 적용하지 않는다.

- 공통 Pipeline 라이브러리·템플릿, callback·토픽·Buffer/Assembler·Data 타입 pack·입력 ID.
- Validator·Checker·Policy·Selector의 로직, stage factory, 스테이지 수·순서·입출력 계약.
- HeaderInjector·IO·launch, README·AGENTS.md·메시지 정의·서브모듈.
- `loadStageConfig()`의 로직·입력 이름 map·namespace·호출 개수, YAML의 실행 방식·주기·trigger·큐·입력 한도·policy priority.

CoreSet에 실행·검사·스테이지 순회를 추가하거나 Core 알고리즘을 Policy·Validator·Selector·callback·설정 로더로 옮겨 우회하지 않는다.

**허용 범위 밖 변경이 필요하면 해당 변경을 중단하고 리드와 상의하며 반드시 명시적 허가를 요구한다.**
입력 전달·타입·검사·선택·헤더·의존성·빌드 등 모든 범위 밖 변경에 같은 절차를 적용한다.
보고에는 다음을 포함한다.

1. 무엇이 불가능한지와 확인한 파일·위치·코드·실제 전달 경로.
2. 제한 또는 충돌 근거인 문서 링크·위치와 실제 지침 문구. 문서·코드 충돌이면 양쪽을 제시하고 높은 권한의 문서에 맞출 해결을 제안한다.
3. 필요한 파일별 diff·사유·직접 필요한 연쇄 변경과 등록 diff만으로 해결할 수 없는 이유.
4. **허용 범위 안에서 개발 가능한 방향과 한계**, 적용·검증한 결과와 미적용 변경.
5. 위 변경에 한정한 리드 협의·명시적 허가 요청.

예: “SectionAnnotation을 사용하려는데 callback은 `onSection()`만 호출하고 Data에 원본 슬롯이 없어 개별 Core 인자로 전달할 수 없다”를 실제 코드로 보고한다.
필요한 입력 연결 diff와 제한 문구를 제시하고, 범위 안 대안으로 “Core·설정을 작성하고 native 메시지를 직접 주는 계산 테스트까지 진행”을 제안한다.
구독 없음·Core 전달 없음·실행 중 수신 없음·health/policy 후 Missing을 구분한다.

사용자 결정 전에는 해당 변경과 이에 의존하는 등록을 보류한다. 독립적인 범위 안 작업은 계속할 수 있다.
Core 개발 요청만으로 범위 밖 허가를 추정하지 않는다. 이미 명시적으로 승인된 같은 변경은 다시 묻지 않으며 질문·이의 제기·무응답은 승인으로 해석하지 않는다.
다른 대화의 승인은 현재 작업의 권한으로 가져오지 않는다. 자동 승인 검토가 거절하면 거절된 동작과 이유를 별도로 보고한다.

완료할 때 자신의 변경을 이 diff 범위·적용 문서와 대조하고 대상 Core 테스트와 노드 컴파일·링크를 확인한다.
Core 계산 완료·등록 완료·실제 발행 확인을 구분하고 선택 조건·검증 결과·미확인 항목을 짧게 보고한다. 실행 환경 부족은 사유와 함께 미확인으로 기록한다.
검증을 이유로 다른 노드·Pipeline·환경을 임의 변경하거나 전체 스택을 시작하지 않는다. 통과한 검증은 새 변경·실패가 없으면 반복하지 않는다.
범위 밖 변경을 잘못 적용했다면 변경·영향·현재 상태를 보고하고 자신의 변경을 바로잡는다. 사용자 변경과 겹치는 수정은 먼저 협의한다.
