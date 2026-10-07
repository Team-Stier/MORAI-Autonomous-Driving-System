# Core 등록 가이드

[루트 계약](../../README.md)·[Pipeline 계약](README.md)·대상 패키지 README·메시지 정의를 따른다. 리드가 제공한 기존 Stage에 Core를 등록한다.

**Core 파일 밖에서는 아래 fenced `diff`의 `+`·`-` 행만 수정한다.** 실제 이름·타입·인자 순서·계수로 치환하되 다른 실행 경로를 추가하지 않는다. 별도 callback·dispatch·CoreSet·main은 작성하지 않는다.

## 등록 전 확인

- 입력 이름은 `node.yaml`의 구독 입력 또는 앞 Stage Core의 출력 이름이다. `/section_annotation`은 입력으로 등록하지 않는다.
- `inputs`의 순서·타입은 native `process()` 인자와 일치하고 모두 필수다. `required_inputs`는 사용하지 않는다.
- 출력 이름·개수·순서·타입은 실제 반환값과 일치한다. 앞 Core가 다음 Core에 필요한 값을 함께 반환한다. 마지막 Stage는 발행 메시지를 반환한다.
- 리드가 제공한 Validator·Policy·Bank·설정 namespace·자산 선언을 사용한다. Core는 같은 패키지 Bank만 참조할 수 있다.

## 허용 등록 diff

Core 본체는 `include/<package>/cores/new_core.hpp` 하나에 작성한다. 설정이 없으면 로더 항목을 생략한다. 설정이 있으면 해당 Core 전용 로더만 추가한다.

```diff
--- /dev/null
+++ src/<package>/include/<package>/parameters/new_core.hpp
+#pragma once
+#include <node_pipeline/sdk.hpp>
+#include <package/cores/new_core.hpp>
+namespace package { namespace parameters {
+struct NewCore {
+  NewCoreParameters operator()(const node_pipeline::Parameters& parameters) const {
+    NewCoreParameters value;
+    value.coefficient = parameters.get<double>("coefficient", value.coefficient);
+    return value;
+  }
+};
+}}

--- /dev/null
+++ src/<package>/registrations/new_core.yaml
+name: new_core.hpp
+type: package::NewCore
+stage: existing_stage
+inputs: [first, second]
+outputs:
+  - {name: result, type: native_package/NativeMessage}
+validator: {header: package/validators/existing.hpp, type: package::validators::Existing}
+policy: {header: package/policies/existing.hpp, type: package::policies::Existing}
+parameter_loader: {header: package/parameters/new_core.hpp, type: package::parameters::NewCore}
+parameter_namespace: new_core
+bank: existing_bank
+allowed_issues: {}

--- src/<package>/node.yaml
+++ src/<package>/node.yaml
 stages:
   existing_stage:
-    cores: [existing_core.hpp]
+    cores: [existing_core.hpp, new_core.hpp]
     section_core_map:
-      "0": existing_core.hpp
+      "0": new_core.hpp
```

`bank`는 Bank가 필요한 경우에만 사용한다. `parameter_namespace`는 해당 Core의 설정 영역이다. 로더는 `Parameters`의 읽기 전용 설정과 선언된 `asset(name)` 경로만 읽는다. ROS handle이나 실행기 객체를 전달받지 않는다. Core 설정 타입은 Core 본체 또는 기존 native 설정 헤더에 둔다.

수정·패스스루 출력의 원본 시간 정보를 유지해야 하면 해당 출력에만 `time_from`을 선언한다. 이 항목은 값 전달을 대신하지 않는다.

```diff
--- src/<package>/registrations/new_core.yaml
+++ src/<package>/registrations/new_core.yaml
 outputs:
-  - {name: result, type: native_package/NativeMessage}
+  - {name: result, type: native_package/NativeMessage, time_from: first}
```

health 조건의 Core 교체가 필요한 경우 기존 입력 이름과 issue를 사용한다. `when`은 AND이며 일치 규칙 중 최고 priority가 적용된다. 최고 priority 동률은 오류다.

```diff
--- src/<package>/node.yaml
+++ src/<package>/node.yaml
   existing_stage:
-    health_core_map: []
+    health_core_map:
+      - when: [{input: first, issue: Stale}]
+        priority: 100
+        core: new_core.hpp
```

Core 계수만 해당 Core namespace에 추가할 수 있다. Pipeline 설정·큐·검사 상한·토픽·Stage 구조는 변경하지 않는다.

```diff
--- src/<package>/config/<package>.yaml
+++ src/<package>/config/<package>.yaml
+new_core:
+  coefficient: 1.0
```

## 공식 확장점과 금지 경로

Validator는 `InputView`와 `ProfileHealth`로 도메인 고유 검사만 추가한다. Policy의 `apply(InputView&, const ProfileHealth&)`는 payload를 보정·제거하고 반복 여부를 `bool`로 반환한다. 보정 strategy는 값을 수정하며 반환값은 없다. Validator·Policy 로직 수정은 리드의 명시적 작업 범위에만 포함한다.

Core에는 입력 유효성 검사·누락 감지·대체·fallback·상태 반환·최외곽 발행 헤더 주입을 넣지 않는다. 중첩 메시지 필드는 native 반환 payload로 구성한다. 다른 계산 Core를 소유·참조·호출하거나 Pipeline 내부 API에 접근하지 않는다. ROS 통신은 공식 `/visualizer/*` 발행만 허용한다. 파라미터 로더·Validator·Policy에 계산 알고리즘을 옮겨 이 제한을 우회하지 않는다.

등록 YAML 변경 후 재빌드한다. 빌드는 YAML 이름·타입·연결 검사, native 시그니처 검사와 Clang AST 계약 검사를 필수 수행한다. 등록을 위해 Pipeline C++이나 생성 소스를 수정하지 않는다.

## 범위 밖 변경과 완료 보고

위 diff로 등록할 수 없으면 해당 변경을 멈춘다. 충돌 문서·실제 전달 경로·필요한 파일별 diff·범위 안 대안과 한계를 제시하고 리드 협의·명시적 허가를 요청한다. 이미 승인된 같은 변경은 다시 묻지 않는다. 독립적인 허용 작업은 계속한다.

완료 전 실제 diff를 허용 범위와 대조한다. 계산 테스트·등록 검사·노드 컴파일과 링크를 확인하고, 계산 완료·등록 완료·실제 발행 확인을 구분해 보고한다. 검증을 이유로 전체 스택·시뮬레이터를 시작하거나 다른 노드·메시지·서브모듈·AGENTS.md를 수정하지 않는다.
