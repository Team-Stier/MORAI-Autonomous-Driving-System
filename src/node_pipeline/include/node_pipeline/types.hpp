// 여러 파일이 이 헤더를 include해도 타입·함수를 한 번만 정의한다.
#pragma once

// 큐 용량과 검사 횟수 등에 사용하는 std::size_t 정의다.
#include <cstddef>
// 시각(ns)과 sample ID를 고정 크기 정수로 표현한다.
#include <cstdint>
// 설정 오류를 std::invalid_argument로 알린다.
#include <stdexcept>
// 정책 거부·계산 실패 등의 이유를 문자열로 전달한다.
#include <string>
// 생성자로 전달받은 helper를 std::move로 멤버에 옮긴다.
#include <utility>
// 한 input에 동시에 존재하는 모든 오류를 목록으로 보관한다.
#include <vector>
#include <type_traits>

// ROS나 특정 노드의 타입과 구분되는 공통 라이브러리 이름 공간이다.
namespace node_pipeline {

// 한 번의 읽기로 받은 두 시계의 값을 전달한다. 두 값은 서로 빼지 않는다.
struct Clocks {
  // 메시지 stamp의 나이·미래 여부를 판정하는 ROS 시각(ns)이다.
  std::int64_t ros_ns = 0;
  // 수신 무응답·처리 경과를 판정하는 단조 증가 시각(ns)이다.
  std::int64_t steady_ns = 0;
};

// Periodic은 timer, OnInput은 지정 input callback이 계산을 시작한다.
enum class ExecutionMode { Periodic, OnInput };
// 각 실행 노드가 자신의 input 식별자와 오류 타입을 지정한다.
template <class InputId, class IssueKind>
// 하나의 input 상태를 보관한다. 대표 오류 하나로 나머지 오류를 덮지 않는다.
struct InputHealth {
  // 이 오류 목록이 어느 input에 속하는지 표시한다.
  InputId input;
  // Missing·범위 초과·Stale 등 동시에 검출된 오류를 모두 유지한다.
  std::vector<IssueKind> issues;
  // 현재 오류가 하나도 없을 때만 Green으로 판단한다.
  bool green() const { return issues.empty(); }
};

// 보정 대상 하나를 표현할 때도 노드별 input/오류 타입을 그대로 사용한다.
template <class InputId, class IssueKind>
// 이번 policy 호출에서 선택된 오류의 위치와 종류를 묶는다.
struct Issue {
  // 보정할 input을 지정한다.
  InputId input;
  // 해당 input에서 이번에 처리할 오류 종류를 지정한다.
  IssueKind kind;
};

// 음수 priority는 policy에서 dismiss하여 추가 처리하지 않는다. 동률은 input/issue의 순회 순서를 유지한다.
// Health 구조와 priority 함수, 선택 결과의 타입을 호출자가 제공한다.
template <class Health, class Priority, class SelectedIssue>
// 모든 input의 오류를 훑고 지금 보정/거부할 최고 우선순위 오류 하나를 찾는다.
bool highestPriorityIssue(const Health& health, Priority priority,
                          // 결과는 selected 참조에 쓴다. 반환값이 false이면 selected를 사용하지 않는다.
                          SelectedIssue& selected) {
  // 아직 선택이 없다는 초기값이다. -1보다 작은 priority도 보정 대상에서 제외된다.
  int best_rank = -1;
  // Health에 저장된 input 순서로 순회한다. 이 순서가 동률 해결 기준이다.
  for (const auto& input : health.inputs) {
    // 하나의 input 안에서는 checker가 기록한 issue 순서로 순회한다.
    for (const auto& kind : input.issues) {
      // input과 오류 종류를 policy의 priority 함수에 넘겨 현재 처리 순위를 얻는다.
      const int rank = priority(input.input, kind);
      // 엄격히 큰 순위만 교체하므로 같은 priority의 뒤쪽 오류가 앞쪽을 덮지 않는다.
      if (rank > best_rank) {
        // 현재까지 찾은 최고 처리 순위를 갱신한다.
        best_rank = rank;
        // 선택된 input/오류를 함께 저장한다. 오류를 Health에서 삭제하지는 않는다.
        selected = {input.input, kind};
      }
    }
  }
  // 0 이상의 priority를 가진 오류가 있어야 true다. dismiss issue만 있으면 false다.
  return best_rank >= 0;
}

// 실행 trigger 설정도 노드의 input ID 타입에 맞춘다.
template <class InputId>
// 노드의 계산 시작 방식과 health 재검사 한도를 담는 설정이다.
struct ExecutionConfig {
  // 기본은 주기 실행이다. 실제 ROS timer 생성은 실행 패키지가 담당한다.
  ExecutionMode mode = ExecutionMode::Periodic;
  // OnInput에서 계산을 시작할 upstream ID다. Periodic에서는 사용하지 않는다.
  InputId trigger_input{};
  // 하나의 snapshot에 대한 최대 healthCheck 호출 횟수다. 기본값은 10회다.
  std::size_t max_health_checks = 10;
  // 주기 실행에서도 이미 Core를 시도한 snapshot을 반복하지 않을 수 있다.
  bool require_new_snapshot = false;
};

// 입력 ID 타입은 큐와 실행 설정에서 쓰는 타입과 같아야 한다.
template <class InputId>
// 어떤 계기로 계산을 시작했는지 assembler에 전달한다.
struct Trigger {
  // 기본 생성된 Trigger는 periodic tick을 뜻한다.
  ExecutionMode mode = ExecutionMode::Periodic;
  // OnInput의 기준 input이다. Tick에서는 기본값을 해석하지 않는다.
  InputId input{};
  // 방금 push한 sample을 정확히 찾기 위한 ID다. stamp가 같아도 구별할 수 있다.
  std::uint64_t sample_id = 0;
  // callback 입력의 원본 stamp(ns)다. 사용 여부와 정렬 방법은 assembler가 정한다.
  std::int64_t stamp_ns = 0;
};

// core ID의 enum/타입은 각 노드가 정의한다.
template <class CoreId>
// selector가 호환 core를 찾았는지와 찾은 ID를 전달한다.
struct Selection {
  // false면 이번에는 core를 호출하지 않는다. id의 기본값을 임의 선택하지 않는다.
  bool selected = false;
  // selected=true일 때만 의미가 있는 core ID다.
  CoreId id{};
};

// 노드 처리 결과다. 알고리즘 core의 반환 데이터와는 별개다.
enum class ProcessStatus {
  // 적재만/중복 건너뜀/정책 거부/core 미선택/core 미선택 상태다.
  Queued, Skipped, Rejected, NoCore,
  // 발행 호출 완료 상태다.
  Published
};
// 호출자와 IO 진단에 동일한 처리 결과를 전달한다.
struct ProcessReport {
  // 기본 생성 반환은 계산 없이 입력을 큐에 저장한 Queued를 뜻한다.
  ProcessStatus status = ProcessStatus::Queued;
  // 이번 snapshot에서 실제로 호출한 healthCheck 횟수다.
  std::size_t health_checks = 0;
  // 종료 원인을 사람이 확인할 수 있도록 남긴다.
  std::string reason;
};

}  // namespace node_pipeline
