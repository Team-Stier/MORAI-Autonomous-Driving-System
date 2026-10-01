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

// callback/process는 직렬 호출해야 한다. 각 helper는 노드가 값 멤버로 소유한다.
// snapshot의 작업용 data만 보정하고 buffer에 저장된 원본 입력은 변경하지 않는다.
// Buffer/Assembler/Checker/Policy는 노드별 입력 타입과 검사·보정 규칙을 제공한다.
template <class Buffer, class Assembler, class Checker, class Policy,
          // Selector/CoreSet/IO는 선택 규칙, 지속 계산 상태, 외부 입출력을 각각 맡는다.
          class Selector, class CoreSet, class HeaderInjector, class IO>
// 공통 순서만 구현한다. 센서 종류나 추정기 알고리즘은 이 템플릿에 넣지 않는다.
class PipelineNode {
 public:
  // Buffer가 공개한 입력 식별자 타입을 callback/trigger 전반에서 공유한다.
  using InputId = typename Buffer::InputId;
  // 해당 input ID에 맞는 실행 설정 타입의 별칭이다.
  using Config = ExecutionConfig<InputId>;

  // 필요한 helper 인스턴스를 완성해 전달한다. 런타임 factory나 inputSpecs 조회는 없다.
  PipelineNode(Config config, Buffer buffer, Assembler assembler, Checker checker,
               // Policy/Selector/CoreSet/IO도 외부에서 전문화한 객체를 받는다.
               Policy policy, Selector selector, CoreSet cores, HeaderInjector header_injector, IO io)
      // 설정을 복사하고 buffer/assembler의 소유권을 노드 멤버로 옮긴다.
      : config_(config), buffer_(std::move(buffer)), assembler_(std::move(assembler)),
        // checker의 시작 시각과 policy 설정을 함께 보관한다.
        checker_(std::move(checker)), policy_(std::move(policy)),
        // selector·core 상태·IO를 보관한다. 매 프레임 core를 새로 만들지 않는다.
        selector_(std::move(selector)), cores_(std::move(cores)), header_injector_(std::move(header_injector)), io_(std::move(io)) {
    // health 검사를 0회로 설정하면 정책을 판단할 수 없으므로 생성 실패로 처리한다.
    if (config_.max_health_checks == 0 ||
        // enum 강제 변환 등으로 들어온 잘못된 실행 모드도 검사한다.
        (config_.mode != ExecutionMode::Periodic &&
         // 두 지원 모드에 모두 속하지 않으면 무효다.
         config_.mode != ExecutionMode::OnInput) ||
        // callback 실행일 때에만 기준 input의 등록 여부를 확인한다.
        (config_.mode == ExecutionMode::OnInput &&
         // 등록되지 않은 trigger를 지정하면 계산이 시작되지 않을 수 있어 거부한다.
         !buffer_.containsInput(config_.trigger_input))) {
      // 초기화 호출자가 설정 실패를 알 수 있도록 예외를 던진다.
      throw std::invalid_argument("invalid execution configuration");
    }
  }

  // 입력마다 서로 다른 메시지 타입을 받을 수 있다. 실제 수용은 Buffer의 overload가 정한다.
  template <class InputTag, class Message>
  // ROS adapter나 다른 호출자가 원본 입력을 공통 pipeline으로 전달하는 진입점이다.
  ProcessReport onInput(InputTag input, const Message& message) {
    // 내용은 검사하지 않고 원본과 수신 시각을 저장한 뒤 새 sample의 메타데이터를 받는다.
    const auto sample = buffer_.pushInput(input, message, io_.readClocks());
    // OnInput이면서 설정된 기준 input인 경우에만 같은 callback 안에서 계산한다.
    if (config_.mode == ExecutionMode::OnInput && static_cast<InputId>(input) == config_.trigger_input) {
      // 방금 적재한 sample ID/stamp를 assembler의 기준으로 넘긴다.
      return process({ExecutionMode::OnInput, static_cast<InputId>(input), sample.id, sample.stamp_ns});
    }
    // 보조 input 또는 periodic 모드는 Queued로 끝낸다. finish 진단은 호출하지 않는다.
    return {};
  }

  // section 메시지의 실제 타입도 실행 패키지가 정한다.
  template <class Message>
  // section 수신은 raw 상태 저장 진입점이다. 그 자체로 계산을 시작하지 않는다.
  void onSection(const Message& message) {
    // 최신 section 원본을 buffer에 맡긴다. 유효성 판단은 이후 checker가 담당한다.
    buffer_.storeRawSection(message);
  }

  // Periodic timer가 계산을 요청하는 공통 진입점이다.
  ProcessReport onPeriodic() {
    // OnInput 노드에 실수로 tick을 보내도 계산을 중복 시작하지 않는다.
    if (config_.mode != ExecutionMode::Periodic) {
      // 비활성 trigger였음을 호출자와 진단에 알린다.
      return finish(ProcessStatus::Skipped, 0, "periodic trigger disabled");
    }
    // 기본 Trigger는 Periodic이다. 같은 process를 callback 경로와 공유한다.
    return process({});
  }

 private:
  // 조립→health/policy 반복→선택→계산→헤더 주입→발행의 실제 실행 순서다.
  ProcessReport process(const Trigger<InputId>& trigger) {
    // 검사 횟수는 snapshot별 지역 변수다. 다음 프레임에는 다시 0부터 시작한다.
    std::size_t checks = 0;
    // 루프 바깥에서 한 번만 조립한다. 이후 Continue에서 큐를 다시 읽지 않는다.
    auto snapshot = assembler_.assemble(buffer_, trigger);
    // 입력이 전혀 없어도 assembler를 먼저 호출해 빈 snapshot을 받는다.
    // callback 모드에서 이미 core를 시도한 bundle이면 계산을 다시 하지 않는다.
    if ((trigger.mode == ExecutionMode::OnInput || config_.require_new_snapshot) && !snapshot.is_new) {
      // 중복 snapshot은 health 검사 전 종료하므로 checks는 0이다.
      return finish(ProcessStatus::Skipped, checks, "bundle already attempted");
    }
    // 모든 health 재검사는 루프 진입 시각을 공유한다. 보정 중 시간 경과로 판정 기준이 바뀌지 않는다.
    const auto health_clocks = io_.readClocks();
    // 마지막 검사 결과를 루프 밖의 core 선택에 사용한다.
    decltype(checker_.healthCheck(snapshot.data, snapshot.context, health_clocks)) health{};
    // AllGreen 또는 dismiss 오류만 남았는지 기록한다.
    bool settled = false;
    // 같은 데이터와 시각으로 검사하고 보정한다.
    for (; checks < config_.max_health_checks;) {
      // 검사 횟수 한도는 snapshot마다 적용한다.
      ++checks;
      // 이전 보정이 반영된 데이터의 오류를 다시 산출한다.
      health = checker_.healthCheck(snapshot.data, snapshot.context, health_clocks);
      // 이번 검사 결과를 진단한다.
      io_.reportHealth(health, checks);
      // AllGreen 또는 dismiss issue만 남았으면 루프를 빠져나간다.
      if (!policy_.apply(snapshot.data, health)) {
        // 마지막 회차에서 완료됐어도 정상적으로 수용한다.
        settled = true;
        break;
      }
      // 보정이 수행됐다면 본문 종료 후 자연스럽게 다음 검사 회차로 간다.
    }
    // 한도까지 보정해도 완료를 확인하지 못하면 현재 snapshot을 폐기한다.
    if (!settled) return finish(ProcessStatus::Rejected, checks, "health check limit");
    // 루프 종료 후 마지막 health로 core를 고른다.
    const auto selection = selector_.select(health.section, health);
    // 현재 data와 호환되는 core가 하나도 없는 경우다.
    if (!selection.selected) {
      // 기본 ID를 임의 사용하지 않고 계산·발행을 보류한다.
      return finish(ProcessStatus::NoCore, checks, "no compatible core");
    }
    // 선택된 Core의 필수 입력이 누락되면 계산과 발행 없이 종료한다.
    if (!snapshot.data.hasInputs(cores_.requiredInputs(selection.id))) {
      return finish(ProcessStatus::NoCore, checks, "required core input missing");
    }
    // core를 호출하기 전에 시도 token을 기록한다. 실패/예외도 이미 한 계산 시도로 본다.
    // OnInput 중복 실행 판정에 사용할 token을 assembler에 저장한다.
    assembler_.markAttempted(snapshot.token);
    // Core가 목표 발행 메시지의 payload를 완성한다.
    auto message = cores_.process(selection.id, snapshot.data);
    // Core 밖에서 발행 헤더만 주입한다. payload는 변경하지 않는다.
    header_injector_.injectHeader(message, snapshot.context);
    // 완성된 메시지를 발행한다. 하류 수신 확인을 뜻하지는 않는다.
    // IO는 헤더 주입을 담당하지 않는다.
    // IO에는 완성된 메시지만 전달한다.
    io_.publish(message);
    // 이번 process를 완료한다.
    return finish(ProcessStatus::Published, checks, {});
  }

  // 모든 종료 경로에서 동일한 ProcessReport를 만들어 진단과 반환에 사용한다.
  ProcessReport finish(ProcessStatus status, std::size_t checks,
                       // 이 함수는 예외를 내보내지 않는 계약이다. IO::report도 예외를 던지면 안 된다.
                       const std::string& reason) noexcept {
    // 종료 상태·실제 검사 횟수·사유를 한 결과로 묶는다.
    ProcessReport report{status, checks, reason};
    // IO의 동기 진단으로 전달한다. noexcept를 지킬 책임은 IO 구현에 있다.
    io_.report(report);  // IO::report must be noexcept.
    // callback 호출자 또는 process 호출자에게 같은 결과를 반환한다.
    return report;
  }

  // 주기/callback 선택, trigger input, health 검사 한도를 저장한다.
  Config config_;
  // 노드별 원본 입력 큐와 수신 메타데이터를 소유한다.
  Buffer buffer_;
  // stamp 정렬·snapshot 구성·bundle 시도 이력을 소유한다.
  Assembler assembler_;
  // 입력 검사을 소유한다.
  Checker checker_;
  // 오류별 데이터 보정 규칙과 우선순위를 소유한다.
  Policy policy_;
  // section 기본 선택과 health 기반 예외 선택 규칙을 소유한다.
  Selector selector_;
  // core 객체를 계속 보관한다. EKF를 넣으면 x/P 등도 이 객체 내부에 남는다.
  CoreSet cores_;
  // 외부 시계·발행·진단을 담당한다. 공통 라이브러리는 ROS 타입을 직접 알지 않는다.
  // Core 반환 메시지에 헤더를 주입하는 helper를 소유한다.
  HeaderInjector header_injector_;
  IO io_;
};

}  // namespace node_pipeline
