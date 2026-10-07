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
#include <node_pipeline/detail/sdk_types.hpp>

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
