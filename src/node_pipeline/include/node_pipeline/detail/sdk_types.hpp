#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace node_pipeline {
enum class Kind { Missing, InvalidStamp, FutureStamp, FrameMismatch, NaN, Inf,
                  OutRangePositive, OutRangeNegative, Stale, ReceiveTimeout, Skew, Count };
struct HealthLimits {
  double lower = -1, upper = 1;
  std::int64_t stale_ns = 300000000, receive_timeout_ns = 300000000, max_skew_ns = 100000000;
  std::string frame = "map";
  bool check_stamp = true;
  bool check_receive_timeout = true;
  bool check_skew = true;
  void validate() const {
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper || frame.empty() || stale_ns <= 0 || receive_timeout_ns <= 0 || max_skew_ns <= 0)
      throw std::invalid_argument("invalid health limits");
  }
};
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

inline void checkNumber(double value, const HealthLimits& limits, std::vector<Kind>& issues) {
  if (std::isnan(value)) issues.push_back(Kind::NaN);
  else if (std::isinf(value)) issues.push_back(Kind::Inf);
  else if (value > limits.upper) issues.push_back(Kind::OutRangePositive);
  else if (value < limits.lower) issues.push_back(Kind::OutRangeNegative);
}
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

}
