#pragma once
#include <speed_filter/libraries/speed_limiter.hpp>
#include <set>
#include <string>
#include <utility>

namespace speed_filter {
struct SpeedLimit60Parameters : SpeedFilterParameters {
  std::set<std::string> speed_exemption_link_ids;
};
class SpeedLimit60 {
 public:
  explicit SpeedLimit60(SpeedLimit60Parameters parameters = {})
      : limiter_(parameters), exemption_links_(std::move(parameters.speed_exemption_link_ids)) {}
  explicit SpeedLimit60(SpeedFilterParameters parameters) : limiter_(parameters) {}

  morai_msgs::CtrlCmd process(const morai_msgs::CtrlCmd& command,
                             const interfaces::CompetitionVehicleStatus& status) {
    const auto limited = limiter_.process(command, status);
    // #REGULATION_SPEED_EXCEPTION
    // 일반 구간의 60km/h 제한과 공식 고주로 예외를 실제 Competition Status의
    // Link ID로 구분한다. 섹션 반경은 Link 시작/끝과 다르므로 판정에 쓰지 않는다.
    // 예외에서도 상위 MPC의 가속·제동·조향 명령은 그대로 보존하며, 새 가속을
    // 만들거나 명시적 제동을 해제하지 않는다. 제한기 내부 시간 상태는 매번
    // 갱신해 예외 진입·이탈 시 오래된 정지 타이머가 남지 않도록 한다.
    return exemption_links_.count(status.link_id) ? command : limited;
  }

 private:
  SpeedLimiter limiter_;
  std::set<std::string> exemption_links_;
};
}  // namespace speed_filter
