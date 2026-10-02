#pragma once
#include <speed_filter/types.hpp>
#include <speed_filter/cores/speed_limit60.hpp>
#include <speed_filter/cores/no_speed_limit.hpp>
#include <array>
#include <stdexcept>
namespace speed_filter {
enum class CoreId { SpeedLimit60, NoSpeedLimit };
struct CoreSet {
  SpeedLimit60 speed_limit60;
  NoSpeedLimit no_speed_limit;

  std::array<std::size_t, 2> requiredInputs(CoreId id) const {
    switch (id) {
      case CoreId::SpeedLimit60:
      case CoreId::NoSpeedLimit: return {{0, 1}};
    }
    throw std::invalid_argument("unknown speed filter core");
  }
  morai_msgs::CtrlCmd process(CoreId id, const Data& data) {
    switch (id) {
      case CoreId::SpeedLimit60:
        return speed_limit60.process(data.get<0>().message(), data.get<1>().message());
      case CoreId::NoSpeedLimit:
        return no_speed_limit.process(data.get<0>().message(), data.get<1>().message());
    }
    throw std::invalid_argument("unknown speed filter core");
  }
};
}  // namespace speed_filter
