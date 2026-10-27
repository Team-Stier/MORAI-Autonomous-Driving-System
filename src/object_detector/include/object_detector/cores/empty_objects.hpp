#pragma once
#include <interfaces/CompetitionVehicleStatus.h>
#include <interfaces/Objects.h>
#include <tuple>

namespace object_detector {
class EmptyObjects {
 public:
  std::tuple<interfaces::Objects, interfaces::CompetitionVehicleStatus> process(
      const interfaces::CompetitionVehicleStatus& competition) {
    return std::make_tuple(interfaces::Objects{}, competition);
  }
};
}  // namespace object_detector
