#include "morai_path_tracking/common/control_timing_bounds.hpp"

#include <cmath>
#include <stdexcept>

namespace morai_path_tracking {

ControlTimingBounds::ControlTimingBounds(double minimum_dt_sec,
                                         double maximum_dt_sec)
    : minimum_dt_sec_(minimum_dt_sec), maximum_dt_sec_(maximum_dt_sec) {
  if (!std::isfinite(minimum_dt_sec_) || minimum_dt_sec_ <= 0.0 ||
      !std::isfinite(maximum_dt_sec_) || maximum_dt_sec_ <= 0.0 ||
      minimum_dt_sec_ > maximum_dt_sec_) {
    throw std::invalid_argument(
        "control dt bounds must be finite, positive, and ordered");
  }
}

bool ControlTimingBounds::contains(double dt_sec) const {
  return std::isfinite(dt_sec) && dt_sec >= minimum_dt_sec_ &&
         dt_sec <= maximum_dt_sec_;
}

}  // namespace morai_path_tracking
