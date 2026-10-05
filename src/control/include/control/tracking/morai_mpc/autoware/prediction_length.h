#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace control { namespace morai_mpc { namespace autoware {

inline int moraiPredictionSteps(int base, int maximum, double dt, double speed, double metres) {
  if (base<2 || !std::isfinite(dt) || dt<=0. || !std::isfinite(speed) ||
      !std::isfinite(metres) || metres<0. || (metres>0. && maximum<base))
    throw std::invalid_argument("Invalid prediction length parameters");
  if (metres==0.) return base;
  const double needed=std::ceil(metres/(std::max(.5,std::abs(speed))*dt))+1.;
  // Five-step blocks reduce repeated solver resizes as speed changes.
  return std::max(base,static_cast<int>(std::min(double(maximum),5.*std::ceil(needed/5.))));
}

}}}  // namespace control::morai_mpc::autoware
