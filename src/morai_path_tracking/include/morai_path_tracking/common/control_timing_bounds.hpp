#pragma once

namespace morai_path_tracking {

class ControlTimingBounds {
 public:
  ControlTimingBounds(double minimum_dt_sec, double maximum_dt_sec);

  bool contains(double dt_sec) const;

 private:
  double minimum_dt_sec_;
  double maximum_dt_sec_;
};

}  // namespace morai_path_tracking
