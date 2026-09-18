#include <limits>

#include <gtest/gtest.h>

#include "morai_path_tracking/common/control_timing_bounds.hpp"

namespace morai_path_tracking {
namespace {

TEST(ControlTimingBounds, RejectsValuesOutsideInclusiveFiniteRange) {
  const ControlTimingBounds bounds(0.005, 0.10);

  EXPECT_FALSE(bounds.contains(0.004999));
  EXPECT_TRUE(bounds.contains(0.005));
  EXPECT_TRUE(bounds.contains(1.0 / 30.0));
  EXPECT_TRUE(bounds.contains(0.10));
  EXPECT_FALSE(bounds.contains(0.100001));
  EXPECT_FALSE(bounds.contains(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(bounds.contains(std::numeric_limits<double>::infinity()));
}

TEST(ControlTimingBounds, RejectsInvalidBounds) {
  EXPECT_THROW(ControlTimingBounds(0.0, 0.10), std::invalid_argument);
  EXPECT_THROW(ControlTimingBounds(0.005, 0.0), std::invalid_argument);
  EXPECT_THROW(ControlTimingBounds(0.10, 0.005), std::invalid_argument);
  EXPECT_THROW(ControlTimingBounds(std::numeric_limits<double>::quiet_NaN(),
                                   0.10),
               std::invalid_argument);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
