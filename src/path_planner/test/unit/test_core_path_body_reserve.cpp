#include <path_planner/cores/core_path_planner.hpp>
#include <gtest/gtest.h>

namespace {
TEST(CorePathBodyReserve, PreservesAValidForwardPathWhenTheRoadCannotProvideThePreferredReserve) {
  stier::PathParameters parameters;path_planner::CorePathPlanner core(parameters);
  const double wide=parameters.vehicle.width_m/2+1;
  const double narrow=parameters.vehicle.width_m/2+1.5*parameters.road_margin_m;
  const std::vector<stier::ReferencePoint> road{{{-20,0},-wide,wide},{{4,0},-wide,wide},
      {{5,0},-narrow,narrow},{{200,0},-narrow,narrow}};
  stier::EgoState ego;ego.stamp_sec=10;ego.speed_mps=5;
  stier::Predictions predictions;predictions.reference_stamp_sec=10;
  const auto path=core.plan(ego,road,predictions,stier::PlanningConstraints{});
  ASSERT_GE(path.size(),2u);EXPECT_GT(path.back().speed_mps,0);
  EXPECT_DOUBLE_EQ(path.front().speed_mps,ego.speed_mps);
  EXPECT_DOUBLE_EQ(path.front().acceleration_mps2,ego.acceleration_mps2);
  EXPECT_TRUE(core.checkPlannedPath(ego,road,predictions,stier::PlanningConstraints{},path));
}
}  // namespace
