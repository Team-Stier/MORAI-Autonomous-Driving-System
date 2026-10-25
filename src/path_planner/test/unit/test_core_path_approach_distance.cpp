#include <path_planner/cores/core_path_planner.hpp>
#include <gtest/gtest.h>
TEST(CorePathApproachDistance, ImpossibleBrakingStateCannotLatchCautionForARemotePedestrian) {
  stier::PathParameters p;p.normal_speed_limit_mps=60./3.6;
  p.pedestrian_caution_speed_mps=10./3.6;p.pedestrian_caution_distance_m=40;
  path_planner::CorePathPlanner core(p);
  const std::vector<stier::ReferencePoint> road{{{-20,0},-1.75,1.75},{{500,0},-1.75,1.75}};
  stier::Predictions predictions;predictions.reference_stamp_sec=10;
  stier::PredictedObject person;person.id=1;person.kind=stier::ObjectKind::Pedestrian;
  person.pose.position={300,0};person.length_m=.6;person.width_m=.6;person.source_stamp_sec=10;
  predictions.current.push_back(person);for(auto& future:predictions.future)future.push_back(person);
  stier::EgoState noisy;noisy.stamp_sec=10;noisy.speed_mps=.001;noisy.acceleration_mps2=-.5;
  EXPECT_TRUE(core.plan(noisy,road,predictions,stier::PlanningConstraints{}).empty());
  stier::EgoState moving;moving.stamp_sec=10;moving.speed_mps=9;
  const auto path=core.plan(moving,road,predictions,stier::PlanningConstraints{});
  ASSERT_GE(path.size(),2u);
  for(const auto& q:path)if(q.time_sec<=1)EXPECT_GE(q.speed_mps,moving.speed_mps-1e-8);
  EXPECT_TRUE(core.checkPlannedPath(moving,road,predictions,stier::PlanningConstraints{},path));
}
