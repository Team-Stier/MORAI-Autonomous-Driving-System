#include <path_planner/cores/integrated_roundabout.hpp>
#include <gtest/gtest.h>
#include <path_planner/parameters/integrated_roundabout.hpp>
#include <path_planner/parameters/shared_map.hpp>
namespace pp=path_planner;
namespace {
struct Fixture {
 pp::IntegratedRoundaboutParameters p;nav_msgs::Odometry ego;interfaces::CompetitionVehicleStatus status;interfaces::Objects objects;pp::PredictedObjects predictions;
 pp::SharedMap shared{pp::SharedMapParameters{std::make_shared<lanelet::LaneletMap>()}};
 Fixture(){ros::Time::init();p.reference_before_entry_m=50;p.reference_after_entry_m=50;p.entry_check_length_m=8;p.desired_speed_mps=6;
 p.reference_lanelet_ids={100,200};p.stop_line_id=500;p.departure_lanelet_id=200;
 for(double y:{-2.,2.}){geometry_msgs::Point point;point.x=50;point.y=y;p.stop_line.push_back(point);}
 lanelet::Id id=1000;
 for(int i=0;i<2;++i){
  const double start=i==0?0:70,end=i==0?70:100;
  lanelet::LineString3d left(id++),right(id++),center(id++);
  for(double x:{start,end}){left.push_back(lanelet::Point3d(id++,x,2,0));right.push_back(lanelet::Point3d(id++,x,-2,0));center.push_back(lanelet::Point3d(id++,x,0,0));}
  lanelet::Lanelet lane(i==0?100:200,left,right);lane.setCenterline(center);shared.map->add(lane);
 }
 shared.map->add(lanelet::LineString3d(500,{lanelet::Point3d(id++,50,-2,0),lanelet::Point3d(id++,50,2,0)}));
 ego.header.stamp.fromSec(10);ego.pose.pose.orientation.w=1;ego.pose.pose.position.x=45.2;objects.header=ego.header;predictions.fill(objects);}

 void npc(double x,double vx=0){interfaces::Object o;o.id=1;o.bbox_pose.position.x=x;o.position=o.bbox_pose.position;o.bbox_pose.orientation.w=1;o.bbox_size.x=4.8;o.bbox_size.y=2.1;o.bbox_size.z=1.6;objects.objects={o};predictions.fill(objects);for(unsigned i=0;i<10;++i){predictions[i].objects[0].position.x+=vx*.5*(i+1);predictions[i].objects[0].bbox_pose.position=predictions[i].objects[0].position;}}
 pp::Message run(pp::IntegratedRoundabout& core){return core.process(ego,status,objects,predictions,ego.header.stamp);}
};
}
TEST(IntegratedRoundaboutCompatibility, ApproachesYieldWithoutAnEarlyFullStop) {Fixture f;f.ego.pose.pose.position.x=30;f.status.velocity_x_mps=5;pp::IntegratedRoundabout c(f.p,f.shared);const auto path=f.run(c);ASSERT_GT(path.points.size(),20u);EXPECT_GT(path.points.front().velocity,4);EXPECT_DOUBLE_EQ(path.points.back().velocity,0);}
TEST(IntegratedRoundaboutCompatibility, StopsOnceBeforeAcceptingAnEmptyGap) {Fixture f;pp::IntegratedRoundabout c(f.p,f.shared);f.ego.pose.pose.position.x=43;f.status.velocity_x_mps=1;const auto approach=f.run(c);EXPECT_GT(approach.points.front().velocity,0);f.ego.pose.pose.position.x=45.2;f.status.velocity_x_mps=0;const auto go=f.run(c);EXPECT_GT(go.points.front().velocity,1);f.ego.pose.pose.position.x=45.6;f.status.velocity_x_mps=.4;EXPECT_GT(f.run(c).points.front().velocity,1);}
TEST(IntegratedRoundaboutCompatibility, OccupiedEntryWaitsAtTheYieldTargetAndReleasesAfterItClears) {Fixture f;f.npc(50);pp::IntegratedRoundabout c(f.p,f.shared);const auto wait=f.run(c);EXPECT_DOUBLE_EQ(wait.points.front().velocity,0);f.objects.objects.clear();f.predictions.fill(f.objects);EXPECT_GT(f.run(c).points.front().velocity,1);}
TEST(IntegratedRoundaboutCompatibility, RearFollowingVehicleDoesNotCauseAStopInsideTheRoundabout) {Fixture f;f.ego.pose.pose.position.x=60;f.status.velocity_x_mps=4;f.npc(50,7);pp::IntegratedRoundabout c(f.p,f.shared);const auto go=f.run(c);for(const auto& p:go.points)EXPECT_GT(p.velocity,3);}
TEST(IntegratedRoundaboutCompatibility, EnteredVehicleStopsForAStationaryBodyImmediatelyAhead) {Fixture f;f.ego.pose.pose.position.x=60;f.status.velocity_x_mps=1;f.npc(64);pp::IntegratedRoundabout c(f.p,f.shared);const auto path=f.run(c);EXPECT_DOUBLE_EQ(path.points.front().velocity,0);}

TEST(IntegratedRoundaboutCompatibility, AcceptedGapIsNotRevokedByAFarUnmeasuredTrack) {
 Fixture f;pp::IntegratedRoundabout core(f.p,f.shared);ASSERT_GT(f.run(core).points.front().velocity,1);
 f.ego.pose.pose.position.x=45.4;f.status.velocity_x_mps=.5;f.npc(75);
 const auto path=f.run(core);ASSERT_GT(path.points.size(),10u);EXPECT_GT(path.points.front().velocity,1);
}

TEST(IntegratedRoundaboutCompatibility, EntryRemainsCommittedWhileMatchingTheMovingLeader) {
 Fixture f;pp::IntegratedRoundabout core(f.p,f.shared);ASSERT_GT(f.run(core).points.front().velocity,1);
 f.ego.pose.pose.position.x=60;f.status.velocity_x_mps=3;f.npc(72,3);
 for(auto& frame:f.predictions){frame.objects.front().velocity_available=true;frame.objects.front().velocity.x=3;}
 f.objects.objects.front().velocity_available=true;f.objects.objects.front().velocity.x=3;
 const auto following=f.run(core);EXPECT_GT(following.points.front().velocity,3);EXPECT_LT(following.points.front().velocity,6);
 f.objects.objects.clear();f.predictions.fill(f.objects);EXPECT_DOUBLE_EQ(f.run(core).points.front().velocity,6);
}
TEST(IntegratedRoundaboutCompatibility, AdjacentCirculatingBodyDoesNotSlowTheCommittedVehicle) {
 Fixture f;f.ego.pose.pose.position.x=60;f.status.velocity_x_mps=3;f.npc(70);
 f.objects.objects.front().position.y=4;f.objects.objects.front().bbox_pose.position.y=4;f.predictions.fill(f.objects);
 pp::IntegratedRoundabout core(f.p,f.shared);EXPECT_DOUBLE_EQ(f.run(core).points.front().velocity,6);
}

TEST(IntegratedRoundaboutCompatibility, EntryDecisionChecksTheCommittedCruiseInsteadOfASlowerFollowingCandidate) {
 Fixture f;f.npc(55,2);f.objects.objects.front().velocity_available=true;f.objects.objects.front().velocity.x=2;
 for(auto& frame:f.predictions){frame.objects.front().velocity_available=true;frame.objects.front().velocity.x=2;}
 pp::IntegratedRoundabout core(f.p,f.shared);const auto wait=f.run(core);EXPECT_DOUBLE_EQ(wait.points.front().velocity,0);
}

TEST(IntegratedRoundaboutCompatibility, UnmeasuredMovingBodyDoesNotCreateAZeroHeadwayCommandAtADistance) {
 Fixture f;f.ego.pose.pose.position.x=60;f.status.velocity_x_mps=6;f.npc(70);
 pp::IntegratedRoundabout core(f.p,f.shared);const auto path=f.run(core);EXPECT_GT(path.points.front().velocity,3);
}

TEST(IntegratedRoundaboutCompatibility, LoadsTheActiveLaneletMapAndItsIndependentStopLineAsset) {
 ros::Time::init();const std::string root=NP_PACKAGE_DIR;
 auto read=[&](const std::string& n){return node_pipeline::ParamReader::fromFile(root+"/config/path_planner/parameters/"+n+".yaml",root);};
 pp::SharedMap shared(pp::loadSharedMapParameters(read("shared_map")));
 const auto p=pp::loadIntegratedRoundaboutParameters(read("integrated_roundabout"));ASSERT_EQ(p.stop_line.size(),2u);
 pp::IntegratedRoundabout core(p,shared);nav_msgs::Odometry ego;ego.header.stamp.fromSec(10);ego.pose.pose.orientation.w=1;
 ego.pose.pose.position.x=-99.4;ego.pose.pose.position.y=300;
 pp::PredictedObjects predictions;const auto path=core.process(ego,{}, {},predictions,ego.header.stamp);
 ASSERT_GT(path.points.size(),30u);EXPECT_GT(path.points.front().velocity,2);EXPECT_DOUBLE_EQ(path.points.back().velocity,0);
}

TEST(IntegratedRoundaboutRollingEntry, EmptyEntryDoesNotRequireACompleteStop) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=2;
 pp::IntegratedRoundabout core(f.p,f.shared);const auto path=f.run(core);
 ASSERT_GT(path.points.size(),20u);
 for(const auto& point:path.points)EXPECT_DOUBLE_EQ(point.velocity,6);
}

TEST(IntegratedRoundaboutRollingEntry, OccupiedEntryPreservesTheStopProfile) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=2;f.npc(50);
 f.objects.objects.front().velocity_available=true;
 for(auto& frame:f.predictions)frame.objects.front().velocity_available=true;
 pp::IntegratedRoundabout core(f.p,f.shared);const auto path=f.run(core);
 ASSERT_GT(path.points.size(),20u);EXPECT_LT(path.points.front().velocity,3);
 EXPECT_DOUBLE_EQ(path.points.back().velocity,0);
}

TEST(IntegratedRoundaboutRollingEntry, ClearTrafficRequiresFreshGapConfirmation) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=2;f.npc(75);
 f.objects.objects.front().position.y=10;f.objects.objects.front().bbox_pose.position.y=10;
 f.objects.objects.front().velocity_available=true;f.predictions.fill(f.objects);
 pp::IntegratedRoundabout core(f.p,f.shared);
 EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
 for(int i=0;i<5;++i)EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
 f.ego.header.stamp.fromSec(10.05);f.objects.header=f.ego.header;f.predictions.fill(f.objects);
 EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
 f.ego.header.stamp.fromSec(10.2);f.objects.header=f.ego.header;f.predictions.fill(f.objects);
 EXPECT_DOUBLE_EQ(f.run(core).points.front().velocity,6);
}

TEST(IntegratedRoundaboutRollingEntry, UnmeasuredTrafficDoesNotAuthorizeEntry) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=2;f.npc(75);
 pp::IntegratedRoundabout core(f.p,f.shared);
 EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
}

TEST(IntegratedRoundaboutRollingEntry, FastApproachStillDeceleratesBeforeDeciding) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=4;
 pp::IntegratedRoundabout core(f.p,f.shared);
 EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
}

TEST(IntegratedRoundaboutRollingEntry, InsufficientForecastDoesNotAuthorizeEntry) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=2;
 f.ego.header.stamp.fromSec(14.7);
 pp::IntegratedRoundabout core(f.p,f.shared);
 EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
}

TEST(IntegratedRoundaboutRollingEntry, CrossingDuringPedalResponseStillRequiresStopping) {
 Fixture f;f.ego.pose.pose.position.x=44;f.status.velocity_x_mps=1;f.npc(48.5);
 auto& object=f.objects.objects.front();object.position.y=-4.4;
 object.bbox_pose.position=object.position;object.velocity_available=true;
 object.velocity.y=8;object.bbox_pose.orientation.w=std::sqrt(.5);
 object.bbox_pose.orientation.z=std::sqrt(.5);
 f.predictions.fill(f.objects);
 for(std::size_t i=0;i<f.predictions.size();++i) {
  auto& future=f.predictions[i].objects.front();future.position.y+=8*.5*(i+1);
  future.bbox_pose.position=future.position;
 }
 pp::IntegratedRoundabout core(f.p,f.shared);
 EXPECT_DOUBLE_EQ(f.run(core).points.back().velocity,0);
}
