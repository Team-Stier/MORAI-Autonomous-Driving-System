#include <path_planner/cores/integrated_highway_merge.hpp>
#include <path_planner/parameters/integrated_highway_merge.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <gtest/gtest.h>
namespace {
namespace pp=path_planner;
struct HighwayLidarFixture {
 pp::SharedMap shared{pp::SharedMapParameters{std::make_shared<lanelet::LaneletMap>()}};
 pp::HighwayMergeMissionParameters params;pp::Route route;long id=1;
 lanelet::LineString3d edge(double y){lanelet::LineString3d line(id++,{lanelet::Point3d(id++,-20,y,0),lanelet::Point3d(id++,600,y,0)});line.attributes()["type"]="line_thin";line.attributes()["subtype"]="dashed";return line;}
 HighwayLidarFixture(){params.gap_confirmation_sec=.3;ros::Time::init();auto shared_edge=edge(1.75);lanelet::Lanelet source(id++,shared_edge,edge(-1.75)),target(id++,edge(5.25),shared_edge);
  for(auto lane:{source,target}){lane.attributes()["subtype"]="road";lane.attributes()["location"]="urban";lane.attributes()["one_way"]="yes";shared.map->add(lane);}
  route=lanelet::routing::LaneletPath({source,target});
 }
 pp::IntegratedHighwayMergeParameters integrated()const{pp::IntegratedHighwayMergeParameters p;p.mission=params;return p;}
 nav_msgs::Odometry ego(double t,double x=0,double y=0){nav_msgs::Odometry e;e.header.stamp.fromSec(t);e.pose.pose.position.x=x;e.pose.pose.position.y=y;e.pose.pose.orientation.w=1;e.twist.twist.linear.x=10;return e;}
 interfaces::CompetitionVehicleStatus status(){interfaces::CompetitionVehicleStatus s;s.velocity_x_mps=10;return s;}
 interfaces::Objects objects(double t,double x=1000,double vx=10,bool known=true,double y=3.5){interfaces::Objects o;o.header.stamp.fromSec(t);if(x!=1000){interfaces::Object a;a.id=1;a.bbox_pose.position.x=x;a.bbox_pose.position.y=y;a.bbox_pose.orientation.w=1;a.bbox_size.x=4;a.bbox_size.y=1.8;a.bbox_size.z=1.5;a.velocity_available=known;a.velocity.x=vx;o.objects.push_back(a);}return o;}
};
}

TEST(IntegratedHighwayMerge, ConfiguredAccOpensTenMetresWithoutDroppingToCrawl) {
 HighwayLidarFixture f;const auto root=std::string(NP_PACKAGE_DIR);
 f.params=pp::loadIntegratedHighwayMergeParameters(node_pipeline::ParamReader::fromFile(
     root+"/config/path_planner/parameters/integrated_highway_merge.yaml",root)).mission;
 f.params.mission_end_y_m=1;
 f.params.entry_speed_mps=f.params.cruise_speed_mps;
 pp::Route route=lanelet::routing::LaneletPath({(*f.route)[0]});pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 double x=0,speed=60./3.6,min_speed=speed,gap=8;
 for(int tick=0;tick<300;++tick) {
  const double elapsed=.1*tick,time=1+elapsed;
  const double leader=8+f.params.vehicle_front_m+2+(60./3.6)*elapsed;
  auto status=f.status();status.velocity_x_mps=speed;auto e=f.ego(time,x);e.twist.twist.linear.x=speed;
  const auto path=core.process(e,status,f.objects(time,leader,60./3.6,true,0),route);
  ASSERT_FALSE(path.points.empty());
  speed=std::max(0.,speed+.1*std::max(-3.,std::min(1.,(path.points.front().velocity-speed)/.4)));
  min_speed=std::min(min_speed,speed);x+=.1*speed;
  gap=leader+(60./3.6)*.1-x-f.params.vehicle_front_m-2;
 }
 EXPECT_GT(min_speed,55./3.6);EXPECT_NEAR(gap,10,1.0);
}

TEST(IntegratedHighwayMerge, ClosingLeaderSettlesAtTenMetresWithDelayedSpeedResponse) {
 HighwayLidarFixture f;const auto root=std::string(NP_PACKAGE_DIR);
 f.params=pp::loadIntegratedHighwayMergeParameters(node_pipeline::ParamReader::fromFile(
     root+"/config/path_planner/parameters/integrated_highway_merge.yaml",root)).mission;
 f.params.mission_end_y_m=1;f.params.entry_speed_mps=f.params.cruise_speed_mps;
 pp::Route route=lanelet::routing::LaneletPath({(*f.route)[0]});
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 const double lead_speed=11,initial_gap=16.7,dt=.1;
 double x=0,speed=13,min_gap=initial_gap,min_speed=speed,gap=initial_gap;
 std::deque<double> commands(8,speed);
 for(int tick=0;tick<300;++tick) {
  const double time=1+dt*tick;
  const double leader=initial_gap+f.params.vehicle_front_m+2+lead_speed*dt*tick;
  auto status=f.status();status.velocity_x_mps=speed;
  auto ego=f.ego(time,x);ego.twist.twist.linear.x=speed;
  const auto path=core.process(ego,status,f.objects(time,leader,lead_speed,true,0),route);
  ASSERT_FALSE(path.points.empty());
  commands.push_back(path.points.front().velocity);
  const double acceleration=std::max(-3.,std::min(1.,(commands.front()-speed)/.4));
  commands.pop_front();
  x+=speed*dt+.5*acceleration*dt*dt;speed=std::max(0.,speed+acceleration*dt);
  gap=leader+lead_speed*dt-x-f.params.vehicle_front_m-2;
  min_gap=std::min(min_gap,gap);min_speed=std::min(min_speed,speed);
 }
 RecordProperty("minimum_gap_m",std::to_string(min_gap));
 RecordProperty("minimum_speed_mps",std::to_string(min_speed));
 RecordProperty("final_gap_m",std::to_string(gap));
 EXPECT_GT(min_gap,9.5);EXPECT_GT(min_speed,lead_speed-.5);
 EXPECT_NEAR(gap,10,.3);EXPECT_NEAR(speed,lead_speed,.1);
}

TEST(IntegratedHighwayMerge, OneFalseZeroVelocityAgreesWithMovingPositionHistory) {
 HighwayLidarFixture f;f.params.mission_end_y_m=1;f.params.entry_speed_mps=f.params.cruise_speed_mps;
 pp::Route route=lanelet::routing::LaneletPath({(*f.route)[0]});pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 auto status=f.status();status.velocity_x_mps=60./3.6;
 for(int tick=0;tick<10;++tick) {
  const double time=1+.1*tick,x=(60./3.6)*.1*tick;
  const auto path=core.process(f.ego(time,x),status,
      f.objects(time,x+20+f.params.vehicle_front_m+2,tick==5?0:60./3.6,true,0),route);
  ASSERT_FALSE(path.points.empty());EXPECT_GT(path.points.front().velocity,55./3.6)<<tick;
 }
}

TEST(IntegratedHighwayMerge, StationaryObstacleStillCommandsImmediateStop) {
 HighwayLidarFixture f;f.params.mission_end_y_m=1;f.params.entry_speed_mps=f.params.cruise_speed_mps;
 pp::Route route=lanelet::routing::LaneletPath({(*f.route)[0]});pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 auto status=f.status();status.velocity_x_mps=60./3.6;
 for(int tick=0;tick<5;++tick) {
  const double time=1+.1*tick;
  const auto path=core.process(f.ego(time),status,
      f.objects(time,20+f.params.vehicle_front_m+2,0,true,0),route);
  ASSERT_FALSE(path.points.empty());EXPECT_DOUBLE_EQ(path.points.front().velocity,0);
 }
}

TEST(IntegratedHighwayMerge, ClearedEmergencyDoesNotKeepRequestingAStopWhileMoving) {
 HighwayLidarFixture f;f.params.mission_end_y_m=1;f.params.entry_speed_mps=f.params.cruise_speed_mps;
 pp::Route route=lanelet::routing::LaneletPath({(*f.route)[0]});pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 auto status=f.status();status.velocity_x_mps=60./3.6;
 const auto stopped=core.process(f.ego(1),status,f.objects(1,20+f.params.vehicle_front_m+2,0,true,0),route);
 ASSERT_FALSE(stopped.points.empty());EXPECT_DOUBLE_EQ(stopped.points.front().velocity,0);
 const auto clear=core.process(f.ego(1.1),status,f.objects(1.1),route);
 ASSERT_FALSE(clear.points.empty());EXPECT_GE(clear.points.front().velocity,60./3.6);
}

TEST(IntegratedHighwayMerge, OpenRearGapBeforeStartingLateralMotion) {
 HighwayLidarFixture f;f.params.rear_gap_m=20;f.params.rear_headway_sec=1.2;f.params.rear_clear_sec=3;
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);auto status=f.status();status.velocity_x_mps=20;
 for(int frame=0;frame<10;++frame) {
  const double time=1+.1*frame,ego_x=20*.1*frame;
  const auto path=core.process(f.ego(time,ego_x),status,f.objects(time,-5+5*.1*frame,5,true),f.route);
  ASSERT_FALSE(path.points.empty());
  for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
  EXPECT_GT(path.points.front().velocity,10);
 }
}
TEST(IntegratedHighwayMerge, EmptyTargetLaneDoesNotWaitForAnObjectToAppear) {
 HighwayLidarFixture f;pp::IntegratedHighwayMerge core(f.integrated(),f.shared);bool changed=false;
 for(int frame=0;frame<8;++frame) {
  const double time=1+.1*frame;
  const auto path=core.process(f.ego(time),f.status(),f.objects(time),f.route);
  ASSERT_FALSE(path.points.empty());EXPECT_GT(path.points.front().velocity,0);
  for(const auto& point:path.points)changed|=point.pose.pose.position.y>1;
 }
 EXPECT_TRUE(changed);
}

TEST(IntegratedHighwayMerge, RecedingRearClearsBeforePredictedLaneOverlap) {
 HighwayLidarFixture f;
 const auto root=std::string(NP_PACKAGE_DIR);
 auto tuned=pp::loadIntegratedHighwayMergeParameters(node_pipeline::ParamReader::fromFile(
     root+"/config/path_planner/parameters/integrated_highway_merge.yaml",root));
 pp::IntegratedHighwayMerge current(tuned,f.shared);
 auto status=f.status();status.velocity_x_mps=13;
 bool changed=false;
 for(int frame=0;frame<6;++frame) {
  const double elapsed=.1*frame,time=1+elapsed;
  const auto e=f.ego(time,13*elapsed);const auto objects=f.objects(time,-10+5*elapsed,5,true);
  const auto path=current.process(e,status,objects,f.route);
  ASSERT_FALSE(path.points.empty());
  for(const auto& point:path.points)changed|=point.pose.pose.position.y>1;
 }
 EXPECT_TRUE(changed);
}

TEST(IntegratedHighwayMerge, ConfiguredRearGapStillBlocksCloseAndFastApproachingVehicles) {
 HighwayLidarFixture f;
 const auto root=std::string(NP_PACKAGE_DIR);
 const auto tuned=pp::loadIntegratedHighwayMergeParameters(node_pipeline::ParamReader::fromFile(
     root+"/config/path_planner/parameters/integrated_highway_merge.yaml",root));
 for(const auto& scenario:std::vector<std::pair<double,double>>{{-8,10},{-35,20}}) {
  pp::IntegratedHighwayMerge core(tuned,f.shared);
  for(int frame=0;frame<8;++frame) {
   const double elapsed=.1*frame,time=1+elapsed;
   const auto path=core.process(f.ego(time,10*elapsed),f.status(),
       f.objects(time,scenario.first+scenario.second*elapsed,scenario.second,true),f.route);
   ASSERT_FALSE(path.points.empty());
   for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
  }
 }
}


TEST(IntegratedHighwayMerge, DistantUnknownRearCanRemainOutsideReachableMergeCorridor) {
 HighwayLidarFixture f;pp::IntegratedHighwayMerge core(f.integrated(),f.shared);bool changed=false;
 for(int frame=0;frame<6;++frame) {
  const double t=1+.1*frame;
  // New IDs prevent a fitted speed from masking the unknown-motion case.
  auto objects=f.objects(t,-180,0,false);objects.objects.front().id=100+frame;
  const auto path=core.process(f.ego(t,10*.1*frame),f.status(),objects,f.route);
  for(const auto& point:path.points)changed|=point.pose.pose.position.y>1;
 }
 EXPECT_TRUE(changed);
}

TEST(IntegratedHighwayMerge, UnknownRearWithinReachableMergeCorridorStillWaits) {
 HighwayLidarFixture f;pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 for(int frame=0;frame<6;++frame) {
  const double t=1+.1*frame;
  auto objects=f.objects(t,-60,0,false);objects.objects.front().id=100+frame;
  const auto path=core.process(f.ego(t),f.status(),objects,f.route);
  for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
 }
}

TEST(IntegratedHighwayMerge, StoppedMergeAccountsForAccelerationBeforeRearArrival) {
 HighwayLidarFixture f;pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 auto status=f.status();status.velocity_x_mps=0;
 for(int frame=0;frame<6;++frame) {
  const double t=1+.1*frame;
  const auto path=core.process(f.ego(t),status,f.objects(t,-140+20*.1*frame,20,true),f.route);
  for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
 }
}

TEST(IntegratedHighwayMerge, RearVehicleDoesNotActAsAFrontLeaderWhileWaiting) {
 HighwayLidarFixture f;pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 auto status=f.status();status.velocity_x_mps=10;
 for(int frame=0;frame<6;++frame) {
  const double t=1+.1*frame;
  const auto path=core.process(f.ego(t,10*.1*frame),status,f.objects(t,-40+16*.1*frame,16,true),f.route);
  ASSERT_FALSE(path.points.empty());EXPECT_GE(path.points.front().velocity,10);
  for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
 }
}

TEST(IntegratedHighwayMerge, WaitingPreservesEnoughRoadForANormalMerge) {
 HighwayLidarFixture f;
 for(auto point:f.shared.map->pointLayer)if(point.x()>500)point.x()=100;
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 double x=0,speed=10;
 for(int tick=0;tick<160;++tick) {
  const double t=1+.1*tick;auto status=f.status();status.velocity_x_mps=speed;
  const auto path=core.process(f.ego(t,x),status,f.objects(t,x-5,speed,true),f.route);
  ASSERT_FALSE(path.points.empty());
  for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
  const double next=std::max(0.,speed+.1*std::max(-3.,std::min(1.,(path.points.front().velocity-speed)/.4)));
  x+=.05*(speed+next);speed=next;
 }
 EXPECT_LT(speed,.1);EXPECT_GT(x,20);EXPECT_LT(x,50);
 bool merged=false;
 for(int tick=0;tick<6;++tick) {
  const double t=18+.1*tick;auto status=f.status();status.velocity_x_mps=speed;
  const auto path=core.process(f.ego(t,x),status,f.objects(t),f.route);
  for(const auto& point:path.points)merged|=point.pose.pose.position.y>1;
 }
 EXPECT_TRUE(merged);
}

TEST(IntegratedHighwayMerge, EqualSpeedRearWithAdequateBumperGapAllowsMerge) {
 HighwayLidarFixture f;
 f.params.rear_gap_m=10;f.params.rear_headway_sec=1.1;f.params.rear_clear_sec=3;
 f.params.entry_speed_mps=20;
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 bool changed=false;
 for(int i=0;i<8;++i) {
  const double dt=.1*i,time=1+dt;
  auto status=f.status();status.velocity_x_mps=17.9;
  const auto path=core.process(f.ego(time,17.9*dt),status,
      f.objects(time,-18+17.9*dt,17.9,true),f.route);
  for(const auto& point:path.points)changed|=point.pose.pose.position.y>1;
  EXPECT_GE(path.points.front().velocity,17.9-1e-4);
 }
 EXPECT_TRUE(changed);
}

TEST(IntegratedHighwayMerge, RecordedOpeningRearGapDoesNotWaitUntilRoadReserveForcesBraking) {
 HighwayLidarFixture f;
 f.params.rear_gap_m=10;f.params.rear_headway_sec=1.1;f.params.rear_clear_sec=3;
 f.params.entry_speed_mps=27.7777777778;
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 // Bag 15-56-23 at +208.36 s: ego 17.96 m/s, ID 42 16.70 m/s, 14.17 m behind.
 bool changed=false;
 for(int i=0;i<8;++i) {
  const double dt=.1*i,time=1+dt;
  auto status=f.status();status.velocity_x_mps=17.96;
  auto objects=f.objects(time,-14.17+16.70*dt,16.70,true);
  objects.objects.front().bbox_size.x=5.0;
  const auto path=core.process(f.ego(time,17.96*dt),status,objects,f.route);
  for(const auto& point:path.points)changed|=point.pose.pose.position.y>1;
  EXPECT_GE(path.points.front().velocity,17.96-1e-4);
 }
 EXPECT_TRUE(changed);
}

TEST(IntegratedHighwayMerge, EqualSpeedRearInsideMinimumBumperGapStillWaits) {
 HighwayLidarFixture f;f.params.rear_gap_m=10;f.params.entry_speed_mps=20;
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 for(int i=0;i<8;++i) {
  const double dt=.1*i,time=1+dt;
  auto status=f.status();status.velocity_x_mps=17.9;
  const auto path=core.process(f.ego(time,17.9*dt),status,
      f.objects(time,-9+17.9*dt,17.9,true),f.route);
  for(const auto& point:path.points)EXPECT_NEAR(point.pose.pose.position.y,0,1e-8);
 }
}

TEST(IntegratedHighwayMerge, RelativeHeadwayUsesMinimumGapInsteadOfAddingToIt) {
 HighwayLidarFixture f;
 f.params.rear_gap_m=10;f.params.rear_headway_sec=1.1;f.params.rear_clear_sec=3;
 f.params.entry_speed_mps=17;
 pp::IntegratedHighwayMerge core(f.integrated(),f.shared);
 bool changed=false;
 for(int i=0;i<8;++i) {
  const double elapsed=.1*i,time=1+elapsed;
  auto status=f.status();status.velocity_x_mps=17;
  const auto path=core.process(f.ego(time,17*elapsed),status,
      f.objects(time,-21+18*elapsed,18,true),f.route);
  for(const auto& point:path.points)changed|=point.pose.pose.position.y>1;
 }
 EXPECT_TRUE(changed);
}
