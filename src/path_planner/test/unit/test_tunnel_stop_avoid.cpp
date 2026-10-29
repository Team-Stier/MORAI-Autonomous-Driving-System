#include <gtest/gtest.h>
#include <path_planner/cores/tunnel_stop_avoid.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <path_planner/libraries/map_parameters.hpp>
#include <fstream>
#include <sstream>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <lanelet2_core/geometry/Polygon.h>

namespace {
class TunnelStopAvoidTest : public ::testing::Test {
 protected:
  std::shared_ptr<lanelet::LaneletMap> map=std::make_shared<lanelet::LaneletMap>();
  std::unique_ptr<path_planner::SharedMap> bank;
  std::unique_ptr<path_planner::TunnelStopAvoid> core;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus status;
  interfaces::Objects objects;
  path_planner::Route route;
  void SetUp()override {
    ros::Time::init();
    lanelet::LineString3d right(1,{lanelet::Point3d(11,-30,-1.75,0),lanelet::Point3d(12,100,-1.75,0)});
    lanelet::LineString3d middle(2,{lanelet::Point3d(21,-30,1.75,0),lanelet::Point3d(22,100,1.75,0)});
    lanelet::LineString3d left(3,{lanelet::Point3d(31,-30,5.25,0),lanelet::Point3d(32,100,5.25,0)});
    map->add(lanelet::Lanelet(101,middle,right));map->add(lanelet::Lanelet(102,left,middle));
    bank.reset(new path_planner::SharedMap({map}));
    path_planner::TunnelStopAvoidParameters parameters;parameters.tunnel_region_enabled=false;
    core.reset(new path_planner::TunnelStopAvoid(parameters,*bank));
    ego.pose.pose.orientation.w=1;route=lanelet::routing::LaneletPath({map->laneletLayer.get(101)});
  }
  void expectAvoidsRectangle(const interfaces::Path& path,double x=25,double y=0,double half_x=.375,double half_y=.375) {
    const lanelet::BasicPolygon2d obstacle{{x-half_x,y-half_y},{x+half_x,y-half_y},
        {x+half_x,y+half_y},{x-half_x,y+half_y}};
    ASSERT_GE(path.points.size(),2u);
    for(const auto& point:path.points) {
      const auto& q=point.pose.pose;const auto& r=q.orientation;
      const double yaw=std::atan2(2*r.w*r.z,1-2*r.z*r.z),c=std::cos(yaw),sn=std::sin(yaw);
      lanelet::BasicPolygon2d vehicle;
      for(const auto& corner:std::vector<std::pair<double,double>>{{3.845008,1.080418},{3.845008,-1.080418},{-.79,-1.080418},{-.79,1.080418}})
        vehicle.push_back({q.position.x+c*corner.first-sn*corner.second,q.position.y+sn*corner.first+c*corner.second});
      EXPECT_TRUE(boost::geometry::disjoint(vehicle,obstacle));
    }
  }
  interfaces::Path step(double time,double x=25,double y=0,bool visible=true,unsigned id=7,double width=.75) {
    objects.header.stamp=ros::Time(100+time);objects.objects.clear();
    if(visible) {
      interfaces::Object object;object.id=id;object.position.x=object.bbox_pose.position.x=x;
      object.position.y=object.bbox_pose.position.y=y;object.bbox_pose.orientation.w=1;
      object.bbox_size.x=.75;object.bbox_size.y=width;object.bbox_size.z=.43;objects.objects.push_back(object);
    }
    return core->process(ego,status,objects,route,ros::Time(100+time));
  }
};
}  // namespace

TEST_F(TunnelStopAvoidTest, EmptySceneCruisesOnProvidedRoute) {
  ego.pose.pose.position.z=28.6;
  const auto result=step(0,25,0,false);
  ASSERT_GT(result.points.size(),2u);EXPECT_NEAR(result.points[0].velocity,25./3.6,1e-8);
  for(const auto& q:result.points)EXPECT_DOUBLE_EQ(q.pose.pose.position.z,28.6);
}
TEST_F(TunnelStopAvoidTest, CruiseContinuesAcrossConsecutiveRouteLanelets) {
  lanelet::LineString3d right(4,{lanelet::Point3d(41,100,-1.75,0),lanelet::Point3d(42,160,-1.75,0)});
  lanelet::LineString3d left(5,{lanelet::Point3d(51,100,1.75,0),lanelet::Point3d(52,160,1.75,0)});
  const lanelet::Lanelet next(103,left,right);map->add(next);
  route=lanelet::routing::LaneletPath({map->laneletLayer.get(101),next});
  ego.pose.pose.position.x=96;
  const auto result=step(0,25,0,false);
  ASSERT_GT(result.points.size(),2u);EXPECT_GT(result.points.back().pose.pose.position.x,120);
  EXPECT_GT(result.points.front().velocity,0);
}
TEST_F(TunnelStopAvoidTest, FeasibleBypassStartsWhileMovingWithoutClassification) {
  status.velocity_x_mps=3;
  const auto result=step(0);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  double max_y=0;for(const auto& q:result.points)max_y=std::max(max_y,q.pose.pose.position.y);
  EXPECT_GT(max_y,1.75);expectAvoidsRectangle(result);
}

TEST_F(TunnelStopAvoidTest, StandstillThenStationaryBypassSurvivesMissingObservations) {
  interfaces::Path result;
  for(int i=0;i<=16;++i)result=step(i*.1);
  ASSERT_GT(result.points.size(),2u);
  double max_y=0;for(const auto& q:result.points)max_y=std::max(max_y,q.pose.pose.position.y);
  EXPECT_GT(max_y,1.75);EXPECT_GT(result.points.front().velocity,0);
  const auto without_observation=step(1.7,25,0,false);
  ASSERT_EQ(result.points.size(),without_observation.points.size());
  for(std::size_t i=0;i<result.points.size();++i)
    EXPECT_DOUBLE_EQ(result.points[i].pose.pose.position.y,without_observation.points[i].pose.pose.position.y);
}
TEST_F(TunnelStopAvoidTest, CrossingWithNoSafeBypassStopsAndRetainsOccupancyWhenOccluded) {
  status.velocity_x_mps=7;
  interfaces::Path result;
  for(int i=0;i<=8;++i)result=step(i*.1,25,3-i*.14,true,7,8.);
  ASSERT_EQ(result.points.size(),2u);
  for(const auto& q:result.points)EXPECT_EQ(q.velocity,0);
  status.velocity_x_mps=0;
  for(int i=9;i<80;++i)result=step(i*.1,25,0,false);
  for(const auto& q:result.points)EXPECT_EQ(q.velocity,0);
}

TEST_F(TunnelStopAvoidTest, CrossingClearanceReturnsToNominalInsteadOfKeepingBypass) {
  interfaces::Path result;
  for(int i=0;i<=65;++i)result=step(i*.1,25,3-i*.1);
  ASSERT_GT(result.points.size(),2u);EXPECT_NEAR(result.points.front().velocity,25./3.6,1e-8);
  for(const auto& q:result.points)EXPECT_NEAR(q.pose.pose.position.y,0,1e-8);
}
TEST_F(TunnelStopAvoidTest, RepeatedSameCloudRetainsTheCommittedCollisionFreeBypass) {
  const auto initial=step(0);
  interfaces::Path result;
  for(int i=1;i<60;++i)result=core->process(ego,status,objects,route,ros::Time(100+i*.1));
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  ASSERT_EQ(result.points.size(),initial.points.size());
  for(std::size_t i=0;i<result.points.size();++i)
    EXPECT_DOUBLE_EQ(result.points[i].pose.pose.position.y,initial.points[i].pose.pose.position.y);
  expectAvoidsRectangle(result);
}

TEST_F(TunnelStopAvoidTest, LostUnclassifiedObjectRetainsBypassAndReacquisitionUpdatesOccupancy) {
  step(0);
  interfaces::Path result;
  for(double time:{.5,2.1,5.,10.,30.}) {
    result=step(time,25,0,false);
    ASSERT_GT(result.points.size(),2u);EXPECT_GT(result.points.back().velocity,0);
    expectAvoidsRectangle(result);
  }
  result=step(30.1,25,0,true,7,2.0);
  expectAvoidsRectangle(result,25,0,.375,1.);
}

TEST_F(TunnelStopAvoidTest, LostObjectIsAvoidedWithoutDrivingThroughRememberedOccupancy) {
  step(0);
  for(double time:{2.1,5.,10.,30.}) {
    const auto result=step(time,25,0,false);
    expectAvoidsRectangle(result);
  }
  ego.pose.pose.position=step(30.1,25,0,false).points[35].pose.pose.position;
  const auto result=step(30.2,25,0,false);
  expectAvoidsRectangle(result);
}

TEST_F(TunnelStopAvoidTest, ReacquiredStaticObjectWithNewIdCompletesExistingEvidence) {
  step(0);
  step(2.1,25,0,false);
  interfaces::Path result;
  for(int i=0;i<14;++i)result=step(2.2+i*.1,25,0,true,99);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  double max_y=0;for(const auto& p:result.points)max_y=std::max(max_y,p.pose.pose.position.y);
  EXPECT_GT(max_y,1.75);
}
TEST_F(TunnelStopAvoidTest, ClearedDynamicObjectWithSameIdBlocksAgainWhenItOccupiesBothLanes) {
  for(int i=0;i<=65;++i)step(i*.1,25,3-i*.1);
  const auto result=step(6.6,25,0,true,7,8.);
  ASSERT_EQ(result.points.size(),2u);
  EXPECT_DOUBLE_EQ(result.points.front().velocity,0);
}

TEST_F(TunnelStopAvoidTest, DistantObstacleUsesMovingBypassWhenCurrentSpeedIsFeasible) {
  status.velocity_x_mps=6;
  const auto result=step(0,40);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_NEAR(result.points.front().velocity,8./3.6,1e-8);
  EXPECT_GT(result.points.back().velocity,0);
  expectAvoidsRectangle(result,40);
}

TEST_F(TunnelStopAvoidTest, AlongRoadMotionUsesCollisionConstraintsInsteadOfClassificationWait) {
  interfaces::Path result;
  for(int i=0;i<30;++i)result=step(i*.1,25+i*.1,0);
  expectAvoidsRectangle(result,27.9);
  if(result.points.size()>2)EXPECT_GT(result.points.back().velocity,0);
}

TEST(TunnelStopAvoidRecorded, StaticTunnelObservationsGenerateSteeringFeasibleBypass) {
  ros::Time::init();
  std::ifstream map_file(std::string(NP_PACKAGE_DIR)+"/../map/kcity_lanelet2_smooth_circle.osm");
  std::stringstream data;data<<map_file.rdbuf();
  auto map=path_planner::readLaneletMap(data.str());path_planner::SharedMap bank({map});
  path_planner::TunnelStopAvoid core({},bank);
  path_planner::Route route=lanelet::routing::LaneletPath({map->laneletLayer.get(104944)});
  nav_msgs::Odometry ego;ego.pose.pose.position.x=3.0470370413;ego.pose.pose.position.y=-502.8485028596;
  ego.pose.pose.orientation.z=std::sin(-2.6490616543/2);ego.pose.pose.orientation.w=std::cos(-2.6490616543/2);
  interfaces::CompetitionVehicleStatus status;
  // Observations from 2026-10-29-03-16-56; unavailable velocity is retained.
  const double observations[][7]={
    {0.0,-22.113486495043578,-516.3996559847361,-1.0827452811251377,0.6668444015371411,0.017693003792813133,0.008821487426757812},
    {0.11558866500854492,-22.331855594712557,-516.6161671040487,-1.080817818306167,0.7415232822093448,0.0011614070881990068,0.00909423828125},
    {0.23993158340454102,-22.5310349701007,-516.726074371399,-1.3056138139072133,0.5735138500135679,0.37866726624171176,0.019557952880859375},
    {1.5230607986450195,-22.20471803218108,-516.5189885298965,-1.0815694704909844,0.6960953718348089,0.016522531259391154,0.003726959228515625},
    {1.6382203102111816,-22.39274070080021,-516.5948648365402,-1.082042136666285,0.7164689163837201,0.0019214894609091704,0.003086090087890625},
    {1.7548010349273682,-22.682111322210996,-516.7974206340775,-1.1627531018520445,0.7189025786222061,0.22524463140737738,0.021745681762695312},
    {2.222702980041504,-21.467207987515543,-516.1359786552955,-1.0799677969128207,0.7110158589259616,0.02146249086635421,0.004161834716796875},
    {2.3375978469848633,-22.422449706125878,-516.6135324544176,-1.0865752041111776,0.7039750915445306,0.001336508483285967,0.0029087066650390625},
    {2.5705478191375732,-21.479420745467163,-516.1144681718016,-1.082081924686142,0.6596828395221273,0.021306552033386276,0.0034580230712890625},
    {2.6835384368896484,-21.371111046784304,-516.0311648358719,-1.0882178205354138,0.7168033664204952,0.07761874611942438,0.2931976318359375},
    {2.8025379180908203,-21.47231452135376,-516.1452544850979,-1.0826394685752998,0.6753129124179927,0.021303661193259327,0.00925445556640625},
    {2.9219107627868652,-21.514184402022988,-516.0330703109062,1.5412268542611989,0.8824207302455105,0.4631434017205162,0.26100730895996094},
    {3.0290799140930176,-21.38540490331126,-515.9678898968856,-0.7841135296840298,0.7498482578439383,0.568803251373499,0.25964927673339844},
    {3.139512538909912,-21.77580653351301,-516.3120109210298,-1.0811495538201128,0.7064796234852755,0.6745750040194903,0.053813934326171875},
    {3.248929023742676,-22.17330075220101,-516.6233443713634,-0.9813318579237887,0.5037890850785708,0.04288904938357868,0.013095855712890625}};
  interfaces::Path result;
  for(unsigned i=0;i<4;++i) {
    const auto& r=observations[i];
    interfaces::Objects objects;objects.header.stamp.fromSec(100+r[0]);
    interfaces::Object object;object.id=72;object.position.x=object.bbox_pose.position.x=r[1];
    object.position.y=object.bbox_pose.position.y=r[2];object.bbox_pose.orientation.z=std::sin(r[3]/2);
    object.bbox_pose.orientation.w=std::cos(r[3]/2);object.bbox_size.x=r[4];object.bbox_size.y=r[5];object.bbox_size.z=r[6];
    objects.objects.push_back(object);
    result=core.process(ego,status,objects,route,objects.header.stamp);
  }
  ASSERT_GT(result.points.size(),2u);EXPECT_GT(result.points.front().velocity,0);  path_planner::TunnelStopAvoid late({},bank);
  for(unsigned i=3;i<sizeof(observations)/sizeof(observations[0]);++i) {
    const auto& r=observations[i];interfaces::Objects objects;objects.header.stamp.fromSec(100+r[0]);
    interfaces::Object object;object.id=72;object.position.x=object.bbox_pose.position.x=r[1];
    object.position.y=object.bbox_pose.position.y=r[2];object.bbox_pose.orientation.z=std::sin(r[3]/2);
    object.bbox_pose.orientation.w=std::cos(r[3]/2);object.bbox_size.x=r[4];object.bbox_size.y=r[5];object.bbox_size.z=r[6];
    objects.objects.push_back(object);result=late.process(ego,status,objects,route,objects.header.stamp);
  }
  // The periodic node continues to call the Core while occupancy is retained.
  // Allow standstill confirmation after the last observed envelope update.
  for(int i=1;i<=8;++i) {
    interfaces::Objects empty;empty.header.stamp.fromSec(103.25+i*.1);
    result=late.process(ego,status,empty,route,empty.header.stamp);
  }
  ASSERT_GT(result.points.size(),2u);EXPECT_GT(result.points.front().velocity,0);
}

TEST_F(TunnelStopAvoidTest, StationaryExtentGrowthUpdatesBypassWhileMoving) {
  interfaces::Path initial;
  for(int i=0;i<=16;++i)initial=step(i*.1);
  ASSERT_GT(initial.points.size(),2u);
  const auto& start=initial.points[10].pose.pose;
  ego.pose.pose=start;
  status.velocity_x_mps=1;
  const auto result=step(1.7,25,0,true,7,1.5);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  EXPECT_NEAR(result.points.front().pose.pose.position.x,start.position.x,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.position.y,start.position.y,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.orientation.z,start.orientation.z,1e-8);
  const auto curvature=[](const geometry_msgs::Pose& a,const geometry_msgs::Pose& b) {
    const auto yaw=[](const geometry_msgs::Quaternion& q) {
      return std::atan2(2*q.w*q.z,1-2*q.z*q.z);
    };
    const double delta=yaw(b.orientation)-yaw(a.orientation);
    return std::atan2(std::sin(delta),std::cos(delta))/
        std::hypot(b.position.x-a.position.x,b.position.y-a.position.y);
  };
  EXPECT_NEAR(curvature(result.points[0].pose.pose,result.points[2].pose.pose),
              curvature(initial.points[9].pose.pose,initial.points[11].pose.pose),.01);
  double before_y=0,after_y=0;
  for(const auto& q:initial.points)before_y=std::max(before_y,q.pose.pose.position.y);
  for(const auto& q:result.points)after_y=std::max(after_y,q.pose.pose.position.y);
  EXPECT_GT(after_y,before_y);

}
TEST_F(TunnelStopAvoidTest, NewBlockingOccupancyStopsButPreservesLateralReference) {
  interfaces::Path initial;
  for(int i=0;i<=16;++i)initial=step(i*.1);
  ASSERT_GT(initial.points.size(),2u);
  interfaces::Object obstacle;obstacle.id=88;
  obstacle.position.x=obstacle.bbox_pose.position.x=12;
  obstacle.position.y=obstacle.bbox_pose.position.y=3;
  obstacle.bbox_pose.orientation.w=1;obstacle.bbox_size.x=obstacle.bbox_size.y=2;
  objects.objects.push_back(obstacle);
  objects.header.stamp=ros::Time(101.7);status.velocity_x_mps=1;
  auto result=core->process(ego,status,objects,route,ros::Time(101.7));
  ASSERT_EQ(result.points.size(),2u);
  EXPECT_DOUBLE_EQ(result.points.front().velocity,0);
  ASSERT_EQ(result.lateral_reference.size(),initial.points.size());
  for(std::size_t i=0;i<result.lateral_reference.size();++i)
    EXPECT_DOUBLE_EQ(result.lateral_reference[i].position.y,initial.points[i].pose.pose.position.y);
  status.velocity_x_mps=0;
  for(int i=0;i<8;++i) {
    objects.header.stamp.fromSec(101.8+i*.1);
    result=core->process(ego,status,objects,route,objects.header.stamp);
  }
  // A failed replan must not treat the retained old path as a new safe candidate.
  ASSERT_EQ(result.points.size(),2u);
  EXPECT_DOUBLE_EQ(result.points.back().velocity,0);
  EXPECT_FALSE(result.lateral_reference.empty());
}

TEST_F(TunnelStopAvoidTest, AdjacentLaneObjectDoesNotBecomeStopTarget) {
  interfaces::Path result;
  for(int i=0;i<30;++i)result=step(i*.1,25,3.5);
  ASSERT_GT(result.points.size(),2u);
  for(const auto& point:result.points)EXPECT_NEAR(point.velocity,25./3.6,1e-8);
}
TEST_F(TunnelStopAvoidTest, AdjacentWideObjectIntrudingIntoCruiseCorridorIsAvoidedOrStopped) {
  const auto result=step(0,25,3.5,true,7,5.0);
  EXPECT_LT(result.points.front().velocity,25./3.6);
  expectAvoidsRectangle(result,25,3.5,.375,2.5);
}

TEST_F(TunnelStopAvoidTest, ObjectOnActualLaneReturnCorridorStillStops) {
  ego.pose.pose.position.y=1.5;
  const auto result=step(0,6,2.8);
  EXPECT_DOUBLE_EQ(result.points.back().velocity,0);
}

TEST_F(TunnelStopAvoidTest, TunnelObjectDoesNotActivateMissionBeforeVehicleEntry) {
  path_planner::TunnelStopAvoidParameters p;
  p.tunnel_origin_x_m=p.tunnel_origin_y_m=p.tunnel_axis_yaw_rad=0;
  p.tunnel_min_station_m=15;p.tunnel_max_station_m=40;
  core.reset(new path_planner::TunnelStopAvoid(p,*bank));
  interfaces::Path result;
  for(int i=0;i<30;++i)result=step(i*.1,25,0);
  ASSERT_GT(result.points.size(),2u);
  for(const auto& q:result.points)EXPECT_NEAR(q.velocity,25./3.6,1e-8);
  ego.pose.pose.position.x=16;
  result=step(3.,25,0);
  EXPECT_LT(result.points.front().velocity,25./3.6);
  expectAvoidsRectangle(result);
}
TEST_F(TunnelStopAvoidTest, OutsideTunnelObjectIsNotSelectedEvenAfterEntry) {
  path_planner::TunnelStopAvoidParameters p;
  p.tunnel_origin_x_m=p.tunnel_origin_y_m=p.tunnel_axis_yaw_rad=0;
  p.tunnel_min_station_m=-5;p.tunnel_max_station_m=10;
  core.reset(new path_planner::TunnelStopAvoid(p,*bank));
  const auto result=step(0,25,0);
  ASSERT_GT(result.points.size(),2u);
  for(const auto& q:result.points)EXPECT_NEAR(q.velocity,25./3.6,1e-8);
}
TEST_F(TunnelStopAvoidTest, ObjectPartlyInsideTunnelRemainsAStopCandidate) {
  path_planner::TunnelStopAvoidParameters p;
  p.tunnel_origin_x_m=p.tunnel_origin_y_m=p.tunnel_axis_yaw_rad=0;
  p.tunnel_min_station_m=-5;p.tunnel_max_station_m=20;
  core.reset(new path_planner::TunnelStopAvoid(p,*bank));
  const auto result=step(0,20.2,0);
  EXPECT_LT(result.points.front().velocity,25./3.6);
  expectAvoidsRectangle(result,20.2);
}
TEST_F(TunnelStopAvoidTest, VehicleFrontEntryEnablesTunnelHandling) {
  path_planner::TunnelStopAvoidParameters p;
  p.tunnel_origin_x_m=p.tunnel_origin_y_m=p.tunnel_axis_yaw_rad=0;
  p.tunnel_min_station_m=3;p.tunnel_max_station_m=40;
  core.reset(new path_planner::TunnelStopAvoid(p,*bank));
  const auto result=step(0,25,0);
  EXPECT_LT(result.points.front().velocity,25./3.6);
  expectAvoidsRectangle(result);
}

TEST_F(TunnelStopAvoidTest, MidBypassUpdateDoesNotRequireInitialDepartureSpaceAgain) {
  interfaces::Path initial;
  for(int i=0;i<=16;++i)initial=step(i*.1);
  ASSERT_GT(initial.points.size(),72u);
  const auto start=initial.points[72].pose.pose;
  ego.pose.pose=start;status.velocity_x_mps=2;
  // Only 5.45 m remains before the object clearance line. This is an update
  // of an existing lateral maneuver, not a fresh departure from lane centre.
  const auto result=step(1.7,25,0,true,7,2.0);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  EXPECT_NEAR(result.points.front().pose.pose.position.x,start.position.x,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.position.y,start.position.y,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.orientation.z,start.orientation.z,1e-8);
}

TEST_F(TunnelStopAvoidTest, MidReturnUpdateStartsAtCurrentPoseAndContinuesTowardLaneCentre) {
  interfaces::Path initial;
  for(int i=0;i<=16;++i)initial=step(i*.1);
  ASSERT_GT(initial.points.size(),160u);
  const auto start=initial.points[160].pose.pose;
  ego.pose.pose=start;status.velocity_x_mps=2;
  interfaces::Object obstacle;obstacle.id=88;
  obstacle.position.x=obstacle.bbox_pose.position.x=40;
  obstacle.position.y=obstacle.bbox_pose.position.y=3;
  obstacle.bbox_pose.orientation.w=1;obstacle.bbox_size.x=obstacle.bbox_size.y=2;
  objects.objects.push_back(obstacle);objects.header.stamp=ros::Time(101.7);
  const auto result=core->process(ego,status,objects,route,objects.header.stamp);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  EXPECT_NEAR(result.points.front().pose.pose.position.x,start.position.x,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.position.y,start.position.y,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.orientation.z,start.orientation.z,1e-8);
  EXPECT_NEAR(result.points.back().pose.pose.position.y,0,1e-8);
}

TEST_F(TunnelStopAvoidTest, RestartMidBypassEvaluatesWholePathWithoutDepartureDistanceBound) {
  interfaces::Path initial;
  for(int i=0;i<=16;++i)initial=step(i*.1);
  ASSERT_GT(initial.points.size(),72u);
  ego.pose.pose=initial.points[72].pose.pose;
  path_planner::TunnelStopAvoidParameters parameters;parameters.tunnel_region_enabled=false;
  core.reset(new path_planner::TunnelStopAvoid(parameters,*bank));
  interfaces::Path result;
  for(int i=0;i<=16;++i)result=step(2+i*.1,25,0,true,7,2.0);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_GT(result.points.back().velocity,0);
  EXPECT_NEAR(result.points.front().pose.pose.position.x,ego.pose.pose.position.x,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.position.y,ego.pose.pose.position.y,1e-8);
}

TEST_F(TunnelStopAvoidTest, InfeasibleMovingBypassFinishesEstopThenRetriesPersistingObstacle) {
  status.velocity_x_mps=20;
  auto result=step(0);
  ASSERT_EQ(result.points.size(),2u);EXPECT_DOUBLE_EQ(result.points.front().velocity,0);
  status.velocity_x_mps=3;result=step(.5);
  ASSERT_EQ(result.points.size(),2u);EXPECT_DOUBLE_EQ(result.points.front().velocity,0);
  status.velocity_x_mps=0;step(.8);result=step(1.2);
  ASSERT_GT(result.points.size(),2u);EXPECT_GT(result.points.back().velocity,0);
  expectAvoidsRectangle(result);
}
TEST_F(TunnelStopAvoidTest, PreviouslyMovingObjectThatRemainsCanBeBypassedAfterEstop) {
  status.velocity_x_mps=20;
  for(int i=0;i<=15;++i)step(i*.1,25,3-i*.2);
  status.velocity_x_mps=0;
  interfaces::Path result;
  for(int i=16;i<=80;++i)result=step(i*.1);
  ASSERT_GT(result.points.size(),2u);EXPECT_GT(result.points.back().velocity,0);
  expectAvoidsRectangle(result);
}

TEST_F(TunnelStopAvoidTest, OutwardClearanceConfirmationDoesNotRequireStandstill) {
  status.velocity_x_mps=2;
  interfaces::Path result;
  for(int i=0;i<=65;++i)result=step(i*.1,25,3-i*.1);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_NEAR(result.points.front().velocity,25./3.6,1e-8);
  for(const auto& q:result.points)EXPECT_NEAR(q.pose.pose.position.y,0,1e-8);
}
TEST_F(TunnelStopAvoidTest, MeasuredOutsideFootprintsAndRecentOutwardMarginClearWithoutMoreClouds) {
  for(int i=0;i<=40;++i)step(i*.1,25,3-i*.1);
  step(4.1,25,-1.45,true,7,.3);
  step(4.2,25,-1.48,true,7,.3);
  core->process(ego,status,objects,route,ros::Time(104.45));
  const auto result=core->process(ego,status,objects,route,ros::Time(104.48));
  ASSERT_GT(result.points.size(),2u);
  EXPECT_NEAR(result.points.front().velocity,25./3.6,1e-8);
  for(const auto& q:result.points)EXPECT_NEAR(q.pose.pose.position.y,0,1e-8);
}
TEST_F(TunnelStopAvoidTest, CrossingDisappearingInsideCorridorDoesNotReleaseStraightTravel) {
  interfaces::Path result;
  for(int i=0;i<=30;++i)result=step(i*.1,25,3-i*.1);
  for(int i=31;i<80;++i)result=step(i*.1,25,0,false);
  expectAvoidsRectangle(result);
  EXPECT_LT(result.points.front().velocity,25./3.6);
}
TEST_F(TunnelStopAvoidTest, ClearingOneCrossingDoesNotReleaseOtherBlockingOccupancy) {
  interfaces::Path result;
  for(int i=0;i<=65;++i) {
    interfaces::Object crossing;crossing.id=7;
    crossing.position.x=crossing.bbox_pose.position.x=25;
    crossing.position.y=crossing.bbox_pose.position.y=3-i*.1;
    crossing.bbox_pose.orientation.w=1;crossing.bbox_size.x=crossing.bbox_size.y=.75;
    auto blocking=crossing;blocking.id=88;blocking.position.x=blocking.bbox_pose.position.x=35;
    blocking.position.y=blocking.bbox_pose.position.y=0;
    objects.objects={crossing,blocking};objects.header.stamp=ros::Time(100+i*.1);
    result=core->process(ego,status,objects,route,objects.header.stamp);
  }
  expectAvoidsRectangle(result,35);
  EXPECT_LT(result.points.front().velocity,25./3.6);
}

TEST_F(TunnelStopAvoidTest, ClearedCrossingConnectsMovingOffCentrePoseToNominalLane) {
  ego.pose.pose.position.x=6;ego.pose.pose.position.y=.8;
  ego.pose.pose.orientation.z=std::sin(.15/2);ego.pose.pose.orientation.w=std::cos(.15/2);
  status.velocity_x_mps=2;
  interfaces::Path result;
  for(int i=0;i<=65;++i)result=step(i*.1,25,3-i*.1);
  ASSERT_GT(result.points.size(),2u);
  EXPECT_NEAR(result.points.front().pose.pose.position.x,6,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.position.y,.8,1e-8);
  EXPECT_NEAR(result.points.front().pose.pose.orientation.z,std::sin(.15/2),1e-8);
  EXPECT_NEAR(result.points.back().pose.pose.position.y,0,1e-8);
  double maximum_lateral=0;
  for(const auto& q:result.points)maximum_lateral=std::max(maximum_lateral,q.pose.pose.position.y);
  EXPECT_LT(maximum_lateral,2.);
}

TEST_F(TunnelStopAvoidTest, ReentryDuringNominalReturnIsNotReleasedAgainAtLaneCentre) {
  ego.pose.pose.position.x=6;ego.pose.pose.position.y=.8;
  ego.pose.pose.orientation.z=std::sin(.15/2);ego.pose.pose.orientation.w=std::cos(.15/2);
  status.velocity_x_mps=2;
  for(int i=0;i<=65;++i)step(i*.1,25,3-i*.1);
  ego.pose.pose.position.y=0;ego.pose.pose.orientation.z=0;ego.pose.pose.orientation.w=1;
  const auto result=step(6.6,25,0,true,7,8.);
  ASSERT_EQ(result.points.size(),2u);
  EXPECT_DOUBLE_EQ(result.points.front().velocity,0);
}

TEST_F(TunnelStopAvoidTest, CrossingOutsideOriginalLaneStillBlocksOffCentreVehicleFootprint) {
  ego.pose.pose.position.x=24;ego.pose.pose.position.y=2.3;
  interfaces::Path result;
  for(int i=0;i<=65;++i)result=step(i*.1,25,-3+i*.1);
  ASSERT_EQ(result.points.size(),2u);
  EXPECT_DOUBLE_EQ(result.points.front().velocity,0);
}
