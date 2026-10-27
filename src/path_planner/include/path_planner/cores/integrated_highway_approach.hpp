#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/integrated_path_mission.hpp>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <algorithm>
#include <cmath>
#include <set>
namespace path_planner {
struct IntegratedHighwayApproachParameters {
  PathPlannerMissionParameters mission;
  lanelet::Id entry_target_lane_id=96937;
};
// Keep the existing exit tracking; S11 owns the highway entry gap and maneuver.
class IntegratedHighwayApproach {
 public:
static Message publishLegacyPrefix(const Message& reference,double measured_speed,
                                   double minimum_length_m,double horizon_sec=5.) {
  if(reference.points.size()<2)return reference;
  std::vector<double> station(reference.points.size(),0.);
  for(std::size_t i=1;i<reference.points.size();++i) {
    const auto& a=reference.points[i-1].pose.pose.position;
    const auto& b=reference.points[i].pose.pose.position;
    station[i]=station[i-1]+std::hypot(b.x-a.x,b.y-a.y);
  }
  double speed=std::max(0.,measured_speed),distance=0.;
  for(double time=0;time<horizon_sec-1e-9;time+=1./30) {
    const auto after=std::upper_bound(station.begin(),station.end(),distance);
    const auto i=std::min<std::size_t>(reference.points.size()-1,after-station.begin());
    const double cap=std::isfinite(reference.points[i].velocity)?reference.points[i].velocity:
        std::max(0.,measured_speed);
    const double next=std::max(0.,speed+std::max(-4./30,std::min(2./30,cap-speed)));
    distance+=(speed+next)/60.;speed=next;
  }
  const double extent=std::max(minimum_length_m,distance);
  const auto after=std::lower_bound(station.begin(),station.end(),extent);
  const std::size_t count=std::min(reference.points.size(),std::max<std::size_t>(2,after-station.begin()+1));
  Message output=reference;
  output.points.resize(count);
  for(auto& point:output.points)point.time_from_start=ros::Duration(0);
  return output;
}

  IntegratedHighwayApproach(IntegratedHighwayApproachParameters p,SharedMap& map)
      : p_(std::move(p)),mission_(p_.mission,*map.map),graph_(lanelet::routing::RoutingGraph::build(*map.map,
          *lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,lanelet::Participants::Vehicle))) {}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects,const Route& route) {
    lanelet::ConstLanelets lanes(route->begin(),route->end());
    for(std::size_t i=0;i+1<lanes.size();++i) {
      if(lanes[i+1].id()!=p_.entry_target_lane_id)continue;
      const auto left=graph_->left(lanes[i]),right=graph_->right(lanes[i]);
      if(!(left && left->id()==p_.entry_target_lane_id) && !(right && right->id()==p_.entry_target_lane_id))continue;
      lanes.resize(i+1);std::set<lanelet::Id> visited;for(const auto& lane:lanes)visited.insert(lane.id());
      double length=0;
      for(int step=0;step<20 && length<p_.mission.local_path_length_m+10;++step) {
        const auto next=graph_->following(lanes.back(),false);if(next.empty())break;
        const auto tail=lanes.back().centerline3d().basicLineString();const auto direction=(tail.back()-tail[tail.size()-2]).head<2>().normalized().eval();
        auto chosen=next.front();double best=-2;
        for(const auto& candidate:next) {
          const auto line=candidate.centerline3d().basicLineString();const double aligned=direction.dot((line[1]-line[0]).head<2>().normalized());
          if(aligned>best){best=aligned;chosen=candidate;}
        }
        if(!visited.insert(chosen.id()).second)break;
        const auto line=chosen.centerline3d().basicLineString();for(std::size_t j=1;j<line.size();++j)length+=(line[j]-line[j-1]).head<2>().norm();
        lanes.push_back(chosen);
      }
      break;
    }
    const Route approach=lanelet::routing::LaneletPath(std::move(lanes));
    return publishLegacyPrefix(mission_.process(ego,competition,objects,approach),
        std::max(0.,double(competition.velocity_x_mps)),47.);
  }
 private:
  IntegratedHighwayApproachParameters p_;
  PathPlannerMission mission_;
  lanelet::routing::RoutingGraphPtr graph_;
};
}
