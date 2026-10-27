#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/integrated_path_mission.hpp>
#include <algorithm>
#include <cmath>
#include <vector>
namespace path_planner {
using IntegratedFinalCircleParameters=PathPlannerMissionParameters;
class IntegratedFinalCircle {
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
  IntegratedFinalCircle(IntegratedFinalCircleParameters p,SharedMap& map):mission_(std::move(p),*map.map) {}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& competition,
      const interfaces::Objects& objects,const Route& route) {
    return publishLegacyPrefix(mission_.process(ego,competition,objects,route),
        std::max(0.,double(competition.velocity_x_mps)),47.);
  }
 private:
  PathPlannerMission mission_;
};
}
