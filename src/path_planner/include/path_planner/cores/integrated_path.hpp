#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/integrated_path_mission.hpp>

#include <path_planner/libraries/integrated_solver.hpp>
#include <ros/console.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <set>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>


namespace path_planner {
struct IntegratedPathTrajectoryParameters {
  integrated_native::PathParameters solver;
  bool publish_lateral_reference=false;
  double curve_preview_m=45.,curve_spacing_m=2.,curve_deceleration_mps2=1.7;
  double minimum_curve_speed_mps=15./3.6;
  double corridor_half_width_m=1.8;
  double horizon_sec=5.0;
  double recovery_deceleration_mps2=8.,recovery_jerk_mps3=8.,recovery_response_sec=.3;
  double recovery_lateral_acceleration_mps2=6.;
  double dynamic_zone_speed_mps=25./3.6,dynamic_zone_approach_m=30;
};


struct IntegratedPathParameters {
  PathPlannerMissionParameters mission;
  IntegratedPathTrajectoryParameters trajectory;
};
class IntegratedPath {
class IntegratedTrajectory {
 public:
  IntegratedTrajectory(IntegratedPathTrajectoryParameters p,const lanelet::LaneletMap& map)
      : p_(std::move(p)),solver_(p_.solver,map),recovery_solver_(solver_,recoveryParameters(p_)) {}
  const char* outcome()const{return outcome_;}
  std::string diagnostics()const{return used_recovery_ ? "dynamics-recovery "+recovery_solver_.diagnostics():solver_.diagnostics();}
  Message calculate(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& status,
      const interfaces::Objects& objects,const sensor_msgs::Imu& imu,const Route& route,
      const PredictedObjects& predictions,const ros::Time& stamp,double ceiling,bool dynamic_zone,
      double detection,double clear_sec) {
    used_recovery_=false;outcome_="native-search";
    try {
      integrated_native::EgoState state;
      const auto& q=ego.pose.pose.orientation;
      state.pose={{ego.pose.pose.position.x,ego.pose.pose.position.y},heading(q)};
      state.stamp_sec=stamp.toSec();
      state.speed_mps=std::max(0.,double(status.velocity_x_mps));
      const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
      state.acceleration_mps2=imu.linear_acceleration.x-
          (rotation.conjugate()*Eigen::Vector3d(0,0,9.81)).x();
      // At measured standstill, brake/contact force is not a continuing
      // backwards acceleration. Preserve ground speed and restart the model
      // from zero acceleration when the vehicle is held by its brake.
      if(state.speed_mps<.001 && std::abs(state.acceleration_mps2)<.05)
        state.acceleration_mps2=0;
      if(status.brake>0 && state.acceleration_mps2<0) {
        // Brake force cannot be extended through rest into reverse motion.
        // Project only an infeasible braking seed onto the forward-only
        // reference model. Measured speed and all road/obstacle limits stay
        // unchanged; assuming less braking is conservative for stopping space.
        const double release_bound=std::sqrt(2*p_.solver.max_longitudinal_jerk_mps3*state.speed_mps);
        state.acceleration_mps2=state.speed_mps<.05 ? 0. :
            std::max(state.acceleration_mps2,-release_bound*(1-1e-8));
      }
      // MORAI status angular velocity uses the opposite yaw convention to
      // ROS IMU/map heading. Use the IMU for curve and pose propagation.
      state.curvature_inv_m=state.speed_mps>.5?imu.angular_velocity.z/state.speed_mps:0;
      if(stamp.toSec()<last_stamp_)previous_.clear();
      last_stamp_=stamp.toSec();
      const double ego_dt=std::max(0.,stamp.toSec()-ego.header.stamp.toSec());
      const double rate=imu.angular_velocity.z,angle=rate*ego_dt;
      const double forward=ego.twist.twist.linear.x,lateral=ego.twist.twist.linear.y;
      const double a=std::abs(rate)>1e-9?std::sin(angle)/rate:ego_dt;
      const double b=std::abs(rate)>1e-9?(1-std::cos(angle))/rate:0.;
      const double dx=forward*a-lateral*b,dy=forward*b+lateral*a;
      state.pose.position.x+=std::cos(state.pose.yaw_rad)*dx-std::sin(state.pose.yaw_rad)*dy;
      state.pose.position.y+=std::sin(state.pose.yaw_rad)*dx+std::cos(state.pose.yaw_rad)*dy;
      state.pose.yaw_rad+=angle;

      auto reference=solver_.mapReference(state,route);
      Message guide;
      // The numerical reference includes supports behind the rear axle.
      // Curvature speed preview starts at the vehicle, not eight metres back.
      std::size_t guide_begin=0;double guide_distance=std::numeric_limits<double>::infinity();
      for(std::size_t i=0;i<reference.size();++i) {
        const double d=std::hypot(reference[i].position.x-state.pose.position.x,
            reference[i].position.y-state.pose.position.y);
        if(d<guide_distance){guide_distance=d;guide_begin=i;}
      }
      for(std::size_t i=guide_begin;i<reference.size();++i) {
        interfaces::PathPoint point;point.pose.pose.position.x=reference[i].position.x;point.pose.pose.position.y=reference[i].position.y;
        const auto a=i==0?0:i-1,b=i+1<reference.size()?i+1:i;
        const double yaw=std::atan2(reference[b].position.y-reference[a].position.y,reference[b].position.x-reference[a].position.x);
        point.pose.pose.orientation.z=std::sin(yaw/2);point.pose.pose.orientation.w=std::cos(yaw/2);guide.points.push_back(point);
      }
      if(dynamic_zone) {
        if(dynamicBlocked(reference,state,objects,predictions,detection))last_blocked_sec_=stamp.toSec();
        if(stamp.toSec()-last_blocked_sec_<clear_sec) {
          outcome_="dynamic-zone-stop";previous_.clear();return hold(ego);
        }
      } else last_blocked_sec_=-std::numeric_limits<double>::infinity();
      auto native=convert(objects,predictions);

      std::vector<bool> coverage;

      integrated_native::PlanningConstraints limits;
      limits.maximum_speed_mps=std::min(p_.solver.max_speed_mps,ceiling);
      limits.preferred_speed_mps=referenceCurveTarget(guide,limits.maximum_speed_mps);
      auto planned=solver_.planInMap(state,reference,native,limits,route,previous_,{},coverage);
      const bool stopped_free=!planned.empty() && planned.back().speed_mps==0 && objects.objects.empty();
      if((planned.empty() || stopped_free) && std::string(solver_.failureReason())!="initial-collision") {
        auto alternative=recovery_solver_.planInMap(state,reference,native,limits,route,previous_,{},coverage);
        if(!alternative.empty() && (planned.empty() || alternative.back().speed_mps>planned.back().speed_mps)) {
          planned=std::move(alternative);used_recovery_=true;
        }
      }
      if(planned.empty()) {
        outcome_=used_recovery_?recovery_solver_.failureReason():solver_.failureReason();
        ROS_WARN_THROTTLE(1.,"[native planner] no trajectory: %s objects=%zu %s",outcome_,objects.objects.size(),diagnostics().c_str());
        return hold(ego);
      }
      previous_=planned;outcome_=used_recovery_?"native-generated-recovery":"native-generated";
      Message output;
      for(const auto& q:planned) {
        interfaces::PathPoint point;point.pose.pose.position.x=q.pose.position.x;point.pose.pose.position.y=q.pose.position.y;
        point.pose.pose.position.z=ego.pose.pose.position.z;point.pose.pose.orientation.z=std::sin(q.pose.yaw_rad/2);
        point.pose.pose.orientation.w=std::cos(q.pose.yaw_rad/2);point.velocity=q.speed_mps;point.acceleration=q.acceleration_mps2;
        point.time_from_start=ros::Duration(q.time_sec);
        if(!output.points.empty() && point.time_from_start<=output.points.back().time_from_start) {
          if(!output.points.back().time_from_start.isZero())output.points.back()=point;
        } else output.points.push_back(point);
      }
      if(p_.publish_lateral_reference && planned.front().spatial_reference)for(const auto& q:*planned.front().spatial_reference) {
        geometry_msgs::Pose point;point.position.x=q.position.x;point.position.y=q.position.y;point.position.z=ego.pose.pose.position.z;
        point.orientation.z=std::sin(q.yaw_rad/2);point.orientation.w=std::cos(q.yaw_rad/2);output.lateral_reference.push_back(point);
      }
      return output;
    } catch(const std::exception& e) {
      outcome_="calculation-domain";ROS_WARN_THROTTLE(1.,"[native planner] %s",e.what());previous_.clear();return hold(ego);
    }
  }
 private:
  static Message hold(const nav_msgs::Odometry& ego) {
    Message output;interfaces::PathPoint point;point.pose.pose=ego.pose.pose;point.velocity=0;point.acceleration=0;
    output.points.push_back(point);point.time_from_start=ros::Duration(5);output.points.push_back(point);return output;
  }
  bool dynamicBlocked(const std::vector<integrated_native::ReferencePoint>& reference,
      const integrated_native::EgoState& state,const interfaces::Objects& current,
      const PredictedObjects& future,double detection)const {
    double station=0,best=std::numeric_limits<double>::infinity(),origin=0;
    std::vector<double> stations(reference.size(),0);
    for(std::size_t i=1;i<reference.size();++i) {
      const auto a=reference[i-1].position,b=reference[i].position;const double length=std::hypot(b.x-a.x,b.y-a.y);
      stations[i]=stations[i-1]+length;
      const double d=std::hypot(a.x-state.pose.position.x,a.y-state.pose.position.y);
      if(d<best){best=d;origin=stations[i-1];}
    }
    const auto blocked=[&](const interfaces::Objects& objects) {
      for(const auto& object:objects.objects) {
        double nearest=std::numeric_limits<double>::infinity(),along=0,lateral=0,road_yaw=0;
        for(std::size_t i=1;i<reference.size();++i) {
          const auto a=reference[i-1].position,b=reference[i].position;const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
          if(length<1e-9)continue;
          const double raw=((object.bbox_pose.position.x-a.x)*dx+(object.bbox_pose.position.y-a.y)*dy)/(length*length);
          const double u=std::max(0.,std::min(1.,raw));
          const double ex=object.bbox_pose.position.x-a.x-u*dx,ey=object.bbox_pose.position.y-a.y-u*dy;
          const double distance=ex*ex+ey*ey;
          if(distance>=nearest)continue;
          // Keep true signed distance outside the finite reference ends. A
          // short predecessor must not project a distant rear object under us.
          const double signed_u=(i==1 && raw<0) || (i+1==reference.size() && raw>1) ? raw : u;
          nearest=distance;along=stations[i-1]+signed_u*length-origin;road_yaw=std::atan2(dy,dx);
          lateral=std::abs(-ex*dy+ey*dx)/length;
        }
        if(along < -p_.solver.vehicle.rear_m || along>detection)continue;
        const double relative=heading(object.bbox_pose.orientation)-road_yaw;
        const double half=(std::abs(std::sin(relative))*object.bbox_size.x+std::abs(std::cos(relative))*object.bbox_size.y)/2;
        if(lateral<=p_.solver.vehicle.width_m/2+half+.25)return true;
      }
      return false;
    };
    (void)station;
    if(blocked(current))return true;
    for(std::size_t i=0;i<6;++i)if(blocked(future[i]))return true;
    return false;
  }

  static integrated_native::PathParameters recoveryParameters(const IntegratedPathTrajectoryParameters& p) {
    auto result=p.solver;
    // A measured departure must recover with the original hard 5 cm margin;
    // the larger nominal tracking reserve must not turn it into a crawl.
    result.road_margin_m=std::min(result.road_margin_m,.05);
    result.max_deceleration_mps2=p.recovery_deceleration_mps2;
    result.max_longitudinal_jerk_mps3=p.recovery_jerk_mps3;
    result.reentry_response_sec=p.recovery_response_sec;
    result.max_lateral_accel_mps2=p.recovery_lateral_acceleration_mps2;
    result.corridor_half_width_m=std::max(result.corridor_half_width_m,p.corridor_half_width_m);
    return result;
  }

  double referenceCurveTarget(const Message& path,double target) const {
    // Match the source curvature preview against the geometry actually tracked
    // laterally. The numerical trajectory may be straighter than this reference.
    // This is a cost target: measured v/a and hard collision/road limits stay intact.
    struct Sample {double x,y,s;};
    const double spacing=p_.curve_spacing_m;
    std::vector<Sample> samples;
    if(path.points.empty())return target;
    const auto& first=path.points.front().pose.pose.position;samples.push_back({first.x,first.y,0});
    double station=0,next=spacing;
    for(std::size_t i=1;i<path.points.size() && next<=p_.curve_preview_m+spacing+1e-9;++i) {
      const auto& a=path.points[i-1].pose.pose.position;const auto& b=path.points[i].pose.pose.position;
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      if(length<1e-9)continue;
      while(next<=station+length+1e-9 && next<=p_.curve_preview_m+spacing+1e-9) {
        const double u=std::max(0.,std::min(1.,(next-station)/length));
        samples.push_back({a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),next});next+=spacing;
      }
      station+=length;
    }
    for(std::size_t i=1;i+1<samples.size();++i) {
      const auto& a=samples[i-1];const auto& b=samples[i];const auto& c=samples[i+1];
      const double denominator=std::hypot(b.x-a.x,b.y-a.y)*std::hypot(c.x-b.x,c.y-b.y)*std::hypot(c.x-a.x,c.y-a.y);
      if(denominator<1e-9)continue;
      const double curvature=2*std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/denominator;
      if(curvature<=.001)continue;
      const double curve=std::max(p_.minimum_curve_speed_mps,std::sqrt(p_.solver.preferred_lateral_accel_mps2/curvature));
      target=std::min(target,std::sqrt(curve*curve+2*p_.curve_deceleration_mps2*b.s));
    }
    return target;
  }

  static double heading(const geometry_msgs::Quaternion& q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  static integrated_native::PredictedObject object(const interfaces::Object& input) {
    integrated_native::PredictedObject o;o.id=input.id;
    o.kind=input.label=="vehicle"?integrated_native::ObjectKind::Vehicle:
        input.label=="pedestrian"?integrated_native::ObjectKind::Pedestrian:integrated_native::ObjectKind::Unknown;
    o.pose={{input.position.x,input.position.y},heading(input.bbox_pose.orientation)};
    o.velocity_mps=input.velocity_available?integrated_native::Vec2{input.velocity.x,input.velocity.y}:integrated_native::Vec2{};
    o.length_m=input.bbox_size.x;o.width_m=input.bbox_size.y;
    return o;
  }
  static integrated_native::Predictions convert(const interfaces::Objects& current,const PredictedObjects& future) {
    integrated_native::Predictions result;result.reference_stamp_sec=current.header.stamp.toSec();
    for(const auto& input:current.objects) {
      auto o=object(input);o.source_stamp_sec=result.reference_stamp_sec;
      if(!input.velocity_available) {
        const auto inferred=std::find_if(future[0].objects.begin(),future[0].objects.end(),
            [&](const interfaces::Object& q){return q.id==input.id && q.velocity_available;});
        if(inferred!=future[0].objects.end())o.velocity_mps={inferred->velocity.x,inferred->velocity.y};
      }
      result.current.push_back(o);
      for(std::size_t i=0;i<10;++i) {
        const auto found=std::find_if(future[i].objects.begin(),future[i].objects.end(),
            [&](const interfaces::Object& q){return q.id==input.id;});
        auto p=found==future[i].objects.end()?o:object(*found);
        p.source_stamp_sec=result.reference_stamp_sec;
        // The upstream solver models one rigid physical body. An enclosing
        // forecast OBB is uncertainty, not a different car at each time slice.
        p.radius_error_m=std::max({0.,(p.length_m-o.length_m)/2,(p.width_m-o.width_m)/2});
        p.length_m=o.length_m;p.width_m=o.width_m;p.kind=o.kind;p.model=o.model;
        result.future[i].push_back(p);
      }
    }
    return result;
  }

  const char* outcome_="native-search";
  IntegratedPathTrajectoryParameters p_;
  integrated_native::IntegratedSolver solver_,recovery_solver_;
  bool used_recovery_=false;
  std::vector<integrated_native::PathPoint> previous_;
  double last_stamp_=0,last_blocked_sec_=-std::numeric_limits<double>::infinity();
};
 public:
  IntegratedPath(IntegratedPathParameters p,SharedMap& map):mission_parameters_(p.mission),
      dynamic_zone_speed_mps_(p.trajectory.dynamic_zone_speed_mps),dynamic_zone_approach_m_(p.trajectory.dynamic_zone_approach_m),
      trajectory_(p.trajectory,*map.map) {}
  const char* planningOutcome()const{return trajectory_.outcome();}
  std::string planningDiagnostics()const{return trajectory_.diagnostics();}
  Message process(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& status,
      const interfaces::Objects& objects,const tf2_msgs::TFMessage& tf,const sensor_msgs::Imu& imu,
      const Route& route,const PredictedObjects& predictions,const ros::Time& stamp) {
    (void)tf;
    const auto& p=mission_parameters_;const auto& q=ego.pose.pose.position;
    const bool inside=p.dynamic_zone_enabled && q.x>=p.dynamic_zone_min_x_m && q.x<=p.dynamic_zone_max_x_m &&
        q.y>=p.dynamic_zone_min_y_m && q.y<=p.dynamic_zone_max_y_m;
    const bool approach=p.dynamic_zone_enabled && q.x>=p.dynamic_zone_min_x_m && q.x<=p.dynamic_zone_max_x_m &&
        q.y>=p.dynamic_zone_min_y_m-dynamic_zone_approach_m_ && q.y<=p.dynamic_zone_max_y_m;
    const double speed=std::abs(status.velocity_x_mps);
    const double detection=std::max(p.dynamic_zone_minimum_detection_m,speed*p.dynamic_zone_reaction_sec+
        speed*speed/(2*p.dynamic_zone_braking_mps2)+p.vehicle_front_m+p.stop_buffer_m);
    return trajectory_.calculate(ego,status,objects,imu,route,predictions,stamp,
        approach?dynamic_zone_speed_mps_:std::numeric_limits<double>::infinity(),inside,detection,p.dynamic_zone_clear_sec);
  }
 private:
  PathPlannerMissionParameters mission_parameters_;
  double dynamic_zone_speed_mps_,dynamic_zone_approach_m_;
  IntegratedTrajectory trajectory_;
};
}
