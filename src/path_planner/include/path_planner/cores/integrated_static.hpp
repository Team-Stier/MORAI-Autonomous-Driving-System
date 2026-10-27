#pragma once
#include <path_planner/banks/shared_map.hpp>
#include <path_planner/libraries/integrated_static_mission.hpp>

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
struct IntegratedStaticTrajectoryParameters {
  integrated_native::PathParameters solver;
  bool publish_lateral_reference=false;
  double curve_preview_m=45.,curve_spacing_m=2.,curve_deceleration_mps2=1.7;
  double minimum_curve_speed_mps=15./3.6;
  double corridor_half_width_m=1.8;
  double physical_guard_m=.25;
  double minimum_obstacle_extent_m=2.;
  double horizon_sec=5.0;
  double recovery_deceleration_mps2=8.,recovery_jerk_mps3=8.,recovery_response_sec=.3;
  double recovery_lateral_acceleration_mps2=6.;
  double dynamic_zone_speed_mps=25./3.6,dynamic_zone_approach_m=30;
};


struct IntegratedStaticParameters {
  PathPlannerStaticMissionParameters mission;
  IntegratedStaticTrajectoryParameters trajectory;
};
class IntegratedStatic {
class IntegratedTrajectory {
 public:
  IntegratedTrajectory(IntegratedStaticTrajectoryParameters p,const lanelet::LaneletMap& map)
      : p_(std::move(p)),solver_(p_.solver,map),recovery_solver_(solver_,recoveryParameters(p_)) {}
  const char* outcome()const{return outcome_;}
  std::string diagnostics()const{return used_recovery_ ? "dynamics-recovery "+recovery_solver_.diagnostics():solver_.diagnostics();}
  Message calculate(const nav_msgs::Odometry& ego,const interfaces::CompetitionVehicleStatus& status,
      const interfaces::Objects& objects,const sensor_msgs::Imu& imu,const Route& route,
      const PredictedObjects& predictions,const ros::Time& stamp,double ceiling,bool dynamic_zone,
      double detection,double clear_sec,double avoidance_speed) {
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
      if(stamp.toSec()<last_stamp_){previous_.clear();avoidance_route_=Route{};}
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

      if(avoidance_route_ && (dynamic_zone ||
          (state.pose.position.x-return_goal_.x)*return_direction_.x+
          (state.pose.position.y-return_goal_.y)*return_direction_.y>=0))
        avoidance_route_=Route{};
      const Route planning_route=avoidance_route_?avoidance_route_:route;
      auto reference=solver_.mapReference(state,planning_route);
      // Changing the matched lane while passing an obstacle must not change
      // the maneuver's destination corridor. Search a fresh native path in
      // that corridor on every call, until the observed obstacle is passed.
      if(!dynamic_zone && !avoidance_route_ &&
         dynamicBlocked(reference,state,objects,predictions,p_.curve_preview_m)) {
        const interfaces::Object* nearest=nullptr;double distance=std::numeric_limits<double>::infinity();
        for(const auto& object:objects.objects) {
          const double dx=object.position.x-state.pose.position.x,dy=object.position.y-state.pose.position.y;
          const double d=std::hypot(dx,dy);
          if(dx*std::cos(state.pose.yaw_rad)+dy*std::sin(state.pose.yaw_rad)>0 && d<distance) {
            nearest=&object;distance=d;
          }
        }
        if(nearest) {
          double best=std::numeric_limits<double>::infinity();
          integrated_native::Vec2 direction{std::cos(state.pose.yaw_rad),std::sin(state.pose.yaw_rad)};
          for(std::size_t i=1;i<reference.size();++i) {
            const auto a=reference[i-1].position,b=reference[i].position;
            const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
            if(length<1e-9)continue;
            const double u=std::max(0.,std::min(1.,((nearest->position.x-a.x)*dx+(nearest->position.y-a.y)*dy)/(length*length)));
            const double d=std::hypot(nearest->position.x-a.x-u*dx,nearest->position.y-a.y-u*dy);
            if(d<best){best=d;direction={dx/length,dy/length};}
          }
          avoidance_route_=route;return_direction_=direction;
          return_goal_={nearest->position.x+20*direction.x,nearest->position.y+20*direction.y};
        }
      }
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
        if(dynamicBlocked(reference,state,objects,predictions,detection,p_.minimum_obstacle_extent_m))last_blocked_sec_=stamp.toSec();
        if(stamp.toSec()-last_blocked_sec_<clear_sec) {
          outcome_="dynamic-zone-stop";previous_.clear();return hold(ego);
        }
      } else last_blocked_sec_=-std::numeric_limits<double>::infinity();
      auto native=convert(objects,predictions,reference);

      // Search/track the smooth target, but construct road permission from
      // the original map supports. Reference fitting must not change road area.
      auto physical_reference=solver_.mapReference(state,planning_route,false);
      const auto coverage=physicalCorridor(physical_reference);
      physicalCorridor(reference);

      integrated_native::PlanningConstraints limits;
      limits.maximum_speed_mps=std::min(p_.solver.max_speed_mps,ceiling);
      limits.preferred_speed_mps=referenceCurveTarget(guide,limits.maximum_speed_mps);
      bool passing=false;
      for(const auto& object:objects.objects)
        passing=passing || std::hypot(object.position.x-state.pose.position.x,
            object.position.y-state.pose.position.y)<10.;
      if(!dynamic_zone && (passing || dynamicBlocked(reference,state,objects,predictions,p_.curve_preview_m))) {
        limits.maximum_speed_mps=std::min(limits.maximum_speed_mps,avoidance_speed);
        limits.preferred_speed_mps=std::min(limits.preferred_speed_mps,avoidance_speed);
      }
      auto planned=solver_.planInMap(state,reference,native,limits,planning_route,previous_,{},coverage,nullptr,&physical_reference);
      const bool stopped_free=!planned.empty() && planned.back().speed_mps==0 && objects.objects.empty();
      if((planned.empty() || stopped_free) && std::string(solver_.failureReason())!="initial-collision") {
        auto alternative=recovery_solver_.planInMap(state,reference,native,limits,planning_route,previous_,{},coverage,nullptr,&physical_reference);
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
      const PredictedObjects& future,double detection,double minimum_extent=0.)const {
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
        // Use the planning envelope for a partial first detection as well.
        // Waiting for a thin crossing box to overlap can enter slow avoidance
        // search before motion has been estimated, delaying the stop decision.
        const double half=(std::abs(std::sin(relative))*std::max(object.bbox_size.x,minimum_extent)+
            std::abs(std::cos(relative))*std::max(object.bbox_size.y,minimum_extent))/2;
        if(lateral<=p_.solver.vehicle.width_m/2+half+.25)return true;
      }
      return false;
    };
    (void)station;
    if(blocked(current))return true;
    for(std::size_t i=0;i<6;++i)if(blocked(future[i]))return true;
    return false;
  }
  std::vector<bool> physicalCorridor(std::vector<integrated_native::ReferencePoint>& reference) const {
    std::vector<bool> covered(reference.size(),false);
    for(std::size_t i=0;i<reference.size();++i) {
      const auto& a=reference[i==0?0:i-1].position;
      const auto& b=reference[i+1<reference.size()?i+1:i].position;
      const double length=std::hypot(b.x-a.x,b.y-a.y);
      if(length<1e-9)continue;
      const double nx=-(b.y-a.y)/length,ny=(b.x-a.x)/length;
      const auto origin=reference[i].position;
      const double old_min=reference[i].min_l_m,old_max=reference[i].max_l_m;bool near_paint=false;
      double negative=-std::numeric_limits<double>::infinity(),positive=std::numeric_limits<double>::infinity();
      for(const auto& line:p_.solver.physical_boundaries)for(std::size_t j=1;j<line.size();++j) {
        const double dx=line[j].x-line[j-1].x,dy=line[j].y-line[j-1].y;
        const double squared=dx*dx+dy*dy;
        if(squared>1e-9) {
          const double u=std::max(0.,std::min(1.,((origin.x-line[j-1].x)*dx+(origin.y-line[j-1].y)*dy)/squared));
          near_paint=near_paint || std::hypot(origin.x-line[j-1].x-u*dx,origin.y-line[j-1].y-u*dy)<=6;
        }
        const double den=nx*dy-ny*dx;if(std::abs(den)<1e-9)continue;
        const double ax=line[j-1].x-origin.x,ay=line[j-1].y-origin.y;
        const double d=(ax*dy-ay*dx)/den,u=(ax*ny-ay*nx)/den;
        if(u<0 || u>1 || std::abs(d)>6)continue;
        if(d<0)negative=std::max(negative,d);else positive=std::min(positive,d);
      }
      if(std::isfinite(negative)){reference[i].min_l_m=negative+p_.physical_guard_m;covered[i]=true;}
      if(std::isfinite(positive)){reference[i].max_l_m=positive-p_.physical_guard_m;covered[i]=true;}
      if(reference[i].max_l_m-reference[i].min_l_m<p_.solver.vehicle.width_m+2*p_.solver.road_margin_m) {
        reference[i].min_l_m=old_min;reference[i].max_l_m=old_max;
      }
      // Paint endpoints at open junctions need not intersect the reference
      // normal. Keep that local connection continuous; exact body/paint sweeps
      // remain hard constraints in the numerical solver.
      covered[i]=near_paint;
    }
    return covered;
  }

  static integrated_native::PathParameters recoveryParameters(const IntegratedStaticTrajectoryParameters& p) {
    auto result=p.solver;
    // Recover an already consumed tracking reserve with the original hard
    // footprint margin; keep physical paint and all collision checks active.
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
  integrated_native::Predictions convert(const interfaces::Objects& current,const PredictedObjects& future,
      const std::vector<integrated_native::ReferencePoint>& reference) const {
    (void)future;
    integrated_native::Predictions result;result.reference_stamp_sec=current.header.stamp.toSec();
    // The supplied static mission deliberately treats section-4 observations
    // as fixed obstacles. Preserve that model in numerical collision timing;
    // centroid motion from changing LiDAR faces is not vehicle motion here.
    // The mission's dynamic-zone stop is evaluated before this calculation.
    for(const auto& input:current.objects) {
      auto o=object(input);o.source_stamp_sec=result.reference_stamp_sec;o.velocity_mps={0,0};
      // Partial vehicle faces rotate the fitted PCA box as the observer
      // passes. Use the road-axis prior of this static vehicle mission; keep
      // the measured dimensions on their nearest longitudinal/lateral axes.
      if(input.label=="unknown" || input.label=="vehicle") {
        double best=std::numeric_limits<double>::infinity(),yaw=o.pose.yaw_rad;
        for(std::size_t i=1;i<reference.size();++i) {
          const auto a=reference[i-1].position,b=reference[i].position;
          const double dx=b.x-a.x,dy=b.y-a.y,n=dx*dx+dy*dy;
          if(n<1e-9)continue;
          const double u=std::max(0.,std::min(1.,((o.pose.position.x-a.x)*dx+(o.pose.position.y-a.y)*dy)/n));
          const double distance=std::hypot(o.pose.position.x-a.x-u*dx,o.pose.position.y-a.y-u*dy);
          if(distance<best){best=distance;yaw=std::atan2(dy,dx);}
        }
        const double delta=o.pose.yaw_rad-yaw;
        if(std::abs(std::sin(delta))>std::abs(std::cos(delta)))std::swap(o.length_m,o.width_m);
        o.pose.yaw_rad=yaw;
      }
      // A thin LiDAR face is not the full static obstacle. Preserve the
      // original static mission's unseen-face reserve in native search too.
      o.length_m=std::max(o.length_m,p_.minimum_obstacle_extent_m);
      o.width_m=std::max(o.width_m,p_.minimum_obstacle_extent_m);
      result.current.push_back(o);
      for(auto& frame:result.future)frame.push_back(o);
    }
    return result;
  }

  const char* outcome_="native-search";
  IntegratedStaticTrajectoryParameters p_;
  integrated_native::IntegratedSolver solver_,recovery_solver_;
  bool used_recovery_=false;
  std::vector<integrated_native::PathPoint> previous_;
  Route avoidance_route_;
  integrated_native::Vec2 return_goal_{},return_direction_{};
  double last_stamp_=0,last_blocked_sec_=-std::numeric_limits<double>::infinity();
};
 public:
  IntegratedStatic(IntegratedStaticParameters p,SharedMap& map):mission_parameters_(p.mission),
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
        approach?dynamic_zone_speed_mps_:std::numeric_limits<double>::infinity(),inside,detection,p.dynamic_zone_clear_sec,p.avoidance_speed_mps);
  }
 private:
  PathPlannerStaticMissionParameters mission_parameters_;
  double dynamic_zone_speed_mps_,dynamic_zone_approach_m_;
  IntegratedTrajectory trajectory_;
};
}
