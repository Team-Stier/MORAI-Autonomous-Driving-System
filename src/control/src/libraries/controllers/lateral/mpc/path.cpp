#include "control/libraries/tracking/controllers/lateral/mpc/path.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cstdint>

namespace morai_mpc {
namespace { double dist(Point2d a, Point2d b){return std::hypot(b.x-a.x,b.y-a.y);} }

ReferencePath::ReferencePath(const std::vector<Point2d>& input, bool closed, CoordinateFrame frame) : closed_(closed), frame_(frame) {
  if(input.size()<2) throw std::invalid_argument("경로에는 서로 다른 점이 2개 이상 필요함");
  std::vector<Point2d> p=input;
  for(const auto& q:p) if(!valid(q)) throw std::invalid_argument("경로에 NaN/Inf가 있음");
  if(closed_ && dist(p.front(),p.back())<1e-8) p.pop_back();
  if(p.size()<2) throw std::invalid_argument("유효한 경로점 부족");
  for(std::size_t i=1;i<p.size();++i) if(dist(p[i-1],p[i])<1e-8) throw std::invalid_argument("연속 중복 경로점");
  if(closed_ && dist(p.back(),p.front())<1e-8) throw std::invalid_argument("폐경로의 마지막 구간이 너무 짧음");
  points_.resize(p.size());
  double s=0.0;
  for(std::size_t i=0;i<p.size();++i){ if(i) s+=dist(p[i-1],p[i]); points_[i].x=p[i].x;points_[i].y=p[i].y;points_[i].s=s; }
  length_=s+(closed_?dist(p.back(),p.front()):0.0);
  const std::size_t n=p.size();
  for(std::size_t i=0;i<n;++i){
    Point2d a,b;
    if(closed_){a=p[(i+n-1)%n];b=p[(i+1)%n];}
    else if(i==0){a=p[0];b=p[1];} else if(i+1==n){a=p[n-2];b=p[n-1];} else {a=p[i-1];b=p[i+1];}
    points_[i].heading=std::atan2(b.y-a.y,b.x-a.x);
  }
  for(std::size_t i=0;i<n;++i){
    if(!closed_ && (i==0||i+1==n)){points_[i].curvature=0.0;continue;}
    std::size_t im=(i+n-1)%n, ip=(i+1)%n;
    double ds=0.5*(dist(p[im],p[i])+dist(p[i],p[ip]));
    points_[i].curvature=normalizeAngle(points_[ip].heading-points_[im].heading)/(2.0*ds);
  }
  if(!closed_ && n>2){points_[0].curvature=points_[1].curvature;points_[n-1].curvature=points_[n-2].curvature;}
}

ReferencePoint ReferencePath::sample(double s) const {
  if(points_.empty()) throw std::runtime_error("빈 경로");
  if(closed_){s=std::fmod(s,length_);if(s<0)s+=length_;} else s=clamp(s,0.0,length_);
  std::size_t i=0,j=1;
  if(closed_ && s>=points_.back().s){i=points_.size()-1;j=0;}
  else {auto it=std::upper_bound(points_.begin(),points_.end(),s,[](double v,const ReferencePoint& p){return v<p.s;});
    if(it==points_.begin()) return points_.front();
    if(it==points_.end()) return points_.back();
    j=static_cast<std::size_t>(it-points_.begin());i=j-1;}
  double s0=points_[i].s,s1=(j==0?length_:points_[j].s),t=(s-s0)/(s1-s0);
  ReferencePoint r; r.x=lerp(points_[i].x,points_[j].x,t);r.y=lerp(points_[i].y,points_[j].y,t);r.s=s;
  r.heading=normalizeAngle(points_[i].heading+t*normalizeAngle(points_[j].heading-points_[i].heading));
  r.curvature=lerp(points_[i].curvature,points_[j].curvature,t);return r;
}

Projection ReferencePath::project(const Pose2d& pose) const {
  Projection out; if(points_.size()<2||!finite(pose.x)||!finite(pose.y)||!finite(pose.yaw)) return out;
  double best=std::numeric_limits<double>::infinity(); std::size_t count=closed_?points_.size():points_.size()-1;
  for(std::size_t i=0;i<count;++i){std::size_t j=(i+1)%points_.size();double dx=points_[j].x-points_[i].x,dy=points_[j].y-points_[i].y,l2=dx*dx+dy*dy;
    double t=clamp(((pose.x-points_[i].x)*dx+(pose.y-points_[i].y)*dy)/l2,0.0,1.0);double qx=points_[i].x+t*dx,qy=points_[i].y+t*dy,d2=(pose.x-qx)*(pose.x-qx)+(pose.y-qy)*(pose.y-qy);
    if(d2<best){best=d2;out.segment=i;out.ratio=t;out.point={qx,qy};double seglen=std::sqrt(l2);out.s=points_[i].s+t*seglen;
      double h=std::atan2(dy,dx);out.lateral_error=(-std::sin(h)*(pose.x-qx)+std::cos(h)*(pose.y-qy));out.heading_error=normalizeAngle(pose.yaw-h);}
  }
  out.reference=sample(out.s); out.reference.x=out.point.x;out.reference.y=out.point.y;out.valid=true;return out;
}

Projection ReferencePath::projectHeadingAware(const Pose2d& pose,double tolerance)const{
  Projection nearest=project(pose);if(!nearest.valid||!finite(tolerance)||tolerance<0)return Projection{};
  const double nearest_d2=(pose.x-nearest.point.x)*(pose.x-nearest.point.x)+(pose.y-nearest.point.y)*(pose.y-nearest.point.y);
  double best_score=std::numeric_limits<double>::infinity();Projection out;std::size_t count=closed_?points_.size():points_.size()-1;
  for(std::size_t i=0;i<count;++i){std::size_t j=(i+1)%points_.size();double dx=points_[j].x-points_[i].x,dy=points_[j].y-points_[i].y,l2=dx*dx+dy*dy;if(l2<=1e-16)continue;double t=clamp(((pose.x-points_[i].x)*dx+(pose.y-points_[i].y)*dy)/l2,0.0,1.0);double qx=points_[i].x+t*dx,qy=points_[i].y+t*dy,d2=(pose.x-qx)*(pose.x-qx)+(pose.y-qy)*(pose.y-qy);if(d2>nearest_d2+tolerance*tolerance)continue;double h=std::atan2(dy,dx),dh=std::abs(normalizeAngle(pose.yaw-h));double score=d2+.5*dh*dh;if(score<best_score){best_score=score;out.segment=i;out.ratio=t;out.point={qx,qy};out.s=points_[i].s+t*std::sqrt(l2);out.lateral_error=-std::sin(h)*(pose.x-qx)+std::cos(h)*(pose.y-qy);out.heading_error=normalizeAngle(pose.yaw-h);}}
  if(!std::isfinite(best_score))return nearest;
  out.reference=sample(out.s);out.reference.x=out.point.x;out.reference.y=out.point.y;out.valid=true;return out;
}

Projection ReferencePath::projectWindow(const Pose2d& pose, double center, double backward,
                                        double forward, double* projected_unwrapped_s) const {
  Projection out;
  if (projected_unwrapped_s) *projected_unwrapped_s=center;
  if(points_.size()<2||!finite(pose.x)||!finite(pose.y)||!finite(pose.yaw)||
     !finite(center)||!finite(backward)||!finite(forward)||backward<0||forward<=0) return out;
  double best=std::numeric_limits<double>::infinity(),best_unwrapped=center;
  const std::size_t count=closed_?points_.size():points_.size()-1;
  for(std::size_t i=0;i<count;++i){
    std::size_t j=(i+1)%points_.size();double dx=points_[j].x-points_[i].x,dy=points_[j].y-points_[i].y,l2=dx*dx+dy*dy;
    if(l2<=1e-16)continue;
    double t=clamp(((pose.x-points_[i].x)*dx+(pose.y-points_[i].y)*dy)/l2,0.0,1.0);
    double base=points_[i].s+t*std::sqrt(l2),candidate=base;
    if(closed_){double laps=std::round((center-base)/length_);candidate=base+laps*length_;}
    if(candidate<center-backward||candidate>center+forward)continue;
    double qx=points_[i].x+t*dx,qy=points_[i].y+t*dy,d2=(pose.x-qx)*(pose.x-qx)+(pose.y-qy)*(pose.y-qy);
    if(d2<best){best=d2;best_unwrapped=candidate;out.segment=i;out.ratio=t;out.point={qx,qy};out.s=closed_?std::fmod(candidate+length_,length_):candidate;
      double h=std::atan2(dy,dx);out.lateral_error=-std::sin(h)*(pose.x-qx)+std::cos(h)*(pose.y-qy);out.heading_error=normalizeAngle(pose.yaw-h);}
  }
  if(!std::isfinite(best))return out;
  out.reference=sample(out.s);out.reference.x=out.point.x;out.reference.y=out.point.y;out.valid=true;
  if(projected_unwrapped_s)*projected_unwrapped_s=best_unwrapped;
  return out;
}

PathProgressTracker::PathProgressTracker(PathProgressConfig config):config_(config){
  if(config.backward_window<0||config.forward_window<=0||config.relocalize_distance<=0||
     config.teleport_distance<=0||config.jump_threshold<=0)throw std::invalid_argument("invalid path progress config");
}
void PathProgressTracker::reset(){initialized_=false;unwrapped_s_=0;path_signature_=0;last_pose_={};}
std::uint64_t PathProgressTracker::signature(const ReferencePath& path)const{
  if(path.empty())return 0;
  const auto&p=path.points();
  auto quant=[](double v){return static_cast<std::uint64_t>(std::llround(v*1000.0));};
  std::uint64_t h=1469598103934665603ULL;
  auto mix=[&](std::uint64_t v){h^=v;h*=1099511628211ULL;};
  mix(p.size());mix(quant(path.length()));mix(quant(p.front().x));mix(quant(p.front().y));mix(quant(p.back().x));mix(quant(p.back().y));mix(path.closed()?1:0);return h;
}
ProgressProjection PathProgressTracker::project(const ReferencePath& path,const Pose2d& pose,double speed){
  ProgressProjection result;if(path.empty()||!finite(pose.x)||!finite(pose.y)||!finite(pose.yaw)||!finite(speed))return result;
  const auto sig=signature(path);bool path_changed=initialized_&&sig!=path_signature_;
  double moved=initialized_?std::hypot(pose.x-last_pose_.x,pose.y-last_pose_.y):0;
  if(!initialized_||path_changed||moved>config_.teleport_distance){result.projection=path.projectHeadingAware(pose);if(!result.projection.valid)return result;result.unwrapped_s=result.projection.s;result.relocalized=initialized_;}
  else{
    double fw=std::abs(speed)<config_.low_speed_threshold?config_.low_speed_forward_window:config_.forward_window;
    double candidate=unwrapped_s_;result.projection=path.projectWindow(pose,unwrapped_s_,config_.backward_window,fw,&candidate);
    double distance=result.projection.valid?std::hypot(pose.x-result.projection.point.x,pose.y-result.projection.point.y):std::numeric_limits<double>::infinity();
    if(!result.projection.valid||distance>config_.relocalize_distance){result.projection=path.projectHeadingAware(pose);if(!result.projection.valid)return result;candidate=result.projection.s;if(path.closed())candidate+=std::round((unwrapped_s_-candidate)/path.length())*path.length();result.relocalized=true;}
    if(!result.relocalized&&candidate<unwrapped_s_-config_.backward_window){candidate=unwrapped_s_;result.backtrack_prevented=true;result.projection=path.projectWindow(pose,candidate,0,fw,&candidate);}
    result.jump_detected=std::abs(candidate-unwrapped_s_)>config_.jump_threshold&&!result.relocalized;result.unwrapped_s=candidate;
  }
  initialized_=true;unwrapped_s_=result.unwrapped_s;path_signature_=sig;last_pose_=pose;
  result.lap=path.closed()?static_cast<int>(std::floor(unwrapped_s_/path.length())):0;return result;
}

ReferencePath ReferencePath::resample(double ds) const {
  if(!finite(ds)||ds<=0) throw std::invalid_argument("재샘플링 간격은 양수");
  std::vector<Point2d> p; int n=std::max(2,static_cast<int>(std::ceil(length_/ds))+(closed_?0:1));
  double actual=closed_?length_/n:length_/(n-1); for(int i=0;i<n;++i){auto r=sample(i*actual);p.push_back({r.x,r.y});}
  return ReferencePath(p,closed_,frame_);
}

ReferencePath ReferencePath::toGlobal(const Pose2d& pose) const {
  if(frame_==CoordinateFrame::Global) return *this;
  if(!finite(pose.x)||!finite(pose.y)||!finite(pose.yaw)) throw std::invalid_argument("invalid transform pose");
  const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);std::vector<Point2d> p;p.reserve(points_.size());
  for(const auto&r:points_)p.push_back({pose.x+c*r.x-s*r.y,pose.y+s*r.x+c*r.y});
  return ReferencePath(p,closed_,CoordinateFrame::Global);
}

ReferencePath makeStraight(double length,double spacing){if(length<=0||spacing<=0)throw std::invalid_argument("invalid straight");std::vector<Point2d> p;int n=std::max(2,int(std::ceil(length/spacing))+1);for(int i=0;i<n;++i)p.push_back({length*i/(n-1),0});return ReferencePath(p);}
ReferencePath makeArc(double radius,double angle,double spacing){if(radius<=0||std::abs(angle)<1e-6||spacing<=0)throw std::invalid_argument("invalid arc");int n=std::max(3,int(std::ceil(std::abs(radius*angle)/spacing))+1);std::vector<Point2d>p;double sign=angle>0?1:-1;for(int i=0;i<n;++i){double a=angle*i/(n-1);p.push_back({radius*std::sin(std::abs(a)),sign*radius*(1-std::cos(std::abs(a))) });}return ReferencePath(p);}
ReferencePath makeCircle(double radius,double spacing){if(radius<=0||spacing<=0)throw std::invalid_argument("invalid circle");int n=std::max(12,int(std::ceil(2*kPi*radius/spacing)));std::vector<Point2d>p;for(int i=0;i<n;++i){double a=2*kPi*i/n;p.push_back({radius*std::sin(a),radius*(1-std::cos(a))});}return ReferencePath(p,true);}
ReferencePath makeSCurve(double length,double amplitude,double spacing){if(length<=0||spacing<=0)throw std::invalid_argument("invalid s curve");int n=std::max(10,int(std::ceil(length/spacing))+1);std::vector<Point2d>p;for(int i=0;i<n;++i){double x=length*i/(n-1);double y=amplitude*(1-std::cos(2*kPi*x/length));p.push_back({x,y});}return ReferencePath(p);}
}  // namespace morai_mpc
