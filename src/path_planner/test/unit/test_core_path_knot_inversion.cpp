#include <sstream>
#include <sensor_msgs/Imu.h>
#include <path_planner/libraries/types.hpp>
#include <Eigen/Geometry>
// Standalone core regression: no ROS node, message conversion, or simulator.
// Compile with C++14, Eigen3, GoogleTest, and src/path_planner/include.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
// Inspect existing numerical helpers without adding a testing API to the core.
#define private public
#include <path_planner/cores/core_path_planner.hpp>
#undef private
#include <gtest/gtest.h>
#include <path_planner/cores/global_path.hpp>
#include <path_planner/parameters/shared_map.hpp>
#include <path_planner/parameters/core_path_planner.hpp>

namespace {
using namespace stier;
using PathGenerationCore = path_planner::CorePathPlanner;
using geometry = path_planner::CorePathPlanner::Geometry;
using C=PathGenerationCore;
struct FrozenSampler : C {
  std::vector<PathPoint> oldSampleEdge(const ArcPath& arc,const SpeedEdge& edge,
                                    bool for_output=false) {
    std::vector<double> times{0,edge.h};
    const double end=edge.s0+edge.distance(edge.h);
    // #OUTPUT_KNOT_ROUNDTRIP
    // map 공간 knot와 속도 phase 끝이 약 0.0001초 차이면 출력 v/a를 다시
    // Hermite 식에 넣을 때 반올림만으로 jerk 4가 4.00000007처럼 계산된다.
    // 최종 출력에만 phase 끝 1ms 안의 공간 knot를 생략해 이 퇴화를 피한다.
    // 4ms보다 짧은 phase는 그 길이의 1/4까지만 적용하며 phase 끝 자체와
    // 실측 초기 v/a는 유지한다. 탐색의 충돌·횡가속도 표본은 원래 그대로다.
    // 새 출력 chord는 기존 전체 도로·swept 충돌·곡률·가감속·jerk와 거리
    // 재검사를 통과해야 한다. 한도·오차 허용값·경로 시각은 바꾸지 않는다.
    // 이후 ROI/차선 경계가 바뀌어도 이 최종 재검사를 생략하면 안 된다.
    auto knot=std::upper_bound(arc.interpolation_knots.begin(),arc.interpolation_knots.end(),edge.s0+1e-10);
    for(;knot!=arc.interpolation_knots.end() && *knot<end-1e-10;++knot) {
      double lo=0,hi=edge.h;
      for(int k=0;k<45;++k) {
        const double mid=(lo+hi)/2;
        if(edge.s0+edge.distance(mid)<*knot) lo=mid; else hi=mid;
      }
      const double knot_time_sec=(lo+hi)/2;
      const double minimum_span_sec=std::min(.001,edge.h/4);
      if(!for_output || (knot_time_sec>=minimum_span_sec && edge.h-knot_time_sec>=minimum_span_sec))
        times.push_back(knot_time_sec);
    }
    std::sort(times.begin(),times.end());
    times.erase(std::unique(times.begin(),times.end(),[](double a,double b){return std::abs(a-b)<1e-10;}),times.end());
    // Subdivide AFTER inserting geometry knots. Unioning two independent grids
    // creates near-coincident samples and ill-conditioned Hermite coefficients.
    std::vector<double> refined{times.front()};
    for(std::size_t i=1;i<times.size();++i) {
      const double span=times[i]-times[i-1];
      const int count=std::max(1,static_cast<int>(std::ceil(span/p_.trajectory_output_step_sec)));
      for(int k=1;k<=count;++k) refined.push_back(times[i-1]+span*k/count);
    }
    std::vector<PathPoint> out;
    for(double t:refined) {
      auto q=arc.at(edge.s0+edge.distance(t));
      q.time_sec=edge.t0+t; q.speed_mps=edge.speed(t); q.acceleration_mps2=edge.acceleration(t);
      // Endpoint arithmetic must retain exact zero v/a for a stopped state.
      if(t==0) { q.speed_mps=edge.v0; q.acceleration_mps2=edge.a0; }
      if(t==edge.h) { q.speed_mps=edge.v1; q.acceleration_mps2=edge.a1; }
      out.push_back(q);
    }
    return out;
  }

};

TEST(CorePathKnotInversion, PreservesEverySampleBitForMovingBrakingAndRestEdges) {
  C core;FrozenSampler oracle;
  std::vector<PathPoint> points;
  for(int i=0;i<=400;++i){const double a=i*.25/100;points.push_back({{{100*std::sin(a),100*(1-std::cos(a))},a},i*.25,0,.01});}
  const C::ArcPath arc(points);
  const std::vector<C::SpeedEdge> special{{0,2,0,2,0,0,.5},{0,2,0,2.5,1,0,.5},
      {0,0,0,2,4,0,1},{0,2,-4,0,0,0,1},{2,4,0,4,0,.2,.3}};
  for(const auto& edge:special)for(bool output:{false,true}){
    const auto expected=oracle.oldSampleEdge(arc,edge,output),actual=core.sampleEdge(arc,edge,output);
    ASSERT_EQ(actual.size(),expected.size());
    for(std::size_t i=0;i<actual.size();++i){EXPECT_EQ(actual[i].time_sec,expected[i].time_sec);EXPECT_EQ(actual[i].pose.position.x,expected[i].pose.position.x);EXPECT_EQ(actual[i].speed_mps,expected[i].speed_mps);}
  }
  // Finite deterministic sweep includes different initial stations and curvatures.
  for(int seed=0;seed<180;++seed){
    const double v0=2+(seed*31%170)/10.,v1=2+(seed*73%170)/10.;
    const double a0=-3+(seed*13%50)/10.,a1=-3+(seed*19%50)/10.;
    const C::SpeedEdge edge{(seed%13)*.13,v0,a0,v1,a1,(seed%7)*.5,.5};
    for(bool output:{false,true}){
      const auto expected=oracle.oldSampleEdge(arc,edge,output),actual=core.sampleEdge(arc,edge,output);
      ASSERT_EQ(actual.size(),expected.size())<<seed;
      for(std::size_t i=0;i<actual.size();++i){EXPECT_EQ(actual[i].time_sec,expected[i].time_sec)<<seed;EXPECT_EQ(actual[i].pose.position.x,expected[i].pose.position.x)<<seed;EXPECT_EQ(actual[i].pose.position.y,expected[i].pose.position.y)<<seed;EXPECT_EQ(actual[i].speed_mps,expected[i].speed_mps)<<seed;EXPECT_EQ(actual[i].acceleration_mps2,expected[i].acceleration_mps2)<<seed;}
    }
  }
}
}
