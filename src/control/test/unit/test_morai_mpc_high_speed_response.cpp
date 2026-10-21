#include <control/cores/morai_mpc.hpp>
#include <control/parameters/morai_mpc.hpp>
#include <gtest/gtest.h>
#include <deque>

namespace {
using namespace control::autoware_ai_mpc;

LongitudinalMpcConfig responseConfig() {
  const std::string package=NP_PACKAGE_DIR;
  return control::loadMoraiMpcParameters(node_pipeline::ParamReader::fromFile(
      package+"/config/control/parameters/morai_mpc.yaml",package)).longitudinal;
}

TEST(MoraiMpcHighSpeedResponse, BrakesForTheRecordedHighwayDecelerationRamp) {
  ros::Time::init();const auto config=responseConfig();LongitudinalMpc core(config);
  std::vector<LongitudinalReference> reference;
  // 출구 실패 직전 입력: 22m/s에서 양의 실측 가속도를 jerk 4로 줄인다.
  // 0.3초 경과한 계획의 감속 참조에 가속 페달을 내면 실제 차량은 빨라진다.
  constexpr double initial_acceleration=.335;
  const double switch_time=(initial_acceleration+5)/4;
  const double switch_speed=22+initial_acceleration*switch_time-2*switch_time*switch_time;
  for(int k=1;k<=config.horizon_steps;++k) {
    const double t=.3+k*config.prediction_dt_sec;
    reference.push_back(t<=switch_time ?
        LongitudinalReference{22+initial_acceleration*t-2*t*t,initial_acceleration-4*t} :
        LongitudinalReference{switch_speed-5*(t-switch_time),-5});
  }
  const auto command=core.updateReference(reference,22,.05);
  EXPECT_DOUBLE_EQ(command.accel,0);
  EXPECT_GT(command.brake,0);
  EXPECT_LE(command.brake,config.maximum_brake_command);
}

// 모델 계수를 그대로 plant로 복제하지 않는다. 세 주행의 관측 범위에서
// 저속/중속 응답 전환과 0.1초 입력 지연을 갖는 독립 차량을 사용한다.
double maximumTrackingError(double initial,double target,int variation) {
  auto config=responseConfig();LongitudinalMpc core(config);
  std::deque<double> delay(2,0);double speed=initial,acceleration=0,maximum=0;
  const auto reference=[&](double time) {
    const double x=std::max(0.,std::min(1.,(time-1)/5));
    return LongitudinalReference{initial+(target-initial)*(3*x*x-2*x*x*x),
        (x==0 || x==1) ? 0 : (target-initial)*(6*x-6*x*x)/5};
  };
  for(int i=0;i<200;++i) {
    const double time=i*.05;
    std::vector<LongitudinalReference> references;
    for(int k=1;k<=config.horizon_steps;++k)
      references.push_back(reference(time+k*config.prediction_dt_sec));
    const auto command=core.updateReference(references,speed,.05);
    EXPECT_GE(command.accel,0);EXPECT_LE(command.accel,config.maximum_accel_command);
    EXPECT_GE(command.brake,0);EXPECT_LE(command.brake,config.maximum_brake_command);
    delay.push_back(command.signed_command);const double u=delay.front();delay.pop_front();
    double ratio=std::max(0.,std::min(1.,(speed-3)/3));ratio=ratio*ratio*(3-2*ratio);
    const double gain=(5.74+.2*variation)*(1-ratio)+(2.7+.2*variation)*ratio;
    double brake_gain=speed<6 ? 18.76 : 9.23456506625;
    double pressure=std::max(0.,std::min(1.,(-u-.18)/.24));pressure=pressure*pressure*(3-2*pressure);
    brake_gain+=(18.76-brake_gain)*pressure;
    const double desired=(u>=0 ? gain*u : brake_gain*u)-(.07+.02*variation)-
        (.0005+.0001*variation)*speed*speed;
    acceleration+=(1-std::exp(-.05/(.075+.02*variation)))*(desired-acceleration);
    speed=std::max(0.,speed+acceleration*.05);
    if(speed==0 && acceleration<0)acceleration=0;
    maximum=std::max(maximum,std::abs(speed-reference(time+.05).speed_mps));
  }
  return maximum;
}

TEST(MoraiMpcHighSpeedResponse, PreservesLowSpeedDepartureAndStandstill) {
  ros::Time::init();
  for(int variation:{-1,0,1}) {
    EXPECT_LT(maximumTrackingError(0,1,variation),.10);
    EXPECT_DOUBLE_EQ(maximumTrackingError(0,0,variation),0);
  }
}

TEST(MoraiMpcHighSpeedResponse, TracksFeasibleHighSpeedCruiseAndDeceleration) {
  ros::Time::init();
  for(int variation:{-1,0,1}) {
    SCOPED_TRACE(variation);
    EXPECT_LT(maximumTrackingError(22,23,variation),.15);
    EXPECT_LT(maximumTrackingError(22,14,variation),.15);
  }
}
}  // namespace
