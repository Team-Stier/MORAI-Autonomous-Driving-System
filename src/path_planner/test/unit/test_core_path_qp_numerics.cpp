#include <chrono>
#include <iostream>
#include <string>
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
namespace {
using Core=path_planner::CorePathPlanner;
TEST(CorePathQpNumerics, SolvesABandedSystemWithKnownInteriorOptimum) {
  Core core;constexpr Eigen::Index n=300;
  Eigen::MatrixXd h=2*Eigen::MatrixXd::Identity(n,n);
  for(Eigen::Index i=1;i<n;++i) { h(i-1,i-1)+=3;h(i,i)+=3;h(i-1,i)=-3;h(i,i-1)=-3; }
  Eigen::VectorXd wanted(n);for(Eigen::Index i=0;i<n;++i)wanted[i]=.4*std::sin(.03*i);
  const Eigen::VectorXd q=-h*wanted,lo=Eigen::VectorXd::Constant(n,-1),hi=Eigen::VectorXd::Constant(n,1);
  const Eigen::MatrixXd a=Eigen::MatrixXd::Identity(n,n);Eigen::VectorXd x=Eigen::VectorXd::Zero(n);
  ASSERT_TRUE(core.solveBoundedQp(h,q,a,lo,hi,x));
  EXPECT_LT((x-wanted).lpNorm<Eigen::Infinity>(),1e-5);
}
TEST(CorePathQpNumerics, PreservesIndependentActiveBounds) {
  Core core;constexpr Eigen::Index n=300;
  const Eigen::MatrixXd h=2*Eigen::MatrixXd::Identity(n,n),a=Eigen::MatrixXd::Identity(n,n);
  Eigen::VectorXd q(n),wanted(n);for(Eigen::Index i=0;i<n;++i) { wanted[i]=i%2?1:-1;q[i]=-4*wanted[i]; }
  const Eigen::VectorXd lo=Eigen::VectorXd::Constant(n,-1),hi=Eigen::VectorXd::Constant(n,1);Eigen::VectorXd x=Eigen::VectorXd::Zero(n);
  ASSERT_TRUE(core.solveBoundedQp(h,q,a,lo,hi,x));
  EXPECT_LT((x-wanted).lpNorm<Eigen::Infinity>(),1e-5);
}
TEST(CorePathQpNumerics, DoesNotShareMatricesBetweenSeparateProblems) {
  Core core;const Eigen::MatrixXd a=Eigen::MatrixXd::Identity(4,4);
  const Eigen::VectorXd lo=Eigen::VectorXd::Constant(4,-1),hi=Eigen::VectorXd::Constant(4,1);
  for(double scale:{2.,7.,2.}) {
    const Eigen::MatrixXd h=scale*a;const Eigen::VectorXd q=Eigen::VectorXd::Constant(4,-3*scale);
    Eigen::VectorXd x=Eigen::VectorXd::Zero(4);
    ASSERT_TRUE(core.solveBoundedQp(h,q,a,lo,hi,x));
    EXPECT_LT((x-Eigen::VectorXd::Ones(4)).lpNorm<Eigen::Infinity>(),1e-5);
  }
}
TEST(CorePathQpNumerics, RejectsIncompatibleConstraints) {
  Core core;const Eigen::MatrixXd h=Eigen::MatrixXd::Identity(1,1);Eigen::MatrixXd a(2,1);a<<1,1;
  const Eigen::VectorXd q=Eigen::VectorXd::Zero(1);Eigen::VectorXd lo(2),hi(2),x=Eigen::VectorXd::Zero(1);lo<<-1,1;hi<<0,2;
  EXPECT_FALSE(core.solveBoundedQp(h,q,a,lo,hi,x));
}
}
