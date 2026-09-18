#include <vector>

#include <gtest/gtest.h>

#include "morai_path_tracking/common/osqp_sqp_solver.hpp"

namespace morai_path_tracking {
namespace {

TEST(OsqpQuadraticProgramTest, PreservesAdjacentControlCoupling) {
  constexpr int kVariables = 3;
  constexpr double kSmoothnessWeight = 10.0;
  std::vector<double> hessian(kVariables * kVariables, 0.0);
  std::vector<double> gradient{-2.0, 2.0, -2.0};
  for (int index = 0; index < kVariables; ++index) {
    hessian[index * kVariables + index] += 2.0;
  }
  for (int index = 1; index < kVariables; ++index) {
    hessian[index * kVariables + index] += 2.0 * kSmoothnessWeight;
    hessian[(index - 1) * kVariables + index - 1] +=
        2.0 * kSmoothnessWeight;
    hessian[index * kVariables + index - 1] -=
        2.0 * kSmoothnessWeight;
    hessian[(index - 1) * kVariables + index] -=
        2.0 * kSmoothnessWeight;
  }
  const std::vector<double> constraints{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0,
  };
  const std::vector<double> lower{-2.0, -2.0, -2.0};
  const std::vector<double> upper{2.0, 2.0, 2.0};

  const OsqpSqpResult result = solveQuadraticProgram(
      hessian, gradient, constraints, kVariables, lower, upper,
      {0.0, 0.0, 0.0}, OsqpQpConfig{});

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.solution.size(), 3U);
  EXPECT_NEAR(result.solution[0], 11.0 / 31.0, 1.0e-4);
  EXPECT_NEAR(result.solution[1], 9.0 / 31.0, 1.0e-4);
  EXPECT_NEAR(result.solution[2], 11.0 / 31.0, 1.0e-4);
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
