#pragma once

#include <functional>
#include <string>
#include <vector>

namespace morai_path_tracking {

struct OsqpSqpConfig {
  int sqp_iterations{2};
  int maximum_iterations{1000};
  double finite_difference_epsilon{0.01};
  double hessian_regularization{1.0e-4};
  double absolute_tolerance{1.0e-4};
  double relative_tolerance{1.0e-4};
  bool polish{true};
};

struct OsqpSqpResult {
  bool success{false};
  std::vector<double> solution;
  double cost{0.0};
  int iterations{0};
  std::string message;
};

struct OsqpQpConfig {
  int maximum_iterations{1000};
  double absolute_tolerance{1.0e-4};
  double relative_tolerance{1.0e-4};
  bool polish{true};
  bool warm_start{true};
};

// Dense row-major A is converted to OSQP's sparse CSC representation. The
// nonlinear objective is handled by sequential diagonal quadratic models.
OsqpSqpResult solveWithOsqpSqp(
    const std::function<double(const std::vector<double>&)>& objective,
    const std::vector<double>& initial,
    const std::vector<double>& constraint_matrix, int constraint_rows,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds, const OsqpSqpConfig& config);

// Solves 0.5*x'*H*x + g'*x with l <= A*x <= u. H is a dense,
// row-major, symmetric positive-semidefinite matrix. Only its upper triangle
// is passed to OSQP. This entry point is used by the condensed linear MPC so
// temporal coupling terms are represented exactly rather than estimated by
// a diagonal finite-difference Hessian.
OsqpSqpResult solveQuadraticProgram(
    const std::vector<double>& hessian,
    const std::vector<double>& gradient,
    const std::vector<double>& constraint_matrix, int constraint_rows,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds,
    const std::vector<double>& warm_start,
    const OsqpQpConfig& config);

}  // namespace morai_path_tracking
