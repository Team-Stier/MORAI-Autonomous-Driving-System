#include "control/libraries/tracking/common/osqp_sqp_solver.hpp"

namespace control { namespace tracking {

OsqpSqpResult solveWithOsqpSqp(
    const std::function<double(const std::vector<double>&)>&,
    const std::vector<double>&, const std::vector<double>&, int,
    const std::vector<double>&, const std::vector<double>&,
    const OsqpSqpConfig&) {
  OsqpSqpResult result;
  result.message = "OSQP support is not built; use projected_gradient";
  return result;
}

OsqpSqpResult solveQuadraticProgram(
    const std::vector<double>&, const std::vector<double>&,
    const std::vector<double>&, int, const std::vector<double>&,
    const std::vector<double>&, const std::vector<double>&,
    const OsqpQpConfig&) {
  OsqpSqpResult result;
  result.message = "OSQP support is not built; use projected_gradient";
  return result;
}

} }  // namespace control::tracking
