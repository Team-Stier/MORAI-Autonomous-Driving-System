#include "morai_path_tracking/common/osqp_sqp_solver.hpp"

namespace morai_path_tracking {

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

}  // namespace morai_path_tracking
