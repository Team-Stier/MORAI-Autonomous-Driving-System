#include "control/libraries/tracking/morai_mpc/osqp_sqp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <osqp.h>

namespace control { namespace morai_mpc {
namespace {

struct CscStorage {
  std::vector<c_float> values;
  std::vector<c_int> rows;
  std::vector<c_int> columns;
};

CscStorage denseToCsc(const std::vector<double>& dense, int row_count,
                      int column_count) {
  CscStorage result;
  result.columns.reserve(static_cast<std::size_t>(column_count + 1));
  result.columns.push_back(0);
  for (int column = 0; column < column_count; ++column) {
    for (int row = 0; row < row_count; ++row) {
      const double value = dense[static_cast<std::size_t>(row * column_count + column)];
      if (std::abs(value) > 1.0e-14) {
        result.values.push_back(static_cast<c_float>(value));
        result.rows.push_back(static_cast<c_int>(row));
      }
    }
    result.columns.push_back(static_cast<c_int>(result.values.size()));
  }
  return result;
}

CscStorage upperTriangleToCsc(const std::vector<double>& dense, int size) {
  CscStorage result;
  result.columns.reserve(static_cast<std::size_t>(size + 1));
  result.columns.push_back(0);
  for (int column = 0; column < size; ++column) {
    for (int row = 0; row <= column; ++row) {
      const double value = dense[static_cast<std::size_t>(row * size + column)];
      if (std::abs(value) > 1.0e-14) {
        result.values.push_back(static_cast<c_float>(value));
        result.rows.push_back(static_cast<c_int>(row));
      }
    }
    result.columns.push_back(static_cast<c_int>(result.values.size()));
  }
  return result;
}

}  // namespace

OsqpSqpResult solveQuadraticProgram(
    const std::vector<double>& hessian,
    const std::vector<double>& gradient,
    const std::vector<double>& constraint_matrix, int constraint_rows,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds,
    const std::vector<double>& warm_start,
    const OsqpQpConfig& config) {
  OsqpSqpResult result;
  const int variables = static_cast<int>(gradient.size());
  if (variables <= 0 || constraint_rows <= 0 ||
      hessian.size() != static_cast<std::size_t>(variables * variables) ||
      constraint_matrix.size() !=
          static_cast<std::size_t>(constraint_rows * variables) ||
      lower_bounds.size() != static_cast<std::size_t>(constraint_rows) ||
      upper_bounds.size() != static_cast<std::size_t>(constraint_rows) ||
      (!warm_start.empty() &&
       warm_start.size() != static_cast<std::size_t>(variables))) {
    result.message = "invalid OSQP QP dimensions";
    return result;
  }
  for (double value : hessian) {
    if (!std::isfinite(value)) {
      result.message = "non-finite OSQP Hessian";
      return result;
    }
  }
  for (double value : gradient) {
    if (!std::isfinite(value)) {
      result.message = "non-finite OSQP gradient";
      return result;
    }
  }

  const CscStorage p_storage = upperTriangleToCsc(hessian, variables);
  const CscStorage a_storage =
      denseToCsc(constraint_matrix, constraint_rows, variables);
  std::vector<c_float> q(gradient.begin(), gradient.end());
  std::vector<c_float> lower(lower_bounds.begin(), lower_bounds.end());
  std::vector<c_float> upper(upper_bounds.begin(), upper_bounds.end());
  csc* p_matrix = csc_matrix(
      variables, variables, static_cast<c_int>(p_storage.values.size()),
      const_cast<c_float*>(p_storage.values.data()),
      const_cast<c_int*>(p_storage.rows.data()),
      const_cast<c_int*>(p_storage.columns.data()));
  csc* a_matrix = csc_matrix(
      constraint_rows, variables,
      static_cast<c_int>(a_storage.values.size()),
      const_cast<c_float*>(a_storage.values.data()),
      const_cast<c_int*>(a_storage.rows.data()),
      const_cast<c_int*>(a_storage.columns.data()));
  OSQPData data;
  data.n = variables;
  data.m = constraint_rows;
  data.P = p_matrix;
  data.A = a_matrix;
  data.q = q.data();
  data.l = lower.data();
  data.u = upper.data();
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;
  settings.max_iter = config.maximum_iterations;
  settings.eps_abs = config.absolute_tolerance;
  settings.eps_rel = config.relative_tolerance;
  settings.polish = config.polish;
  settings.warm_start = config.warm_start;
  OSQPWorkspace* workspace = nullptr;
  const c_int setup_status = osqp_setup(&workspace, &data, &settings);
  if (setup_status != 0 || workspace == nullptr) {
    c_free(p_matrix);
    c_free(a_matrix);
    result.message = "OSQP QP setup failed";
    return result;
  }
  if (config.warm_start && !warm_start.empty()) {
    std::vector<c_float> initial(warm_start.begin(), warm_start.end());
    osqp_warm_start_x(workspace, initial.data());
  }
  osqp_solve(workspace);
  const c_int status = workspace->info->status_val;
  result.iterations = static_cast<int>(workspace->info->iter);
  const bool solved =
      status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;
  if (solved) {
    result.solution.resize(static_cast<std::size_t>(variables));
    for (int index = 0; index < variables; ++index) {
      result.solution[static_cast<std::size_t>(index)] =
          workspace->solution->x[index];
    }
    result.cost = workspace->info->obj_val;
    result.success = true;
    result.message = "osqp_linear_qp";
  } else {
    result.message = std::string("OSQP did not solve linear MPC QP: ") +
                     workspace->info->status;
  }
  osqp_cleanup(workspace);
  c_free(p_matrix);
  c_free(a_matrix);
  return result;
}

OsqpSqpResult solveWithOsqpSqp(
    const std::function<double(const std::vector<double>&)>& objective,
    const std::vector<double>& initial,
    const std::vector<double>& constraint_matrix, int constraint_rows,
    const std::vector<double>& lower_bounds,
    const std::vector<double>& upper_bounds, const OsqpSqpConfig& config) {
  OsqpSqpResult result;
  const int variables = static_cast<int>(initial.size());
  if (variables == 0 || constraint_rows <= 0 ||
      constraint_matrix.size() != static_cast<std::size_t>(constraint_rows * variables) ||
      lower_bounds.size() != static_cast<std::size_t>(constraint_rows) ||
      upper_bounds.size() != static_cast<std::size_t>(constraint_rows)) {
    result.message = "invalid OSQP dimensions";
    return result;
  }
  result.solution = initial;
  result.cost = objective(result.solution);
  if (!std::isfinite(result.cost)) {
    result.message = "non-finite OSQP initial cost";
    return result;
  }
  const CscStorage a_storage =
      denseToCsc(constraint_matrix, constraint_rows, variables);
  for (int outer = 0; outer < config.sqp_iterations; ++outer) {
    std::vector<double> diagonal(static_cast<std::size_t>(variables), 0.0);
    std::vector<c_float> q(static_cast<std::size_t>(variables));
    std::vector<double> gradient(static_cast<std::size_t>(variables), 0.0);
    for (int index = 0; index < variables; ++index) {
      std::vector<double> plus = result.solution;
      std::vector<double> minus = result.solution;
      plus[static_cast<std::size_t>(index)] += config.finite_difference_epsilon;
      minus[static_cast<std::size_t>(index)] -= config.finite_difference_epsilon;
      const double plus_cost = objective(plus);
      const double minus_cost = objective(minus);
      if (!std::isfinite(plus_cost) || !std::isfinite(minus_cost)) {
        result.message = "non-finite OSQP finite difference";
        return result;
      }
      gradient[static_cast<std::size_t>(index)] =
          (plus_cost - minus_cost) /
          (2.0 * config.finite_difference_epsilon);
      const double curvature = std::max(
          config.hessian_regularization,
          (plus_cost - 2.0 * result.cost + minus_cost) /
              (config.finite_difference_epsilon * config.finite_difference_epsilon));
      diagonal[static_cast<std::size_t>(index)] = curvature;
    }
    for (int index = 0; index < variables; ++index) {
      q[static_cast<std::size_t>(index)] = static_cast<c_float>(
          gradient[static_cast<std::size_t>(index)] -
          diagonal[static_cast<std::size_t>(index)] *
              result.solution[static_cast<std::size_t>(index)]);
    }

    std::vector<c_float> p_values(diagonal.begin(), diagonal.end());
    std::vector<c_int> p_rows(static_cast<std::size_t>(variables));
    std::vector<c_int> p_columns(static_cast<std::size_t>(variables + 1));
    for (int index = 0; index < variables; ++index) {
      p_rows[static_cast<std::size_t>(index)] = index;
      p_columns[static_cast<std::size_t>(index)] = index;
    }
    p_columns[static_cast<std::size_t>(variables)] = variables;
    std::vector<c_float> lower(lower_bounds.begin(), lower_bounds.end());
    std::vector<c_float> upper(upper_bounds.begin(), upper_bounds.end());
    csc* p_matrix = csc_matrix(variables, variables,
                               static_cast<c_int>(p_values.size()), p_values.data(),
                               p_rows.data(), p_columns.data());
    csc* a_matrix = csc_matrix(constraint_rows, variables,
                               static_cast<c_int>(a_storage.values.size()),
                               const_cast<c_float*>(a_storage.values.data()),
                               const_cast<c_int*>(a_storage.rows.data()),
                               const_cast<c_int*>(a_storage.columns.data()));
    OSQPData data;
    data.n = variables;
    data.m = constraint_rows;
    data.P = p_matrix;
    data.A = a_matrix;
    data.q = q.data();
    data.l = lower.data();
    data.u = upper.data();
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.max_iter = config.maximum_iterations;
    settings.eps_abs = config.absolute_tolerance;
    settings.eps_rel = config.relative_tolerance;
    settings.polish = config.polish;
    OSQPWorkspace* workspace = nullptr;
    const c_int setup_status = osqp_setup(&workspace, &data, &settings);
    if (setup_status != 0 || workspace == nullptr) {
      c_free(p_matrix);
      c_free(a_matrix);
      result.message = "OSQP setup failed";
      return result;
    }
    osqp_solve(workspace);
    const c_int status = workspace->info->status_val;
    result.iterations += static_cast<int>(workspace->info->iter);
    const bool solved = status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;
    std::vector<double> candidate(static_cast<std::size_t>(variables));
    if (solved) {
      for (int index = 0; index < variables; ++index) {
        candidate[static_cast<std::size_t>(index)] = workspace->solution->x[index];
      }
    }
    osqp_cleanup(workspace);
    c_free(p_matrix);
    c_free(a_matrix);
    if (!solved) {
      result.message = "OSQP did not solve QP";
      return result;
    }
    double blend = 1.0;
    bool improved = false;
    for (int line_search = 0; line_search < 8; ++line_search) {
      std::vector<double> trial = result.solution;
      for (int index = 0; index < variables; ++index) {
        trial[static_cast<std::size_t>(index)] += blend *
            (candidate[static_cast<std::size_t>(index)] - trial[static_cast<std::size_t>(index)]);
      }
      const double trial_cost = objective(trial);
      if (std::isfinite(trial_cost) && trial_cost <= result.cost) {
        result.solution = trial;
        result.cost = trial_cost;
        improved = true;
        break;
      }
      blend *= 0.5;
    }
    if (!improved) break;
  }
  result.success = true;
  result.message = "osqp";
  return result;
}

}}  // namespace control::morai_mpc
