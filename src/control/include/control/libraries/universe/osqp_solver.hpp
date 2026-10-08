#pragma once
#include <control/libraries/universe/mpc/qp_solver/qp_solver_interface.hpp>
#include <memory>
namespace control { namespace universe {
// OSQP 0.6 C API implementation of Universe's QP interface.
class OsqpSolver final : public autoware::motion::control::mpc_lateral_controller::QPSolverInterface {
 public:
  OsqpSolver();
  ~OsqpSolver() override;
  autoware::motion::control::mpc_lateral_controller::QPSolverResult solve(
    const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&,
    const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
    const Eigen::VectorXd&, Eigen::VectorXd&) override;
  int64_t getTakenIter() const override;
  double getRunTime() const override;
  double getObjVal() const override;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}}
