/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "control/libraries/tracking/morai_mpc/autoware/qp_solver/qp_solver_qpoases.h"

namespace control { namespace morai_mpc { namespace autoware {

QPSolverQpoasesHotstart::QPSolverQpoasesHotstart(const int max_iter)
  : is_solver_initialized_(false), max_iter_(max_iter){};

bool QPSolverQpoasesHotstart::solve(const Eigen::MatrixXd& Hmat, const Eigen::MatrixXd& fvec, const Eigen::MatrixXd& A,
                                    const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::MatrixXd& lbA,
                                    const Eigen::MatrixXd& ubA, Eigen::VectorXd& U)
{
  const int n = Hmat.rows(), rows = A.rows();
  U = Eigen::VectorXd::Zero(n);
  if (n <= 0 || Hmat.cols()!=n || fvec.rows()!=n || fvec.cols()!=1 ||
      A.cols()!=n || lb.size()!=n || ub.size()!=n || lbA.rows()!=rows || ubA.rows()!=rows ||
      lbA.cols()!=1 || ubA.cols()!=1 || !Hmat.allFinite() || !fvec.allFinite() || !A.allFinite() ||
      !lb.allFinite() || !ub.allFinite() || !lbA.allFinite() || !ubA.allFinite() ||
      (lb.array()>ub.array()).any() || (lbA.array()>ubA.array()).any()) return false;
  // Legacy callers represent no linear constraints by zero rows with 0 in the interval.
  const bool no_constraints = A.isZero(0.) &&
      (rows==0 || ((lbA.array()<=0.).all() && (ubA.array()>=0.).all()));
  const int nc = no_constraints ? 0 : rows;
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> next_h=Hmat, next_a=A;
  int iterations=max_iter_;
  qpOASES::returnValue result;
  if (!is_solver_initialized_ || solver_.getNV()!=n || solver_.getNC()!=nc) {
    solver_=qpOASES::SQProblem(n,nc);
    solver_.setPrintLevel(qpOASES::PL_NONE);
    result=solver_.init(next_h.data(),fvec.data(),nc ? next_a.data() : nullptr,
        lb.data(),ub.data(),nc ? lbA.data() : nullptr,nc ? ubA.data() : nullptr,iterations);
  } else {
    result=solver_.hotstart(next_h.data(),fvec.data(),nc ? next_a.data() : nullptr,
        lb.data(),ub.data(),nc ? lbA.data() : nullptr,nc ? ubA.data() : nullptr,iterations);
  }
  h_storage_.swap(next_h);
  a_storage_.swap(next_a);
  is_solver_initialized_=result==qpOASES::SUCCESSFUL_RETURN;
  if (!is_solver_initialized_ || solver_.getPrimalSolution(U.data())!=qpOASES::SUCCESSFUL_RETURN || !U.allFinite())
    return false;
  const Eigen::VectorXd constrained=A*U;
  const double tolerance=1e-6;
  return (U.array()>=lb.array()-tolerance).all() && (U.array()<=ub.array()+tolerance).all() &&
      (constrained.array()>=lbA.array()-tolerance).all() && (constrained.array()<=ubA.array()+tolerance).all();
};

}}}  // namespace control::morai_mpc::autoware
