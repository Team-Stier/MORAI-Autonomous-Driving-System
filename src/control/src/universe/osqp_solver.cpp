#include <control/universe/osqp_solver.hpp>
#include <osqp.h>
#include <vector>
#include <stdexcept>
#include <limits>
namespace control { namespace universe {
struct OsqpSolver::Impl {
  OSQPWorkspace* work = nullptr;
  std::vector<c_float> px, ax, q, l, u;
  std::vector<c_int> pi, pp, ai, ap;
  ~Impl() { if (work) osqp_cleanup(work); }
};
OsqpSolver::OsqpSolver() : impl_(new Impl) {}
OsqpSolver::~OsqpSolver() = default;
autoware::motion::control::mpc_lateral_controller::QPSolverResult OsqpSolver::solve(
  const Eigen::MatrixXd& h, const Eigen::MatrixXd& f, const Eigen::MatrixXd& a,
  const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lba,
  const Eigen::VectorXd& uba, Eigen::VectorXd& result) {
  auto& s=*impl_; const c_int n=h.rows(), m=n+a.rows();
  s.px.clear();s.pi.clear();s.pp.clear();s.ax.clear();s.ai.clear();s.ap.clear();
  // Keep a fixed sparsity pattern for OSQP matrix updates and warm starts.
  for(c_int col=0;col<n;++col) {
    s.pp.push_back(s.px.size());
    for(c_int row=0;row<=col;++row) { s.pi.push_back(row);s.px.push_back(h(row,col)); }
    s.ap.push_back(s.ax.size());s.ai.push_back(col);s.ax.push_back(1.0);
    for(c_int row=0;row<a.rows();++row) { s.ai.push_back(n+row);s.ax.push_back(a(row,col)); }
  }
  s.pp.push_back(s.px.size());s.ap.push_back(s.ax.size());
  s.q.resize(n);s.l.resize(m);s.u.resize(m);
  for(c_int i=0;i<n;++i) {s.q[i]=f(i);s.l[i]=lb(i);s.u[i]=ub(i);}
  for(c_int i=0;i<a.rows();++i) {s.l[n+i]=lba(i);s.u[n+i]=uba(i);}
  if(s.work && (s.work->data->n!=n || s.work->data->m!=m)) {osqp_cleanup(s.work);s.work=nullptr;}
  if(!s.work) {
    csc p{static_cast<c_int>(s.px.size()),n,n,s.pp.data(),s.pi.data(),s.px.data(),-1};
    csc c{static_cast<c_int>(s.ax.size()),m,n,s.ap.data(),s.ai.data(),s.ax.data(),-1};
    OSQPData data{}; data.n=n;data.m=m;data.P=&p;data.A=&c;data.q=s.q.data();data.l=s.l.data();data.u=s.u.data();
    OSQPSettings settings;osqp_set_default_settings(&settings);
    settings.verbose=0;settings.eps_abs=std::numeric_limits<c_float>::epsilon();
    settings.eps_rel=1e-4;settings.eps_prim_inf=1e-4;settings.eps_dual_inf=1e-4;
    settings.alpha=1.6;settings.max_iter=4000;
    settings.polish=1;settings.warm_start=1;
    if(osqp_setup(&s.work,&data,&settings)) throw std::runtime_error("OSQP setup failed");
  } else {
    if(osqp_update_P_A(s.work,s.px.data(),nullptr,s.px.size(),s.ax.data(),nullptr,s.ax.size()) ||
       osqp_update_lin_cost(s.work,s.q.data()) || osqp_update_bounds(s.work,s.l.data(),s.u.data()))
      throw std::runtime_error("OSQP update failed");
  }
  if(osqp_solve(s.work)) return {false,"OSQP solve failed"};
  // Match Universe's OSQP interface: only the solved status is accepted.
  if(s.work->info->status_val!=OSQP_SOLVED) return {false,s.work->info->status};
  result.resize(n);for(c_int i=0;i<n;++i)result(i)=s.work->solution->x[i];
  return {true,{}};
}
int64_t OsqpSolver::getTakenIter() const {return impl_->work ? impl_->work->info->iter : 0;}
double OsqpSolver::getRunTime() const {return impl_->work ? impl_->work->info->run_time : 0.0;}
double OsqpSolver::getObjVal() const {return impl_->work ? impl_->work->info->obj_val : 0.0;}
}}
