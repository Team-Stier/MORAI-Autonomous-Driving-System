#include "morai_path_tracking/controllers/lateral/mpc/optimizer.hpp"
#include "morai_path_tracking/common/osqp_sqp_solver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
namespace morai_mpc {
void ProjectedCoordinateSearch::project(std::vector<double>& u,double steering,const MpcConfig& c){
  double d=clamp(steering,-c.max_steering,c.max_steering);
  for(double& v:u){v=clamp(v,-c.max_steering_rate,c.max_steering_rate);double next=clamp(d+v*c.dt,-c.max_steering,c.max_steering);v=(next-d)/c.dt;d=next;}
}
OptimizationResult ProjectedCoordinateSearch::solve(const std::function<double(const std::vector<double>&)>& f,const std::vector<double>& initial,double steering,const MpcConfig& c){
  OptimizationResult r;r.controls=initial;if(r.controls.size()!=static_cast<std::size_t>(c.horizon))r.controls.assign(c.horizon,0.0);project(r.controls,steering,c);r.cost=f(r.controls);
  if(!finite(r.cost)){r.message="초기 비용이 비정상";return r;}double step=c.optimizer_initial_step;
  for(int it=0;it<c.optimizer_iterations;++it){bool improved=false;
    for(int i=0;i<c.horizon;++i){auto best=r.controls;double bestcost=r.cost;
      for(double sign:{-1.0,1.0}){auto trial=r.controls;trial[i]+=sign*step;project(trial,steering,c);double cost=f(trial);if(finite(cost)&&cost<bestcost){bestcost=cost;best=trial;}}
      if(bestcost<r.cost){r.cost=bestcost;r.controls=best;improved=true;}
    }r.iterations=it+1;
    if(c.optimizer_multi_resolution){
      if(step<=c.optimizer_min_step*(1.0+1e-12))break;
      step=std::max(c.optimizer_min_step,step*c.optimizer_step_decay);
    }else{
      step*=0.5;if(!improved&&step<deg2rad(1.0))break;
    }
  }
  r.success=finite(r.cost)&&!r.controls.empty();r.message=r.success?"ok":"최적화 결과 비정상";return r;
}
OptimizationResult ProjectedGradientSearch::solve(
    const std::function<double(const std::vector<double>&)>& objective,
    const std::vector<double>& initial, double steering,
    const MpcConfig& config) {
  OptimizationResult result;
  result.controls = initial;
  if (result.controls.size() != static_cast<std::size_t>(config.horizon)) {
    result.controls.assign(static_cast<std::size_t>(config.horizon), 0.0);
  }
  ProjectedCoordinateSearch::project(result.controls, steering, config);
  result.cost = objective(result.controls);
  if (!finite(result.cost)) {
    result.message = "projected gradient initial cost is not finite";
    return result;
  }

  std::vector<double> gradient(result.controls.size(), 0.0);
  for (int iteration = 0; iteration < config.optimizer_iterations;
       ++iteration) {
    double squared_gradient_norm = 0.0;
    for (std::size_t index = 0; index < result.controls.size(); ++index) {
      std::vector<double> plus = result.controls;
      std::vector<double> minus = result.controls;
      plus[index] += config.optimizer_gradient_epsilon;
      minus[index] -= config.optimizer_gradient_epsilon;
      ProjectedCoordinateSearch::project(plus, steering, config);
      ProjectedCoordinateSearch::project(minus, steering, config);
      const double denominator = plus[index] - minus[index];
      if (std::abs(denominator) <= 1.0e-12) {
        gradient[index] = 0.0;
      } else {
        const double plus_cost = objective(plus);
        const double minus_cost = objective(minus);
        gradient[index] = finite(plus_cost) && finite(minus_cost)
                              ? (plus_cost - minus_cost) / denominator
                              : 0.0;
      }
      squared_gradient_norm += gradient[index] * gradient[index];
    }
    result.iterations = iteration + 1;
    if (!finite(squared_gradient_norm)) {
      result.message = "projected gradient norm is not finite";
      return result;
    }
    if (std::sqrt(squared_gradient_norm) <=
        config.optimizer_gradient_tolerance) {
      break;
    }

    bool improved = false;
    double step = config.optimizer_gradient_step;
    for (int line_search = 0;
         line_search < config.optimizer_line_search_steps; ++line_search) {
      std::vector<double> candidate = result.controls;
      for (std::size_t index = 0; index < candidate.size(); ++index) {
        candidate[index] -= step * gradient[index];
      }
      ProjectedCoordinateSearch::project(candidate, steering, config);
      const double candidate_cost = objective(candidate);
      if (finite(candidate_cost) && candidate_cost < result.cost) {
        result.controls = std::move(candidate);
        result.cost = candidate_cost;
        improved = true;
        break;
      }
      step *= 0.5;
    }
    if (!improved) {
      break;
    }
  }
  result.success = finite(result.cost) && !result.controls.empty();
  result.message = result.success ? "ok" : "projected gradient failed";
  return result;
}
OptimizationResult OsqpSqpSearch::solve(
    const std::function<double(const std::vector<double>&)>& objective,
    const std::vector<double>& initial, double steering,
    const MpcConfig& config) {
  const int horizon = config.horizon;
  std::vector<double> seed = initial;
  if (seed.size() != static_cast<std::size_t>(horizon)) {
    seed.assign(static_cast<std::size_t>(horizon), 0.0);
  }
  ProjectedCoordinateSearch::project(seed, steering, config);
  const int rows = 2 * horizon;
  std::vector<double> matrix(static_cast<std::size_t>(rows * horizon), 0.0);
  std::vector<double> lower(static_cast<std::size_t>(rows));
  std::vector<double> upper(static_cast<std::size_t>(rows));
  for (int index = 0; index < horizon; ++index) {
    matrix[static_cast<std::size_t>(index * horizon + index)] = 1.0;
    lower[static_cast<std::size_t>(index)] = -config.max_steering_rate;
    upper[static_cast<std::size_t>(index)] = config.max_steering_rate;
    for (int column = 0; column <= index; ++column) {
      matrix[static_cast<std::size_t>((horizon + index) * horizon + column)] =
          config.dt;
    }
    lower[static_cast<std::size_t>(horizon + index)] =
        -config.max_steering - steering;
    upper[static_cast<std::size_t>(horizon + index)] =
        config.max_steering - steering;
  }
  morai_path_tracking::OsqpSqpConfig solver_config;
  solver_config.sqp_iterations = std::min(3, config.optimizer_iterations);
  solver_config.finite_difference_epsilon =
      config.optimizer_gradient_epsilon;
  const morai_path_tracking::OsqpSqpResult solved =
      morai_path_tracking::solveWithOsqpSqp(objective, seed, matrix, rows,
                                            lower, upper, solver_config);
  OptimizationResult result;
  result.success = solved.success;
  result.controls = solved.solution;
  result.cost = solved.cost;
  result.iterations = solved.iterations;
  result.message = solved.message;
  return result;
}
std::vector<double> shiftedWarmStart(const std::vector<double>& solution,int horizon,bool enabled){
  if(horizon<=0)return {};
  if(!enabled||solution.empty())return std::vector<double>(static_cast<std::size_t>(horizon),0.0);
  std::vector<double> shifted(static_cast<std::size_t>(horizon),solution.back());
  for(int i=0;i<horizon-1;++i){std::size_t source=static_cast<std::size_t>(i+1);shifted[static_cast<std::size_t>(i)]=source<solution.size()?solution[source]:solution.back();}
  return shifted;
}
}  // namespace morai_mpc
