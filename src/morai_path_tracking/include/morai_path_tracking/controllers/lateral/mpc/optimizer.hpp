#pragma once
#include "morai_path_tracking/controllers/lateral/mpc/types.hpp"
#include <functional>
#include <vector>
namespace morai_mpc {
struct OptimizationResult { bool success{false}; std::vector<double> controls; double cost{0.0}; int iterations{0}; std::string message; };
class Optimizer {
 public: virtual ~Optimizer() = default;
  virtual OptimizationResult solve(const std::function<double(const std::vector<double>&)>& objective,
      const std::vector<double>& initial, double steering, const MpcConfig& config) = 0;
};
class ProjectedCoordinateSearch : public Optimizer {
 public: OptimizationResult solve(const std::function<double(const std::vector<double>&)>& objective,
      const std::vector<double>& initial, double steering, const MpcConfig& config) override;
  static void project(std::vector<double>& u, double steering, const MpcConfig& config);
};
class ProjectedGradientSearch : public Optimizer {
 public:
  OptimizationResult solve(
      const std::function<double(const std::vector<double>&)>& objective,
      const std::vector<double>& initial, double steering,
      const MpcConfig& config) override;
};
class OsqpSqpSearch : public Optimizer {
 public:
  OptimizationResult solve(
      const std::function<double(const std::vector<double>&)>& objective,
      const std::vector<double>& initial, double steering,
      const MpcConfig& config) override;
};
std::vector<double> shiftedWarmStart(const std::vector<double>& solution, int horizon, bool enabled);
}  // namespace morai_mpc
