#pragma once
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/prediction_pass_through.hpp>
#include <path_planner/cores/prediction_core.hpp>
#include <path_planner/cores/local_path.hpp>
namespace path_planner {
enum class FirstCoreId { GlobalPath };
enum class SecondCoreId { PredictionPassThrough, PredictionCore };
enum class ThirdCoreId { LocalPath };
struct CoreSet {
  GlobalPath global_path;
  PredictionPassThrough prediction;
  LocalPath local_path;
  PredictionCore prediction_core;
};
}  // namespace path_planner
