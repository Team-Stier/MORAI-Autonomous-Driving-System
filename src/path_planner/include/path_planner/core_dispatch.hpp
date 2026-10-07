#pragma once
#include <node_pipeline/stages.hpp>
#include <path_planner/core_set.hpp>
namespace path_planner {
using Stage2Data = node_pipeline::Data<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
    interfaces::Objects, tf2_msgs::TFMessage, interfaces::SectionAnnotation, Route>;
using Stage3Data = node_pipeline::Data<nav_msgs::Odometry, interfaces::CompetitionVehicleStatus,
    interfaces::Objects, tf2_msgs::TFMessage, interfaces::SectionAnnotation, Route, PredictedObjects>;

inline bool processStage1(CoreSet& cores, FirstCoreId id, const node_pipeline::StageData& data,
                          node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case FirstCoreId::GlobalPath:
      if (!data.hasInputs({0, 4})) return false;
      before();
      result = node_pipeline::wrapResult(cores.global_path.process(
        data.message<nav_msgs::Odometry>(0), data.message<interfaces::SectionAnnotation>(4)));
      return true;
  }
  throw std::invalid_argument("unknown first stage core");
}
inline bool processStage2(CoreSet& cores, SecondCoreId id, const node_pipeline::StageData& data,
                          node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case SecondCoreId::PredictionPassThrough:
      if (!data.hasInputs({2, 5})) return false;
      before();
      result = node_pipeline::wrapResult(cores.prediction.process(
        data.message<interfaces::Objects>(2), data.message<Route>(5)));
      return true;
    case SecondCoreId::PredictionCore:
      if (!data.hasInputs({1, 2, 5})) return false;
      before();
      result = node_pipeline::wrapResult(cores.prediction_core.process(
        data.message<interfaces::Objects>(2), data.message<interfaces::CompetitionVehicleStatus>(1),
        data.message<Route>(5)));
      return true;
  }
  throw std::invalid_argument("unknown second stage core");
}
inline bool processStage3(CoreSet& cores, ThirdCoreId id, const node_pipeline::StageData& data,
                          node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case ThirdCoreId::LocalPath:
      if (!data.hasInputs({0, 4, 5, 6})) return false;
      before();
      result = node_pipeline::wrapResult(cores.local_path.process(data.message<nav_msgs::Odometry>(0),
        data.message<interfaces::SectionAnnotation>(4), data.message<Route>(5),
        data.message<PredictedObjects>(6)));
      return true;
  }
  throw std::invalid_argument("unknown third stage core");
}
}  // namespace path_planner
