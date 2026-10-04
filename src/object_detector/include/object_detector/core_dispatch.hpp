#pragma once
#include <node_pipeline/stages.hpp>
#include <object_detector/core_set.hpp>
namespace object_detector {
using Stage2Data = node_pipeline::Data<interfaces::Objects>;
// TODO: Add a case to the existing stage: presence check, before(), native process call, wrapResult.
inline bool processStage1(CoreSet&, FirstCoreId, const node_pipeline::StageData&,
                          node_pipeline::CoreResult&, const node_pipeline::BeforeCore&) {
  throw std::logic_error("no registered first stage core");
}
inline bool processStage2(CoreSet&, SecondCoreId, const node_pipeline::StageData&,
                          node_pipeline::CoreResult&, const node_pipeline::BeforeCore&) {
  throw std::logic_error("no registered second stage core");
}
}  // namespace object_detector
