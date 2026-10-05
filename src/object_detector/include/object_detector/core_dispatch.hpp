#pragma once
#include <node_pipeline/stages.hpp>
#include <object_detector/core_set.hpp>
namespace object_detector {
using Stage2Data = node_pipeline::Data<interfaces::Objects>;
inline bool processStage1(CoreSet& cores, FirstCoreId id, const node_pipeline::StageData& data,
                          node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case FirstCoreId::Dbscan:
      if (!data.hasInputs({0})) return false;
      before();
      result = node_pipeline::wrapResult(cores.dbscan.process(
        data.message<sensor_msgs::PointCloud2>(0)));
      return true;
  }
  throw std::logic_error("no registered first stage core");
}
inline bool processStage2(CoreSet& cores, SecondCoreId id, const node_pipeline::StageData& data,
                          node_pipeline::CoreResult& result, const node_pipeline::BeforeCore& before) {
  switch (id) {
    case SecondCoreId::ObjectsPassThrough:
      if (!data.hasInputs({0})) return false;
      before();
      result = node_pipeline::wrapResult(cores.objects_pass_through.process(
        data.message<interfaces::Objects>(0)), {0});
      return true;
  }
  throw std::logic_error("no registered second stage core");
}
}  // namespace object_detector
