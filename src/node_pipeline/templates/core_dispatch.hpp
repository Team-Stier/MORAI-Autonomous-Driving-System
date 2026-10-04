// Guide only. Each case connects one Core to an existing stage contract.
#pragma once
#include <node_pipeline/stages.hpp>
#include "core_set.hpp"
namespace pipeline_template {
template<class Message, class Access, class OutputMessage>
bool processCore(CoreSet<Message, Access, OutputMessage>& cores, CoreId id,
                 const node_pipeline::StageData& data, node_pipeline::CoreResult& result,
                 const node_pipeline::BeforeCore& before) {
  switch (id) {
    case CoreId::Primary:
      if (!data.hasInputs({0})) return false;
      before();
      result = node_pipeline::wrapResult(cores.primary.process(data.template message<Message>(0)));
      return true;
    case CoreId::Auxiliary:
      if (!data.hasInputs({1})) return false;
      before();
      result = node_pipeline::wrapResult(cores.auxiliary.process(data.template message<Message>(1)));
      return true;
  }
  throw std::invalid_argument("unknown core");
}
}
