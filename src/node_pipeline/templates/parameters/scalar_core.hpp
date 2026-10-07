#pragma once
#include <node_pipeline/sdk.hpp>
#include <pipeline_template/cores/scalar_core.hpp>
namespace pipeline_template { namespace parameters {
struct ScalarCore {
  ScalarParameters operator()(const node_pipeline::Parameters& parameters)const {
    return {parameters.get<double>("coefficient",1)};
  }
};
}}
