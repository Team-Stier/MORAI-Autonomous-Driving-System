#pragma once
#include <std_msgs/Float64.h>
namespace pipeline_template {
struct ScalarParameters { double coefficient=1; };
class ScalarCore {
 public:
  explicit ScalarCore(ScalarParameters parameters):parameters_(parameters){}
  std_msgs::Float64 process(const std_msgs::Float64& input) {
    std_msgs::Float64 output; output.data=input.data*parameters_.coefficient;return output;
  }
 private:
  ScalarParameters parameters_;
};
}
