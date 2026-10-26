#pragma once
#include <velocity_planner/cores/scored_stop_line_pass_through.hpp>
#include <node_pipeline/extension.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace velocity_planner {
inline ScoredStopLinePassThroughParameters loadScoredStopLinePassThroughParameters(
    const node_pipeline::ParamReader& params) {
  ScoredStopLinePassThroughParameters result;
  const auto read_ids = [&params](const std::string& key) {
    std::vector<std::int64_t> ids;
    std::unordered_set<std::int64_t> seen;
    for (const double value : params.numbers(key)) {
      if (!std::isfinite(value) || value == 0 || std::floor(value) != value ||
          value < -std::ldexp(1.0, 63) || value >= std::ldexp(1.0, 63))
        throw std::invalid_argument("stop-line ID must be a finite nonzero signed integer");
      const auto id = static_cast<std::int64_t>(value);
      if (!seen.insert(id).second) throw std::invalid_argument("duplicate configured stop-line ID");
      ids.push_back(id);
    }
    return ids;
  };
  result.scored_stop_line_ids = read_ids("scored_stop_line_ids");
  result.left_signal_stop_line_ids = read_ids("left_signal_stop_line_ids");
  if (result.scored_stop_line_ids.empty())
    throw std::invalid_argument("scored_stop_line_ids must not be empty");
  for (const auto id : result.left_signal_stop_line_ids)
    if (std::find(result.scored_stop_line_ids.begin(), result.scored_stop_line_ids.end(), id) ==
        result.scored_stop_line_ids.end())
      throw std::invalid_argument("left_signal_stop_line_ids must be a subset of scored_stop_line_ids");
  return result;
}
}  // namespace velocity_planner
