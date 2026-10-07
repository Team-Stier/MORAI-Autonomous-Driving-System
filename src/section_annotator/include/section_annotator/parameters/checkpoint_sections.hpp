#pragma once
#include <section_annotator/types.hpp>
#include <node_pipeline/sdk.hpp>
#include <istream>
#include <sstream>
namespace section_annotator {
inline CheckpointParameters readCheckpoints(std::istream& stream, double radius_m) {
  if (!std::isfinite(radius_m) || radius_m <= 0)
    throw std::invalid_argument("checkpoint_sections/radius_m must be finite and positive");
  CheckpointParameters parameters;
  parameters.radius_m = radius_m;
  std::string line;
  if (!std::getline(stream, line)) throw std::invalid_argument("empty checkpoints CSV");
  if (!line.empty() && line.back() == '\r') line.pop_back();
  if (line != "x,y,z,yaw,lanelet_id") throw std::invalid_argument("expected checkpoints CSV header x,y,z,yaw,lanelet_id");
  for (auto& point : parameters.checkpoints) {
    if (!std::getline(stream, line)) throw std::invalid_argument("expected 16 checkpoints");
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::istringstream row(line);
    std::array<double, 4> values;
    for (auto& value : values) {
      std::string field;
      if (!std::getline(row, field, ',') || field.empty())
        throw std::invalid_argument("missing checkpoint field");
      std::size_t parsed = 0;
      value = std::stod(field, &parsed);
      if (parsed != field.size() || !std::isfinite(value))
        throw std::invalid_argument("invalid checkpoint field");
    }
    std::string id_field;
    if (!std::getline(row, id_field, ',') || id_field.empty() ||
        id_field.find_first_not_of("0123456789") != std::string::npos || std::stoll(id_field) <= 0)
      throw std::invalid_argument("expected positive integer checkpoint lanelet_id");
    if (row.peek() != std::char_traits<char>::eof() || line.back() == ',')
      throw std::invalid_argument("expected five checkpoint fields");
    point = {values[0], values[1]};
  }
  if (std::getline(stream, line)) throw std::invalid_argument("expected exactly 16 checkpoints");
  if (stream.bad()) throw std::runtime_error("cannot read checkpoints CSV");
  return parameters;
}

namespace parameters {
struct CheckpointSections {
  CheckpointParameters operator()(const node_pipeline::Parameters& params) const {
    auto stream = params.openAsset("checkpoints");
    return readCheckpoints(*stream, params.get("radius_m", 10.0));
  }
};
}}
