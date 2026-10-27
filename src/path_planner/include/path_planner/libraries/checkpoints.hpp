#pragma once
#include <path_planner/libraries/types.hpp>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace path_planner {
// checkpoints.csv의 x,y,z,yaw,lanelet_id 16행을 읽는다. yaw는 deg를 rad로 변환한다.
inline std::array<Checkpoint, 16> readCheckpoints(const std::string& checkpoints_data) {
  std::array<Checkpoint, 16> checkpoints{};
  std::istringstream csv(checkpoints_data);
  std::string line;
  auto read_line = [&] {
    if (!std::getline(csv, line)) throw std::invalid_argument("expected 16 checkpoints with x,y,z,yaw,lanelet_id header");
    if (!line.empty() && line.back() == '\r') line.pop_back();
  };
  read_line();
  if (line != "x,y,z,yaw,lanelet_id") throw std::invalid_argument("expected checkpoints header x,y,z,yaw,lanelet_id");
  for (auto& checkpoint : checkpoints) {
    read_line();
    std::istringstream row(line);
    std::array<double, 4> values;
    for (auto& value : values) {
      std::string field;
      if (!std::getline(row, field, ',')) throw std::invalid_argument("missing checkpoint field");
      std::size_t parsed = 0;
      value = std::stod(field, &parsed);
      if (parsed != field.size() || !std::isfinite(value))
        throw std::invalid_argument("invalid checkpoint field");
    }
    std::string id_field;
    if (!std::getline(row, id_field, ',') || id_field.empty() ||
        id_field.find_first_not_of("0123456789") != std::string::npos)
      throw std::invalid_argument("expected positive integer checkpoint lanelet_id");
    const auto lanelet_id = std::stoll(id_field);
    if (lanelet_id <= 0) throw std::invalid_argument("expected positive integer checkpoint lanelet_id");
    if (row.peek() != std::char_traits<char>::eof() || line.back() == ',')
      throw std::invalid_argument("expected five checkpoint fields");
    checkpoint = {values[0], values[1], values[2], values[3] * std::acos(-1.0) / 180.0, lanelet_id};
  }
  if (std::getline(csv, line)) throw std::invalid_argument("expected exactly 16 checkpoints");
  return checkpoints;
}
}  // namespace path_planner
