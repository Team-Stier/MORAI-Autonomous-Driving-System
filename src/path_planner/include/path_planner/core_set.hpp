#pragma once
#include <path_planner/cores/global_path.hpp>
#include <path_planner/cores/local_path.hpp>
#include <stdexcept>
#include <tuple>
namespace path_planner {
enum class FirstCoreId { GlobalPath };
enum class SecondCoreId { LocalPath };
struct CoreIds { FirstCoreId first{}; SecondCoreId second{}; };
struct CoreSet {
  GlobalPath global_path;
  LocalPath local_path;
  std::array<std::size_t, 2> requiredInputs(CoreIds ids) const {
    switch (ids.first) {
      case FirstCoreId::GlobalPath: return {{0, 4}};
    }
    throw std::invalid_argument("unknown first core");
  }
  Message process(CoreIds ids, const Data& snapshot) {
    switch (ids.first) {
      case FirstCoreId::GlobalPath: {
        const auto data = global_path.process(snapshot.get<0>().message(), snapshot.get<4>().message());
        switch (ids.second) {
          case SecondCoreId::LocalPath:
            return local_path.process(std::get<0>(data), std::get<1>(data), std::get<2>(data));
        }
        throw std::invalid_argument("unknown second core");
      }
    }
    throw std::invalid_argument("unknown first core");
  }
};
}  // namespace path_planner
