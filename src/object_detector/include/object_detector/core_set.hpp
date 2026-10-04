#pragma once
#include <object_detector/types.hpp>
#include <object_detector/cores/dbscan.hpp>
#include <object_detector/cores/objects_pass_through.hpp>
#include <array>
#include <stdexcept>
namespace object_detector {
enum class FirstCoreId { Dbscan };
enum class SecondCoreId { ObjectsPassThrough };
struct CoreId { FirstCoreId first{}; SecondCoreId second{}; };
struct CoreSet {
  Dbscan dbscan{DbscanParameters{}};
  ObjectsPassThrough objects_pass_through;
  std::array<std::size_t, 1> requiredInputs(CoreId ids) const {
    switch (ids.first) {
      case FirstCoreId::Dbscan: return {{0}};
    }
    throw std::invalid_argument("unknown first core");
  }
  interfaces::Objects process(CoreId ids, const Data& snapshot) {
    const interfaces::Objects objects = processFirst(ids.first, snapshot);
    return processSecond(ids.second, objects);
  }
 private:
  interfaces::Objects processFirst(FirstCoreId id, const Data& snapshot) {
    switch (id) {
      case FirstCoreId::Dbscan: return dbscan.process(snapshot.get<0>().message());
    }
    throw std::invalid_argument("unknown first core");
  }
  interfaces::Objects processSecond(SecondCoreId id, const interfaces::Objects& objects) {
    switch (id) {
      case SecondCoreId::ObjectsPassThrough: return objects_pass_through.process(objects);
    }
    throw std::invalid_argument("unknown second core");
  }
};
}  // namespace object_detector
