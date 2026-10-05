#pragma once
#include <object_detector/types.hpp>
#include <object_detector/cores/dbscan.hpp>
#include <object_detector/cores/objects_pass_through.hpp>
namespace object_detector {
enum class FirstCoreId { Dbscan };
enum class SecondCoreId { ObjectsPassThrough };
struct CoreSet {
  Dbscan dbscan{DbscanParameters{}};
  ObjectsPassThrough objects_pass_through;
};
}  // namespace object_detector
