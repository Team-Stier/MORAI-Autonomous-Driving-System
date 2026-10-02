#pragma once
#include <object_detector/types.hpp>
namespace object_detector {
// TODO: 단계별 Core 파일을 include하고 ID·객체·필수 입력·process 인자를 등록한다.
enum class FirstCoreId {};
enum class SecondCoreId {};
struct CoreId { FirstCoreId first{}; SecondCoreId second{}; };
struct CoreSet {
  // TODO: 첫 단계 Core가 사용하는 구독 입력의 tuple 인덱스를 선언한다.
  std::array<std::size_t, 0> requiredInputs(CoreId) const { return {}; }
  interfaces::Objects process(CoreId ids, const Data& snapshot) {
    const interfaces::Objects objects = processFirst(ids.first, snapshot);
    return processSecond(ids.second, objects);
  }
 private:
  // TODO: Core1.process(snapshot.get<0>().message())로 구독 점군을 전달한다.
  interfaces::Objects processFirst(FirstCoreId, const Data&) {
    throw std::logic_error("no registered first core");
  }
  // TODO: Core2.process(objects)로 중간 Objects 메시지 하나를 전달한다.
  interfaces::Objects processSecond(SecondCoreId, const interfaces::Objects&) {
    throw std::logic_error("no registered second core");
  }
};
}  // namespace object_detector
