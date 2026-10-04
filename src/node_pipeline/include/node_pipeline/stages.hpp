#pragma once
#include "components.hpp"
#include <functional>
#include <typeindex>

namespace node_pipeline {
// Native payload ownership and original sample metadata survive stage boundaries.
struct StageSlot {
  std::shared_ptr<const void> message;
  std::type_index type{typeid(void)};
  std::uint64_t id = 0;
  std::int64_t stamp_ns = 0, receipt_steady_ns = 0;
  bool has_stamp = false, has_receipt = false;
  bool present() const { return bool(message); }
};
struct StageData {
  std::vector<StageSlot> inputs;
  template<class Indices> bool hasInputs(const Indices& indices) const {
    return std::all_of(indices.begin(), indices.end(), [&](std::size_t i) {
      return i < inputs.size() && inputs[i].present();
    });
  }
  bool hasInputs(std::initializer_list<std::size_t> indices) const { return hasInputs<std::initializer_list<std::size_t>>(indices); }
  template<class Message> const Message& message(std::size_t index) const {
    const auto& slot = inputs.at(index);
    if (!slot.present()) throw std::invalid_argument("core input missing");
    if (slot.type != std::type_index(typeid(Message))) throw std::invalid_argument("incompatible stage input type");
    return *static_cast<const Message*>(slot.message.get());
  }
  template<class DataType> DataType typed() const {
    DataType result;
    each(result.inputs, [&](auto tag, auto& slot) {
      using Message = typename std::decay<decltype(slot.message())>::type;
      if (tag >= inputs.size()) throw std::invalid_argument("incompatible stage input count");
      const auto& source = inputs[tag];
      if (source.present()) {
        message<Message>(tag);
        slot.sample.message = std::static_pointer_cast<const Message>(source.message);
      }
      slot.sample.id = source.id;
      slot.sample.stamp_ns = source.stamp_ns;
      slot.sample.receipt_steady_ns = source.receipt_steady_ns;
      slot.sample.has_stamp = source.has_stamp;
      slot.sample.has_receipt = source.has_receipt;
    });
    return result;
  }
  template<class DataType> void update(const DataType& data) {
    constexpr auto count = std::tuple_size<decltype(data.inputs)>::value;
    if (inputs.size() < count) inputs.resize(count);
    each(data.inputs, [&](auto tag, const auto& slot) {
      using Message = typename std::decay<decltype(slot.message())>::type;
      inputs[tag] = {slot.sample.message, std::type_index(typeid(Message)), slot.sample.id,
        slot.sample.stamp_ns, slot.sample.receipt_steady_ns, slot.sample.has_stamp, slot.sample.has_receipt};
    });
  }
};

using BeforeCore = std::function<void()>;
using CoreResult = std::function<StageData(const StageData&)>;

template<class Value>
void appendResult(StageData& output, Value&& value, const StageData& input,
                  const std::vector<int>& origins) {
  using Message = typename std::decay<Value>::type;
  StageSlot slot;
  const auto i = output.inputs.size();
  if (!origins.empty()) {
    if (origins.at(i) < -1) throw std::invalid_argument("invalid result origin");
    if (origins[i] >= 0) slot = input.inputs.at(origins[i]);
  }
  // Pass-through references retain their original payload owner.
  if (!slot.message || slot.type != std::type_index(typeid(Message)) || slot.message.get() != std::addressof(value))
    slot.message = std::make_shared<Message>(std::forward<Value>(value));
  slot.type = std::type_index(typeid(Message));
  output.inputs.push_back(std::move(slot));
}
template<class Value> void packResult(StageData& output, Value&& value,
                                     const StageData& input, const std::vector<int>& origins) {
  appendResult(output, std::forward<Value>(value), input, origins);
}
template<class... Values> void packResult(StageData& output, std::tuple<Values...>&& values,
                                         const StageData& input, const std::vector<int>& origins) {
  each(values, [&](auto tag, auto&) { appendResult(output, std::get<decltype(tag)::value>(std::move(values)), input, origins); });
}
template<class A, class B> void packResult(StageData& output, std::pair<A, B>&& values,
                                          const StageData& input, const std::vector<int>& origins) {
  appendResult(output, std::forward<A>(values.first), input, origins);
  appendResult(output, std::forward<B>(values.second), input, origins);
}
template<class Value> CoreResult wrapResult(Value&& value, std::vector<int> origins = {}) {
  using Result = typename std::decay<Value>::type;
  auto owned = std::make_shared<Result>(std::forward<Value>(value));
  return [owned, origins](const StageData& input) {
    StageData output;
    packResult(output, std::move(*owned), input, origins);
    if (!origins.empty() && origins.size() != output.inputs.size()) throw std::invalid_argument("invalid result origin count");
    return output;
  };
}
template<class Snapshot> void replaceInputs(Snapshot& snapshot, const CoreResult& result) {
  snapshot.data.inputs = result(snapshot.data).inputs;
}

struct StageHealth {
  std::vector<InputHealth<std::size_t, Kind>> inputs;
  std::vector<std::uint64_t> evictions;
  std::uint8_t section = 0;
  bool invalid_section = false;
};
template<class Cores, class ContextType> struct Stage {
  std::function<StageHealth(const StageData&, const ContextType&, const Clocks&)> healthCheck;
  std::function<bool(StageData&, const StageHealth&)> apply;
  std::function<Selection<std::size_t>(std::uint8_t, const StageHealth&)> select;
  std::function<bool(Cores&, std::size_t, const StageData&, CoreResult&, const BeforeCore&)> process;
};
// Adapters retain each stage's typed helpers; the Pipeline owns all execution loops.
template<class Cores, class ContextType, class DataType, class Checker, class Policy, class Selector, class Dispatch>
Stage<Cores, ContextType> makeStage(Checker checker, Policy policy, Selector selector, Dispatch dispatch) {
  using HealthType = decltype(checker.healthCheck(std::declval<DataType>(), std::declval<ContextType>(), std::declval<Clocks>()));
  using Id = decltype(selector.select(0, std::declval<HealthType>()).id);
  auto health = std::make_shared<HealthType>();
  Stage<Cores, ContextType> stage;
  stage.healthCheck = [checker, health](const StageData& data, const ContextType& context, const Clocks& clocks) {
    *health = checker.healthCheck(data.template typed<DataType>(), context, clocks);
    StageHealth result;
    for (const auto& input : health->inputs) result.inputs.push_back({static_cast<std::size_t>(input.input), input.issues});
    result.evictions.assign(health->evictions.begin(), health->evictions.end());
    result.section = health->section; result.invalid_section = health->invalid_section;
    return result;
  };
  stage.apply = [policy, health](StageData& data, const StageHealth&) {
    auto typed = data.template typed<DataType>();
    const bool corrected = policy.apply(typed, *health);
    if (corrected) data.update(typed);
    return corrected;
  };
  stage.select = [selector, health](std::uint8_t section, const StageHealth&) {
    const auto selected = selector.select(section, *health);
    return Selection<std::size_t>{selected.selected, static_cast<std::size_t>(selected.id)};
  };
  stage.process = [dispatch](Cores& cores, std::size_t id, const StageData& data, CoreResult& result, const BeforeCore& before) {
    return dispatch(cores, static_cast<Id>(id), data, result, before);
  };
  return stage;
}
}  // namespace node_pipeline
