#pragma once
// 공통 실행기 내부 타입이다. 확장점 파일은 include하지 않는다.
#include <node_pipeline/extension.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace node_pipeline {
namespace detail {

template<class... Ts> struct Types { static constexpr std::size_t size = sizeof...(Ts); };
template<std::size_t I, class List> struct At;
template<std::size_t I, class... Ts> struct At<I, Types<Ts...>> {
  using type = typename std::tuple_element<I, std::tuple<Ts...>>::type;
};
template<std::size_t I, class List> using AtT = typename At<I, List>::type;
template<class T> struct TypeTag { using type = T; };

template<class Fn, std::size_t... I> void forIndices(Fn&& fn, std::index_sequence<I...>) {
  using Expand = int[];
  (void)Expand{0, (fn(std::integral_constant<std::size_t, I>{}), 0)...};
}
template<std::size_t N, class Fn> void forEachIndex(Fn&& fn) { forIndices(std::forward<Fn>(fn), std::make_index_sequence<N>{}); }

inline const std::array<const char*, static_cast<std::size_t>(Kind::Count)>& kindNames() {
  static const std::array<const char*, static_cast<std::size_t>(Kind::Count)> names{{
    "Missing", "InvalidStamp", "FutureStamp", "FrameMismatch", "NaN", "Inf",
    "OutRangePositive", "OutRangeNegative", "Stale", "ReceiveTimeout", "Skew"}};
  return names;
}
inline Kind kindFromName(const std::string& name) {
  const auto& names = kindNames();
  for (std::size_t i = 0; i < names.size(); ++i)
    if (name == names[i]) return static_cast<Kind>(i);
  throw std::invalid_argument("unknown health issue: " + name);
}

// 한 번의 읽기로 받은 두 시계다. 두 값은 서로 빼지 않는다.
struct Clocks {
  std::int64_t ros_ns = 0;
  std::int64_t steady_ns = 0;
};

enum class ExecutionMode { Periodic, OnInput, Latched };
struct ExecutionConfig {
  ExecutionMode mode = ExecutionMode::Periodic;
  std::size_t trigger_input = 0;
  std::size_t max_health_checks = 10;
  bool require_new_snapshot = false;
};
// 계산을 시작한 계기다. OnInput은 방금 저장한 sample을 가리킨다.
struct Trigger {
  ExecutionMode mode = ExecutionMode::Periodic;
  std::size_t input = 0;
  std::uint64_t sample_id = 0;
};
template<class CoreId> struct Selection {
  bool selected = false;
  CoreId id{};
};

enum class ProcessStatus { Queued, Skipped, Rejected, NoCore, Published };
struct ProcessReport {
  ProcessStatus status = ProcessStatus::Queued;
  std::size_t health_checks = 0;
  std::string reason;
};

struct Section {
  bool present = false;
  std::uint8_t state = 0;
  std::int64_t stamp_ns = 0;
  std::string frame;
};
// 조립 기준 메타데이터다. Core에 전달하지 않는다.
struct Context {
  std::int64_t stamp_ns = 0;
  Section section;
};

// 입력별 모든 issue를 보관한다.
struct InputHealth {
  std::size_t input = 0;
  std::string name;
  std::vector<Kind> issues;
};
template<std::size_t N> struct Health {
  std::string stage;
  std::array<InputHealth, N> inputs{};
};

// 메시지 타입별 stamp·frame 접근과 헤더 주입이다.
template<class T, class = void> struct HasHeader : std::false_type {};
template<class T> struct HasHeader<T, decltype(void(std::declval<const T&>().header.stamp.toNSec()),
                                               void(std::declval<const T&>().header.frame_id))> : std::true_type {};
template<class T, class = void> struct HasTransforms : std::false_type {};
template<class T> struct HasTransforms<T, decltype(void(std::declval<const T&>().transforms.front().header.stamp.toNSec()))>
    : std::true_type {};
template<class T, class = void> struct HasPosePoints : std::false_type {};
template<class T> struct HasPosePoints<T, decltype(void(std::declval<T&>().points.front().pose.header = std::declval<T&>().header))>
    : std::true_type {};

struct Access {
  template<class T> static constexpr bool hasFrame() { return HasHeader<T>::value || HasTransforms<T>::value; }
  template<class T> static bool stamp(const T& message, std::int64_t& stamp_ns) {
    return stampOf(message, stamp_ns, HasHeader<T>{}, HasTransforms<T>{});
  }
  template<class T> static std::string frame(const T& message) { return frameOf(message, HasHeader<T>{}, HasTransforms<T>{}); }
  template<class T> static void inject(T& message, std::int64_t stamp_ns, const std::string& frame_id) {
    injectInto(message, stamp_ns, frame_id, HasHeader<T>{}, HasTransforms<T>{});
  }

 private:
  template<class T, class AnyTransforms>
  static bool stampOf(const T& message, std::int64_t& stamp_ns, std::true_type, AnyTransforms) {
    stamp_ns = static_cast<std::int64_t>(message.header.stamp.toNSec());
    return true;
  }
  template<class T> static bool stampOf(const T& message, std::int64_t& stamp_ns, std::false_type, std::true_type) {
    stamp_ns = message.transforms.empty() ? 0 : static_cast<std::int64_t>(message.transforms.front().header.stamp.toNSec());
    return true;
  }
  template<class T> static bool stampOf(const T&, std::int64_t& stamp_ns, std::false_type, std::false_type) {
    stamp_ns = 0;
    return false;
  }
  template<class T, class AnyTransforms> static std::string frameOf(const T& message, std::true_type, AnyTransforms) {
    return message.header.frame_id;
  }
  template<class T> static std::string frameOf(const T& message, std::false_type, std::true_type) {
    return message.transforms.empty() ? std::string() : message.transforms.front().header.frame_id;
  }
  template<class T> static std::string frameOf(const T&, std::false_type, std::false_type) { return {}; }
  template<class T, class AnyTransforms>
  static void injectInto(T& message, std::int64_t stamp_ns, const std::string& frame_id, std::true_type, AnyTransforms) {
    message.header.stamp.fromNSec(static_cast<std::uint64_t>(stamp_ns));
    message.header.frame_id = frame_id;
    injectPoints(message, HasPosePoints<T>{});
  }
  template<class T>
  static void injectInto(T& message, std::int64_t stamp_ns, const std::string& frame_id, std::false_type, std::true_type) {
    for (auto& transform : message.transforms) {
      transform.header.stamp.fromNSec(static_cast<std::uint64_t>(stamp_ns));
      transform.header.frame_id = frame_id;
    }
  }
  template<class T> static void injectInto(T&, std::int64_t, const std::string&, std::false_type, std::false_type) {}
  template<class T> static void injectPoints(T& message, std::true_type) {
    for (auto& point : message.points) point.pose.header = message.header;
  }
  template<class T> static void injectPoints(T&, std::false_type) {}
};

// 수치 검사 필드가 없는 입력이다.
struct NoNumbers {
  template<class T> static std::vector<double> collect(const T&) { return {}; }
};
struct NoValidator {};
struct NoPolicy {};
}  // namespace detail
}  // namespace node_pipeline
