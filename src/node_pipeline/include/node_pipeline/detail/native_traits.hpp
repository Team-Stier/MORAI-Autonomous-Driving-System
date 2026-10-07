#pragma once
#include <node_pipeline/detail/sdk_types.hpp>
#include <array>
#include <boost/array.hpp>
#include <tuple>
#include <type_traits>
#include <utility>

namespace node_pipeline { namespace detail {
inline void inspectFrame(const std::string& frame, const HealthLimits& limits, std::vector<Kind>& issues) {
  if (!limits.frame.empty() && frame != limits.frame) issues.push_back(Kind::FrameMismatch);
}
template<class T, class Enable=void> struct NumericInspector {
  static void inspect(const T&, const HealthLimits&, std::vector<Kind>&) {
    static_assert(sizeof(T) == 0, "native payload needs a numeric inspector");
  }
};
template<class T> struct NumericInspector<T, typename std::enable_if<std::is_arithmetic<T>::value>::type> {
  static void inspect(T value, const HealthLimits& limits, std::vector<Kind>& issues) {
    checkNumber(value, limits, issues);
  }
};
template<> struct NumericInspector<std::string> {
  static void inspect(const std::string&, const HealthLimits&, std::vector<Kind>&) {}
};
template<class Range> void inspectRange(const Range& values, const HealthLimits& limits, std::vector<Kind>& issues) {
  for (const auto& value : values) NumericInspector<typename std::remove_cv<typename std::remove_reference<decltype(value)>::type>::type>::inspect(value, limits, issues);
}
template<class T, class Allocator> struct NumericInspector<std::vector<T, Allocator>> {
  static void inspect(const std::vector<T, Allocator>& values, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectRange(values, limits, issues);
  }
};
template<class Allocator> struct NumericInspector<std::vector<bool, Allocator>> {
  static void inspect(const std::vector<bool, Allocator>& values, const HealthLimits& limits, std::vector<Kind>& issues) {
    for (bool value : values) NumericInspector<bool>::inspect(value, limits, issues);
  }
};
template<class T, std::size_t N> struct NumericInspector<std::array<T, N>> {
  static void inspect(const std::array<T, N>& values, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectRange(values, limits, issues);
  }
};
template<class T, std::size_t N> struct NumericInspector<boost::array<T, N>> {
  static void inspect(const boost::array<T, N>& values, const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectRange(values, limits, issues);
  }
};
template<class T, std::size_t N> struct NumericInspector<T[N]> {
  static void inspect(const T (&values)[N], const HealthLimits& limits, std::vector<Kind>& issues) {
    inspectRange(values, limits, issues);
  }
};
template<class... T> struct NumericInspector<std::tuple<T...>> {
  template<std::size_t... I> static void values(const std::tuple<T...>& value, const HealthLimits& limits,
                                              std::vector<Kind>& issues, std::index_sequence<I...>) {
    const int checks[] = {0, (NumericInspector<T>::inspect(std::get<I>(value), limits, issues), 0)...};
    (void)checks;
  }
  static void inspect(const std::tuple<T...>& value, const HealthLimits& limits, std::vector<Kind>& issues) {
    values(value, limits, issues, std::index_sequence_for<T...>{});
  }
};
template<class A, class B> struct NumericInspector<std::pair<A, B>> {
  static void inspect(const std::pair<A, B>& value, const HealthLimits& limits, std::vector<Kind>& issues) {
    NumericInspector<A>::inspect(value.first, limits, issues);
    NumericInspector<B>::inspect(value.second, limits, issues);
  }
};
template<class Message, class Enable=void> struct MessageTraits {
  static void inspect(const Message& value, const HealthLimits& limits, std::vector<Kind>& issues) {
    NumericInspector<Message>::inspect(value, limits, issues);
  }
};
}}
