#pragma once
// 확장점 파일(Core·Validator·Policy·파라미터 로더·Bank·라이브러리)이 include하는 유일한 Pipeline 헤더다.
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <utility>
#include <vector>

namespace node_pipeline {
// 공통 검사층과 Validator가 기록하는 issue 종류다.
enum class Kind { Missing, InvalidStamp, FutureStamp, FrameMismatch, NaN, Inf,
                  OutRangePositive, OutRangeNegative, Stale, ReceiveTimeout, Skew, Count };

// node.yaml에서 정한 입력별 검사 한도다. Validator는 읽기만 한다.
struct HealthLimits {
  std::string frame;
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
  std::int64_t stale_ns = 0;
  std::int64_t receive_timeout_ns = 0;
  bool check_stamp = true;
  bool check_receive_timeout = true;
  bool check_skew = true;
};

// Policy가 보정할 입력 하나다. 원본은 유지하고 같은 타입의 복사본을 수정하거나 누락값을 채운다.
class EditableInput {
 public:
  EditableInput(std::shared_ptr<const void>& message, std::type_index type) : message_(message), type_(type) {}
  void remove() { message_.reset(); }
  template<class T> bool fill(T value) {
    if (message_ || type_ != std::type_index(typeid(T))) return false;
    message_ = std::make_shared<T>(std::move(value));
    return true;
  }
  template<class T, class Fn> void modify(Fn fn) {
    if (!message_) return;
    if (type_ != std::type_index(typeid(T))) throw std::invalid_argument("policy modify type mismatch");
    auto copy = std::make_shared<T>(*static_cast<const T*>(message_.get()));
    fn(*copy);
    message_ = std::move(copy);
  }

 private:
  std::shared_ptr<const void>& message_;
  std::type_index type_;
};

// 해당 Core의 파라미터 파일만 읽는다. 키가 없거나 타입이 다르면 예외를 던지며, 시작 단계 실패로 보고된다.
class ParamReader {
 public:
  struct Impl;
  ParamReader();
  static ParamReader fromFile(const std::string& path, const std::string& base_dir);
  static ParamReader fromString(const std::string& yaml, const std::string& base_dir = ".");
  bool has(const std::string& key) const;
  double number(const std::string& key) const;
  std::int64_t integer(const std::string& key) const;
  std::string text(const std::string& key) const;
  bool flag(const std::string& key) const;
  std::vector<double> numbers(const std::string& key) const;
  std::vector<std::string> keys(const std::string& key) const;
  // 파라미터로 지정한 정적 자산 파일의 내용이다. 상대 경로는 패키지 디렉터리 기준이다.
  std::string asset(const std::string& key) const;

 private:
  explicit ParamReader(std::shared_ptr<const Impl> impl);
  std::shared_ptr<const Impl> impl_;
};

// Core가 소유하는 시각화 발행기다. 토픽은 /visualizer/<name>으로 고정한다.
template<class T> class Visualizer {
 public:
  explicit Visualizer(const std::string& name) : topic_("/visualizer/" + name) {
    if (name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") != std::string::npos)
      throw std::invalid_argument("visualizer name must be lower snake_case: " + name);
  }
  void publish(const T& message) {
    if (!ros::isInitialized()) return;
    if (!publisher_) publisher_ = ros::NodeHandle().advertise<T>(topic_, 1, true);
    publisher_.publish(message);
  }

 private:
  std::string topic_;
  ros::Publisher publisher_;
};
}  // namespace node_pipeline
