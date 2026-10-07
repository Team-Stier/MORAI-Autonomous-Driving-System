#pragma once
#include <node_pipeline/detail/sdk_types.hpp>
#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <type_traits>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <typeindex>

namespace node_pipeline {
template<class Message> class VisualizerPublisher;
namespace detail {
template<class Message> VisualizerPublisher<Message> bindVisualizer(const std::shared_ptr<void>&, const std::string&);
}
// The SDK exposes native payloads and read-only Core configuration only.
namespace detail {
struct ParameterValue {
  enum class Type { Null, Integer, Number, Boolean, String, Object, Array } type = Type::Null;
  double number = 0;
  bool boolean = false;
  std::string string;
  std::map<std::string, ParameterValue> object;
  std::vector<ParameterValue> array;
};
template<class T> struct ParameterCast {
  static T read(const ParameterValue& v) {
    if (v.type != ParameterValue::Type::Number && v.type != ParameterValue::Type::Integer) throw std::invalid_argument("parameter must be numeric");
    if (!std::isfinite(v.number)) throw std::invalid_argument("parameter must be finite");
    if (std::is_integral<T>::value && (v.type!=ParameterValue::Type::Integer ||
        static_cast<long double>(v.number)<std::numeric_limits<T>::lowest() ||
        static_cast<long double>(v.number)>std::numeric_limits<T>::max()))
      throw std::invalid_argument("parameter must be an integer in range");
    return static_cast<T>(v.number);
  }
};
template<> struct ParameterCast<std::string> {
  static std::string read(const ParameterValue& v) {
    if (v.type != ParameterValue::Type::String) throw std::invalid_argument("parameter must be a string");
    return v.string;
  }
};
template<> struct ParameterCast<bool> {
  static bool read(const ParameterValue& v) {
    if (v.type != ParameterValue::Type::Boolean) throw std::invalid_argument("parameter must be boolean");
    return v.boolean;
  }
};
template<class T> struct ParameterCast<std::vector<T>> {
  static std::vector<T> read(const ParameterValue& v) {
    if (v.type != ParameterValue::Type::Array) throw std::invalid_argument("parameter must be an array");
    std::vector<T> out;
    for (const auto& item : v.array) out.push_back(ParameterCast<T>::read(item));
    return out;
  }
};
struct Slot {
  std::shared_ptr<const void> value;
  std::type_index type{typeid(void)};
  std::uint64_t id = 0;
  std::int64_t stamp_ns = 0, receipt_ns = 0;
  bool has_stamp = false, has_receipt = false;
  std::function<void(const void*, const HealthLimits&, std::vector<Kind>&)> inspect;
};
using Frame = std::map<std::string, Slot>;
class Engine;
class RosRuntime;
template<class C, class L, class V, class P, class B> class CoreModel;
}
class Parameters {
 public:
  Parameters() = default;
  bool contains(const std::string& key) const { return find(key) != nullptr; }
  template<class T> T require(const std::string& key) const {
    const auto* v = find(key);
    if (!v) throw std::invalid_argument("missing Core parameter: " + key);
    return detail::ParameterCast<T>::read(*v);
  }
  template<class T> T get(const std::string& key, T fallback) const {
    return contains(key) ? require<T>(key) : fallback;
  }
  Parameters child(const std::string& key) const {
    const auto* v=find(key);
    return Parameters(v ? *v : detail::ParameterValue{}, assets_);
  }
  std::unique_ptr<std::istream> openAsset(const std::string& name) const {
    const auto found=assets_.find(name);
    if (found==assets_.end()) throw std::invalid_argument("undeclared Core asset: "+name);
    std::unique_ptr<std::istream> stream(new std::ifstream(found->second));
    if (!*stream) throw std::runtime_error("cannot open declared Core asset: "+name);
    return stream;
  }
 private:
  friend class detail::RosRuntime;
  explicit Parameters(detail::ParameterValue values, std::map<std::string,std::string> assets = {})
      : values_(std::move(values)), assets_(std::move(assets)) {}
  const detail::ParameterValue* find(const std::string& key) const {
    if (key.empty() || key.front() == '/' || key.back() == '/')
      throw std::invalid_argument("Core parameter path must be relative and nonempty");
    const detail::ParameterValue* value=&values_;
    std::istringstream path(key); std::string part;
    while (std::getline(path,part,'/')) {
      if (part.empty() || part == "." || part == "..")
        throw std::invalid_argument("Core parameter path cannot traverse namespaces");
      if (!value) continue;
      const auto it=value->object.find(part);
      value=it==value->object.end() ? nullptr : &it->second;
    }
    return value;
  }
  detail::ParameterValue values_;
  std::map<std::string,std::string> assets_;
};
template<class Message> class VisualizerPublisher {
 public:
  void publish(const Message& message) const { publish_(message); }
 private:
  friend VisualizerPublisher<Message> detail::bindVisualizer<Message>(const std::shared_ptr<void>&, const std::string&);
  explicit VisualizerPublisher(std::function<void(const Message&)> publish) : publish_(std::move(publish)) {}
  std::function<void(const Message&)> publish_;
};
class Visualizer {
 public:
  template<class Message> VisualizerPublisher<Message> advertise(const std::string& topic) const {
    return detail::bindVisualizer<Message>(owner_, topic);
  }
 private:
  friend class detail::RosRuntime;
  explicit Visualizer(std::shared_ptr<void> owner) : owner_(std::move(owner)) {}
  std::shared_ptr<void> owner_;
};
struct NoParameters { NoParameters operator()(const Parameters&) const { return {}; } };
struct ProfileHealth {
  std::vector<InputHealth<std::string,Kind>> inputs;
  std::vector<Kind>& issues(const std::string& name) {
    for (auto& input:inputs) if(input.input==name) return input.issues;
    throw std::invalid_argument("unknown health input: "+name);
  }
  const std::vector<Kind>& issues(const std::string& name) const {
    for (const auto& input:inputs) if(input.input==name) return input.issues;
    throw std::invalid_argument("unknown health input: "+name);
  }
};
class InputView {
 public:
  bool present(const std::string& name) const {
    if (!allowed(name)) return false;
    const auto found=frame_.find(name); return found!=frame_.end() && bool(found->second.value);
  }
  template<class T> const T& get(const std::string& name) const {
    if (!allowed(name)) throw std::invalid_argument("input outside Core contract: "+name);
    const auto& slot=frame_.at(name);
    if (!slot.value || slot.type!=std::type_index(typeid(T))) throw std::invalid_argument("invalid native input: "+name);
    return *static_cast<const T*>(slot.value.get());
  }
  template<class T,class Fn> void modify(const std::string& name, Fn fn) {
    requireWritable();
    auto copy=std::make_shared<T>(get<T>(name)); fn(*copy); frame_.at(name).value=std::move(copy);
  }
  void remove(const std::string& name) { requireWritable();if(!allowed(name))throw std::invalid_argument("input outside Core contract: "+name);auto found=frame_.find(name); if(found!=frame_.end()) found->second.value.reset(); }
 private:
  InputView(detail::Frame& frame,const std::vector<std::string>& names,bool writable=false):frame_(frame),names_(names),writable_(writable){}
  void requireWritable()const {if(!writable_)throw std::logic_error("Validator inputs are read-only");}
  bool allowed(const std::string& name)const {return std::find(names_.begin(),names_.end(),name)!=names_.end();}
  friend class detail::Engine;
  template<class C,class L,class V,class P,class B> friend class detail::CoreModel;
  detail::Frame& frame_;
  const std::vector<std::string>& names_;
  bool writable_;
};
struct NoValidator { void operator()(const InputView&, ProfileHealth&) const {} };
struct RemoveInvalid {
  bool apply(InputView& data,const ProfileHealth& health) const {
    Issue<std::string,Kind> selected;
    const auto priority = [&data](const std::string& name, Kind kind) {
      const int ranks[] = {-1,200,200,200,200,200,100,100,-1,-1,-1};
      const auto index = static_cast<std::size_t>(kind);
      return data.present(name) && index < sizeof(ranks)/sizeof(ranks[0]) ? ranks[index] : -1;
    };
    if (!highestPriorityIssue(health, priority, selected)) return false;
    data.remove(selected.input);
    return true;
  }
};
}  // namespace node_pipeline
