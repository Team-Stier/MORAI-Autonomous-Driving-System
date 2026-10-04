#include <tf_publisher/components.hpp>
#include <node_pipeline/ros.hpp>
namespace pkg = tf_publisher;
namespace np = node_pipeline;
struct RosAccess {
  template<class Message> static std::int64_t stamp(const Message& m) { return m.header.stamp.toNSec(); }
  template<class Message> static const std::string& frame(const Message& m) { return m.header.frame_id; }
};
using Message = pkg::Message;
using Components = pkg::Components<RosAccess>;
using IO = np::RosIO<Message>;
struct RosHeaderInjector {
  void injectHeader(tf2_msgs::TFMessage& message, const np::Context<1>& context) const {
    for (auto& transform : message.transforms) {
      transform.header.stamp.fromNSec(context.stamp_ns);
      transform.header.frame_id = "map";
    }
  }
};
using Node = Components::Node<IO, RosHeaderInjector>;
struct RosCallbacks {
  Node& node;
  void onEgo(const nav_msgs::Odometry::ConstPtr& msg) { node.onInput(pkg::InputTag<pkg::Input::Ego>{}, np::retainMessage(msg)); }
  void onPeriodic(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_publisher_node");
  ros::NodeHandle nh, params("~");
  auto selector = Components::selector();
  auto policy = Components::policy(np::HealthLimits{});
  const auto settings = np::loadRosConfig(nh, params,
    std::map<std::string, pkg::Input>{{nh.resolveName("/localization/ego"), pkg::Input::Ego}},
    std::map<std::string, pkg::Input>{{"ego", pkg::Input::Ego}},
    std::map<std::string, pkg::CoreId>{}, selector, policy);
  ros::NodeHandle stage1_params(params, "stage1");
  np::loadStageConfig(stage1_params,
    std::map<std::string, pkg::Input>{{"ego", pkg::Input::Ego}},
    std::map<std::string, pkg::CoreId>{{"ego_tf.hpp", pkg::CoreId::EgoTf}}, selector, policy);
  std::array<np::HealthLimits, 1> limits;
  limits.fill(settings.health);
  limits[0].frame = "map";
  limits[0].stale_ns = np::readDurationNs(params, "inputs/ego/stale_after_sec", 0.3);
  limits[0].receive_timeout_ns = np::readDurationNs(params, "inputs/ego/receive_timeout_sec", 0.3);
  IO io{nh.advertise<tf2_msgs::TFMessage>("/tf", 1)};
  Node node(settings.execution, Components::Buffer(settings.queue_capacity), {},
    {np::makeStage<pkg::CoreSet, Components::Buffer::ContextType, pkg::Data>(
      Components::Checker(limits, io.readClocks().steady_ns), policy, selector, pkg::processCore)}, {}, {}, io);
  RosCallbacks callbacks{node};
  const auto sub_ego = nh.subscribe("/localization/ego", 1, &RosCallbacks::onEgo, &callbacks);
  // 정적 장착 변환은 노드 설정이며 계산 Core와 독립이다.
  const auto static_publisher = nh.advertise<tf2_msgs::TFMessage>("/tf_static", 1, true);
  XmlRpc::XmlRpcValue mounts;
  if (!params.getParam("static_transforms", mounts) || mounts.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::invalid_argument("invalid static_transforms");
  tf2_msgs::TFMessage static_message;
  for (int i = 0; i < mounts.size(); ++i) {
    auto& mount = mounts[i];
    if (mount.getType() != XmlRpc::XmlRpcValue::TypeStruct || !mount.hasMember("child") ||
        !mount.hasMember("translation") || !mount.hasMember("rotation") ||
        mount["child"].getType() != XmlRpc::XmlRpcValue::TypeString ||
        mount["translation"].getType() != XmlRpc::XmlRpcValue::TypeArray || mount["translation"].size() != 3 ||
        mount["rotation"].getType() != XmlRpc::XmlRpcValue::TypeArray || mount["rotation"].size() != 4)
      throw std::invalid_argument("invalid static transform entry");
    auto number = [](XmlRpc::XmlRpcValue& value) {
      const double result = value.getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(value)
                          : value.getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<double>(value)
                          : throw std::invalid_argument("invalid transform number");
      if (!std::isfinite(result)) throw std::invalid_argument("nonfinite transform number");
      return result;
    };
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = static_cast<std::string>(mount["child"]);
    transform.transform.translation.x = number(mount["translation"][0]);
    transform.transform.translation.y = number(mount["translation"][1]);
    transform.transform.translation.z = number(mount["translation"][2]);
    transform.transform.rotation.x = number(mount["rotation"][0]);
    transform.transform.rotation.y = number(mount["rotation"][1]);
    transform.transform.rotation.z = number(mount["rotation"][2]);
    transform.transform.rotation.w = number(mount["rotation"][3]);
    const auto& r = transform.transform.rotation;
    if (transform.child_frame_id.empty() || transform.child_frame_id == "base_link" ||
        std::fabs(r.x*r.x + r.y*r.y + r.z*r.z + r.w*r.w - 1) > 1e-6)
      throw std::invalid_argument("invalid static transform frame or quaternion");
    static_message.transforms.push_back(transform);
  }
  static_publisher.publish(static_message);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &RosCallbacks::onPeriodic, &callbacks);
  ros::spin();
}
