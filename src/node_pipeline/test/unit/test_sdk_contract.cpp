#include <node_pipeline/detail/message_traits.hpp>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cassert>
#include <cstdio>
#include <unistd.h>

using namespace node_pipeline;
using namespace node_pipeline::detail;

// The fixture is confined to this translation unit; extensions cannot define runtime internals.
namespace node_pipeline { namespace detail {
class RosRuntime {
 public:
  static Parameters scopedParameters(const std::string& asset) {
    ParameterValue root;
    root.object["own"].object["coefficient"].type = ParameterValue::Type::Integer;
    root.object["own"].object["coefficient"].number = 7;
    root.object["sibling"].object["secret"].type = ParameterValue::Type::Integer;
    root.object["sibling"].object["secret"].number = 99;
    return Parameters(root, {{"declared", asset}}).child("own");
  }
};
}}
struct VectorProducer {
  std::vector<double> process(const int&) { return {1, std::numeric_limits<double>::quiet_NaN()}; }
};
struct VectorConsumer {
  double process(const std::vector<double>& values) { return values.front(); }
};

int main() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto limits = numericLimits();
  std::vector<Kind> issues;
  const auto values = std::make_tuple(std::vector<double>{nan}, std::array<double, 1>{{nan}}, std::make_pair(1, nan));
  NumericInspector<typename std::decay<decltype(values)>::type>::inspect(values, limits, issues);
  assert(issues.size() == 3);
  issues.clear();
  const double matrix[1][2] = {{0, nan}};
  NumericInspector<double[1][2]>::inspect(matrix, limits, issues);
  assert(issues == std::vector<Kind>{Kind::NaN});

  Engine engine;
  engine.input<int>({"source"});
  CoreSpec producer; producer.name = "producer"; producer.inputs = {"source"}; producer.outputs = {{"derived", {}}};
  CoreSpec consumer; consumer.name = "consumer"; consumer.inputs = {"derived"}; consumer.outputs = {{"result", {}}};
  engine.core<VectorProducer>(producer, {}); engine.core<VectorConsumer>(consumer, {});
  StageSpec first; first.name = "first"; first.section_core_map[0] = "producer";
  StageSpec second; second.name = "second"; second.section_core_map[0] = "consumer";
  engine.stage(first); engine.stage(second);
  engine.push("source", std::make_shared<const int>(1), {100, 100}, 100);
  assert(engine.process({100, 100}).status == ProcessStatus::Rejected);

  std_msgs::String text;
  text.data = "headerless";
  assert(!RosMessageAccess<std_msgs::String>::hasStamp());
  assert(RosMessageAccess<std_msgs::String>::stamp(text) == 0);
  issues.clear();
  MessageTraits<std_msgs::String>::inspect(text, limits, issues);
  assert(issues.empty());
  nav_msgs::OccupancyGrid occupancy;
  occupancy.header.frame_id = "wrong";
  occupancy.info.resolution = nan;
  auto framed_limits = limits; framed_limits.frame = "map";
  MessageTraits<nav_msgs::OccupancyGrid>::inspect(occupancy, framed_limits, issues);
  assert(std::find(issues.begin(), issues.end(), Kind::FrameMismatch) != issues.end());
  assert(issues.front() == Kind::FrameMismatch);
  sensor_msgs::Imu imu;
  imu.orientation_covariance[2] = nan;
  imu.angular_velocity_covariance[3] = nan;
  imu.linear_acceleration_covariance[4] = nan;
  issues.clear();
  MessageTraits<sensor_msgs::Imu>::inspect(imu, limits, issues);
  assert(issues.empty());
  imu.linear_acceleration.x = nan;
  MessageTraits<sensor_msgs::Imu>::inspect(imu, limits, issues);
  assert(std::find(issues.begin(), issues.end(), Kind::NaN) != issues.end());
  nav_msgs::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  ego.pose.covariance[0] = nan; ego.twist.covariance[1] = nan;
  issues.clear();
  MessageTraits<nav_msgs::Odometry>::inspect(ego, limits, issues);
  assert(issues.empty());
  ego.twist.twist.linear.x = nan;
  MessageTraits<nav_msgs::Odometry>::inspect(ego, limits, issues);
  assert(std::find(issues.begin(), issues.end(), Kind::NaN) != issues.end());
  imu.header.frame_id = "wrong";
  issues.clear();
  MessageTraits<sensor_msgs::Imu>::inspect(imu, framed_limits, issues);
  assert(issues.front() == Kind::FrameMismatch);
  interfaces::Objects objects;
  objects.objects.resize(1);
  objects.objects[0].bbox_pose.orientation.w = 1;
  objects.objects[0].velocity.x = nan;
  issues.clear();
  MessageTraits<interfaces::Objects>::inspect(objects, limits, issues);
  assert(issues.empty());
  objects.objects[0].velocity_available = true;
  MessageTraits<interfaces::Objects>::inspect(objects, limits, issues);
  assert(std::find(issues.begin(), issues.end(), Kind::NaN) != issues.end());
  sensor_msgs::PointCloud2 cloud;
  cloud.width = cloud.height = 1; cloud.point_step = cloud.row_step = sizeof(float);
  cloud.fields.resize(1); cloud.fields[0].name = "x";
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32; cloud.fields[0].count = 1;
  cloud.data.resize(sizeof(float));
  const float nan_float = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(cloud.data.data(), &nan_float, sizeof(float));
  issues.clear();
  MessageTraits<sensor_msgs::PointCloud2>::inspect(cloud, limits, issues);
  assert(std::find(issues.begin(), issues.end(), Kind::NaN) != issues.end());

  const std::string asset = "/tmp/pipeline-sdk-" + std::to_string(getpid());
  { std::ofstream stream(asset); stream << "42"; }
  const auto parameters = RosRuntime::scopedParameters(asset);
  assert(parameters.require<int>("coefficient") == 7);
  assert(!parameters.contains("sibling/secret"));
  auto stream = parameters.openAsset("declared"); int number = 0; *stream >> number;
  assert(number == 42);
  for (const std::string path : {"", "/sibling/secret", "../sibling/secret", "a//b", "coefficient/"}) {
    bool rejected = false;
    try { parameters.child(path); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
  bool denied = false;
  try { parameters.openAsset("undeclared"); } catch (const std::invalid_argument&) { denied = true; }
  assert(denied);
  std::remove(asset.c_str());
}
