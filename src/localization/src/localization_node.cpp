#include <localization/components.hpp>
#include <interfaces/SectionAnnotation.h>
namespace loc = localization;
namespace np = node_pipeline;
struct Callbacks {
  loc::Node& node;
  void gps(const sensor_msgs::NavSatFix::ConstPtr& msg) { node.onInput(loc::InputTag<loc::Input::Gps>{}, np::retainMessage(msg)); }
  void imu(const sensor_msgs::Imu::ConstPtr& msg) { node.onInput(loc::InputTag<loc::Input::Imu>{}, np::retainMessage(msg)); }
  void competition(const interfaces::CompetitionVehicleStatus::ConstPtr& msg) { node.onInput(loc::InputTag<loc::Input::Competition>{}, np::retainMessage(msg)); }
  void lidar(const sensor_msgs::PointCloud2::ConstPtr& msg) { node.onInput(loc::InputTag<loc::Input::Lidar>{}, np::retainMessage(msg)); }
  void section(const interfaces::SectionAnnotation::ConstPtr& msg) {
    node.onSection(np::Section{true, msg->state, static_cast<std::int64_t>(msg->header.stamp.toNSec()), msg->header.frame_id});
  }
  void tick(const ros::SteadyTimerEvent&) { node.onPeriodic(); }
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_node");
  ros::NodeHandle nh, params("~");
  const std::map<std::string, loc::Input> topics{
    {nh.resolveName("/sensors/gps/fix"), loc::Input::Gps}, {nh.resolveName("/sensors/imu/data"), loc::Input::Imu},
    {nh.resolveName("/vehicle/competition_status"), loc::Input::Competition}, {nh.resolveName("/velodyne_points/leveled"), loc::Input::Lidar}};
  const std::map<std::string, loc::Input> inputs{{"gps", loc::Input::Gps}, {"imu", loc::Input::Imu},
                                             {"competition_status", loc::Input::Competition}, {"lidar", loc::Input::Lidar}};
  np::CoreSelector<loc::Input, loc::CoreId> mapping;
  for (std::uint8_t section = 0; section <= interfaces::SectionAnnotation::TRAFFIC_LIGHT_STATIC_OBSTACLE; ++section)
    mapping.section_core_map[section] = loc::CoreId::FullEkf;
  // 로더에서 공통 설정을 읽고 실제 처리 strategy는 모두 Remove로 연결한다.
  np::InputPolicy<loc::Input, np::ClampUpper<loc::Access>, np::ClampLower<loc::Access>> configured_policy({1}, {-1});
  const auto settings = np::loadRosConfig(nh, params, topics, inputs,
    std::map<std::string, loc::CoreId>{{"full_EKF", loc::CoreId::FullEkf}}, mapping, configured_policy);
  loc::Policy policy({}, {});
  policy.priority.fill(100); // 모든 issue에 Remove를 적용한다. 제거한 입력은 다시 처리하지 않는다.
  loc::EkfParameters ekf;
  params.param("ekf/acceleration_noise", ekf.acceleration_noise, ekf.acceleration_noise);
  params.param("ekf/yaw_rate_noise", ekf.yaw_rate_noise, ekf.yaw_rate_noise);
  params.param("ekf/gps_position_noise", ekf.gps_position_noise, ekf.gps_position_noise);
  params.param("ekf/velocity_noise", ekf.velocity_noise, ekf.velocity_noise);
  params.param("ekf/yaw_noise", ekf.yaw_noise, ekf.yaw_noise);
  params.param("ekf/gps_height_m", ekf.gps_height_m, ekf.gps_height_m);
  params.param("ekf/yaw_offset_rad", ekf.yaw_offset_rad, ekf.yaw_offset_rad);
  for (double value : {ekf.acceleration_noise, ekf.yaw_rate_noise, ekf.gps_position_noise, ekf.velocity_noise, ekf.yaw_noise})
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid EKF noise parameter");
  if (!std::isfinite(ekf.yaw_offset_rad) || !std::isfinite(ekf.gps_height_m)) throw std::invalid_argument("invalid heading offset");
  std::array<np::HealthLimits, 3> input_limits;
  input_limits.fill(settings.health);
  const std::array<std::string, 3> input_names{{"gps", "imu", "competition_status"}};
  const std::array<double, 3> default_timeouts{{0.1, 0.06, 0.06}};
  for (std::size_t i = 0; i < input_limits.size(); ++i) {
    input_limits[i].stale_ns = np::readDurationNs(params, "inputs/" + input_names[i] + "/stale_after_sec", default_timeouts[i]);
    input_limits[i].receive_timeout_ns = np::readDurationNs(params, "inputs/" + input_names[i] + "/receive_timeout_sec", default_timeouts[i]);
  }
  loc::IO io{nh.advertise<geometry_msgs::PoseStamped>("/localization/ego", 1)};
  loc::Node node(settings.execution, loc::Buffer(settings.queue_capacity), {},
    loc::Checker(input_limits, io.readClocks().steady_ns), policy, loc::Selector{mapping},
    loc::CoreSet{loc::FullEkf(ekf)}, {}, io);
  Callbacks callbacks{node};
  const auto gps = nh.subscribe("/sensors/gps/fix", 1, &Callbacks::gps, &callbacks);
  const auto imu = nh.subscribe("/sensors/imu/data", 1, &Callbacks::imu, &callbacks);
  const auto competition = nh.subscribe("/vehicle/competition_status", 1, &Callbacks::competition, &callbacks);
  const auto lidar = nh.subscribe("/velodyne_points/leveled", 1, &Callbacks::lidar, &callbacks);
  const auto section = nh.subscribe("/section_annotation", 1, &Callbacks::section, &callbacks);
  ros::SteadyTimer timer;
  if (settings.execution.mode == np::ExecutionMode::Periodic)
    timer = nh.createSteadyTimer(ros::WallDuration(1.0 / settings.frequency_hz), &Callbacks::tick, &callbacks);
  ros::spin();
}
