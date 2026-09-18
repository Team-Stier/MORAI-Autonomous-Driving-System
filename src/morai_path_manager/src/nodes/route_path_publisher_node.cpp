#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "morai_path_manager/route/route_path.hpp"
#include "morai_path_manager/route/route_path_messages.hpp"

namespace morai_path_manager {
namespace {

template <typename Value>
Value requiredParameter(const ros::NodeHandle& private_node,
                        const std::string& name) {
  Value value;
  if (!private_node.getParam(name, value)) {
    throw std::runtime_error("required private parameter '~" + name +
                             "' is missing");
  }
  return value;
}

struct PublisherConfig {
  std::string path_file;
  std::string localization_topic = "/localization/pose";
  std::string global_path_topic = "/global_path";
  std::string local_path_topic = "/local_path";
  std::string frame_id = "map";
  std::string local_path_mode = "point_count";
  int local_path_point_count = 20;
  double local_path_length_m = 100.0;
  double search_backward_m = 20.0;
  double search_forward_m = 200.0;
  double reacquire_distance_m = 10.0;
  RouteLoadOptions route_options;
};

void requireName(const std::string& name, const std::string& value) {
  if (value.empty()) {
    throw std::invalid_argument(name + " must not be empty");
  }
}

void requirePositive(const std::string& name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and positive");
  }
}

PublisherConfig loadPublisherConfig(const ros::NodeHandle& private_node) {
  PublisherConfig config;
  config.path_file =
      requiredParameter<std::string>(private_node, "path_file");
  private_node.param<std::string>(
      "localization_topic", config.localization_topic,
      config.localization_topic);
  private_node.param<std::string>(
      "global_path_topic", config.global_path_topic,
      config.global_path_topic);
  private_node.param<std::string>(
      "local_path_topic", config.local_path_topic,
      config.local_path_topic);
  private_node.param<std::string>("frame_id", config.frame_id,
                                  config.frame_id);
  private_node.param<std::string>("local_path_mode", config.local_path_mode,
                                  config.local_path_mode);
  private_node.param("local_path_point_count",
                     config.local_path_point_count,
                     config.local_path_point_count);
  private_node.param("local_path_length_m", config.local_path_length_m,
                     config.local_path_length_m);
  private_node.param("search_backward_m", config.search_backward_m,
                     config.search_backward_m);
  private_node.param("search_forward_m", config.search_forward_m,
                     config.search_forward_m);
  private_node.param("reacquire_distance_m", config.reacquire_distance_m,
                     config.reacquire_distance_m);
  private_node.param("max_path_point_spacing_m",
                     config.route_options.max_point_spacing_m,
                     config.route_options.max_point_spacing_m);
  private_node.param("flatten_z", config.route_options.flatten_z,
                     config.route_options.flatten_z);
  private_node.param("require_closed_loop",
                     config.route_options.require_closed_loop,
                     config.route_options.require_closed_loop);

  requireName("path_file", config.path_file);
  requireName("localization_topic", config.localization_topic);
  requireName("global_path_topic", config.global_path_topic);
  requireName("local_path_topic", config.local_path_topic);
  requireName("frame_id", config.frame_id);
  if (config.local_path_mode != "point_count" &&
      config.local_path_mode != "distance") {
    throw std::invalid_argument(
        "local_path_mode must be 'point_count' or 'distance'");
  }
  if (config.local_path_point_count <= 0) {
    throw std::invalid_argument("local_path_point_count must be positive");
  }
  requirePositive("local_path_length_m", config.local_path_length_m);
  requirePositive("search_backward_m", config.search_backward_m);
  requirePositive("search_forward_m", config.search_forward_m);
  requirePositive("reacquire_distance_m", config.reacquire_distance_m);
  requirePositive("max_path_point_spacing_m",
                  config.route_options.max_point_spacing_m);
  return config;
}

class RoutePathPublisherNode {
 public:
  RoutePathPublisherNode()
      : private_node_("~"),
        config_(loadPublisherConfig(private_node_)),
        route_(RoutePath::loadFromFile(config_.path_file,
                                       config_.route_options)) {
    if (config_.local_path_mode == "point_count" &&
        static_cast<std::size_t>(config_.local_path_point_count) >
            route_.points().size()) {
      throw std::invalid_argument(
          "local_path_point_count exceeds loaded route point count");
    }
    global_publisher_ =
        node_.advertise<nav_msgs::Path>(config_.global_path_topic, 1, true);
    local_publisher_ =
        node_.advertise<nav_msgs::Path>(config_.local_path_topic, 1, false);
    localization_subscriber_ =
        node_.subscribe(config_.localization_topic, 10,
                        &RoutePathPublisherNode::handleLocalization, this);

    global_publisher_.publish(makePathMessage(
        route_.globalPoints(), config_.frame_id, ros::Time::now()));
    ROS_INFO_STREAM("route path publisher loaded "
                    << route_.rawPointCount() << " raw points as "
                    << route_.points().size() << " route points ("
                    << route_.totalLength() << " m)");
  }

 private:
  void handleLocalization(
      const geometry_msgs::PoseStamped::ConstPtr& pose) {
    if (!std::isfinite(pose->pose.position.x) ||
        !std::isfinite(pose->pose.position.y)) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding non-finite localization pose");
      return;
    }
    if (!pose->header.frame_id.empty() &&
        pose->header.frame_id != config_.frame_id) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding localization pose in unexpected frame '"
                   << pose->header.frame_id << "'");
      return;
    }

    const NearestResult nearest =
        has_previous_index_
            ? route_.nearestWithContinuity(
                  pose->pose.position.x, pose->pose.position.y,
                  previous_index_,
                  config_.search_backward_m, config_.search_forward_m,
                  config_.reacquire_distance_m)
            : route_.nearest(pose->pose.position.x,
                             pose->pose.position.y);

    previous_index_ = nearest.index;
    has_previous_index_ = true;
    const std::vector<RoutePoint> local_points =
        config_.local_path_mode == "point_count"
            ? route_.extractForwardPoints(
                  nearest.index,
                  static_cast<std::size_t>(
                      config_.local_path_point_count))
            : route_.extractForward(
                  nearest.index, config_.local_path_length_m);
    local_publisher_.publish(makePathMessage(
        local_points, config_.frame_id, pose->header.stamp));
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  PublisherConfig config_;
  RoutePath route_;
  ros::Publisher global_publisher_;
  ros::Publisher local_publisher_;
  ros::Subscriber localization_subscriber_;
  bool has_previous_index_ = false;
  std::size_t previous_index_ = 0U;
};

}  // namespace
}  // namespace morai_path_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "route_path_publisher");
  try {
    morai_path_manager::RoutePathPublisherNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("failed to start route path publisher: "
                     << error.what());
    return 1;
  }
  return 0;
}
