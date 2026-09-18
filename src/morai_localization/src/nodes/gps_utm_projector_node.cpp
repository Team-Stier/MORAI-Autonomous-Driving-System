#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include "morai_localization/projection/utm_projector.hpp"

namespace morai_localization {
namespace {

template <typename Value>
Value getRequiredParameter(const ros::NodeHandle& private_node,
                           const std::string& name) {
  Value value;
  if (!private_node.getParam(name, value)) {
    throw std::runtime_error("required private parameter '~" + name +
                             "' is missing");
  }
  return value;
}

class GpsUtmProjectorNode {
 public:
  GpsUtmProjectorNode() : private_node_("~") {
    private_node_.param("config_verified", config_.config_verified, false);
    if (!config_.config_verified) {
      throw std::invalid_argument(
          "GPS map projection config is not verified for the final scenario");
    }
    config_.utm_zone = getRequiredParameter<int>(private_node_, "utm_zone");
    private_node_.param("utm_northern", config_.utm_northern, true);
    config_.east_offset =
        getRequiredParameter<double>(private_node_, "east_offset");
    config_.north_offset =
        getRequiredParameter<double>(private_node_, "north_offset");
    private_node_.param<std::string>("frame_id", config_.frame_id, "map");
    validateProjectionConfig(config_);

    std::string input_topic;
    std::string output_topic;
    private_node_.param<std::string>("input_topic", input_topic,
                                     "/sensors/gps/fix");
    private_node_.param<std::string>("output_topic", output_topic,
                                     "/localization/gps/local_point");

    publisher_ = node_.advertise<geometry_msgs::PointStamped>(output_topic, 10);
    subscriber_ = node_.subscribe(input_topic, 10,
                                  &GpsUtmProjectorNode::handleFix, this);
  }

 private:
  void handleFix(const sensor_msgs::NavSatFix::ConstPtr& fix) {
    if (fix->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      return;
    }
    if (!std::isfinite(fix->latitude) || !std::isfinite(fix->longitude)) {
      return;
    }

    try {
      const LocalCoordinate local =
          projectToLocal(fix->latitude, fix->longitude, config_);
      geometry_msgs::PointStamped point;
      point.header.stamp = fix->header.stamp;
      point.header.frame_id = config_.frame_id;
      point.point.x = local.x;
      point.point.y = local.y;
      point.point.z = 0.0;
      publisher_.publish(point);
    } catch (const std::invalid_argument& error) {
      ROS_WARN_STREAM_THROTTLE(5.0, "discarding invalid GPS fix: "
                                        << error.what());
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ProjectionConfig config_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_utm_projector");
  try {
    morai_localization::GpsUtmProjectorNode projector;
    ROS_INFO("GPS UTM projector (C++) is running");
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("failed to start GPS UTM projector: " << error.what());
    return 1;
  }
  return 0;
}
