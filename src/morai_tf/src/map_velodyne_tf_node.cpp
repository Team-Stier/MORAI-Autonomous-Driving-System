#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

namespace {

constexpr double kDegreesToRadians = 0.017453292519943295;

class MapVelodyneTfNode {
 public:
  MapVelodyneTfNode() : private_node_("~") {
    private_node_.param<std::string>("gps_topic", gps_topic_,
                                     "/localization/gps/local_point");
    private_node_.param<std::string>("imu_topic", imu_topic_,
                                     "/localization/imu/data");
    private_node_.param<std::string>("parent_frame_id", parent_frame_, "map");
    private_node_.param<std::string>("child_frame_id", child_frame_, "velodyne");
    private_node_.param("max_sensor_skew_sec", max_skew_sec_, 0.2);
    private_node_.param("yaw_sign", yaw_sign_, 1.0);
    private_node_.param("yaw_offset_deg", yaw_offset_deg_, 0.0);
    private_node_.param("translation_offset_x_m", offset_x_, 0.0);
    private_node_.param("translation_offset_y_m", offset_y_, 0.0);
    private_node_.param("translation_offset_z_m", offset_z_, 0.0);
    if (gps_topic_.empty() || imu_topic_.empty() || parent_frame_.empty() ||
        child_frame_.empty() || parent_frame_ == child_frame_ ||
        !std::isfinite(max_skew_sec_) || max_skew_sec_ <= 0.0 ||
        (yaw_sign_ != 1.0 && yaw_sign_ != -1.0)) {
      throw std::invalid_argument("invalid map-Velodyne TF configuration");
    }
    gps_subscriber_ = node_.subscribe(gps_topic_, 10,
                                      &MapVelodyneTfNode::onGps, this);
    imu_subscriber_ = node_.subscribe(imu_topic_, 20,
                                      &MapVelodyneTfNode::onImu, this);
  }

 private:
  void onGps(const geometry_msgs::PointStamped::ConstPtr& gps) {
    if (!std::isfinite(gps->point.x) || !std::isfinite(gps->point.y) ||
        !std::isfinite(gps->point.z)) return;
    gps_ = *gps;
    publish();
  }

  void onImu(const sensor_msgs::Imu::ConstPtr& imu) {
    const double yaw = tf2::getYaw(imu->orientation);
    if (!std::isfinite(yaw)) return;
    imu_ = *imu;
    publish();
  }

  void publish() {
    if (gps_.header.stamp.isZero() || imu_.header.stamp.isZero() ||
        std::abs((gps_.header.stamp - imu_.header.stamp).toSec()) >
            max_skew_sec_) return;

    const double yaw = yaw_sign_ * tf2::getYaw(imu_.orientation) +
                       yaw_offset_deg_ * kDegreesToRadians;
    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw);

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = gps_.header.stamp > imu_.header.stamp
                                 ? gps_.header.stamp : imu_.header.stamp;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = gps_.point.x + offset_x_;
    transform.transform.translation.y = gps_.point.y + offset_y_;
    transform.transform.translation.z = gps_.point.z + offset_z_;
    transform.transform.rotation = tf2::toMsg(rotation);
    broadcaster_.sendTransform(transform);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber gps_subscriber_;
  ros::Subscriber imu_subscriber_;
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_msgs::PointStamped gps_;
  sensor_msgs::Imu imu_;
  std::string gps_topic_, imu_topic_, parent_frame_, child_frame_;
  double max_skew_sec_{0.2};
  double yaw_sign_{1.0}, yaw_offset_deg_{0.0};
  double offset_x_{0.0}, offset_y_{0.0}, offset_z_{0.0};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_velodyne_tf");
  try {
    MapVelodyneTfNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("map_velodyne_tf failed: %s", error.what());
    return 1;
  }
  return 0;
}
