#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "morai_localization/orientation/imu_orientation.hpp"

namespace morai_localization {
namespace {

class ImuOrientationAdapterNode {
 public:
  ImuOrientationAdapterNode() : private_node_("~") {
    private_node_.param<std::string>("imu_input_topic", input_topic_,
                                     "/sensors/imu/data");
    private_node_.param<std::string>("imu_output_topic", output_topic_,
                                     "/localization/imu/data");
    private_node_.param<std::string>("imu_frame_id", frame_id_, "imu_link");
    private_node_.param("minimum_quaternion_norm", minimum_quaternion_norm_,
                        1.0e-6);

    if (input_topic_.empty() || output_topic_.empty() || frame_id_.empty()) {
      throw std::invalid_argument(
          "IMU topic names and frame ID must not be empty");
    }
    if (!std::isfinite(minimum_quaternion_norm_) ||
        minimum_quaternion_norm_ <= 0.0) {
      throw std::invalid_argument(
          "minimum_quaternion_norm must be finite and positive");
    }

    publisher_ = node_.advertise<sensor_msgs::Imu>(output_topic_, 50);
    subscriber_ =
        node_.subscribe(input_topic_, 50,
                        &ImuOrientationAdapterNode::handleImu, this);
  }

 private:
  void handleImu(const sensor_msgs::Imu::ConstPtr& input) {
    if (!input->header.frame_id.empty() &&
        input->header.frame_id != frame_id_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "discarding IMU message in frame '"
                   << input->header.frame_id << "' (expected '" << frame_id_
                   << "')");
      return;
    }
    if (!isFinite(input->angular_velocity) ||
        !isFinite(input->linear_acceleration)) {
      ROS_WARN_THROTTLE(5.0,
                        "discarding IMU message with non-finite motion data");
      return;
    }

    sensor_msgs::Imu output = *input;
    try {
      output.orientation =
          normalizeQuaternion(input->orientation, minimum_quaternion_norm_);
    } catch (const std::invalid_argument& error) {
      ROS_WARN_STREAM_THROTTLE(5.0, "discarding invalid IMU orientation: "
                                        << error.what());
      return;
    }

    if (output.header.stamp.isZero()) {
      output.header.stamp = ros::Time::now();
    }
    output.header.frame_id = frame_id_;
    publisher_.publish(output);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double minimum_quaternion_norm_ = 1.0e-6;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_orientation_adapter");
  try {
    morai_localization::ImuOrientationAdapterNode adapter;
    ROS_INFO("IMU orientation adapter is running");
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("failed to start IMU orientation adapter: "
                     << error.what());
    return 1;
  }
  return 0;
}
