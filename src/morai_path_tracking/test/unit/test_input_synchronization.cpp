#include <cstddef>
#include <cstdint>

#include <boost/bind/bind.hpp>
#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/duration.h>
#include <ros/time.h>

namespace morai_path_tracking {
namespace {

template <typename Message>
class TestFilter : public message_filters::SimpleFilter<Message> {
 public:
  void add(const boost::shared_ptr<const Message>& message) {
    this->signalMessage(message);
  }
};

class PairCounter {
 public:
  void callback(const nav_msgs::Path::ConstPtr& path,
                const nav_msgs::Odometry::ConstPtr& odometry) {
    ++count;
    last_path_stamp = path->header.stamp;
    last_odometry_stamp = odometry->header.stamp;
  }

  std::size_t count{0U};
  ros::Time last_path_stamp;
  ros::Time last_odometry_stamp;
};

TEST(InputSynchronization, ExactPolicyDoesNotEmitMixedGenerations) {
  using Policy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::Path,
                                                       nav_msgs::Odometry>;
  TestFilter<nav_msgs::Path> paths;
  TestFilter<nav_msgs::Odometry> odometry;
  message_filters::Synchronizer<Policy> synchronizer(Policy(10U), paths,
                                                      odometry);
  synchronizer.setMaxIntervalDuration(ros::Duration(0.0));
  PairCounter counter;
  synchronizer.registerCallback(
      boost::bind(&PairCounter::callback, &counter, boost::placeholders::_1,
                  boost::placeholders::_2));

  auto path_one = boost::make_shared<nav_msgs::Path>();
  auto odometry_one = boost::make_shared<nav_msgs::Odometry>();
  path_one->header.stamp = ros::Time(1, 0);
  odometry_one->header.stamp = ros::Time(1, 0);
  paths.add(path_one);
  odometry.add(odometry_one);
  ASSERT_EQ(counter.count, 1U);

  auto path_two = boost::make_shared<nav_msgs::Path>();
  auto odometry_three = boost::make_shared<nav_msgs::Odometry>();
  path_two->header.stamp = ros::Time(2, 0);
  odometry_three->header.stamp = ros::Time(3, 0);
  paths.add(path_two);
  odometry.add(odometry_three);

  EXPECT_EQ(counter.count, 1U);
  EXPECT_EQ(counter.last_path_stamp, ros::Time(1, 0));
  EXPECT_EQ(counter.last_odometry_stamp, ros::Time(1, 0));
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
