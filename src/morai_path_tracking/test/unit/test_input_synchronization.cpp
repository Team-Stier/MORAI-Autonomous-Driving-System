#include <cstddef>
#include <cstdint>

#include <boost/bind/bind.hpp>
#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <message_filters/simple_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/PoseStamped.h>
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
                const geometry_msgs::PoseStamped::ConstPtr& ego) {
    ++count;
    last_path_stamp = path->header.stamp;
    last_ego_stamp = ego->header.stamp;
  }

  std::size_t count{0U};
  ros::Time last_path_stamp;
  ros::Time last_ego_stamp;
};

TEST(InputSynchronization, ExactPolicyDoesNotEmitMixedGenerations) {
  using Policy =
      message_filters::sync_policies::ApproximateTime<nav_msgs::Path,
                                                       geometry_msgs::PoseStamped>;
  TestFilter<nav_msgs::Path> paths;
  TestFilter<geometry_msgs::PoseStamped> ego;
  message_filters::Synchronizer<Policy> synchronizer(Policy(10U), paths,
                                                      ego);
  synchronizer.setMaxIntervalDuration(ros::Duration(0.0));
  PairCounter counter;
  synchronizer.registerCallback(
      boost::bind(&PairCounter::callback, &counter, boost::placeholders::_1,
                  boost::placeholders::_2));

  auto path_one = boost::make_shared<nav_msgs::Path>();
  auto ego_one = boost::make_shared<geometry_msgs::PoseStamped>();
  path_one->header.stamp = ros::Time(1, 0);
  ego_one->header.stamp = ros::Time(1, 0);
  paths.add(path_one);
  ego.add(ego_one);
  ASSERT_EQ(counter.count, 1U);

  auto path_two = boost::make_shared<nav_msgs::Path>();
  auto ego_three = boost::make_shared<geometry_msgs::PoseStamped>();
  path_two->header.stamp = ros::Time(2, 0);
  ego_three->header.stamp = ros::Time(3, 0);
  paths.add(path_two);
  ego.add(ego_three);

  EXPECT_EQ(counter.count, 1U);
  EXPECT_EQ(counter.last_path_stamp, ros::Time(1, 0));
  EXPECT_EQ(counter.last_ego_stamp, ros::Time(1, 0));
}

}  // namespace
}  // namespace morai_path_tracking

int main(int argc, char** argv) {
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
