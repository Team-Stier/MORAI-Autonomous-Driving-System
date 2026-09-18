#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "morai_path_manager/route/route_path.hpp"
#include "morai_path_manager/route/route_path_messages.hpp"

namespace morai_path_manager {
namespace {

std::string validFixture() {
  return std::string(MORAI_PATH_MANAGER_TEST_DATA_DIR) +
         "/valid_closed_path.txt";
}

TEST(RoutePathMessages, PreservesFrameStampPositionAndTangentYaw) {
  const RoutePath route = RoutePath::loadFromFile(validFixture(), {});
  const nav_msgs::Path message =
      makePathMessage(route.globalPoints(), "map", ros::Time(12, 34));

  ASSERT_EQ(message.poses.size(), 5U);
  EXPECT_EQ(message.header.frame_id, "map");
  EXPECT_EQ(message.header.stamp, ros::Time(12, 34));
  EXPECT_EQ(message.poses.front().header, message.header);
  EXPECT_DOUBLE_EQ(message.poses.front().pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(message.poses.front().pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(message.poses.front().pose.orientation.y, 0.0);
  EXPECT_NEAR(message.poses.front().pose.orientation.z, 0.0, 1e-12);
  EXPECT_NEAR(message.poses.front().pose.orientation.w, 1.0, 1e-12);

  const double quarter_pi = std::acos(-1.0) * 0.25;
  EXPECT_NEAR(message.poses[1].pose.orientation.z,
              std::sin(quarter_pi), 1e-12);
  EXPECT_NEAR(message.poses[1].pose.orientation.w,
              std::cos(quarter_pi), 1e-12);
  for (const geometry_msgs::PoseStamped& pose : message.poses) {
    const double norm =
        std::sqrt(pose.pose.orientation.x * pose.pose.orientation.x +
                  pose.pose.orientation.y * pose.pose.orientation.y +
                  pose.pose.orientation.z * pose.pose.orientation.z +
                  pose.pose.orientation.w * pose.pose.orientation.w);
    EXPECT_NEAR(norm, 1.0, 1e-12);
  }
}

TEST(RoutePathMessages, RejectsEmptyFrame) {
  const RoutePath route = RoutePath::loadFromFile(validFixture(), {});
  EXPECT_THROW(
      makePathMessage(route.globalPoints(), "", ros::Time(1, 0)),
      std::invalid_argument);
}

}  // namespace
}  // namespace morai_path_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
