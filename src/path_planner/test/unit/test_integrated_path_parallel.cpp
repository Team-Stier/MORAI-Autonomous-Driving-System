#include <gtest/gtest.h>
#include <omp.h>
#include <path_planner/cores/integrated_path.hpp>
#include <path_planner/cores/integrated_static.hpp>
#include <ros/serialization.h>

namespace {
struct ParallelFixture {
  path_planner::SharedMap shared{path_planner::SharedMapParameters{
      std::make_shared<lanelet::LaneletMap>()}};
  path_planner::Route route;
  nav_msgs::Odometry ego;
  interfaces::CompetitionVehicleStatus status;
  interfaces::Objects objects;
  sensor_msgs::Imu imu;
  path_planner::PredictedObjects predictions;

  ParallelFixture() {
    ros::Time::init();
    lanelet::LineString3d left(1001), right(1002), center(1003);
    lanelet::Id id = 2000;
    for (int x = -10; x <= 200; x += 5) {
      left.push_back(lanelet::Point3d(id++, x, 3, 0));
      right.push_back(lanelet::Point3d(id++, x, -3, 0));
      center.push_back(lanelet::Point3d(id++, x, 0, 0));
    }
    lanelet::Lanelet lane(100, left, right);
    lane.setCenterline(center);
    lane.attributes()["type"] = "lanelet";
    lane.attributes()["subtype"] = "road";
    lane.attributes()["location"] = "urban";
    lane.attributes()["one_way"] = "yes";
    shared.map->add(lane);
    route = lanelet::routing::LaneletPath({lane});
    ego.header.stamp.fromSec(100);
    ego.pose.pose.orientation.w = 1;
    ego.twist.twist.linear.x = status.velocity_x_mps = 5;
    objects.header = imu.header = ego.header;
    imu.linear_acceleration.z = 9.81;
    predictions.fill(objects);
  }

  template<class Core, class Parameters>
  void compare(Parameters parameters) {
    parameters.trajectory.solver.speed_beam_width = 16;
    parameters.trajectory.solver.qp_max_iterations = 300;
    parameters.trajectory.solver.max_speed_mps = 60. / 3.6;
    parameters.trajectory.solver.trajectory_horizon_m = 150;
    Core serial(parameters, shared), parallel(parameters, shared);
    const int original = omp_get_max_active_levels();
    omp_set_max_active_levels(0);
    const auto expected = serial.process(ego, status, objects, {}, imu, route,
                                         predictions, ego.header.stamp);
    omp_set_max_active_levels(1);
    const auto actual = parallel.process(ego, status, objects, {}, imu, route,
                                         predictions, ego.header.stamp);
    omp_set_max_active_levels(original);
    ASSERT_GT(expected.points.size(), 2u) << serial.planningDiagnostics();
    ASSERT_EQ(actual.points.size(), expected.points.size()) << parallel.planningDiagnostics();
    EXPECT_GT(actual.points.back().time_from_start.toSec(), 0);
    EXPECT_DOUBLE_EQ(actual.points.front().velocity, 5);
    const auto bytes = [](const interfaces::Path& path) {
      std::vector<uint8_t> result(ros::serialization::serializationLength(path));
      ros::serialization::OStream stream(result.data(), result.size());
      ros::serialization::serialize(stream, path);
      return result;
    };
    EXPECT_EQ(bytes(actual), bytes(expected));
  }
};
}

TEST(IntegratedPathParallel, SerialAndParallelSearchReturnIdenticalTrajectories) {
  ParallelFixture fixture;
  fixture.compare<path_planner::IntegratedPath>(
      integrated_path_detail::path_planner::IntegratedPathParameters{});
}

TEST(IntegratedStaticParallel, SerialAndParallelSearchReturnIdenticalTrajectories) {
  ParallelFixture fixture;
  fixture.compare<path_planner::IntegratedStatic>(
      integrated_static_detail::path_planner::IntegratedStaticParameters{});
}
