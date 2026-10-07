#include <lidar_leveler/cores/pass_through.hpp>
#include <lidar_leveler/validators/pass_through.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cstring>
#include <cmath>

TEST(PassThrough, AppliesMountingTfAndPreservesOtherFields) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 7;
  cloud.header.stamp.fromNSec(1234567890);
  cloud.header.frame_id = "velodyne";
  cloud.height = 1;
  cloud.width = 2;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::PointField::FLOAT32,
    "ring", 1, sensor_msgs::PointField::UINT16);
  cloud.point_step = 32;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.assign(cloud.row_step, 0xa5);
  cloud.is_dense = false;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 1; *y = 2; *z = 3;
  ++x; ++y; ++z;
  *x = -1; *y = 0; *z = 0.5;

  tf2_msgs::TFMessage tf_static;
  tf_static.transforms.resize(2);
  tf_static.transforms[0].header.frame_id = "base_link";
  tf_static.transforms[0].child_frame_id = "gps_link";
  auto& mount = tf_static.transforms[1];
  mount.header.frame_id = "base_link";
  mount.child_frame_id = "velodyne";
  mount.transform.translation.x = 10;
  mount.transform.translation.y = 20;
  mount.transform.translation.z = 30;
  mount.transform.rotation.z = std::sqrt(0.5);
  mount.transform.rotation.w = std::sqrt(0.5);

  lidar_leveler::PassThroughParameters parameters;
  parameters.z_min_m = -100;
  parameters.z_max_m = 100;
  lidar_leveler::PassThrough core(parameters);
  const auto output = core.process(cloud, tf_static);

  auto expected_cloud = cloud;
  sensor_msgs::PointCloud2Iterator<float> expected_x(expected_cloud, "x"),
    expected_y(expected_cloud, "y"), expected_z(expected_cloud, "z");
  *expected_x = 8; *expected_y = 21; *expected_z = 33;
  ++expected_x; ++expected_y; ++expected_z;
  *expected_x = 10; *expected_y = 19; *expected_z = 30.5;
  const auto expected = ros::serialization::serializeMessage(expected_cloud);
  const auto actual = ros::serialization::serializeMessage(output);
  ASSERT_EQ(expected.num_bytes, actual.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected.buf.get(), actual.buf.get(), expected.num_bytes));
}

TEST(PassThroughPipeline, RequiresStaticTfAndKeepsCloudTime) {
  namespace np=node_pipeline; namespace nd=np::detail;
  nd::Engine engine(8,1000000000);
  nd::InputSpec cloud_spec("cloud","/velodyne_points"); cloud_spec.limits.frame="velodyne";
  nd::InputSpec tf_spec("tf_static","/tf_static"); tf_spec.limits.frame="base_link";
  tf_spec.limits.check_stamp=false; tf_spec.limits.check_skew=false; tf_spec.limits.check_receive_timeout=false;
  engine.input<sensor_msgs::PointCloud2>(cloud_spec); engine.input<tf2_msgs::TFMessage>(tf_spec);
  nd::CoreSpec declaration; declaration.name="pass_through.hpp"; declaration.stage="stage1";
  declaration.inputs={"cloud","tf_static"}; declaration.outputs={{"cloud","cloud"}};
  engine.core<lidar_leveler::PassThrough,np::NoParameters,lidar_leveler::validators::PassThrough>(declaration,{});
  nd::StageSpec stage; stage.name="stage1"; stage.section_core_map[0]=declaration.name; engine.stage(stage);
  sensor_msgs::PointCloud2 cloud; cloud.header.frame_id="velodyne"; cloud.header.stamp.fromNSec(1000000000);
  sensor_msgs::PointCloud2Modifier modifier(cloud); modifier.setPointCloud2FieldsByString(1,"xyz"); modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z"); *x=1;*y=2;*z=3;
  const np::Clocks clocks{1000000000,1000000000};
  const auto id=engine.push<sensor_msgs::PointCloud2>("cloud",std::make_shared<const sensor_msgs::PointCloud2>(cloud),clocks,1000000000);
  EXPECT_EQ(np::ProcessStatus::Rejected,engine.process(clocks,false,"cloud",id).status);
  EXPECT_TRUE(engine.output().empty());
  tf2_msgs::TFMessage tf;tf.transforms.resize(1);tf.transforms[0].header.frame_id="base_link";
  tf.transforms[0].child_frame_id="velodyne";tf.transforms[0].transform.rotation.w=1;
  engine.push<tf2_msgs::TFMessage>("tf_static",std::make_shared<const tf2_msgs::TFMessage>(tf),clocks,0);
  EXPECT_EQ(np::ProcessStatus::Published,engine.process(clocks,false,"cloud",id).status);
  EXPECT_EQ(id,engine.output().at("cloud").id);
  EXPECT_EQ(1000000000,engine.output().at("cloud").stamp_ns);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
