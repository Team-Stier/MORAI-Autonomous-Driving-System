#include <velocity_planner/cores/plain_velocity60.hpp>
#include <velocity_planner/cores/plain_velocity100.hpp>
#include <velocity_planner/validators/transform.hpp>
#include <node_pipeline/detail/message_traits.hpp>
#include <gtest/gtest.h>
#include <cmath>

TEST(PlainVelocity, InjectsBothSpeedsAndTransformsAllPoses) {
  interfaces::Path path;
  path.header.stamp.fromNSec(1234567890);
  path.header.frame_id = "map";
  path.points.resize(2);
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    auto& point = path.points[i];
    point.pose.header = path.header;
    point.pose.pose.position.x = 10;
    point.pose.pose.position.y = 21 + i;
    point.pose.pose.position.z = 32 + i;
    point.pose.pose.orientation.z = std::sqrt(0.5);
    point.pose.pose.orientation.w = std::sqrt(0.5);
    point.velocity = -1;
    point.acceleration = -2;
  }
  tf2_msgs::TFMessage tf;
  tf.transforms.resize(2);
  tf.transforms[0].header.frame_id = "unrelated";
  auto& transform = tf.transforms[1];
  transform.header = path.header;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = 10;
  transform.transform.translation.y = 20;
  transform.transform.translation.z = 30;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);

  velocity_planner::PlainVelocity60 core60;
  velocity_planner::PlainVelocity100 core100;
  for (const bool sixty : {true, false}) {
    const auto output = sixty ? core60.process(path, tf) : core100.process(path, tf);
    const double speed_mps = (sixty ? 60.0 : 100.0) / 3.6;
    EXPECT_TRUE(output.header.stamp.isZero());
    EXPECT_TRUE(output.header.frame_id.empty());
    ASSERT_EQ(path.points.size(), output.points.size());
    for (std::size_t i = 0; i < output.points.size(); ++i) {
      const auto& point = output.points[i];
      EXPECT_DOUBLE_EQ(speed_mps, point.velocity);
      EXPECT_DOUBLE_EQ(0, point.acceleration);
      EXPECT_NEAR(1 + i, point.pose.pose.position.x, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.position.y, 1e-12);
      EXPECT_NEAR(2 + i, point.pose.pose.position.z, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.orientation.x, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.orientation.y, 1e-12);
      EXPECT_NEAR(0, point.pose.pose.orientation.z, 1e-12);
      EXPECT_NEAR(1, point.pose.pose.orientation.w, 1e-12);
      EXPECT_EQ(path.points[i].pose.header.stamp, point.pose.header.stamp);
      EXPECT_EQ("base_link", point.pose.header.frame_id);
    }
  }
  EXPECT_EQ(-1, path.points.front().velocity);
  EXPECT_EQ(10, path.points.front().pose.pose.position.x);
}

TEST(PlainVelocityPipeline, RequiresTfBeforeCallingCore) {
  namespace np = node_pipeline;
  namespace nd = node_pipeline::detail;
  nd::Engine engine(8, 1000000000);
  nd::InputSpec path_spec("path", "/local_path"); path_spec.limits.frame = "map";
  nd::InputSpec tf_spec("tf", "/tf"); tf_spec.limits.frame = "map";
  engine.input<interfaces::Path>(path_spec); engine.input<tf2_msgs::TFMessage>(tf_spec);
  nd::CoreSpec core; core.name="plain_velocity60.hpp"; core.stage="stage1";
  core.inputs={"path","tf"}; core.outputs={{"path","path"}};
  engine.core<velocity_planner::PlainVelocity60,np::NoParameters,velocity_planner::validators::Transform>(core,{});
  nd::StageSpec stage; stage.name="stage1"; stage.section_core_map[0]=core.name; engine.stage(stage);
  interfaces::Path path; path.header.frame_id="map"; path.header.stamp.fromNSec(1000000000);
  path.points.resize(1); path.points[0].pose.header=path.header; path.points[0].pose.pose.orientation.w=1;
  const np::Clocks clocks{1000000000,1000000000};
  const auto id=engine.push<interfaces::Path>("path",std::make_shared<const interfaces::Path>(path),clocks,1000000000);
  EXPECT_EQ(np::ProcessStatus::Rejected,engine.process(clocks,false,"path",id).status);
  EXPECT_TRUE(engine.output().empty());
  tf2_msgs::TFMessage tf; tf.transforms.resize(1);
  tf.transforms[0].header=path.header; tf.transforms[0].child_frame_id="base_link";
  tf.transforms[0].transform.rotation.w=1;
  engine.push<tf2_msgs::TFMessage>("tf",std::make_shared<const tf2_msgs::TFMessage>(tf),clocks,1000000000);
  EXPECT_EQ(np::ProcessStatus::Published,engine.process(clocks,false,"path",id).status);
  EXPECT_EQ(id,engine.output().at("path").id);
}

TEST(PlainVelocityHeader, PreservesDistinctPointStamps) {
  interfaces::Path input; input.header.stamp.fromNSec(1000000000); input.header.frame_id="map";
  input.points.resize(2);
  input.points[0].pose.header=input.header; input.points[0].pose.header.stamp.fromNSec(900000000);
  input.points[1].pose.header=input.header; input.points[1].pose.header.stamp.fromNSec(950000000);
  for(auto& point:input.points) point.pose.pose.orientation.w=1;
  tf2_msgs::TFMessage tf; tf.transforms.resize(1); tf.transforms[0].header=input.header;
  tf.transforms[0].child_frame_id="base_link"; tf.transforms[0].transform.rotation.w=1;
  velocity_planner::PlainVelocity60 core;
  auto output=core.process(input,tf);
  for(std::size_t i=0;i<input.points.size();++i) {
    EXPECT_EQ(input.points[i].pose.header.stamp,output.points[i].pose.header.stamp);
    EXPECT_EQ("base_link",output.points[i].pose.header.frame_id);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
