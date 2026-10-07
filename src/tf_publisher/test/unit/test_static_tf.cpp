#include <tf_publisher/cores/static_tf.hpp>
#include <cassert>
#include <iostream>

int main() {
  tf_publisher::StaticTfParameters settings;
  const std::string children[] = {"gps_link", "imu_link", "velodyne",
    "camera_front_optical_frame", "camera_left_optical_frame",
    "camera_right_optical_frame", "camera_7_optical_frame"};
  const double translations[][3] = {{0,0,1.2}, {1.5,0,0.5}, {1.5,0,1.403},
    {1.9,0,1.2}, {1.15,0.65,1.2}, {1.15,-0.65,1.2}, {0,0,1.5}};
  const double rotations[][4] = {{0,0,0,1}, {0,0,0,1}, {0,0,0,1},
    {-0.5086500507968373,0.5086500507968372,-0.4911976443595539,0.491197644359554},
    {-0.7544065067354889,0.1330222215594889,-0.11161889704894962,0.6330222215594892},
    {0.13302222155948917,-0.7544065067354888,0.6330222215594892,-0.1116188970489499},
    {0.4545194776720437,-0.45451947767204354,0.5416752204197018,-0.541675220419702}};
  for (std::size_t i = 0; i < 7; ++i) {
    settings.transforms.push_back({children[i],
      {{translations[i][0], translations[i][1], translations[i][2]}},
      {{rotations[i][0], rotations[i][1], rotations[i][2], rotations[i][3]}}});
  }
  tf_publisher::StaticTf core(std::move(settings));
  const auto bundle = core.process();
  assert(bundle.transforms.size() == 7);
  for (std::size_t i = 0; i < 7; ++i) {
    const auto& transform = bundle.transforms[i];
    assert(transform.header.frame_id == "base_link");
    assert(transform.header.stamp.isZero());
    assert(transform.child_frame_id == children[i]);
    assert(transform.transform.translation.x == translations[i][0]);
    assert(transform.transform.translation.y == translations[i][1]);
    assert(transform.transform.translation.z == translations[i][2]);
    assert(transform.transform.rotation.x == rotations[i][0]);
    assert(transform.transform.rotation.y == rotations[i][1]);
    assert(transform.transform.rotation.z == rotations[i][2]);
    assert(transform.transform.rotation.w == rotations[i][3]);
  }
  std::cout << "static TF bundle: 7 mounts, original coefficients/order, base_link, stamp zero\n";
}
