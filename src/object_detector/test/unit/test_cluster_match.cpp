#include <object_detector/cores/cluster_match.hpp>
#include <object_detector/parameters/cluster_match.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <gtest/gtest.h>
#include <cmath>
#include <set>
#include <Eigen/Geometry>
#include <algorithm>

namespace pkg = object_detector;
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

pkg::ClusterMatchParameters withModels(pkg::ClusterMatchParameters parameters, std::vector<pkg::ModelTemplate> models) {
  parameters.models = std::move(models);
  return parameters;
}

// models/npc_models.bin은 Git LFS 자산이다. 파라미터 로더와 같은 경로로 읽는다.
const std::vector<pkg::ModelTemplate>& installedModels() {
  static const std::vector<pkg::ModelTemplate> models = [] {
    const auto params = node_pipeline::ParamReader::fromString("models_file: models/npc_models.bin", NP_PACKAGE_DIR);
    return pkg::NpcModelReader(params.asset("models_file")).read();
  }();
  return models;
}

sensor_msgs::PointCloud2 message(const Cloud& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.seq = 17;
  cloud.header.stamp.fromNSec(1234567890);
  cloud.header.frame_id = "base_link";
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& point : points) {
    *x = point.x; *y = point.y; *z = point.z;
    ++x; ++y; ++z;
  }
  return cloud;
}

void addBox(Cloud& points, float x_min, float y_min, float z_min,
            int x_steps, int y_steps, int z_steps, float step) {
  for (int x = 0; x <= x_steps; ++x)
    for (int y = 0; y <= y_steps; ++y)
      for (int z = 0; z <= z_steps; ++z)
        if (x == 0 || x == x_steps || y == 0 || y == y_steps || z == 0 || z == z_steps)
          points.emplace_back(x_min + step * x, y_min + step * y, z_min + step * z);
}

TEST(ClusterMatch, SeparatesClustersAndReturnsUnknownGeometryWithoutMotion) {
  Cloud points;
  addBox(points, 5, -0.2, 0, 10, 4, 5, 0.1);
  addBox(points, 10, -0.2, 0, 10, 4, 5, 0.1);
  pkg::ClusterMatch core(withModels({}, {}));
  const auto output = core.process(message(points));
  ASSERT_EQ(2u, output.objects.size());
  for (const auto& object : output.objects) {
    EXPECT_EQ("unknown", object.label);
    EXPECT_EQ(0u, object.id);
    EXPECT_FALSE(object.velocity_available);
    EXPECT_FALSE(object.acceleration_available);
    EXPECT_DOUBLE_EQ(0, object.velocity.x);
    EXPECT_DOUBLE_EQ(0, object.velocity.y);
    EXPECT_DOUBLE_EQ(0, object.velocity.z);
    EXPECT_DOUBLE_EQ(0, object.acceleration.x);
    EXPECT_DOUBLE_EQ(0, object.acceleration.y);
    EXPECT_DOUBLE_EQ(0, object.acceleration.z);
    EXPECT_NEAR(1.0, object.bbox_size.x, 1e-5);
    EXPECT_NEAR(0.4, object.bbox_size.y, 1e-5);
    EXPECT_NEAR(0.5, object.bbox_size.z, 1e-5);
    EXPECT_DOUBLE_EQ(object.position.x, object.bbox_pose.position.x);
    EXPECT_DOUBLE_EQ(object.position.y, object.bbox_pose.position.y);
    EXPECT_DOUBLE_EQ(object.position.z, object.bbox_pose.position.z);
    const auto& q = object.bbox_pose.orientation;
    EXPECT_NEAR(1, q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1e-6);
  }
  EXPECT_TRUE(output.header.frame_id.empty());
  EXPECT_TRUE(output.header.stamp.isZero());
}

TEST(ClusterMatch, RemovesRoadPlaneWhileKeepingLowObstacle) {
  Cloud points;
  for (int x = 0; x <= 50; ++x)
    for (int y = -20; y <= 20; ++y)
      points.emplace_back(4 + x * 0.1f, y * 0.1f, -0.38f);
  addBox(points, 6, -0.2, -0.36, 10, 10, 5, 0.04);
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.02;
  pkg::ClusterMatch core(withModels(parameters, {}));
  const auto output = core.process(message(points));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_LT(output.objects.front().bbox_size.x, 0.6);
  EXPECT_LT(output.objects.front().bbox_size.y, 0.6);
  EXPECT_GT(output.objects.front().bbox_size.z, 0.08);
  EXPECT_LT(output.objects.front().position.z, 0);
}

TEST(ClusterMatch, EmptyCloudReturnsNoObjects) {
  pkg::ClusterMatch core(withModels({}, {}));
  EXPECT_TRUE(core.process(message({})).objects.empty());
}

TEST(ClusterMatch, RegionalRoadHeightDoesNotMergeIntoAnObstacle) {
  Cloud points;
  for (int x = -40; x <= 80; ++x)
    for (int y = -60; y <= 60; ++y) {
      const float px = 0.5f * x, py = 0.5f * y;
      if (px < 12 || px > 28 || std::abs(py) > 8)
        points.emplace_back(px, py, -0.48f);
    }
  for (int x = 0; x <= 80; ++x)
    for (int y = -40; y <= 40; ++y)
      points.emplace_back(12 + 0.2f * x, 0.2f * y, -0.36f);
  addBox(points, 20, 0, -0.28f, 6, 6, 6, 0.04f);
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.02;
  pkg::ClusterMatch core(withModels(parameters, {}));
  const auto output = core.process(message(points));
  const auto obstacle = std::find_if(output.objects.begin(), output.objects.end(),
    [](const interfaces::Object& object) {
      return object.position.z + object.bbox_size.z * 0.5 > -0.1;
    });
  ASSERT_NE(output.objects.end(), obstacle);
  EXPECT_LT(obstacle->bbox_size.x, 1.0);
  EXPECT_LT(obstacle->bbox_size.y, 1.0);
}

pkg::ModelTemplate asymmetricModel() {
  pkg::ModelTemplate model;
  model.label = "fixture_asymmetric";
  for (int latitude = 1; latitude < 20; ++latitude) {
    const double theta = latitude * 3.141592653589793 / 20;
    for (int longitude = 0; longitude < 48; ++longitude) {
      const double phi = longitude * 2 * 3.141592653589793 / 48;
      const double bulge = 1 + 0.18 * std::sin(theta) * std::cos(3 * phi);
      model.points.emplace_back(0.9 * std::sin(theta) * std::cos(phi) * bulge,
        0.35 * std::sin(theta) * std::sin(phi),
        0.45 * std::cos(theta) + 0.08 * std::sin(theta) * std::cos(phi));
    }
  }
  model.collider_available = true;
  model.collider_pose.position.x = 0.2;
  model.collider_pose.position.y = -0.1;
  model.collider_pose.position.z = 0.15;
  model.collider_pose.orientation.z = std::sin(0.25 / 2);
  model.collider_pose.orientation.w = std::cos(0.25 / 2);
  model.collider_size.x = 1.8;
  model.collider_size.y = 0.7;
  model.collider_size.z = 0.8;
  return model;
}

Cloud partialObservation(const pkg::ModelTemplate& model) {
  Cloud points;
  for (const auto& point : model.points)
    if (point.x < 0.15)
      points.emplace_back(8 + std::cos(0.6) * point.x - std::sin(0.6) * point.y,
        3 + std::sin(0.6) * point.x + std::cos(0.6) * point.y, 0.6 + point.z);
  return points;
}

pkg::ClusterMatchParameters matchingParameters() {
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.03;
  parameters.normal_radius_m = 0.18;
  parameters.feature_radius_m = 0.3;
  parameters.registration_iterations = 3000;
  parameters.correspondence_distance_m = 0.1;
  parameters.min_inlier_fraction = 0.8;
  parameters.max_rmse_m = 0.025;
  return parameters;
}

TEST(ClusterMatch, MatchesPartialSurfaceAndTransformsColliderOffsetAndOrientation) {
  const auto model = asymmetricModel();
  pkg::ClusterMatch core(withModels(matchingParameters(), {model}));
  const auto output = core.process(message(partialObservation(model)));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  ASSERT_EQ("fixture_asymmetric", object.label);
  EXPECT_NEAR(8.2215313703, object.position.x, 0.025);
  EXPECT_NEAR(3.030394932, object.position.y, 0.025);
  EXPECT_NEAR(0.75, object.position.z, 0.025);
  EXPECT_NEAR(std::sin(0.85 / 2), object.bbox_pose.orientation.z, 0.025);
  EXPECT_NEAR(std::cos(0.85 / 2), object.bbox_pose.orientation.w, 0.025);
  EXPECT_DOUBLE_EQ(1.8, object.bbox_size.x);
  EXPECT_DOUBLE_EQ(0.7, object.bbox_size.y);
  EXPECT_DOUBLE_EQ(0.8, object.bbox_size.z);
  EXPECT_EQ(0u, object.id);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);
}

TEST(ClusterMatch, AmbiguousModelsKeepUnknownObservedBox) {
  const auto model = asymmetricModel();
  auto duplicate = model;
  duplicate.label = "fixture_duplicate";
  pkg::ClusterMatch core(withModels(matchingParameters(), {model, duplicate}));
  const auto output = core.process(message(partialObservation(model)));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("unknown", output.objects.front().label);
  EXPECT_LT(output.objects.front().bbox_size.x, 1.5);
}

TEST(ClusterMatch, ClassifiedModelWithUnverifiedColliderKeepsObservedBox) {
  auto model = asymmetricModel();
  model.collider_available = false;
  pkg::ClusterMatch core(withModels(matchingParameters(), {model}));
  const auto output = core.process(message(partialObservation(model)));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("fixture_asymmetric", output.objects.front().label);
  EXPECT_LT(output.objects.front().bbox_size.x, 1.5);
}

TEST(ClusterMatch, DenseLowObstacleTopIsNotSelectedAsRoad) {
  Cloud points;
  for (int x = 0; x <= 50; ++x)
    for (int y = -20; y <= 20; ++y) {
      const float px = 4 + x * 0.1f, py = y * 0.1f;
      if (!(px >= 6 && px <= 7 && std::abs(py) < 0.25))
        points.emplace_back(px, py, -0.38f);
    }
  for (int x = 0; x <= 100; ++x)
    for (int y = 0; y <= 50; ++y) {
      points.emplace_back(6 + x * 0.01f, -0.25f + y * 0.01f, -0.26f);
      if (x == 0 || x == 100 || y == 0 || y == 50)
        for (int z = 1; z < 12; ++z)
          points.emplace_back(6 + x * 0.01f, -0.25f + y * 0.01f, -0.38f + z * 0.01f);
    }
  pkg::ClusterMatchParameters parameters;
  parameters.voxel_leaf_m = 0.005;
  pkg::ClusterMatch core(withModels(parameters, {}));
  const auto output = core.process(message(points));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  EXPECT_LT(object.bbox_size.x, 1.2);
  EXPECT_LT(object.bbox_size.y, 0.7);
  EXPECT_GT(object.position.z + object.bbox_size.z * 0.5, -0.28);
}

TEST(ClusterMatch, NearPlanarPartialSupportDoesNotInventFullColliderPose) {
  auto model = asymmetricModel();
  for (auto& point : model.points) point.z *= 0.02;
  auto parameters = matchingParameters();
  parameters.model_max_tilt_rad = 3.141592653589793;
  pkg::ClusterMatch core(withModels(parameters, {model}));
  const auto output = core.process(message(partialObservation(model)));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("unknown", output.objects.front().label);
  EXPECT_LT(output.objects.front().bbox_size.z, 0.02);
}

// NPC 모델 자산(models/npc_models.bin) 시험이다.
sensor_msgs::PointCloud2 cloudMessage(const Cloud& points) {
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto& point : points) {
    *x = point.x; *y = point.y; *z = point.z;
    ++x; ++y; ++z;
  }
  return cloud;
}

TEST(NpcModelBank, InstalledCatalogHasUniqueFiniteStaticSurfaces) {
  const auto& models = installedModels();
  ASSERT_EQ(146u, models.size());
  std::set<std::string> labels;
  for (const auto& model : models) {
    SCOPED_TRACE(model.label);
    EXPECT_TRUE(labels.insert(model.label).second);
    if (model.points.empty()) {
      EXPECT_EQ("effect_fire01", model.label);
      continue;
    }
    ASSERT_GE(model.points.size(), 3u);
    for (const auto& point : model.points) EXPECT_TRUE(point.getVector3fMap().allFinite());
    if (model.collider_available) {
      EXPECT_GT(model.collider_size.x, 0);
      EXPECT_GT(model.collider_size.y, 0);
      EXPECT_GT(model.collider_size.z, 0);
      const auto& q = model.collider_pose.orientation;
      EXPECT_NEAR(1, q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1e-6);
    }
  }
}

TEST(NpcModelBank, ExplicitEmptyModelsKeepObservedUnknownGeometry) {
  Cloud points;
  for (int x = 0; x <= 10; ++x)
    for (int y = 0; y <= 4; ++y)
      for (int z = 0; z <= 5; ++z)
        if (x == 0 || x == 10 || y == 0 || y == 4 || z == 0 || z == 5)
          points.emplace_back(5 + x*.1f, y*.1f, z*.1f);
  pkg::ClusterMatch core(withModels({}, {}));
  const auto output = core.process(cloudMessage(points));
  ASSERT_EQ(1u, output.objects.size());
  EXPECT_EQ("unknown", output.objects.front().label);
  EXPECT_NEAR(1, output.objects.front().bbox_size.x, 1e-5);
  EXPECT_FALSE(output.objects.front().velocity_available);
  EXPECT_FALSE(output.objects.front().acceleration_available);
}

TEST(NpcModelBank, FourRecordedVehicleModelsMatchPartialSurfacesAndTransformColliders) {
  for (const auto& label : {"kia_k5", "hyundai_ioniq5",
                            "hyundai_universe_high", "hyundai_hr"}) {
    SCOPED_TRACE(label);
    const auto& models = installedModels();
    const auto found = std::find_if(models.begin(), models.end(),
      [&](const pkg::ModelTemplate& model) { return model.label == label; });
    ASSERT_NE(models.end(), found);
    const auto& model = *found;
    ASSERT_TRUE(model.collider_available);
    float min_x = INFINITY, max_x = -INFINITY;
    for (const auto& point : model.points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
    }
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d translation(20, 8, 1.5);
    Cloud observation;
    for (const auto& point : model.points)
      if (point.x <= min_x + .8f*(max_x - min_x)) {
        const Eigen::Vector3d transformed = rotation*point.getVector3fMap().cast<double>() + translation;
        observation.emplace_back(transformed.x(), transformed.y(), transformed.z());
      }
    auto parameters = pkg::ClusterMatchParameters{};
    parameters.registration_iterations = 4000;
    pkg::ClusterMatch core(withModels(parameters, {model}));
    const auto output = core.process(cloudMessage(observation));
    ASSERT_EQ(1u, output.objects.size());
    const auto& object = output.objects.front();
    ASSERT_EQ(model.label, object.label);
    const auto& center = model.collider_pose.position;
    const Eigen::Vector3d expected = rotation*Eigen::Vector3d(center.x, center.y, center.z) + translation;
    EXPECT_NEAR(expected.x(), object.position.x, .2);
    EXPECT_NEAR(expected.y(), object.position.y, .2);
    EXPECT_NEAR(expected.z(), object.position.z, .2);
    EXPECT_DOUBLE_EQ(model.collider_size.x, object.bbox_size.x);
    EXPECT_DOUBLE_EQ(model.collider_size.y, object.bbox_size.y);
    EXPECT_DOUBLE_EQ(model.collider_size.z, object.bbox_size.z);
    const auto& local_q = model.collider_pose.orientation;
    const Eigen::Quaterniond expected_q = Eigen::Quaterniond(rotation)*
      Eigen::Quaterniond(local_q.w, local_q.x, local_q.y, local_q.z);
    const auto& actual_q = object.bbox_pose.orientation;
    const Eigen::Quaterniond actual(actual_q.w, actual_q.x, actual_q.y, actual_q.z);
    EXPECT_GT(std::abs(expected_q.dot(actual)), .98);
    EXPECT_EQ(0u, object.id);
    EXPECT_FALSE(object.velocity_available);
    EXPECT_FALSE(object.acceleration_available);
  }
}

TEST(NpcModelBank, RecordedSparsePorterUsesVerifiedTwinLabelAndCollider) {
  // Recorded source frame 1791215093.585224, after the unchanged voxel/ground/cluster path.
  // XYZ SHA256: 8cbdf80ef8dc57f783cf8f783819a1833d2c22f1d8c789a15510be309b7fab3e; GT is only the assertion oracle.
  const float xyz[][3] = {
    {7.79512787f, -0.0988917723f, -0.283982545f},
    {7.79715252f, -0.0439487994f, -0.28437078f},
    {7.66054916f, -0.966919601f, -0.0366867483f},
    {7.71101141f, -0.997066677f, -0.0492840074f},
    {7.78056812f, -1.0307337f, -0.0663802847f},
    {7.57235289f, -0.898880482f, -0.0141916424f},
    {7.61958408f, -0.938614249f, -0.0263390001f},
    {7.54199171f, -0.840565979f, -0.00534356758f},
    {7.53954697f, -0.765110731f, -0.00249418756f},
    {7.55741692f, -0.681304216f, -0.00429379568f},
    {7.58430767f, -0.598160565f, -0.00845539011f},
    {7.61143827f, -0.515317142f, -0.0129544111f},
    {7.63435602f, -0.441141695f, -0.0168910548f},
    {7.65820932f, -0.367724955f, -0.0212776009f},
    {7.67727041f, -0.315088868f, -0.0249892939f},
    {7.68773079f, -0.272319436f, -0.0269388687f},
    {7.69804001f, -0.21751833f, -0.0288134608f},
    {7.7870512f, -0.177278265f, -0.0490590557f},
    {7.78086281f, -0.122778378f, -0.0473344289f},
    {7.81857204f, -0.0463132784f, -0.0558075868f},
    {8.58688068f, -1.34292352f, 0.000935502001f},
    {8.04528046f, -1.21664119f, 0.108933397f},
    {7.87967634f, -1.13984013f, 0.143279016f},
    {7.95484734f, -1.17653632f, 0.127632678f},
    {7.81612492f, -1.10461223f, 0.156635642f},
    {7.73548365f, -1.05805862f, 0.173617631f},
    {7.65164185f, -0.999690831f, 0.191553682f},
    {7.56209517f, -0.909284532f, 0.211461425f},
    {7.60347176f, -0.959052622f, 0.202048182f},
    {7.50599575f, -0.825933456f, 0.224563643f},
    {7.52297831f, -0.860414505f, 0.220365852f},
    {7.49772549f, -0.760881305f, 0.227807403f},
    {7.51212168f, -0.68817687f, 0.226726159f},
    {7.52591276f, -0.64717567f, 0.224945262f},
    {7.54596853f, -0.595716834f, 0.222083122f},
    {7.57616949f, -0.51341188f, 0.217694521f},
    {7.59612417f, -0.461579978f, 0.214641571f},
    {7.6072979f, -0.429736376f, 0.212924302f},
    {7.63001347f, -0.367381871f, 0.209298924f},
    {7.66199017f, -0.283287913f, 0.203956276f},
    {7.69185209f, -0.197536066f, 0.198804438f},
    {7.7118125f, -0.122515202f, 0.195306256f},
    {7.72705698f, -0.0684659779f, 0.192507729f},
    {7.80296803f, -0.0253001563f, 0.177815437f},
    {8.65345573f, -1.35812593f, 0.249765128f},
    {8.34549332f, -1.27491724f, 0.300137013f},
    {8.16140842f, -1.21658921f, 0.330485314f},
    {7.98899078f, -1.1371789f, 0.359582126f},
    {8.07033539f, -1.17627156f, 0.345815897f},
    {7.88373661f, -1.08494592f, 0.377415657f},
    {7.7879014f, -1.03531849f, 0.393684834f},
    {7.71421432f, -0.989831209f, 0.406356037f},
    {7.64022589f, -0.923209012f, 0.419548661f},
    {7.57062864f, -0.837051272f, 0.432402372f},
    {7.52375603f, -0.755641937f, 0.441449493f},
    {7.53333569f, -0.682072878f, 0.441319108f},
    {7.55264664f, -0.598250031f, 0.439676583f},
    {7.5795536f, -0.515834212f, 0.436626077f},
    {7.59617901f, -0.463724345f, 0.434670657f},
    {7.61177301f, -0.421286464f, 0.43268919f},
    {7.63487101f, -0.359092385f, 0.429664761f},
    {7.66195297f, -0.285442352f, 0.425988555f},
    {7.69251108f, -0.19944495f, 0.42168659f},
    {7.72430754f, -0.114048123f, 0.416993558f},
    {7.74791431f, -0.0490651168f, 0.413395584f},
    {7.76192284f, -0.0163937118f, 0.4112055f},
    {8.64410591f, -1.33311176f, 0.510672688f},
    {8.29784107f, -1.21822584f, 0.555033088f},
    {8.44759083f, -1.27136159f, 0.535777748f},
    {8.12619495f, -1.13977921f, 0.57745707f},
    {8.20038319f, -1.17663574f, 0.567707539f},
    {7.95336628f, -1.04174995f, 0.6003685f},
    {8.03297997f, -1.08978415f, 0.589765906f},
    {7.88320446f, -0.996199489f, 0.609752417f},
    {7.75245571f, -0.897651076f, 0.627423525f},
    {7.80459499f, -0.93888849f, 0.620355129f},
    {7.70298052f, -0.857454181f, 0.634126306f},
    {7.72360086f, -0.191582084f, 0.638472974f},
    {7.74843025f, -0.116674557f, 0.635649681f},
    {7.77339697f, -0.0404962264f, 0.632704496f},
    {7.65918732f, -0.81852895f, 0.640097916f},
    {7.59579659f, -0.734441161f, 0.649116278f},
    {7.61021137f, -0.779495001f, 0.646678865f},
    {7.58818626f, -0.67932272f, 0.650822401f},
    {7.59109354f, -0.604209006f, 0.651431739f},
    {7.61322975f, -0.520840943f, 0.64966464f},
    {7.64124775f, -0.436143756f, 0.64704448f},
    {7.66690063f, -0.363127232f, 0.644485235f},
    {7.68120384f, -0.320697904f, 0.643022776f},
    {7.69516611f, -0.278056592f, 0.641560316f},
    {7.70892429f, -0.234181881f, 0.640097916f},
    {8.79142189f, -1.24372101f, 0.755868614f},
    {8.52369595f, -1.13507462f, 0.780533671f},
    {8.64746475f, -1.18069196f, 0.769203424f},
    {8.3246994f, -1.05408013f, 0.79883641f},
    {8.42134476f, -1.09375691f, 0.789946496f},
    {8.20300293f, -0.999436498f, 0.810079455f},
    {7.96386337f, -0.906135976f, 0.831955552f},
    {8.0032835f, -0.911676168f, 0.828469336f},
    {7.8465414f, -0.82201916f, 0.843111515f},
    {7.9502821f, -0.858359814f, 0.83369869f},
    {7.7327981f, -0.752082586f, 0.853744507f},
    {7.79797602f, -0.793385983f, 0.847643614f},
    {7.6368103f, -0.675446033f, 0.862852216f},
    {7.60547256f, -0.596495211f, 0.86629492f},
    {7.62469196f, -0.522894204f, 0.865205526f},
    {7.55243254f, -0.431753397f, 0.872134328f},
    {7.64247608f, -0.470481008f, 0.864028871f},
    {7.4966321f, -0.375188589f, 0.877334654f},
    {7.69831371f, -0.333513498f, 0.859932542f},
    {7.71653223f, -0.28083244f, 0.858567059f},
    {7.7387495f, -0.204750955f, 0.856882095f},
    {7.75627995f, -0.150715053f, 0.855487645f},
    {7.77043533f, -0.10724517f, 0.854325533f},
    {7.79246569f, -0.0417176001f, 0.852466285f},
    {8.75420761f, -0.993702888f, 1.0192728f},
    {8.59331608f, -0.946450412f, 1.02796054f},
    {8.36484337f, -0.891587496f, 1.04020715f},
    {8.22024059f, -0.825141788f, 1.04816222f},
    {8.28469086f, -0.857105732f, 1.04460335f},
    {8.05186462f, -0.734909356f, 1.05747807f},
    {8.12924004f, -0.778805733f, 1.05318642f},
    {7.9182353f, -0.650808811f, 1.0649097f},
    {7.95366526f, -0.677168727f, 1.06292093f},
    {8.0027647f, -0.706426322f, 1.0601995f},
    {7.81917715f, -0.57509017f, 1.07045734f},
    {7.87372017f, -0.613754511f, 1.06742191f},
    {7.73436928f, -0.512582958f, 1.07516766f},
    {7.78334284f, -0.549721241f, 1.07244611f},
    {7.70403481f, -0.444172829f, 1.07702541f},
    {7.73004389f, -0.359176636f, 1.07595265f},
    {7.753932f, -0.305868298f, 1.07485354f},
    {7.76851225f, -0.26307714f, 1.07419062f},
    {7.79225349f, -0.196991041f, 1.0730741f},
    {7.82210922f, -0.121355832f, 1.07160878f},
    {7.84977198f, -0.0443142243f, 1.0702132f},
    {8.65993404f, -0.728542089f, 1.27737761f},
    {8.52113438f, -0.689665496f, 1.27985585f},
    {8.42583656f, -0.657123506f, 1.28156614f},
    {8.22908974f, -0.567422986f, 1.28512645f},
    {8.28682899f, -0.596157849f, 1.28407931f},
    {8.35241413f, -0.626030385f, 1.28289258f},
    {8.11018562f, -0.498806834f, 1.28729057f},
    {8.18120003f, -0.539903939f, 1.28599906f},
    {7.98012733f, -0.408830523f, 1.28966403f},
    {8.02676392f, -0.446127832f, 1.28880882f},
    {7.89812613f, -0.337551117f, 1.29116499f},
    {7.93818283f, -0.373502612f, 1.29043198f},
    {7.83584785f, -0.25613904f, 1.29231691f},
    {7.86983299f, -0.302650124f, 1.29168844f},
    {7.84917068f, -0.200621367f, 1.29211903f},
    {7.88076115f, -0.124706611f, 1.2916013f},
    {7.91082525f, -0.0469760299f, 1.29109514f},
    {7.92302132f, -0.00224205735f, 1.29088569f},
    {7.81502151f, 0.0441131704f, -0.28915894f},
    {7.82989693f, 0.0994379669f, -0.293300033f},
    {7.84279966f, 0.132866085f, -0.296923488f},
    {7.84874487f, 0.199535891f, -0.298994035f},
    {7.86129618f, 0.277753711f, -0.303135157f},
    {7.87986755f, 0.334364831f, -0.308829159f},
    {9.32911015f, 2.07877636f, -0.171555832f},
    {9.3162117f, 2.11922359f, -0.171174213f},
    {9.31448078f, 2.16274166f, -0.173082307f},
    {8.23915958f, 0.315453082f, -0.154561102f},
    {7.9343977f, 0.323700786f, -0.0843763724f},
    {8.91596127f, 1.88622439f, -0.0844197273f},
    {8.9374752f, 1.94702351f, -0.0914160535f},
    {9.15944862f, 2.17751598f, -0.144842565f},
    {9.21615028f, 2.16455317f, -0.154764637f},
    {8.73589325f, 2.4731679f, -0.0834020749f},
    {8.72746658f, 2.51263666f, -0.0843561143f},
    {8.77951145f, 2.6021235f, -0.0996844396f},
    {8.78386497f, 2.67564201f, -0.105345108f},
    {7.82096243f, 0.0309220403f, -0.0563324727f},
    {7.85634279f, 0.0754421353f, -0.0645806789f},
    {7.82913017f, 0.130367845f, -0.0585069992f},
    {7.85993767f, 0.0976893082f, -0.0654804856f},
    {7.83713675f, 0.163730189f, -0.0605315603f},
    {7.87346888f, 0.209191218f, -0.0692296699f},
    {7.86297894f, 0.264473081f, -0.0672800839f},
    {7.88915443f, 0.366140515f, -0.0744785294f},
    {7.89253569f, 0.422359079f, -0.0760531798f},
    {8.70901108f, 0.753895283f, -0.0059336219f},
    {8.72644424f, 0.793977201f, -0.0101314196f},
    {8.74362946f, 0.834286988f, -0.0143292183f},
    {8.84450817f, 0.910936415f, -0.0355726182f},
    {8.85273361f, 0.990190208f, -0.0391343869f},
    {8.86785793f, 1.07094705f, -0.0442226268f},
    {8.90126228f, 1.14182734f, -0.0526818261f},
    {8.98728561f, 1.19524825f, -0.0708086789f},
    {8.83499527f, 1.24976933f, -0.0433321856f},
    {8.9307518f, 1.21283758f, -0.0605049953f},
    {8.81762218f, 1.32591689f, -0.0425689518f},
    {8.84726238f, 1.39759433f, -0.0507737361f},
    {8.87199593f, 1.45569921f, -0.0576428585f},
    {8.88641357f, 1.49875557f, -0.0620314665f},
    {8.77733994f, 1.56943214f, -0.0440954193f},
    {8.85847855f, 1.54665112f, -0.0585969053f},
    {8.90036297f, 1.52850103f, -0.0658476427f},
    {8.78628731f, 1.61132896f, -0.0475299843f},
    {8.8514185f, 1.67964888f, -0.0627947003f},
    {8.8539772f, 1.720819f, -0.0650844127f},
    {8.8732748f, 1.77971268f, -0.071381107f},
    {8.89913082f, 1.82698894f, -0.0784410387f},
    {8.67261982f, 2.06620264f, -0.0479115993f},
    {8.59829235f, 2.11200428f, -0.0365584679f},
    {8.5941782f, 2.17838883f, -0.0395160057f},
    {8.65214729f, 2.2235384f, -0.0528726354f},
    {8.6420269f, 2.27521324f, -0.0540174879f},
    {8.68555546f, 2.37220979f, -0.0678829476f},
    {8.68682384f, 2.4284122f, -0.0715719163f},
    {8.66793823f, 0.711665034f, 0.00284359185f},
    {7.91982985f, 0.389279723f, 0.15281947f},
    {8.08775425f, 0.376386821f, 0.120381936f},
    {7.94239187f, 0.435836911f, 0.147858426f},
    {7.9766798f, 0.517736435f, 0.140035257f},
    {8.01518536f, 0.600991666f, 0.131194428f},
    {9.0661993f, 2.20539618f, 0.154757246f},
    {9.09103775f, 2.27020168f, 0.148082718f},
    {7.81021023f, 0.0407698303f, 0.176384374f},
    {7.82643938f, 0.118173845f, 0.173045218f},
    {7.83529949f, 0.162576273f, 0.171137124f},
    {7.84444952f, 0.207152948f, 0.169101834f},
    {7.8623414f, 0.285605013f, 0.165031239f},
    {7.88122177f, 0.342243701f, 0.160833433f},
    {8.85694885f, 1.63775635f, 0.209248602f},
    {8.87602139f, 1.70968413f, 0.203773379f},
    {8.88953972f, 1.71280336f, 0.201583311f},
    {8.91402912f, 1.8005352f, 0.194595918f},
    {8.95105362f, 1.87850714f, 0.185939863f},
    {8.98021793f, 1.95547068f, 0.178430989f},
    {9.00697613f, 2.04668427f, 0.17060928f},
    {9.03059006f, 2.1096642f, 0.16435191f},
    {9.04218292f, 2.14133334f, 0.161223218f},
    {8.70097351f, 2.22216797f, 0.209405035f},
    {8.74843597f, 2.19533587f, 0.203460529f},
    {8.6511631f, 2.27536035f, 0.214410946f},
    {8.66382694f, 2.362324f, 0.208257839f},
    {8.6931591f, 2.44176841f, 0.19986254f},
    {8.71110058f, 2.49002218f, 0.194700196f},
    {8.73428917f, 2.54052877f, 0.188599244f},
    {8.3601265f, 0.702875137f, 0.310774565f},
    {8.30852222f, 0.721612215f, 0.318596274f},
    {8.25753307f, 1.14905035f, 0.317344815f},
    {8.29008484f, 1.24015355f, 0.309757739f},
    {8.31544304f, 1.30628037f, 0.30389145f},
    {8.32981682f, 1.34616137f, 0.300449908f},
    {8.35025024f, 1.39993811f, 0.295600414f},
    {8.37542248f, 1.46772575f, 0.289499491f},
    {7.83503008f, 0.193561301f, 0.399160028f},
    {7.84874535f, 0.227250338f, 0.396813512f},
    {7.86867142f, 0.28367129f, 0.393293768f},
    {7.89778757f, 0.363315791f, 0.388053179f},
    {7.91483688f, 0.409207821f, 0.384924501f},
    {7.93215752f, 0.455446482f, 0.381691545f},
    {7.95722437f, 0.525230885f, 0.376894206f},
    {7.98356342f, 0.584358215f, 0.371940434f},
    {8.00500679f, 0.620632052f, 0.368029565f},
    {8.02639008f, 0.645668685f, 0.364275157f},
    {8.22541809f, 0.772210479f, 0.330798179f},
    {8.25883389f, 0.740212739f, 0.326105148f},
    {8.1338129f, 0.843950808f, 0.34383437f},
    {8.18402863f, 0.80293864f, 0.336742699f},
    {8.1461134f, 0.904508591f, 0.340653539f},
    {8.19508648f, 0.946891665f, 0.332049638f},
    {8.17395306f, 0.991498768f, 0.334344029f},
    {8.21847057f, 1.07017934f, 0.325479418f},
    {9.01944065f, 3.19646096f, 0.399771571f},
    {9.03265667f, 3.24877596f, 0.395749867f},
    {9.05976677f, 3.32325411f, 0.389047086f},
    {9.0880127f, 3.3990798f, 0.382100523f},
    {9.13994598f, 3.48658514f, 0.371863484f},
    {9.16942406f, 3.56493878f, 0.364551306f},
    {7.77758169f, 0.038382519f, 0.408702552f},
    {7.80629301f, 0.115600176f, 0.404009491f},
    {7.82313061f, 0.160055369f, 0.401193678f},
    {8.66969872f, 2.30608034f, 0.478255421f},
    {8.68769455f, 2.35348511f, 0.474355578f},
    {8.70730495f, 2.40174341f, 0.470212042f},
    {8.72751331f, 2.45061707f, 0.465946615f},
    {8.7486763f, 2.51434898f, 0.460949957f},
    {8.77877903f, 2.59621119f, 0.454125285f},
    {8.80992889f, 2.67941666f, 0.44705686f},
    {8.84258938f, 2.76434803f, 0.43966344f},
    {8.87071896f, 2.83378291f, 0.433407515f},
    {8.88113689f, 2.86740136f, 0.430726379f},
    {8.89831734f, 2.9187746f, 0.426460952f},
    {8.92379665f, 2.98888779f, 0.420367479f},
    {8.95049095f, 3.06025171f, 0.414030254f},
    {8.99852085f, 3.14133787f, 0.404768229f},
    {8.4316349f, 0.707756817f, 0.547477186f},
    {8.35677338f, 0.736367285f, 0.556251824f},
    {8.2733345f, 1.18580151f, 0.558689177f},
    {8.29784775f, 1.25135112f, 0.554301918f},
    {8.31627655f, 1.30397356f, 0.550889552f},
    {8.32979393f, 1.34367883f, 0.548330307f},
    {8.35310841f, 1.41051638f, 0.543902397f},
    {8.38258266f, 1.49187517f, 0.538296402f},
    {8.40675926f, 1.56032014f, 0.533584118f},
    {8.43150425f, 1.62963724f, 0.528709352f},
    {8.46096325f, 1.71368206f, 0.522778332f},
    {8.49133778f, 1.79901361f, 0.516603649f},
    {8.51621342f, 1.88395786f, 0.510997653f},
    {8.5393734f, 1.956177f, 0.505919755f},
    {8.5548315f, 2.00030637f, 0.50262928f},
    {8.57189751f, 2.04522657f, 0.499095082f},
    {8.60035324f, 2.12080455f, 0.493123502f},
    {8.62902832f, 2.19728041f, 0.487030029f},
    {8.65231895f, 2.25925541f, 0.482033372f},
    {7.7950716f, 0.0362894572f, 0.630043685f},
    {7.82521009f, 0.124798015f, 0.626204848f},
    {7.85167456f, 0.202959284f, 0.622711182f},
    {7.87927103f, 0.281912327f, 0.618953586f},
    {7.90691996f, 0.361588478f, 0.615074039f},
    {7.93583202f, 0.4421826f, 0.610910177f},
    {7.96608353f, 0.523679614f, 0.606461942f},
    {7.98797655f, 0.58247292f, 0.603171468f},
    {8.00757504f, 0.618584931f, 0.6003685f},
    {8.0290823f, 0.643633783f, 0.59744364f},
    {8.20181847f, 0.790786266f, 0.574410319f},
    {8.28181553f, 0.764298379f, 0.565026402f},
    {8.13325882f, 0.841527462f, 0.582006872f},
    {8.17980099f, 0.918612659f, 0.575100899f},
    {8.20581532f, 0.993844748f, 0.570632398f},
    {8.22828388f, 1.0572412f, 0.566732526f},
    {8.24226189f, 1.09563124f, 0.564295173f},
    {8.25501823f, 1.13405097f, 0.561979651f},
    {8.8550396f, 2.69449806f, 0.717694402f},
    {8.87539291f, 2.74581504f, 0.714469612f},
    {8.88627625f, 2.77926159f, 0.71255219f},
    {8.90324211f, 2.83000684f, 0.709588885f},
    {8.92091179f, 2.91121221f, 0.705579758f},
    {8.93035507f, 2.99001575f, 0.702267826f},
    {8.94597149f, 3.041677f, 0.699304521f},
    {8.9610939f, 3.07828808f, 0.696864188f},
    {9.03674221f, 3.14033008f, 0.688671529f},
    {9.04960823f, 3.17666435f, 0.68640548f},
    {9.06949806f, 3.23181438f, 0.682919264f},
    {9.09790325f, 3.3068428f, 0.678038538f},
    {9.11878204f, 3.36346054f, 0.674377978f},
    {9.13923264f, 3.42042112f, 0.670717478f},
    {9.16592312f, 3.4968214f, 0.665836692f},
    {9.17657089f, 3.56659746f, 0.662437677f},
    {9.16591167f, 3.61057329f, 0.66165328f},
    {8.4900856f, 1.71951234f, 0.773212612f},
    {8.47675896f, 1.78095818f, 0.773038268f},
    {8.49048615f, 1.8234911f, 0.770946562f},
    {8.50577259f, 1.87984359f, 0.76839f},
    {8.53622818f, 1.96724141f, 0.763799846f},
    {8.55874825f, 2.0267303f, 0.760487854f},
    {8.56884861f, 2.05636644f, 0.75891906f},
    {8.59246063f, 2.11709905f, 0.755432844f},
    {8.67180347f, 2.20909548f, 0.746455789f},
    {8.70128059f, 2.28719878f, 0.741952717f},
    {8.73062325f, 2.36614609f, 0.737391591f},
    {8.75781727f, 2.4454f, 0.732946634f},
    {8.79176331f, 2.52787066f, 0.727804422f},
    {8.8152504f, 2.59336042f, 0.723969579f},
    {8.83439064f, 2.64343548f, 0.720919132f},
    {7.81323719f, 0.0352922976f, 0.850650489f},
    {7.83497143f, 0.101736523f, 0.848689497f},
    {7.85004187f, 0.146327317f, 0.847294986f},
    {7.86908388f, 0.202402875f, 0.845493793f},
    {7.89729595f, 0.281589389f, 0.842762947f},
    {7.91852188f, 0.33862716f, 0.840671122f},
    {7.93156815f, 0.373103142f, 0.839363813f},
    {7.95757961f, 0.44254443f, 0.836705565f},
    {7.98862076f, 0.524367273f, 0.833466291f},
    {8.01914787f, 0.595620692f, 0.830270588f},
    {8.06112671f, 0.645636439f, 0.826203287f},
    {8.16090775f, 0.796623349f, 0.816093206f},
    {8.14081764f, 0.829522192f, 0.817487717f},
    {7.87311745f, 0.0334047303f, 1.0689919f},
    {7.90597677f, 0.123035908f, 1.06721258f},
    {7.93396235f, 0.202214047f, 1.06564248f},
    {7.96241522f, 0.282196134f, 1.06399393f},
    {7.99091387f, 0.362921715f, 1.06229293f},
    {8.0168581f, 0.444304496f, 1.0606705f},
    {8.04116154f, 0.526310325f, 1.05908298f},
    {8.06584549f, 0.586001813f, 1.0575304f},
    {8.20996475f, 0.776369452f, 1.04899955f},
    {8.21316338f, 0.80049628f, 1.04868555f},
    {7.94151735f, 0.0427433252f, 1.29055989f},
    {7.97131443f, 0.122033373f, 1.2900219f},
    {7.99620628f, 0.190540314f, 1.28955936f},
    {8.00712109f, 0.224959508f, 1.28934991f},
    {8.02883148f, 0.28281185f, 1.28893113f},
    {8.05686092f, 0.364312798f, 1.28837264f},
    {8.07440186f, 0.422844827f, 1.28800607f},
    {8.08507538f, 0.4581635f, 1.28777921f},
    {8.11216927f, 0.518141091f, 1.28722942f},
    {8.29525185f, 0.759798467f, 1.28364885f},
  };
  Cloud observation;
  for (const auto& point : xyz) observation.emplace_back(point[0], point[1], point[2]);
  const auto& models = installedModels();
  const auto find_model = [&](const char* label) {
    return std::find_if(models.begin(), models.end(),
      [&](const pkg::ModelTemplate& model) { return model.label == label; });
  };
  const auto vehicle = find_model("hyundai_hr"), twin = find_model("obj_hyundai_hr");
  ASSERT_NE(models.end(), vehicle);
  ASSERT_NE(models.end(), twin);
  ASSERT_EQ("hyundai_hr", twin->semantic_label);
  pkg::ClusterMatch core(withModels({}, {*vehicle, *twin}));
  const auto output = core.process(cloudMessage(observation));
  ASSERT_EQ(1u, output.objects.size());
  const auto& object = output.objects.front();
  ASSERT_EQ("hyundai_hr", object.label);
  Eigen::Matrix3d expected_rotation;
  expected_rotation << -0.34448617546118565, 0.9387768896631884, -0.005217887545315166,
                       -0.9387913869036424, -0.3444813410109025, 0.0018269019859438484,
                       -0.00008241153477038374, 0.005527850363453908, 0.9999847179225785;
  const Eigen::Vector3d expected_translation(9.460351979907744, 2.069300134340274, -0.08304439530037087);
  const auto& center = vehicle->collider_pose.position;
  const Eigen::Vector3d expected = expected_rotation*Eigen::Vector3d(center.x, center.y, center.z)+expected_translation;
  EXPECT_NEAR(expected.x(), object.position.x, .2);
  EXPECT_NEAR(expected.y(), object.position.y, .2);
  EXPECT_NEAR(expected.z(), object.position.z, .2);
  EXPECT_DOUBLE_EQ(vehicle->collider_size.x, object.bbox_size.x);
  EXPECT_DOUBLE_EQ(vehicle->collider_size.y, object.bbox_size.y);
  EXPECT_DOUBLE_EQ(vehicle->collider_size.z, object.bbox_size.z);
  const auto& q = object.bbox_pose.orientation;
  EXPECT_GT(std::abs(Eigen::Quaterniond(expected_rotation).dot(Eigen::Quaterniond(q.w, q.x, q.y, q.z))), .98);
  EXPECT_EQ(0u, object.id);
  EXPECT_FALSE(object.velocity_available);
  EXPECT_FALSE(object.acceleration_available);

  auto unverified = *twin;
  unverified.label = "unverified_twin";
  unverified.semantic_label.clear();
  pkg::ClusterMatch ambiguous(withModels({}, {*vehicle, unverified}));
  const auto unknown = ambiguous.process(cloudMessage(observation));
  ASSERT_EQ(1u, unknown.objects.size());
  EXPECT_EQ("unknown", unknown.objects.front().label);
}

TEST(ClusterMatchParameters, LoadsInstalledModelAssetAndRejectsInvalidValues) {
  const std::string common =
      "{voxel_leaf_m: 0.08, cluster_tolerance_m: 0.45, min_cluster_points: 6, max_cluster_points: 100000,"
      " ground_distance_m: 0.06, ground_max_tilt_rad: 0.35, ground_seed_max_z_m: -0.25, ground_cell_m: 0.5,"
      " ground_region_m: 5.0, ground_min_inliers: 50, normal_radius_m: 0.3, feature_radius_m: 0.5,"
      " registration_iterations: 1024, icp_iterations: 40, correspondence_distance_m: 0.15,"
      " min_inlier_fraction: 0.85, max_rmse_m: 0.06, ambiguity_score_margin: 0.10, min_pose_shape_ratio: 0.002,"
      " model_max_tilt_rad: 0.35, exclude_ego: true, ego_min_m: [-0.79, -1.080418, -0.3685],"
      " ego_max_m: [3.845008, 1.080417, 1.275546], models_file: models/npc_models.bin";
  const auto parameters = pkg::loadClusterMatchParameters(node_pipeline::ParamReader::fromString(common + "}", NP_PACKAGE_DIR));
  EXPECT_EQ(146u, parameters.models.size());
  EXPECT_DOUBLE_EQ(3.845008, parameters.ego_max_m[0]);
  std::string negative = common + "}";
  negative.replace(negative.find("voxel_leaf_m: 0.08"), 18, "voxel_leaf_m: -1.0");
  const auto invalid = node_pipeline::ParamReader::fromString(negative, NP_PACKAGE_DIR);
  EXPECT_THROW(pkg::loadClusterMatchParameters(invalid), std::invalid_argument);
}

TEST(ClusterMatchParameters, RejectsTruncatedModelAsset) {
  EXPECT_THROW(pkg::NpcModelReader(std::string("NPCM\x01\x00", 6)).read(), std::invalid_argument);
}
