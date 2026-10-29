#include <object_detector/cores/guardrail_obb.hpp>
#include <object_detector/parameters/guardrail_obb.hpp>
#include <gtest/gtest.h>

namespace od = object_detector;
namespace {
od::GuardrailObbParameters parameters() {
  lanelet::Point3d a(1, -20, 0, 0), b(2, 80, 0, 0);
  lanelet::Point3d c(3, -20, -4, 0), d(4, 80, -4, 0);
  lanelet::LineString3d left(10, {a, b}), right(11, {c, d});
  left.attributes()["subtype"] = "solid";
  right.attributes()["subtype"] = "dashed";
  od::GuardrailObbParameters p;
  p.map = lanelet::utils::createMap(lanelet::Lanelets{lanelet::Lanelet(20, left, right)});
  return p;
}
od::Cloud box(double x, double y, double length, double width, double height = .2, double yaw = 0) {
  od::Cloud cloud;
  for (int i = 0; i <= 20; ++i) for (int j : {-1, 1}) for (int k : {0, 1}) {
    const double u = length*(i/20.0-.5), v = j*width/2;
    cloud.emplace_back(x+std::cos(yaw)*u-std::sin(yaw)*v,
                       y+std::sin(yaw)*u+std::cos(yaw)*v, .6+k*height);
  }
  return cloud;
}
interfaces::CompetitionVehicleStatus status(double time) {
  interfaces::CompetitionVehicleStatus result;
  result.header.stamp = ros::Time(time);
  result.velocity_x_mps = 12;
  return result;
}
}

TEST(GuardrailObb, RemovesLongThinRoadBoundaryReturnAndPreservesOtherPayload) {
  od::GuardrailObb core(parameters());
  const auto result = core.process({box(5, -.4, 7, .15), box(10, -2, 4.5, 1.8, 1.5)}, status(10));
  const auto& objects = std::get<0>(result).objects;
  ASSERT_EQ(objects.size(), 1u);
  EXPECT_NEAR(objects.front().position.x, 10, 1e-5);
  EXPECT_NEAR(objects.front().bbox_size.x, 4.5, 1e-5);
  EXPECT_EQ(objects.front().label, "unknown");
  EXPECT_FALSE(objects.front().velocity_available);
  EXPECT_DOUBLE_EQ(std::get<1>(result).velocity_x_mps, 12);
}

TEST(GuardrailObb, RemovesContinuousBoundaryFragmentsButRetainsIsolatedThinVehicleFace) {
  od::GuardrailObb core(parameters());
  const auto objects = std::get<0>(core.process(
      {box(0, -.2, 2, .15), box(3, -.2, 2, .15), box(6, -.2, 2, .15),
       box(20, -.2, 4.5, .15)}, status(10))).objects;
  ASSERT_EQ(objects.size(), 1u);
  EXPECT_NEAR(objects.front().position.x, 20, 1e-5);
}

TEST(GuardrailObb, PreservesMidLaneThinObjectsCrossingFacesAndTallObjects) {
  od::GuardrailObb core(parameters());
  const auto objects = std::get<0>(core.process(
      {box(5, -2, 7, .15), box(15, -.2, 7, .15, .2, 1.5707963267948966),
       box(25, -.2, 7, .15, 2)}, status(10))).objects;
  EXPECT_EQ(objects.size(), 3u);
}

TEST(GuardrailObb, PreservesTwoSeparateVehicleSidesAndSparseShortReturn) {
  od::GuardrailObb core(parameters());
  const auto objects = std::get<0>(core.process(
      {box(0, -.2, 4.5, .15), box(6, -.2, 4.5, .15), box(25, -.2, 1.5, .02, .01)}, status(10))).objects;
  EXPECT_EQ(objects.size(), 3u);
}

TEST(GuardrailObb, PreservesDisconnectedAndLaterallySeparatedFragments) {
  od::GuardrailObb core(parameters());
  EXPECT_EQ(std::get<0>(core.process(
      {box(0, -.2, 2, .15), box(5, -.2, 2, .15), box(10, -.2, 2, .15)}, status(10))).objects.size(), 3u);
  EXPECT_EQ(std::get<0>(core.process(
      {box(0, -.1, 2, .15), box(3, -1.1, 2, .15), box(6, -.1, 2, .15)}, status(11))).objects.size(), 3u);
}

TEST(GuardrailObb, RemembersMeasuredRailLocallyAndExpiresItsEvidence) {
  od::GuardrailObb core(parameters());
  EXPECT_TRUE(std::get<0>(core.process({box(5, -.2, 7, .15)}, status(10))).objects.empty());
  EXPECT_TRUE(std::get<0>(core.process({box(5, -.2, 2, .15)}, status(11))).objects.empty());
  EXPECT_EQ(std::get<0>(core.process({box(30, -.2, 2, .15)}, status(11.5))).objects.size(), 1u);
  EXPECT_EQ(std::get<0>(core.process({box(5, -.2, 2, .15)}, status(14))).objects.size(), 1u);
}

TEST(GuardrailObb, HandlesNoClustersAndLoadsConfiguredMap) {
  const auto config = od::loadGuardrailObbParameters(node_pipeline::ParamReader::fromFile(
      std::string(NP_PACKAGE_DIR)+"/config/object_detector/parameters/guardrail_obb.yaml", NP_PACKAGE_DIR));
  od::GuardrailObb core(config);
  EXPECT_TRUE(std::get<0>(core.process({}, status(10))).objects.empty());
}

TEST(GuardrailObb, JoinsShortStaticBoundaryFragmentsAcrossAdjacentScans) {
  od::GuardrailObb core(parameters());
  EXPECT_EQ(std::get<0>(core.process(
      {box(0, -.2, 1.7, .1, .65), box(2.8, -.2, 2.4, .18, .8)}, status(10))).objects.size(), 2u);
  EXPECT_TRUE(std::get<0>(core.process(
      {box(3.8, -.2, 1.2, .16, .75), box(7, -.2, 1.4, .18, .85)}, status(10.13))).objects.empty());
}

TEST(GuardrailObb, DoesNotTurnSuccessiveMovingCarSidesIntoRailEvidence) {
  od::GuardrailObb core(parameters());
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(std::get<0>(core.process(
        {box(i, -.2, 4.5, .15), box(i+6, -.2, 4.5, .15)}, status(10+i*.1))).objects.size(), 2u);
  }
}

TEST(GuardrailObb, RecordedS8GuardrailsAreRemovedAndRoundaboutVehicleReturnsRemain) {
  const auto config = od::loadGuardrailObbParameters(node_pipeline::ParamReader::fromFile(
      std::string(NP_PACKAGE_DIR)+"/config/object_detector/parameters/guardrail_obb.yaml", NP_PACKAGE_DIR));
  od::GuardrailObb core(config);
  // Native map clusters recomputed from 2026-10-29-15-34-51-gt.bag.
  od::Clusters frame_0(1);
  frame_0[0].emplace_back(-90.85681152, 214.3010101, 29.07396317);
  frame_0[0].emplace_back(-91.06363678, 214.6309814, 29.06468201);
  frame_0[0].emplace_back(-90.6396637, 213.9676056, 29.08345413);
  frame_0[0].emplace_back(-91.27500153, 214.9562836, 29.05541039);
  frame_0[0].emplace_back(-90.42242432, 213.6370544, 29.09288216);
  frame_0[0].emplace_back(-91.4850769, 215.2939606, 29.0459156);
  frame_0[0].emplace_back(-90.18449402, 213.2970886, 29.10278702);
  frame_0[0].emplace_back(-91.69841003, 215.6365967, 29.03626823);
  frame_0[0].emplace_back(-89.90855408, 212.9377441, 29.11361694);
  frame_0[0].emplace_back(-91.91156006, 215.9821014, 29.02655792);
  frame_0[0].emplace_back(-89.63751984, 212.5846558, 29.1242485);
  frame_0[0].emplace_back(-92.10952759, 216.3316803, 29.01698494);
  frame_0[0].emplace_back(-89.3731842, 212.2387695, 29.13463402);
  frame_0[0].emplace_back(-92.30876923, 216.6743164, 29.00751305);
  frame_0[0].emplace_back(-89.12774658, 211.9069977, 29.14446068);
  frame_0[0].emplace_back(-92.50950623, 217.0208435, 28.99793625);
  frame_0[0].emplace_back(-88.89438629, 211.585144, 29.1539135);
  frame_0[0].emplace_back(-92.71841431, 217.3756714, 28.98806763);
  EXPECT_EQ(std::get<0>(core.process(frame_0, status(161.9303834438))).objects.size(), 0u);
  od::Clusters frame_1(2);
  frame_1[0].emplace_back(-100.9116516, 260.2186279, 29.10126305);
  frame_1[0].emplace_back(-100.9394073, 259.8804016, 29.11903);
  frame_1[0].emplace_back(-100.9021835, 260.7655334, 29.07197952);
  frame_1[0].emplace_back(-100.9680786, 259.571991, 29.13516617);
  frame_1[0].emplace_back(-101.0121078, 259.418396, 29.14272881);
  frame_1[0].emplace_back(-101.0016861, 261.0996399, 29.71653938);
  frame_1[1].emplace_back(-100.8644791, 264.2738647, 28.88355827);
  frame_1[1].emplace_back(-100.9345474, 263.7632141, 29.66660881);
  frame_1[1].emplace_back(-101.0460739, 263.7592773, 29.66539764);
  frame_1[1].emplace_back(-100.8577118, 263.3245544, 28.93481636);
  frame_1[1].emplace_back(-100.8957825, 262.9143372, 28.95636749);
  frame_1[1].emplace_back(-100.9953232, 262.430481, 29.69128609);
  frame_1[1].emplace_back(-100.9213333, 262.3987122, 28.98376656);
  frame_1[1].emplace_back(-100.9389496, 261.8374329, 29.01372719);
  EXPECT_EQ(std::get<0>(core.process(frame_1, status(169.2110598087))).objects.size(), 2u);
  od::Clusters frame_2(2);
  frame_2[0].emplace_back(-100.7828827, 267.0697021, 28.79705429);
  frame_2[0].emplace_back(-100.9099503, 266.5863342, 29.62511063);
  frame_2[0].emplace_back(-100.8275681, 266.5579529, 28.82440758);
  frame_2[0].emplace_back(-100.7370148, 267.6172485, 28.76774216);
  frame_2[0].emplace_back(-100.8421173, 267.9225769, 29.59944344);
  frame_2[0].emplace_back(-100.9270401, 267.9205322, 29.598526);
  frame_2[1].emplace_back(-100.8682404, 264.1610413, 28.95477486);
  frame_2[1].emplace_back(-100.9225845, 263.9579468, 28.96507454);
  frame_2[1].emplace_back(-100.9886627, 263.901001, 29.67729187);
  frame_2[1].emplace_back(-101.0596161, 263.898468, 29.67651558);
  frame_2[1].emplace_back(-100.8333969, 264.6247559, 28.92994881);
  frame_2[1].emplace_back(-100.9461441, 263.4305725, 28.99351501);
  EXPECT_EQ(std::get<0>(core.process(frame_2, status(169.3411099911))).objects.size(), 0u);
  od::Clusters frame_3(3);
  frame_3[0].emplace_back(-109.0960541, 327.6614685, 29.17004585);
  frame_3[0].emplace_back(-109.3381653, 327.7712097, 29.16893578);
  frame_3[0].emplace_back(-109.5849533, 327.9036865, 29.16747665);
  frame_3[0].emplace_back(-109.8335114, 328.0391846, 29.16596413);
  frame_3[0].emplace_back(-110.0848083, 328.1835327, 29.16430664);
  frame_3[0].emplace_back(-110.3378906, 328.3308716, 29.16259384);
  frame_3[0].emplace_back(-110.5886841, 328.457489, 29.16117859);
  frame_3[0].emplace_back(-110.8504028, 328.6382751, 29.1589489);
  frame_3[0].emplace_back(-111.1156998, 328.8297729, 29.15654755);
  frame_3[0].emplace_back(-111.3939438, 329.0811157, 29.15324783);
  frame_3[1].emplace_back(-85.20207214, 335.1623535, 28.92187881);
  frame_3[1].emplace_back(-85.46461487, 335.124939, 28.92466736);
  frame_3[1].emplace_back(-84.87255096, 335.5239868, 28.91359901);
  frame_3[1].emplace_back(-85.7069397, 335.1886597, 28.92573738);
  frame_3[1].emplace_back(-85.94356537, 335.282959, 28.92628098);
  frame_3[1].emplace_back(-86.17555237, 335.4039917, 28.9263649);
  frame_3[1].emplace_back(-86.40174103, 335.5596924, 28.92585945);
  frame_3[2].emplace_back(-117.7113266, 351.058197, 28.83076477);
  frame_3[2].emplace_back(-117.4693451, 351.403656, 28.82519913);
  frame_3[2].emplace_back(-117.9481125, 350.6984253, 28.83651924);
  frame_3[2].emplace_back(-118.2338867, 350.5704651, 28.83882713);
  frame_3[2].emplace_back(-118.538681, 350.5293579, 28.8398304);
  frame_3[2].emplace_back(-118.8766861, 350.6334229, 28.83867073);
  frame_3[2].emplace_back(-119.2634659, 350.9426575, 28.83446312);
  EXPECT_EQ(std::get<0>(core.process(frame_3, status(171.1174893379))).objects.size(), 3u);
}

TEST(GuardrailObb, SingleMovingPartialFaceDoesNotPaintAStaticRail) {
  od::GuardrailObb core(parameters());
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(std::get<0>(core.process(
        {box(i*1.2, -.2, 2.5, .15)}, status(10+i*.1))).objects.size(), 1u);
  }
}

TEST(GuardrailObb, MissingObservationsDoNotRefreshRailMemory) {
  od::GuardrailObb core(parameters());
  EXPECT_TRUE(std::get<0>(core.process(
      {box(0, -.2, 2, .15), box(3, -.2, 2, .15), box(6, -.2, 2, .15)}, status(10))).objects.empty());
  EXPECT_TRUE(std::get<0>(core.process({}, status(10.4))).objects.empty());
  EXPECT_EQ(std::get<0>(core.process({box(3, -.2, 2, .15)}, status(12.1))).objects.size(), 1u);
}
