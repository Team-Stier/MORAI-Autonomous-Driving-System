#include <section_annotator/parameters/checkpoint_sections.hpp>
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace {
nav_msgs::Odometry egoAt(double x_m, double y_m) {
  nav_msgs::Odometry ego;
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = x_m;
  ego.pose.pose.position.y = y_m;
  ego.pose.pose.orientation.w = 1;
  return ego;
}

section_annotator::CheckpointParameters loadCenters() {
  return section_annotator::loadCheckpointSectionsParameters(node_pipeline::ParamReader::fromString(
      "checkpoints_file: models/section_centers.csv\nradius_m: 10.0\n", NP_PACKAGE_DIR));
}
}  // namespace

TEST(CheckpointSections, ReaderAcceptsUpToThirtyTwoRows) {
  std::string csv = "x,y,z,yaw,lanelet_id\n";
  for (int i = 0; i < 15; ++i) csv += "0,0,0,0,1\n";
  std::istringstream too_few(csv);
  EXPECT_THROW(section_annotator::readCheckpoints(too_few, 10), std::invalid_argument);
  csv += "0,0,0,0,1\n";
  std::istringstream sixteen(csv);
  EXPECT_EQ(16u, section_annotator::readCheckpoints(sixteen, 10).checkpoints.size());
  for (int i = 0; i < 4; ++i) csv += "1,2,3,4,5\n";
  std::istringstream twenty(csv);
  const auto extra = section_annotator::readCheckpoints(twenty, 10);
  EXPECT_EQ(20u, extra.checkpoints.size());
  EXPECT_DOUBLE_EQ(1, extra.checkpoints.back().x_m);
}

TEST(CheckpointSections, OfficialSpawnPointsSelectTheirSectionOnTheFirstCall) {
  const auto loaded = loadCenters();
  ASSERT_EQ(20u, loaded.checkpoints.size());
  const struct { double x_m; double y_m; int section; } spawns[] = {
      {-131.68979755061446, -428.3310229377821, 1},
      {73.55110931396484, -238.7718963623047, 18},
      {43.761510782991536, -476.41094469698146, 20},
      {-74.86074459552765, -544.5780326710083, 19},
  };
  for (const auto& spawn : spawns) {
    section_annotator::CheckpointSections core(loaded);
    EXPECT_EQ(spawn.section, core.process(egoAt(spawn.x_m, spawn.y_m)).state);
  }
}

TEST(CheckpointSections, DrivenLatchCoversCp11Cp14AndTheFinish) {
  const auto loaded = loadCenters();
  std::ifstream stream(std::string(NP_PACKAGE_DIR) + "/../map/2026_molit_comp_global_path.txt");
  ASSERT_TRUE(stream.good());
  section_annotator::CheckpointSections core(loaded);
  double distance_m = 0;
  double previous_x = 0;
  double previous_y = 0;
  bool have_previous = false;
  std::uint8_t previous = 0;
  bool saw_17 = false;
  bool finish_from_17 = false;
  double first_15_m = -1;
  bool cp11 = true;
  bool cp13 = true;
  bool cp14 = true;
  int cp11_seen = 0;
  int cp13_seen = 0;
  int cp14_seen = 0;
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty()) continue;
    std::istringstream row(line);
    double x_m = 0;
    double y_m = 0;
    ASSERT_TRUE(static_cast<bool>(row >> x_m >> y_m));
    if (have_previous) distance_m += std::hypot(x_m - previous_x, y_m - previous_y);
    have_previous = true;
    previous_x = x_m;
    previous_y = y_m;
    const auto state = core.process(egoAt(x_m, y_m)).state;
    if (state == 15 && first_15_m < 0) first_15_m = distance_m;
    if (state == 17) saw_17 = true;
    if (previous == 17 && state == 1 && distance_m > 2000) finish_from_17 = true;
    if (std::hypot(x_m - 73.55110931396484, y_m + 238.7718963623047) < 1) {
      ++cp11_seen;
      cp11 = cp11 && state == 18;
    }
    if (std::hypot(x_m - 43.761510782991536, y_m + 476.41094469698146) < 1) {
      ++cp13_seen;
      cp13 = cp13 && state == 20;
    }
    if (std::hypot(x_m + 74.86074459552765, y_m + 544.5780326710083) < 1) {
      ++cp14_seen;
      cp14 = cp14 && state == 19;
    }
    previous = state;
  }
  EXPECT_GE(first_15_m, 1876);
  EXPECT_LT(first_15_m, 1878);
  EXPECT_GT(cp11_seen, 0);
  EXPECT_GT(cp13_seen, 0);
  EXPECT_GT(cp14_seen, 0);
  EXPECT_TRUE(cp11);
  EXPECT_TRUE(cp13);
  EXPECT_TRUE(cp14);
  EXPECT_TRUE(saw_17);
  EXPECT_TRUE(finish_from_17);
  EXPECT_EQ(1, previous);
}

TEST(CheckpointSections, KeepsStoredSectionOutsideEveryRadius) {
  const auto loaded = loadCenters();
  section_annotator::CheckpointSections core(loaded);
  ASSERT_EQ(1, core.process(egoAt(-131.68979755061446, -428.3310229377821)).state);
  EXPECT_EQ(1, core.process(egoAt(1.0e6, 1.0e6)).state);
}
