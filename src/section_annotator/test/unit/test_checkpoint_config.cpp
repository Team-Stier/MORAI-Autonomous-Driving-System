#include <section_annotator/config.hpp>
#include <gtest/gtest.h>

TEST(CheckpointConfig, ReadsCoordinatesFromCsvWithLaneletIds) {
  std::string csv = "x,y,z,yaw,lanelet_id\r\n";
  for (int i = 0; i < 16; ++i) csv += "1.25,-2.5,28,90,100070\r\n";
  std::istringstream stream(csv);
  const auto parameters = section_annotator::readCheckpoints(stream, 10);
  EXPECT_DOUBLE_EQ(parameters.checkpoints.back().x_m, 1.25);
  EXPECT_DOUBLE_EQ(parameters.checkpoints.back().y_m, -2.5);
  const auto id = csv.find("100070");
  csv.replace(id, 6, "100070.0");
  std::istringstream invalid(csv);
  EXPECT_THROW(section_annotator::readCheckpoints(invalid, 10), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
