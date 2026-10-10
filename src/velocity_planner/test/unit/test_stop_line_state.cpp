#include <velocity_planner/parameters/stop_line_state.hpp>
#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
class MapAsset {
 public:
  explicit MapAsset(const std::string& xml) {
    char name[] = "/tmp/stop_line_map_XXXXXX";
    const int descriptor = mkstemp(name);
    if (descriptor < 0) throw std::runtime_error("cannot create map fixture");
    close(descriptor);
    path_ = name;
    std::ofstream(path_) << xml;
  }
  ~MapAsset() { std::remove(path_.c_str()); }
  node_pipeline::ParamReader parameters(const std::string& coefficients =
      "x_m: 5\ny_m: 100\nz_mps2: 3\nn_frames: 3\n") const {
    return node_pipeline::ParamReader::fromString(coefficients + "map_file: " + path_ + "\n");
  }
 private:
  std::string path_;
};

const std::string nodes =
    "<node id='1'><tag k='local_x' v='10'/><tag k='local_y' v='-2'/></node>"
    "<node id='2'><tag k='local_x' v='12.5'/><tag k='local_y' v='-4'/></node>";
const std::string line =
    "<way id='9'><nd ref='1'/><nd ref='2'/><tag k='type' v='stop_line'/></way>";
}  // namespace

TEST(StopLineStateLoader, LoadsAllUnlinkedStopLinesFromRealMap) {
  const auto params = node_pipeline::ParamReader::fromString(
      "x_m: 5\ny_m: 100\nz_mps2: 3\nn_frames: 3\nmap_file: ../map/kcity_lanelet2.osm\n", NP_PACKAGE_DIR);
  const auto result = velocity_planner::loadStopLineStateParameters(params);
  EXPECT_DOUBLE_EQ(5, result.x_m);
  EXPECT_DOUBLE_EQ(100, result.y_m);
  EXPECT_DOUBLE_EQ(3, result.z_mps2);
  EXPECT_EQ(3, result.n_frames);
  ASSERT_EQ(97u, result.stop_lines.size());
  EXPECT_EQ(3, result.stop_lines.front().id);
  ASSERT_EQ(2u, result.stop_lines.front().points.size());
  EXPECT_NEAR(-41.133953261945862, result.stop_lines.front().points.front()[0], 1e-10);
  EXPECT_NEAR(100.3985118442215, result.stop_lines.front().points.front()[1], 1e-10);
}

TEST(StopLineStateLoader, UsesLocalCoordinatesAndIgnoresOtherWays) {
  const MapAsset asset("<osm>" + nodes + line + "<way id='10'><tag k='type' v='virtual'/></way></osm>");
  const auto result = velocity_planner::loadStopLineStateParameters(asset.parameters(
      "x_m: 0\ny_m: 40\nz_mps2: 2.5\nn_frames: 1\n"));
  EXPECT_DOUBLE_EQ(0, result.x_m);
  EXPECT_DOUBLE_EQ(40, result.y_m);
  EXPECT_DOUBLE_EQ(2.5, result.z_mps2);
  EXPECT_EQ(1, result.n_frames);
  ASSERT_EQ(1u, result.stop_lines.size());
  EXPECT_EQ(9, result.stop_lines.front().id);
  ASSERT_EQ(2u, result.stop_lines.front().points.size());
  EXPECT_EQ((std::array<double, 2>{{10, -2}}), result.stop_lines.front().points.front());
  EXPECT_EQ((std::array<double, 2>{{12.5, -4}}), result.stop_lines.front().points.back());
}

class InvalidMap : public testing::TestWithParam<std::string> {};
TEST_P(InvalidMap, RejectsMalformedStopLineAssets) {
  const MapAsset asset(GetParam());
  EXPECT_THROW(velocity_planner::loadStopLineStateParameters(asset.parameters()), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(StopLineStateLoader, InvalidMap, testing::Values(
    "<osm>", "<other/>", "<osm/>",
    "<osm>" + nodes + "<way id='9'><nd ref='1'/><nd ref='3'/><tag k='type' v='stop_line'/></way></osm>",
    "<osm>" + nodes + "<way id='9'><nd ref='1'/><tag k='type' v='stop_line'/></way></osm>",
    "<osm>" + nodes + line + line + "</osm>",
    "<osm>" + nodes + "<way id='0'><nd ref='1'/><nd ref='2'/><tag k='type' v='stop_line'/></way></osm>",
    "<osm>" + nodes + "<way id='9tail'><nd ref='1'/><nd ref='2'/><tag k='type' v='stop_line'/></way></osm>",
    "<osm><node id='1'><tag k='local_x' v='10tail'/><tag k='local_y' v='1'/></node>" + nodes.substr(nodes.find("<node id='2'")) + line + "</osm>",
    "<osm><node id='1'><tag k='local_x' v='nan'/><tag k='local_y' v='1'/></node>" + nodes.substr(nodes.find("<node id='2'")) + line + "</osm>",
    "<osm><node id='1'><tag k='local_x' v='10'/></node>" + nodes.substr(nodes.find("<node id='2'")) + line + "</osm>",
    "<osm>" + nodes + nodes + line + "</osm>"));

class InvalidCoefficients : public testing::TestWithParam<std::string> {};
TEST_P(InvalidCoefficients, RejectsInvalidCoefficients) {
  const MapAsset asset("<osm>" + nodes + line + "</osm>");
  EXPECT_THROW(velocity_planner::loadStopLineStateParameters(asset.parameters(GetParam())), std::invalid_argument);
}
INSTANTIATE_TEST_SUITE_P(StopLineStateLoader, InvalidCoefficients, testing::Values(
    "x_m: -1\ny_m: 100\nz_mps2: 3\nn_frames: 3\n",
    "x_m: .nan\ny_m: 100\nz_mps2: 3\nn_frames: 3\n",
    "x_m: 5\ny_m: 0\nz_mps2: 3\nn_frames: 3\n",
    "x_m: 5\ny_m: .inf\nz_mps2: 3\nn_frames: 3\n",
    "x_m: 5\ny_m: 100\nz_mps2: 0\nn_frames: 3\n",
    "x_m: 5\ny_m: 100\nz_mps2: .inf\nn_frames: 3\n",
    "x_m: 5\ny_m: 100\nz_mps2: 3\nn_frames: 0\n",
    "x_m: 5\ny_m: 100\nz_mps2: 3\nn_frames: 1.5\n",
    "x_m: 5\ny_m: 100\nz_mps2: 3\nn_frames: 2147483648\n"));

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
