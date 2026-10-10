// Core 파라미터 로더가 쓰는 ParamReader의 형식·경로 계약을 검증한다.
#include <node_pipeline/extension.hpp>
#include <node_pipeline/detail/ros.hpp>
#include <gtest/gtest.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <unistd.h>

namespace np = node_pipeline;
namespace npd = node_pipeline::detail;

namespace {
npd::ConfigNode configFromString(const std::string& yaml) {
  const auto directory = std::getenv("TMPDIR");
  const std::string pattern = std::string(directory ? directory : "/tmp") + "/np_registration_XXXXXX";
  std::vector<char> path(pattern.begin(), pattern.end());
  path.push_back('\0');
  const int descriptor = mkstemp(path.data());
  if (descriptor < 0) throw std::runtime_error("cannot create temporary registration");
  close(descriptor);
  {
    std::ofstream stream(path.data());
    stream << yaml;
  }
  const auto config = npd::ConfigNode::fromFile(path.data());
  std::remove(path.data());
  return config;
}
}  // namespace

TEST(ParamReader, ReadsTypedValuesAndNestedKeys) {
  const auto reader = np::ParamReader::fromString(
      "gain: 1.5\ncount: 3\nname: mpc\nenabled: true\nlist: [1, 2.5]\n"
      "mounts:\n  gps_link: {translation: [0, 0, 1.2]}\n  imu_link: {translation: [1.5, 0, 0.5]}\n");
  EXPECT_DOUBLE_EQ(1.5, reader.number("gain"));
  EXPECT_EQ(3, reader.integer("count"));
  EXPECT_EQ("mpc", reader.text("name"));
  EXPECT_TRUE(reader.flag("enabled"));
  EXPECT_EQ((std::vector<double>{1, 2.5}), reader.numbers("list"));
  EXPECT_EQ((std::vector<std::string>{"gps_link", "imu_link"}), reader.keys("mounts"));
  EXPECT_EQ((std::vector<double>{1.5, 0, 0.5}), reader.numbers("mounts/imu_link/translation"));
  EXPECT_TRUE(reader.has("mounts/gps_link"));
  EXPECT_FALSE(reader.has("mounts/velodyne"));
}

TEST(ParamReader, RejectsMissingKeysAndWrongTypes) {
  const auto reader = np::ParamReader::fromString("gain: 1.5\nname: mpc\n");
  EXPECT_THROW(reader.number("missing"), std::invalid_argument);
  EXPECT_THROW(reader.integer("gain"), std::invalid_argument);
  EXPECT_THROW(reader.number("name"), std::invalid_argument);
  EXPECT_THROW(reader.numbers("gain"), std::invalid_argument);
  EXPECT_THROW(np::ParamReader().number("gain"), std::invalid_argument);
}

TEST(ParamReader, ReadsAssetsRelativeToBaseDirectory) {
  const std::string directory = "/tmp";
  const std::string name = "np_param_reader_asset.csv";
  {
    std::ofstream stream(directory + "/" + name);
    stream << "x,y\n1,2\n";
  }
  const auto reader = np::ParamReader::fromString("table: " + name + "\nmissing: nothing.csv\n", directory);
  EXPECT_EQ("x,y\n1,2\n", reader.asset("table"));
  EXPECT_THROW(reader.asset("missing"), std::runtime_error);
  std::remove((directory + "/" + name).c_str());
}

TEST(Registration, AcceptsPayloadInputsWithOptionalStampAndPayloadIssues) {
  const auto stamp = std::numeric_limits<std::size_t>::max();
  const std::map<std::string, std::size_t> inputs{{"gps", 0}, {"imu", 1}};
  npd::StageParams stage;
  stage.name = "estimate";
  const std::vector<std::pair<std::string, std::vector<std::size_t>>> cases{
      {"[gps, imu]", {0, 1}},
      {"[stamp, gps, imu]", {stamp, 0, 1}},
      {"[gps, stamp, imu]", {0, stamp, 1}},
      {"[gps, imu, stamp]", {0, 1, stamp}}};
  for (const auto& row : cases) {
    SCOPED_TRACE(row.first);
    const auto registration = configFromString(
        "stage: estimate\ninputs: " + row.first + "\nallowed_issues: {gps: [Stale]}\n");
    std::vector<std::vector<np::Kind>> allowed(2);
    ASSERT_NO_THROW(npd::loadRegistration(registration, stage, inputs, row.second, "", "", allowed));
    EXPECT_EQ((std::vector<np::Kind>{np::Kind::Stale}), allowed[0]);
    EXPECT_TRUE(allowed[1].empty());
  }
}

TEST(Registration, AcceptsStampWithoutPayloadInputs) {
  npd::StageParams stage;
  stage.name = "estimate";
  const auto registration = configFromString("stage: estimate\ninputs: [stamp]\n");
  std::vector<std::vector<np::Kind>> allowed;
  EXPECT_NO_THROW(npd::loadRegistration(registration, stage, {}, {std::numeric_limits<std::size_t>::max()}, "", "", allowed));
}

TEST(Registration, RejectsStampArgumentOrderOrCountMismatch) {
  npd::StageParams stage;
  stage.name = "estimate";
  const std::map<std::string, std::size_t> inputs{{"gps", 0}, {"imu", 1}};
  const auto registration = configFromString("stage: estimate\ninputs: [gps, stamp, imu]\n");
  std::vector<std::vector<np::Kind>> allowed(2);
  EXPECT_THROW(npd::loadRegistration(registration, stage, inputs, {0, 1, std::numeric_limits<std::size_t>::max()}, "", "", allowed),
               std::invalid_argument);
  EXPECT_THROW(npd::loadRegistration(registration, stage, inputs, {0, 1}, "", "", allowed), std::invalid_argument);
}

TEST(Registration, RejectsAllowedIssuesForStampAndUndeclaredPayloadOrMissing) {
  npd::StageParams stage;
  stage.name = "estimate";
  const std::map<std::string, std::size_t> inputs{{"gps", 0}, {"imu", 1}};
  for (const auto& issues : {"{stamp: [Stale]}", "{imu: [Stale]}", "{gps: [Missing]}"}) {
    SCOPED_TRACE(issues);
    const auto registration = configFromString(
        "stage: estimate\ninputs: [gps, stamp]\nallowed_issues: " + std::string(issues) + "\n");
    std::vector<std::vector<np::Kind>> allowed(2);
    EXPECT_THROW(npd::loadRegistration(registration, stage, inputs, {0, std::numeric_limits<std::size_t>::max()}, "", "", allowed),
                 std::invalid_argument);
  }
}
