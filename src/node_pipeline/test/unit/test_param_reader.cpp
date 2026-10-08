// Core 파라미터 로더가 쓰는 ParamReader의 형식·경로 계약을 검증한다.
#include <node_pipeline/extension.hpp>
#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>

namespace np = node_pipeline;

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
