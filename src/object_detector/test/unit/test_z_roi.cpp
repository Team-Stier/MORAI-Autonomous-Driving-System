#include <object_detector/cores/z_roi.hpp>
#include <object_detector/parameters/z_roi.hpp>
#include <ros/serialization.h>
#include <gtest/gtest.h>
#include <cstring>
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace pkg = object_detector;
namespace {
sensor_msgs::PointCloud2 cloud(std::initializer_list<float> heights, std::uint32_t width,
                              std::uint32_t row_padding = 0) {
  sensor_msgs::PointCloud2 value;
  value.header.seq = 7;
  value.header.stamp.fromNSec(1234567890);
  value.header.frame_id = "base_link";
  value.height = width == 0 ? 1 : heights.size() / width;
  value.width = width;
  value.fields.resize(5);
  const char* names[] = {"intensity", "x", "y", "ring", "z"};
  for (std::size_t i = 0; i < value.fields.size(); ++i) {
    value.fields[i].name = names[i];
    value.fields[i].offset = 4 * i;
    value.fields[i].datatype = i == 3 ? sensor_msgs::PointField::UINT16 : sensor_msgs::PointField::FLOAT32;
    value.fields[i].count = 1;
  }
  value.point_step = 32;
  value.row_step = width * value.point_step + row_padding;
  value.data.assign(value.height * value.row_step, 0xa5);
  value.is_dense = false;
  std::size_t index = 0;
  for (const auto z : heights) {
    auto* record = value.data.data() + (index / width) * value.row_step +
                                     (index % width) * value.point_step;
    const float intensity = 10.5f + index, x = 3.0f + index, y = -4.0f - index;
    const std::uint16_t ring = 8 + index;
    std::memcpy(record, &intensity, sizeof(intensity));
    std::memcpy(record + 4, &x, sizeof(x));
    std::memcpy(record + 8, &y, sizeof(y));
    std::memcpy(record + 12, &ring, sizeof(ring));
    std::memcpy(record + 16, &z, sizeof(z));
    ++index;
  }
  return value;
}
}  // namespace

TEST(ZRoi, IncludesBothBoundsAndPreservesRecordsAndCompetition) {
  const auto input = cloud({-1.1f, -1.0f, 0.0f, 2.0f, 2.1f}, 5);
  interfaces::CompetitionVehicleStatus competition;
  competition.header.seq = 19;
  competition.header.stamp.fromNSec(9876543210);
  competition.header.frame_id = "base_link";
  competition.source_sec = 9;
  competition.source_nsec = 876543210;
  competition.gear = 4;
  competition.map_data_id = 23;
  competition.velocity_x_mps = 5.5f;
  competition.yaw_rad = 0.75f;
  competition.link_id = "test_link";
  pkg::ZRoi core({-1.0, 2.0});

  const auto result = core.process(input, competition);
  const auto& output = std::get<0>(result);
  EXPECT_EQ(input.header, output.header);
  EXPECT_EQ(input.fields, output.fields);
  EXPECT_EQ(input.point_step, output.point_step);
  EXPECT_EQ(input.is_bigendian, output.is_bigendian);
  EXPECT_EQ(input.is_dense, output.is_dense);
  EXPECT_EQ(1u, output.height);
  EXPECT_EQ(3u, output.width);
  EXPECT_EQ(96u, output.row_step);
  EXPECT_EQ((std::vector<std::uint8_t>(input.data.begin() + 32, input.data.begin() + 128)), output.data);
  const auto expected_status = ros::serialization::serializeMessage(competition);
  const auto actual_status = ros::serialization::serializeMessage(std::get<1>(result));
  ASSERT_EQ(expected_status.num_bytes, actual_status.num_bytes);
  EXPECT_EQ(0, std::memcmp(expected_status.buf.get(), actual_status.buf.get(), expected_status.num_bytes));
  EXPECT_EQ(160u, input.data.size());
}

TEST(ZRoi, ReadsOrganizedRowsWithPaddingAndReturnsContiguousPoints) {
  const auto input = cloud({1.0f, 3.0f, -2.0f, 2.0f}, 2, 8);
  pkg::ZRoi core({-1.0, 2.0});
  const auto output = std::get<0>(core.process(input, {}));
  EXPECT_EQ(1u, output.height);
  EXPECT_EQ(2u, output.width);
  EXPECT_EQ(64u, output.row_step);
  std::vector<std::uint8_t> expected(input.data.begin(), input.data.begin() + 32);
  expected.insert(expected.end(), input.data.begin() + 104, input.data.begin() + 136);
  EXPECT_EQ(expected, output.data);
}

TEST(ZRoi, ReturnsEmptyCloudWhenEveryPointIsOutsideBounds) {
  const auto input = cloud({-2.0f, 3.0f}, 2);
  pkg::ZRoi core({-1.0, 2.0});
  const auto output = std::get<0>(core.process(input, {}));
  EXPECT_EQ(1u, output.height);
  EXPECT_EQ(0u, output.width);
  EXPECT_EQ(0u, output.row_step);
  EXPECT_TRUE(output.data.empty());
  EXPECT_EQ(input.fields, output.fields);
}

TEST(ZRoi, AcceptsValidEmptyCloud) {
  const auto input = cloud({}, 0);
  pkg::ZRoi core({-1.0, 2.0});
  const auto output = std::get<0>(core.process(input, {}));
  EXPECT_EQ(1u, output.height);
  EXPECT_EQ(0u, output.width);
  EXPECT_EQ(0u, output.row_step);
  EXPECT_TRUE(output.data.empty());
  EXPECT_EQ(input.header, output.header);
  EXPECT_EQ(input.fields, output.fields);
}

TEST(ZRoi, RemovesVehicleReturnsAndKeepsNearbyAndAboveVehiclePoints) {
  auto input = cloud({1.106f, 1.112f, 1.177f, 1.1f, 1.5f}, 5);
  const float positions[][2] = {{0.04293f, -0.38261f}, {0.04572f, 0.37490f},
                                {0.56065f, -0.01333f}, {4.0f, 0.0f}, {0.5f, 0.0f}};
  for (std::size_t i = 0; i < 5; ++i) {
    auto* record = input.data.data() + i * input.point_step;
    std::memcpy(record + 4, &positions[i][0], sizeof(float));
    std::memcpy(record + 8, &positions[i][1], sizeof(float));
  }
  pkg::ZRoi core({0.2, 2.0});
  const auto output = std::get<0>(core.process(input, {}));
  EXPECT_EQ(2u, output.width);
  EXPECT_EQ((std::vector<std::uint8_t>(input.data.begin() + 96, input.data.end())), output.data);
}

TEST(ZRoi, RemovesVehicleBoundaryPoints) {
  auto input = cloud({1.0f, 1.0f, 1.0f, 1.0f, -0.3684999644756317f,
                      1.2755463123321533f}, 6);
  const float positions[][2] = {{-0.7899996042251587f, 0.0f}, {3.845008373260498f, 0.0f},
                                {0.0f, -1.0804176330566406f}, {0.0f, 1.080417275428772f},
                                {0.0f, 0.0f}, {0.0f, 0.0f}};
  for (std::size_t i = 0; i < 6; ++i) {
    auto* record = input.data.data() + i * input.point_step;
    std::memcpy(record + 4, &positions[i][0], sizeof(float));
    std::memcpy(record + 8, &positions[i][1], sizeof(float));
  }
  pkg::ZRoi core({-1.0, 2.0});
  EXPECT_EQ(0u, std::get<0>(core.process(input, {})).width);
}

TEST(ZRoi, PreservesInputCoordinatesAndOtherFieldsWithNonzeroVehicleAttitude) {
  auto input = cloud({1.1f, 1.1f}, 2, 8);
  interfaces::CompetitionVehicleStatus competition;
  competition.roll_rad = 0.2;
  competition.pitch_rad = -0.3;
  const float points[][3] = {{0.04f, -0.38f, 1.1f}, {4.0f, 0.0f, 1.1f}};
  for (std::size_t i = 0; i < 2; ++i) {
    auto* record = input.data.data() + i * input.point_step;
    std::memcpy(record + 4, &points[i][0], sizeof(float));
    std::memcpy(record + 8, &points[i][1], sizeof(float));
    std::memcpy(record + 16, &points[i][2], sizeof(float));
  }
  pkg::ZRoi core({0.2, 2.0});
  const auto output = std::get<0>(core.process(input, competition));
  ASSERT_EQ(1u, output.width);
  const std::uint32_t offsets[] = {4, 8, 16};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    float value;
    std::memcpy(&value, output.data.data() + offsets[axis], sizeof(value));
    EXPECT_NEAR(points[1][axis], value, 1e-6);
  }
  EXPECT_EQ(input.header, output.header);
  EXPECT_EQ(0, std::memcmp(input.data.data() + 32, output.data.data(), 4));
  EXPECT_EQ(0, std::memcmp(input.data.data() + 44, output.data.data() + 12, 4));
  EXPECT_EQ(0, std::memcmp(input.data.data() + 52, output.data.data() + 20, 12));
}

TEST(ZRoiParameters, LoadsRequiredBoundsAndAllowsSingleHeight) {
  const auto parameters = pkg::loadZRoiParameters(
    node_pipeline::ParamReader::fromString("z_min_m: 1.25\nz_max_m: 1.25\n"));
  EXPECT_DOUBLE_EQ(1.25, parameters.z_min_m);
  EXPECT_DOUBLE_EQ(1.25, parameters.z_max_m);
  pkg::ZRoi core(parameters);
  const auto output = std::get<0>(core.process(cloud({1.0f, 1.25f, 1.5f}, 3), {}));
  EXPECT_EQ(1u, output.width);
}

TEST(ZRoiParameters, RejectsMissingNonfiniteAndReversedBounds) {
  for (const auto* yaml : {"z_min_m: 0\n", "z_max_m: 2\n",
                           "z_min_m: .nan\nz_max_m: 2\n",
                           "z_min_m: -.inf\nz_max_m: 2\n",
                           "z_min_m: 0\nz_max_m: .inf\n",
                           "z_min_m: 2\nz_max_m: 1\n"}) {
    SCOPED_TRACE(yaml);
    EXPECT_THROW(pkg::loadZRoiParameters(node_pipeline::ParamReader::fromString(yaml)), std::invalid_argument);
  }
}
