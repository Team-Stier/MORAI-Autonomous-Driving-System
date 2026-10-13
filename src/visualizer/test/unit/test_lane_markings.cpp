#include <visualizer/cores/lane_markings.hpp>
#include <visualizer/parameters/lane_markings.hpp>
#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <unistd.h>

namespace {
geometry_msgs::Point point(double x, double y = 0, double z = 0) {
  geometry_msgs::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}
}  // namespace

TEST(LaneMarkings, BatchesColorsAndPreservesMapElevationWithoutHeaders) {
  visualizer::LaneMarkingsParameters parameters;
  parameters.flatten_z = false;
  visualizer::LaneMarkingsParameters::Line lane;
  lane.points = {point(0, 0, 28), point(8, 0, 29)};
  parameters.lines.push_back(lane);
  lane.semantic = "center_line";
  parameters.lines.push_back(lane);
  lane.semantic.clear();
  lane.lane_type = "501";
  parameters.lines.push_back(lane);
  const auto marker = visualizer::LaneMarkings(parameters).process();
  ASSERT_EQ(6u, marker.points.size());
  ASSERT_EQ(marker.points.size(), marker.colors.size());
  EXPECT_EQ(visualization_msgs::Marker::LINE_LIST, marker.type);
  EXPECT_TRUE(marker.header.frame_id.empty());
  EXPECT_TRUE(marker.header.stamp.isZero());
  EXPECT_DOUBLE_EQ(28, marker.points[0].z);
  EXPECT_DOUBLE_EQ(29, marker.points[1].z);
  EXPECT_DOUBLE_EQ(0.12, marker.scale.x);
  for (std::size_t i = 0; i < marker.colors.size(); ++i) {
    EXPECT_FLOAT_EQ(1, marker.colors[i].r);
    EXPECT_FLOAT_EQ(1, marker.colors[i].g);
    EXPECT_FLOAT_EQ(1, marker.colors[i].a);
    EXPECT_FLOAT_EQ(i < 2 ? 1 : 0, marker.colors[i].b);
  }
  parameters.flatten_z = true;
  for (const auto& p : visualizer::LaneMarkings(parameters).process().points) EXPECT_DOUBLE_EQ(0, p.z);
}

TEST(LaneMarkings, DashPhaseContinuesAcrossVertices) {
  visualizer::LaneMarkingsParameters parameters;
  parameters.dash_length_m = 3;
  parameters.dash_gap_m = 2;
  parameters.flatten_z = false;
  visualizer::LaneMarkingsParameters::Line line;
  line.subtype = "dashed";
  line.points = {point(0, 0, 4), point(2, 0, 6), point(8, 0, 12)};
  parameters.lines.push_back(line);
  const auto marker = visualizer::LaneMarkings(parameters).process();
  const auto& points = marker.points;
  ASSERT_EQ(6u, points.size());
  const double expected[] = {0, 2, 2, 3, 5, 8};
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(expected[i], points[i].x);
    EXPECT_DOUBLE_EQ(expected[i] + 4, points[i].z);
  }
  parameters.lines[0].points = {line.points.front(), line.points.back()};
  const auto without_vertex = visualizer::LaneMarkings(parameters).process();
  ASSERT_EQ(4u, without_vertex.points.size());
  EXPECT_DOUBLE_EQ(3, without_vertex.points[1].x);
  EXPECT_DOUBLE_EQ(5, without_vertex.points[2].x);
}

TEST(LaneMarkings, MixedSubtypesHaveSolidAndDashedStrandsOnTheirOsmSides) {
  for (const auto& subtype : {"solid_dashed", "dashed_solid"}) {
    visualizer::LaneMarkingsParameters parameters;
    parameters.dash_length_m = 3;
    parameters.dash_gap_m = 2;
    visualizer::LaneMarkingsParameters::Line line;
    line.subtype = subtype;
    line.points = {point(0, 0, 1), point(8, 0, 9)};
    parameters.lines.push_back(line);
    const auto marker = visualizer::LaneMarkings(parameters).process();
    const auto& points = marker.points;
    ASSERT_EQ(6u, points.size());
    const bool left_dashed = line.subtype == "dashed_solid";
    const double offset = (line.width_m + parameters.double_line_gap_m) / 2;
    EXPECT_DOUBLE_EQ(offset, points.front().y);
    EXPECT_DOUBLE_EQ(-offset, points.back().y);
    EXPECT_DOUBLE_EQ(left_dashed ? 3 : 8, points[1].x);
    EXPECT_DOUBLE_EQ(left_dashed ? 0 : 5, points[points.size() - 2].x);
  }
}

TEST(LaneMarkings, LoadsCanonicalPhysicalAndVirtualWaysAndUniqueUnreferencedOriginals) {
  char filename[] = "/tmp/visualizer-lane-markings-XXXXXX";
  const int descriptor = mkstemp(filename);
  ASSERT_GE(descriptor, 0);
  close(descriptor);
  {
    std::ofstream asset(filename);
    asset << R"(<osm version="0.6">
      <node id="1"><tag k="local_x" v="0"/><tag k="local_y" v="0"/><tag k="local_z" v="28"/>
        <tag k="mgeo:asset_edge_adjustment" v="installed_road_mesh"/>
        <tag k="mgeo:asset_edge_adjustment" v="installed_road_mesh"/></node>
      <node id="2"><tag k="local_x" v="8"/><tag k="local_y" v="0"/><tag k="ele" v="29"/></node>
      <way id="10"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="dashed"/>
        <tag k="mgeo:boundary_id" v="a"/><tag k="mgeo:geometry" v="original"/></way>
      <way id="11"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="dashed"/>
        <tag k="mgeo:boundary_id" v="a"/></way>
      <way id="12"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/>
        <tag k="mgeo:boundary_id" v="b"/><tag k="mgeo:geometry" v="original"/>
        <tag k="mgeo:semantic" v="center_line"/></way>
      <way id="13"><nd ref="1"/><nd ref="2"/><tag k="type" v="virtual"/>
        <tag k="mgeo:boundary_id" v="b"/><tag k="mgeo:semantic" v="center_line"/>
        <tag k="color" v="yellow"/></way>
      <way id="14"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/>
        <tag k="mgeo:boundary_id" v="c"/></way>
      <way id="15"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/>
        <tag k="mgeo:boundary_id" v="d"/><tag k="mgeo:geometry" v="original"/></way>
      <way id="16"><nd ref="1"/><nd ref="2"/><tag k="type" v="line_thin"/><tag k="subtype" v="solid"/>
        <tag k="mgeo:boundary_id" v="d"/><tag k="mgeo:geometry" v="original"/></way>
      <way id="17"><nd ref="1"/><nd ref="2"/><tag k="type" v="virtual"/></way>
      <relation id="20"><member type="way" ref="11" role="left"/><member type="way" ref="13" role="right"/>
        <tag k="type" v="lanelet"/></relation>
      <relation id="21"><member type="way" ref="13" role="left"/><member type="way" ref="11" role="right"/>
        <tag k="type" v="lanelet"/></relation>
      </osm>)";
  }
  const auto params = node_pipeline::ParamReader::fromString(
      std::string("map_file: ") + filename
      + "\ndash_length_m: 3\ndash_gap_m: 2\ndouble_line_gap_m: 0.15\nline_width_m: 0.12\nflatten_z: false\n");
  const auto parameters = visualizer::loadLaneMarkingsParameters(params);
  ASSERT_EQ(3u, parameters.lines.size());
  EXPECT_EQ("dashed", parameters.lines[0].subtype);
  EXPECT_DOUBLE_EQ(0, parameters.lines[0].points.front().x);
  EXPECT_DOUBLE_EQ(8, parameters.lines[0].points.back().x);
  EXPECT_DOUBLE_EQ(29, parameters.lines[0].points.back().z);
  EXPECT_EQ("center_line", parameters.lines[1].semantic);
  EXPECT_FALSE(parameters.lines[0].virtual_boundary);
  EXPECT_FALSE(parameters.lines[1].virtual_boundary);
  EXPECT_TRUE(parameters.lines[2].virtual_boundary);
  EXPECT_TRUE(parameters.lines[2].subtype.empty());
  const auto marker = visualizer::LaneMarkings(parameters).process();
  ASSERT_EQ(10u, marker.points.size());
  EXPECT_EQ(marker.points.size(), marker.colors.size());
  EXPECT_FLOAT_EQ(0, marker.colors[4].b);
  for (std::size_t i = 6; i < marker.colors.size(); ++i) {
    EXPECT_FLOAT_EQ(0, marker.colors[i].r);
    EXPECT_FLOAT_EQ(0.7, marker.colors[i].g);
    EXPECT_FLOAT_EQ(1, marker.colors[i].b);
    EXPECT_FLOAT_EQ(0.55, marker.colors[i].a);
  }
  EXPECT_DOUBLE_EQ(0, marker.points[6].x);
  EXPECT_DOUBLE_EQ(3, marker.points[7].x);
  EXPECT_DOUBLE_EQ(5, marker.points[8].x);
  EXPECT_DOUBLE_EQ(8, marker.points[9].x);
  EXPECT_DOUBLE_EQ(28, marker.points[6].z);
  EXPECT_DOUBLE_EQ(28.375, marker.points[7].z);
  EXPECT_DOUBLE_EQ(29, marker.points[9].z);
  std::ofstream(filename) << "<osm><node>";
  EXPECT_THROW(visualizer::loadLaneMarkingsParameters(params), std::exception);
  std::remove(filename);
}

TEST(LaneMarkings, RejectsDerivedDistancesAndSpacingThatOverflow) {
  char filename[] = "/tmp/visualizer-lane-marking-arithmetic-XXXXXX";
  const int descriptor = mkstemp(filename);
  ASSERT_GE(descriptor, 0);
  close(descriptor);
  const auto write_asset = [&](const char* x, const char* width) {
    std::ofstream asset(filename);
    asset << "<osm><node id='1'><tag k='local_x' v='0'/><tag k='local_y' v='0'/>"
             "<tag k='local_z' v='0'/></node><node id='2'><tag k='local_x' v='" << x
          << "'/><tag k='local_y' v='0'/><tag k='local_z' v='0'/></node>"
             "<way id='10'><nd ref='1'/><nd ref='2'/><nd ref='1'/>"
             "<tag k='type' v='line_thin'/><tag k='subtype' v='solid_dashed'/>"
             "<tag k='mgeo:lane_width' v='" << width
          << "'/></way><relation id='20'><member type='way' ref='10' role='left'/>"
             "<member type='way' ref='10' role='right'/><tag k='type' v='lanelet'/></relation></osm>";
  };
  const auto parameters = [&](const char* dash, const char* gap, const char* double_gap) {
    return node_pipeline::ParamReader::fromString(
        std::string("map_file: ") + filename + "\ndash_length_m: " + dash
        + "\ndash_gap_m: " + gap + "\ndouble_line_gap_m: " + double_gap
        + "\nline_width_m: 0.12\nflatten_z: false\n");
  };
  write_asset("8", "0.15");
  EXPECT_THROW(visualizer::loadLaneMarkingsParameters(parameters("1e308", "1e308", "0.15")),
               std::invalid_argument);
  EXPECT_THROW(visualizer::loadLaneMarkingsParameters(parameters("1e-320", "1e-320", "0.15")),
               std::invalid_argument);
  write_asset("1e308", "0.15");
  EXPECT_THROW(visualizer::loadLaneMarkingsParameters(parameters("3", "3", "0.15")),
               std::invalid_argument);
  write_asset("8", "1e308");
  EXPECT_THROW(visualizer::loadLaneMarkingsParameters(parameters("3", "3", "1e308")),
               std::invalid_argument);
  std::remove(filename);
}
