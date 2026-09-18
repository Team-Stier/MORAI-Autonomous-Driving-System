#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "morai_localization/projection/utm_projector.hpp"

namespace morai_localization {
namespace {

TEST(UtmProjector, MatchesProjReferenceForUtm52North) {
  // EPSG:32652 result produced by PROJ/cs2cs.
  const UtmCoordinate result = wgs84ToUtm(37.583465, 127.02741, 52, true);
  EXPECT_NEAR(result.easting, 325827.986968, 0.001);
  EXPECT_NEAR(result.northing, 4161430.305875, 0.001);
}

TEST(UtmProjector, AppliesCompetitionMapOrigin) {
  ProjectionConfig config;
  config.config_verified = true;
  config.utm_zone = 52;
  config.utm_northern = true;
  config.east_offset = 302595.0;
  config.north_offset = 4124145.0;
  config.frame_id = "map";

  const LocalCoordinate result = projectToLocal(37.583465, 127.02741, config);
  EXPECT_NEAR(result.x, 23232.986968, 0.001);
  EXPECT_NEAR(result.y, 37285.305875, 0.001);
}

TEST(UtmProjector, RefusesUnverifiedScenarioConfig) {
  ProjectionConfig config;
  config.utm_zone = 52;
  config.frame_id = "map";
  EXPECT_THROW(validateProjectionConfig(config), std::invalid_argument);
  EXPECT_THROW(projectToLocal(37.0, 127.0, config), std::invalid_argument);
}

TEST(UtmProjector, RejectsInvalidProjectionInputs) {
  EXPECT_THROW(wgs84ToUtm(37.0, 127.0, 0, true), std::invalid_argument);
  EXPECT_THROW(wgs84ToUtm(85.0, 127.0, 52, true), std::invalid_argument);
  EXPECT_THROW(wgs84ToUtm(37.0, 181.0, 52, true), std::invalid_argument);
  EXPECT_THROW(wgs84ToUtm(std::numeric_limits<double>::quiet_NaN(), 127.0,
                          52, true),
               std::invalid_argument);
}

TEST(UtmProjector, UsesSouthernHemisphereFalseNorthing) {
  const UtmCoordinate result = wgs84ToUtm(-33.0, 151.0, 56, false);
  EXPECT_GT(result.northing, 0.0);
  EXPECT_LT(result.northing, 10000000.0);
}

}  // namespace
}  // namespace morai_localization

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
