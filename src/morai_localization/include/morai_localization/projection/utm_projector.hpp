#ifndef MORAI_LOCALIZATION_UTM_PROJECTOR_HPP_
#define MORAI_LOCALIZATION_UTM_PROJECTOR_HPP_

#include <string>

namespace morai_localization {

struct UtmCoordinate {
  double easting;
  double northing;
};

struct LocalCoordinate {
  double x;
  double y;
};

struct ProjectionConfig {
  bool config_verified = false;
  int utm_zone = 0;
  bool utm_northern = true;
  double east_offset = 0.0;
  double north_offset = 0.0;
  std::string frame_id = "map";
};

// Throws std::invalid_argument when the scenario projection must not be used.
void validateProjectionConfig(const ProjectionConfig& config);

// Converts WGS84 degrees to UTM metres. The caller supplies the map's fixed
// UTM zone rather than deriving a zone from longitude.
UtmCoordinate wgs84ToUtm(double latitude, double longitude, int zone,
                         bool northern = true);

LocalCoordinate projectToLocal(double latitude, double longitude,
                               const ProjectionConfig& config);

}  // namespace morai_localization

#endif  // MORAI_LOCALIZATION_UTM_PROJECTOR_HPP_
