#include "morai_localization/projection/utm_projector.hpp"

#include <cmath>
#include <stdexcept>

namespace morai_localization {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSemiMajor = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kScale = 0.9996;

double degreesToRadians(const double degrees) { return degrees * kPi / 180.0; }

}  // namespace

void validateProjectionConfig(const ProjectionConfig& config) {
  if (!config.config_verified) {
    throw std::invalid_argument(
        "GPS map projection config is not verified for the final scenario");
  }
  if (config.utm_zone < 1 || config.utm_zone > 60) {
    throw std::invalid_argument("UTM zone must be between 1 and 60");
  }
  if (!std::isfinite(config.east_offset) ||
      !std::isfinite(config.north_offset)) {
    throw std::invalid_argument("UTM map offsets must be finite");
  }
  if (config.frame_id.empty()) {
    throw std::invalid_argument("output frame_id must not be empty");
  }
}

UtmCoordinate wgs84ToUtm(const double latitude, const double longitude,
                         const int zone, const bool northern) {
  if (!std::isfinite(latitude) || latitude < -80.0 || latitude > 84.0) {
    throw std::invalid_argument(
        "latitude is outside the UTM coverage range");
  }
  if (!std::isfinite(longitude) || longitude < -180.0 || longitude > 180.0) {
    throw std::invalid_argument("longitude is out of range");
  }
  if (zone < 1 || zone > 60) {
    throw std::invalid_argument("UTM zone must be between 1 and 60");
  }

  const double eccentricity_sq = kFlattening * (2.0 - kFlattening);
  const double second_eccentricity_sq =
      eccentricity_sq / (1.0 - eccentricity_sq);

  const double latitude_rad = degreesToRadians(latitude);
  const double central_meridian =
      degreesToRadians((zone - 1) * 6.0 - 180.0 + 3.0);
  const double longitude_delta = degreesToRadians(longitude) - central_meridian;

  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double tan_latitude = std::tan(latitude_rad);
  const double n = kSemiMajor /
                   std::sqrt(1.0 - eccentricity_sq * sin_latitude * sin_latitude);
  const double t = tan_latitude * tan_latitude;
  const double c = second_eccentricity_sq * cos_latitude * cos_latitude;
  const double a = cos_latitude * longitude_delta;

  const double eccentricity_4 = eccentricity_sq * eccentricity_sq;
  const double eccentricity_6 = eccentricity_4 * eccentricity_sq;
  const double meridional_arc =
      kSemiMajor *
      ((1.0 - eccentricity_sq / 4.0 - 3.0 * eccentricity_4 / 64.0 -
        5.0 * eccentricity_6 / 256.0) *
           latitude_rad -
       (3.0 * eccentricity_sq / 8.0 + 3.0 * eccentricity_4 / 32.0 +
        45.0 * eccentricity_6 / 1024.0) *
           std::sin(2.0 * latitude_rad) +
       (15.0 * eccentricity_4 / 256.0 +
        45.0 * eccentricity_6 / 1024.0) *
           std::sin(4.0 * latitude_rad) -
       35.0 * eccentricity_6 / 3072.0 * std::sin(6.0 * latitude_rad));

  const double a2 = a * a;
  const double a3 = a2 * a;
  const double a4 = a2 * a2;
  const double a5 = a4 * a;
  const double a6 = a3 * a3;

  const double easting =
      500000.0 +
      kScale * n *
          (a + (1.0 - t + c) * a3 / 6.0 +
           (5.0 - 18.0 * t + t * t + 72.0 * c -
            58.0 * second_eccentricity_sq) *
               a5 / 120.0);

  double northing =
      kScale *
      (meridional_arc +
       n * tan_latitude *
           (a2 / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * a4 / 24.0 +
            (61.0 - 58.0 * t + t * t + 600.0 * c -
             330.0 * second_eccentricity_sq) *
                a6 / 720.0));
  if (!northern) {
    northing += 10000000.0;
  }

  return {easting, northing};
}

LocalCoordinate projectToLocal(const double latitude, const double longitude,
                               const ProjectionConfig& config) {
  validateProjectionConfig(config);
  const UtmCoordinate utm =
      wgs84ToUtm(latitude, longitude, config.utm_zone, config.utm_northern);
  return {utm.easting - config.east_offset,
          utm.northing - config.north_offset};
}

}  // namespace morai_localization
