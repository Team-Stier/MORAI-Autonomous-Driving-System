#pragma once
#include <array>
#include <cmath>
namespace localization {
// 기존 WGS84→UTM 계산을 사용한다. 입력 검사는 pipeline Validator의 책임이다.
inline std::array<double, 2> projectGps(double latitude, double longitude) {
  constexpr int zone = 52;
  constexpr bool northern = true;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kSemiMajor = 6378137.0;
  constexpr double kFlattening = 1.0 / 298.257223563;
  constexpr double kScale = 0.9996;
  const auto degreesToRadians = [](double degrees) { return degrees * kPi / 180.0; };
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

  return {{easting - 302595.0, northing - 4124145.0}};
}

}
