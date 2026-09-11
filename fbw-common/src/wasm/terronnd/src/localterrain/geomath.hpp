#pragma once

// Ported 1:1 from flybywiresim/simbridge's
// apps/server/src/terrain/processing/generic/helper.ts and gpu/helper.ts,
// so a locally-rendered frame uses the exact same geodesic math as a
// SimBridge-rendered one.

#include <cmath>

namespace localterrain {

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kEarthRadiusMetres = 6371010.0;

inline double deg2rad(double degree) {
  return degree * (kPi / 180.0);
}

inline double rad2deg(double radian) {
  return radian * (180.0 / kPi);
}

inline double normalizeHeading(double angle) {
  return angle - std::floor(angle / 360.0) * 360.0;
}

/**
 * @brief Great-circle distance between two WGS84 points, in nautical miles.
 * Matches simbridge's distanceWgs84() (haversine variant with a fixed mean-earth-diameter constant).
 */
inline double distanceWgs84Nm(double latitude0, double longitude0, double latitude1, double longitude1) {
  const double deltaLatitude = deg2rad(latitude1 - latitude0);
  const double deltaLongitude = deg2rad(longitude1 - longitude0);
  const double latitude0radian = deg2rad(latitude0);
  const double latitude1radian = deg2rad(latitude1);

  const double a = 0.5 - std::cos(deltaLatitude) * 0.5 +
                    std::cos(latitude0radian) * std::cos(latitude1radian) * (1.0 - std::cos(deltaLongitude)) * 0.5;

  const double distanceMetres = 12742020.0 * std::asin(std::sqrt(a));
  return distanceMetres * 0.000539957;
}

/**
 * @brief Projects a WGS84 point by a bearing (degrees) and distance (metres), matching simbridge's projectWgs84().
 * @return {latitude, longitude} in degrees
 */
struct LatLon {
  double latitude;
  double longitude;
};

inline LatLon projectWgs84(double latitude, double longitude, double bearing, double distance) {
  const double latRad = deg2rad(latitude);
  const double longRad = deg2rad(longitude);
  const double bearingRad = deg2rad(bearing);
  const double ratio = distance / kEarthRadiusMetres;

  double latDest =
      std::asin(std::sin(latRad) * std::cos(ratio) + std::cos(latRad) * std::sin(ratio) * std::cos(bearingRad));
  double longDest = longRad + std::atan2(std::sin(bearingRad) * std::sin(ratio) * std::cos(latRad),
                                          std::cos(ratio) - std::sin(latRad) * std::sin(latDest));

  double latDestDeg = rad2deg(latDest);
  if (latDestDeg < -90.0) latDestDeg = -180.0 - latDestDeg;
  if (latDestDeg > 90.0) latDestDeg = 180.0 - latDestDeg;

  double longDestDeg = rad2deg(longDest);
  if (longDestDeg < -180.0) longDestDeg = 360.0 + longDestDeg;
  if (longDestDeg > 180.0) longDestDeg -= 360.0;

  return {latDestDeg, longDestDeg};
}

}  // namespace localterrain
