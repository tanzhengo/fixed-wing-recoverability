#ifndef LIDAR_RAY_ENCODING_H
#define LIDAR_RAY_ENCODING_H

#include <cmath>
#include <limits>

namespace lidar_ray_encoding {
struct Endpoint {
  float x, y, z;
  // 1: occupied return; 0: valid clear ray up to maximum range;
  // -1: invalid measurement, which contributes no occupancy evidence.
  float hit;
};

inline Endpoint endpoint(double range, double near, double far,
                         double yaw, double pitch) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  if (std::isnan(range) || range < near || !std::isfinite(near) ||
      !std::isfinite(far) || far <= near || !std::isfinite(yaw) ||
      !std::isfinite(pitch)) return {nan, nan, nan, -1.0f};
  const bool hit = std::isfinite(range) && range < far - 1e-5;
  const double distance = hit ? range : far;
  return {static_cast<float>(distance * std::cos(pitch) * std::cos(yaw)),
          static_cast<float>(distance * std::cos(pitch) * std::sin(yaw)),
          static_cast<float>(distance * std::sin(pitch)), hit ? 1.0f : 0.0f};
}
}  // namespace lidar_ray_encoding
#endif
