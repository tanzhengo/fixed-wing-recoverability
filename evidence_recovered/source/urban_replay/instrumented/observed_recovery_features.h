#ifndef OBSERVED_RECOVERY_FEATURES_H
#define OBSERVED_RECOVERY_FEATURES_H
#include "sensor_occupancy_map.h"
#include <array>
#include <cmath>
#include <stdexcept>

namespace observed_recovery {
// OBS58 appends these 30 scalars to the unchanged 28 state/task features.
// Fixed angular probes query only the current three-state sensor map.
inline std::array<float, 30> encode(const sensor_occupancy::Map& map,
                                  double x, double y, double z,
                                  double heading, double speed, double horizon) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(heading) || !std::isfinite(speed) ||
        !std::isfinite(horizon) || speed <= 0 || horizon <= 0)
        throw std::invalid_argument("OBS58 requires a finite terminal state and positive speed/horizon");
    std::array<float, 30> features{};
    constexpr double rad = 3.14159265358979323846 / 180.0;
    size_t offset = 0;
    for (double yaw : {-45.0, -22.5, 0.0, 22.5, 45.0}) {
        for (double pitch : {-20.0, 0.0, 20.0}) {
            const double az = heading + yaw * rad, el = pitch * rad;
            int free = 0, unknown = 0;
            for (int distance = 1; distance <= 5; ++distance) {
                const double length = speed * horizon * distance / 5.0;
                const sensor_occupancy::Point point{
                    x + length * std::cos(el) * std::cos(az),
                    y + length * std::cos(el) * std::sin(az),
                    z + length * std::sin(el)};
                const auto cell = map.state(point);
                free += cell == sensor_occupancy::Cell::FREE;
                unknown += cell == sensor_occupancy::Cell::UNKNOWN;
            }
            features[offset++] = static_cast<float>(free) / 5.0f;
            features[offset++] = static_cast<float>(unknown) / 5.0f;
        }
    }
    return features;
}
}  // namespace observed_recovery
#endif
