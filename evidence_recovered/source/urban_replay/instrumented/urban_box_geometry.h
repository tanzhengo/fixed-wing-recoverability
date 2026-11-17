#ifndef URBAN_BOX_GEOMETRY_H
#define URBAN_BOX_GEOMETRY_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace urban_geometry {

// Axis-aligned building component; dimensions are full physical side lengths.
struct Box {
    double x, y, z, size_x, size_y, size_z;
};

constexpr double face_margin = 1.0;

inline bool valid(const Box& box) {
    return std::isfinite(box.x) && std::isfinite(box.y) && std::isfinite(box.z) &&
        std::isfinite(box.size_x) && std::isfinite(box.size_y) && std::isfinite(box.size_z) &&
        box.size_x > 0 && box.size_y > 0 && box.size_z > 0;
}

// Exact Euclidean signed distance to one axis-aligned box expanded by margin
// at each face. Setting margin=0 gives the physical building clearance.
inline double signed_distance(const Box& box, double x, double y, double z,
                              double margin = face_margin) {
    const double qx = std::abs(x - box.x) - box.size_x / 2 - margin;
    const double qy = std::abs(y - box.y) - box.size_y / 2 - margin;
    const double qz = std::abs(z - box.z) - box.size_z / 2 - margin;
    const double dx = std::max(qx, 0.0), dy = std::max(qy, 0.0), dz = std::max(qz, 0.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz) +
        std::min(std::max(qx, std::max(qy, qz)), 0.0);
}

}  // namespace urban_geometry
#endif
