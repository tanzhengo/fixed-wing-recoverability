#ifndef RUNTIME_SWEPT_GEOMETRY_H
#define RUNTIME_SWEPT_GEOMETRY_H

#include <algorithm>
#include <cmath>
#include "urban_box_geometry.h"

// Intersection tests for the piecewise-linear position path produced by the
// runtime integrator. These do not bound model or tracking error.
namespace runtime_swept_geometry {
struct Point { double x, y, z; };

inline bool clip_axis(double start, double finish, double low, double high,
                      double& enter, double& leave) {
    if (std::max(start, finish) < low || std::min(start, finish) > high) return false;
    const double direction = finish - start;
    if (direction == 0.0) return start >= low && start <= high;
    double a = (low - start) / direction, b = (high - start) / direction;
    if (a > b) std::swap(a, b);
    enter = std::max(enter, a);
    leave = std::min(leave, b);
    return enter <= leave;
}

inline bool box(const Point& a, const Point& b, const urban_geometry::Box& obstacle,
                double margin) {
    double enter = 0.0, leave = 1.0;
    return clip_axis(a.x, b.x, obstacle.x - obstacle.size_x / 2 - margin,
                     obstacle.x + obstacle.size_x / 2 + margin, enter, leave) &&
        clip_axis(a.y, b.y, obstacle.y - obstacle.size_y / 2 - margin,
                  obstacle.y + obstacle.size_y / 2 + margin, enter, leave) &&
        clip_axis(a.z, b.z, obstacle.z - obstacle.size_z / 2 - margin,
                  obstacle.z + obstacle.size_z / 2 + margin, enter, leave);
}

inline bool sphere(const Point& a, const Point& b, double radius) {
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const double length2 = dx * dx + dy * dy + dz * dz;
    const double u = length2 == 0.0 ? 0.0 : std::max(0.0, std::min(1.0,
        -(a.x * dx + a.y * dy + a.z * dz) / length2));
    const double x = a.x + u * dx, y = a.y + u * dy, z = a.z + u * dz;
    return x * x + y * y + z * z <= radius * radius;
}

// Local cylinder axis is Z. Clip by the two flat caps before minimizing the
// radial distance, so an intersection at a different height is not accepted.
inline bool cylinder(const Point& a, const Point& b, double radius, double half_height) {
    double enter = 0.0, leave = 1.0;
    if (!clip_axis(a.z, b.z, -half_height, half_height, enter, leave)) return false;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double u = length2 == 0.0 ? enter : std::max(enter, std::min(leave,
        -(a.x * dx + a.y * dy) / length2));
    const double x = a.x + u * dx, y = a.y + u * dy;
    return x * x + y * y <= radius * radius;
}
}  // namespace runtime_swept_geometry
#endif
