#ifndef SENSOR_OCCUPANCY_MAP_H
#define SENSOR_OCCUPANCY_MAP_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

// Static-scene occupancy from measured rays only. A traversed voxel is FREE,
// a hit endpoint is OCCUPIED, and an absent voxel is UNKNOWN. No-return rays
// clear only their measured range; they never create occupied range shells.
// The representation is voxel discretized, not a continuous beam-volume model.
namespace sensor_occupancy {
struct Point {
    double x, y, z;
    Point(double px = 0, double py = 0, double pz = 0) : x(px), y(py), z(pz) {}
};
struct Key {
    int x, y, z;
    Key(int px = 0, int py = 0, int pz = 0) : x(px), y(py), z(pz) {}
    bool operator==(const Key& other) const { return x == other.x && y == other.y && z == other.z; }
};
struct KeyHash {
    std::size_t operator()(const Key& k) const {
        // Unsigned arithmetic keeps negative grid coordinates well-defined.
        std::uint64_t x = static_cast<std::uint32_t>(k.x);
        std::uint64_t y = static_cast<std::uint32_t>(k.y);
        std::uint64_t z = static_cast<std::uint32_t>(k.z);
        x = (x ^ (x >> 16)) * 0x45d9f3bULL;
        y = (y ^ (y >> 16)) * 0x119de1f3ULL;
        z = (z ^ (z >> 16)) * 0x3449f5abULL;
        return static_cast<std::size_t>(x ^ (y << 1) ^ (z << 7));
    }
};
enum class Cell : unsigned char { UNKNOWN = 0, FREE = 1, OCCUPIED = 2 };
struct Ray {
    Point endpoint;
    int hit;  // 1: measured hit, 0: valid no-return, any other value: invalid.
    Ray(const Point& p = Point(), int value = -1) : endpoint(p), hit(value) {}
};
struct Stats {
    std::size_t scans = 0, valid_rays = 0, hit_rays = 0, no_return_rays = 0;
    std::size_t invalid_rays = 0, dropped_capacity = 0, stale_scans = 0;
    std::size_t free_cells = 0, occupied_cells = 0;
    double last_stamp = -1;
};

class Map {
public:
    explicit Map(double resolution = 2.0, std::size_t max_cells = 5000000)
        : resolution_(resolution), max_cells_(max_cells) {
        if (!std::isfinite(resolution_) || resolution_ <= 0 || max_cells_ == 0)
            throw std::invalid_argument("Invalid sensor occupancy resolution or capacity");
    }
    double resolution() const { return resolution_; }
    std::size_t size() const { return cells_.size(); }
    const Stats& stats() const { return stats_; }
    std::uint64_t generation() const { return generation_; }
    void reset() {
        cells_.clear(); clear_query_cache(); stats_ = Stats(); ++generation_;
    }
    bool valid_point(const Point& p) const {
        const double limit = static_cast<double>(std::numeric_limits<int>::max() - 1024);
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
            std::abs(p.x / resolution_) < limit && std::abs(p.y / resolution_) < limit &&
            std::abs(p.z / resolution_) < limit;
    }
    Key key(const Point& p) const {
        if (!valid_point(p)) throw std::invalid_argument("Invalid sensor occupancy position");
        return Key(static_cast<int>(std::floor(p.x / resolution_)),
                   static_cast<int>(std::floor(p.y / resolution_)),
                   static_cast<int>(std::floor(p.z / resolution_)));
    }
    Point center(const Key& k) const {
        return Point((k.x + .5) * resolution_, (k.y + .5) * resolution_, (k.z + .5) * resolution_);
    }
    Cell state(const Key& k) const {
        const auto it = cells_.find(k);
        return it == cells_.end() ? Cell::UNKNOWN : it->second;
    }
    Cell state(const Point& p) const { return valid_point(p) ? state(key(p)) : Cell::UNKNOWN; }

    // All hit endpoints are installed before free-ray updates, so a hit has
    // precedence regardless of beam order. Occupancy persists in this
    // static-scene mapper. A capacity drop leaves a new voxel UNKNOWN (blocked).
    bool integrate_scan(const Point& origin, const std::vector<Ray>& rays, double stamp) {
        if (!valid_point(origin) || !std::isfinite(stamp) || stamp < 0) return false;
        if (stats_.last_stamp >= 0 && stamp < stats_.last_stamp) { ++stats_.stale_scans; return false; }
        std::vector<std::size_t> valid;
        valid.reserve(rays.size());
        for (std::size_t index=0; index<rays.size(); ++index) {
            const auto& ray = rays[index];
            if ((ray.hit != 0 && ray.hit != 1) || !valid_point(ray.endpoint)) {
                ++stats_.invalid_rays; continue;
            }
            const double length = std::abs(ray.endpoint.x-origin.x) +
                std::abs(ray.endpoint.y-origin.y) + std::abs(ray.endpoint.z-origin.z);
            if (length < 1e-9 || length / resolution_ > 100000) {
                ++stats_.invalid_rays; continue;
            }
            valid.push_back(index);
            ++stats_.valid_rays;
            if (ray.hit) {
                ++stats_.hit_rays;
                containing_keys(ray.endpoint, [&](const Key& k) { assign(k,Cell::OCCUPIED); return true; });
            } else ++stats_.no_return_rays;
        }
        // Install all endpoints first. Direct free updates then need neither a
        // per-scan free-cell hash set nor a second pass over those cells.
        for (std::size_t index : valid) {
            const auto& ray = rays[index];
            const double dx=ray.endpoint.x-origin.x, dy=ray.endpoint.y-origin.y;
            const double dz=ray.endpoint.z-origin.z;
            // Exclude a cell touched solely by the far endpoint. In particular,
            // a no-return at a grid face does not clear the next cell past range.
            const double fraction = 1.0 - 1e-9;
            const Point before(origin.x + fraction * dx, origin.y + fraction * dy,
                               origin.z + fraction * dz);
            traverse(origin, before, false, [&](const Key& k) { observe_free(k); return true; });
        }
        ++stats_.scans; stats_.last_stamp = stamp; ++generation_; clear_query_cache();
        return true;
    }

    // Exact closed-segment / voxel-AABB tests for the predicted polyline. The
    // per-face margin sweeps an axis-aligned cube, matching urban box inflation.
    // Every intersected UNKNOWN or OCCUPIED voxel rejects the segment.
    bool segment_is_free(const Point& a, const Point& b, double margin = 0.0) const {
        if (!valid_point(a) || !valid_point(b) || !std::isfinite(margin) || margin < 0) return false;
        // A non-boundary point in a visited base voxel cannot be within the
        // margin of a voxel farther than ceil(margin/resolution) indices away.
        // At exact face/edge/corner tangencies the adjacent containing base
        // voxel is also visited by CLOSED supercover, covering the equality.
        const double requested_radius = std::ceil(margin / resolution_);
        if (requested_radius > 64) return false;
        const int radius = static_cast<int>(requested_radius);
        return traverse(a, b, true, [&](const Key& k) {
            if (state(k) != Cell::FREE) return false;
            if (safe_cell(k, radius)) return true;
            for (int dx = -radius; dx <= radius; ++dx)
                for (int dy = -radius; dy <= radius; ++dy)
                    for (int dz = -radius; dz <= radius; ++dz) {
                        const Key other(k.x + dx, k.y + dy, k.z + dz);
                        if (state(other) != Cell::FREE && intersects_expanded_voxel(a, b, other, margin))
                            return false;
                    }
            return true;
        });
    }
    bool point_is_free(const Point& p, double margin = 0.0) const {
        return segment_is_free(p, p, margin);
    }

    // Conservative distance to the boundary of an all-FREE voxel cube. This is
    // a lower bound on known-free clearance, NOT an exact obstacle ESDF. Unknown
    // and occupied query cells return a negative value rather than open space.
    double clearance_lower_bound(const Point& p, double cap = 30.0) const {
        if (!valid_point(p) || !std::isfinite(cap) || cap <= 0) return 0;
        const Key k = key(p);
        if (state(k) != Cell::FREE) return -std::min(cap, resolution_);
        double value = face_distance(p, k, 0);
        const int maximum = std::min(64, static_cast<int>(std::ceil(cap / resolution_)) + 1);
        for (int r = 1; r <= maximum && value < cap; ++r) {
            bool all_free = true;
            for (int dx = -r; dx <= r && all_free; ++dx)
                for (int dy = -r; dy <= r && all_free; ++dy)
                    for (int dz = -r; dz <= r; ++dz) {
                        if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != r) continue;
                        if (state(Key(k.x + dx, k.y + dy, k.z + dz)) != Cell::FREE) {
                            all_free = false; break;
                        }
                    }
            if (!all_free) break;
            value = face_distance(p, k, r);
        }
        return std::min(cap, value);
    }

    std::vector<Point> snapshot(Cell requested, std::size_t maximum = 0) const {
        std::vector<Point> result;
        for (const auto& item : cells_) {
            if (item.second != requested) continue;
            result.push_back(center(item.first));
            if (maximum && result.size() >= maximum) break;
        }
        return result;
    }

    // Public for independent geometry tests and map auditing. In supercover
    // mode tied face/edge/corner crossings include all touched voxels. Mapping
    // uses the half-open interior traversal so a corner touch is not free data.
    template<class Visitor>
    bool traverse(const Point& a, const Point& b, bool supercover, Visitor visit) const {
        if (!valid_point(a) || !valid_point(b)) return false;
        Key current = key(a);
        const double start[3] = {a.x, a.y, a.z}, finish[3] = {b.x, b.y, b.z};
        const double direction[3] = {b.x-a.x, b.y-a.y, b.z-a.z};
        int step[3]; double next[3], delta[3];
        int static_boundary[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; ++axis) {
            const int c = axis == 0 ? current.x : (axis == 1 ? current.y : current.z);
            step[axis] = direction[axis] > 0 ? 1 : (direction[axis] < 0 ? -1 : 0);
            if (step[axis] == 0) {
                next[axis] = delta[axis] = std::numeric_limits<double>::infinity();
                static_boundary[axis] = on_boundary(start[axis]);
            } else {
                const double boundary = (c + (step[axis] > 0 ? 1 : 0)) * resolution_;
                next[axis] = std::max(0.0, (boundary-start[axis])/direction[axis]);
                delta[axis] = resolution_ / std::abs(direction[axis]);
            }
        }
        const auto emit = [&](const Key& k) {
            if (!supercover) return visit(k);
            int mask = 0;
            for (int axis=0; axis<3; ++axis) if (static_boundary[axis]) mask |= 1 << axis;
            for (int sub=mask;; sub=(sub-1)&mask) {
                const Key shifted(k.x-((sub&1)?1:0), k.y-((sub&2)?1:0), k.z-((sub&4)?1:0));
                if (!visit(shifted)) return false;
                if (sub==0) break;
            }
            return true;
        };
        if (supercover) {
            if (!containing_keys(a, visit)) return false;
        } else if (!emit(current)) return false;
        const Key target = key(b);
        const long long bound = 8 + std::llabs(static_cast<long long>(target.x)-current.x) +
            std::llabs(static_cast<long long>(target.y)-current.y) +
            std::llabs(static_cast<long long>(target.z)-current.z);
        if (bound > 100000) return false;
        for (long long n=0; n<bound; ++n) {
            const double time = std::min(next[0], std::min(next[1], next[2]));
            if (!std::isfinite(time) || time > 1.0 + 1e-12) break;
            int mask=0;
            for (int axis=0; axis<3; ++axis)
                if (std::abs(next[axis]-time) <= 1e-12*std::max(1.0,std::abs(time))) mask |= 1 << axis;
            if (supercover) {
                for (int sub=mask; sub; sub=(sub-1)&mask) {
                    const Key touched(current.x+((sub&1)?step[0]:0),
                                      current.y+((sub&2)?step[1]:0), current.z+((sub&4)?step[2]:0));
                    if (!emit(touched)) return false;
                }
            }
            if (mask&1) current.x += step[0];
            if (mask&2) current.y += step[1];
            if (mask&4) current.z += step[2];
            for (int axis=0; axis<3; ++axis) if (mask&(1<<axis)) next[axis] += delta[axis];
            if (!supercover && !emit(current)) return false;
        }
        return !supercover || containing_keys(b, visit);
    }

private:
    bool on_boundary(double coordinate) const {
        const double scaled = coordinate / resolution_;
        return std::abs(scaled-std::round(scaled)) <= 1e-10;
    }
    template<class Visitor>
    bool containing_keys(const Point& p, Visitor visit) const {
        const Key k = key(p);
        const int mask = (on_boundary(p.x)?1:0) | (on_boundary(p.y)?2:0) | (on_boundary(p.z)?4:0);
        for (int sub=mask;; sub=(sub-1)&mask) {
            if (!visit(Key(k.x-((sub&1)?1:0), k.y-((sub&2)?1:0), k.z-((sub&4)?1:0)))) return false;
            if (sub==0) break;
        }
        return true;
    }
    void observe_free(const Key& k) {
        if (cells_.find(k) != cells_.end()) return; // Static occupied cells persist.
        if (cells_.size() >= max_cells_) { ++stats_.dropped_capacity; return; }
        cells_.emplace(k,Cell::FREE); ++stats_.free_cells;
    }
    void assign(const Key& k, Cell value) {
        auto it = cells_.find(k);
        if (it == cells_.end()) {
            if (cells_.size() >= max_cells_) { ++stats_.dropped_capacity; return; }
            cells_.emplace(k, value);
            if (value == Cell::FREE) ++stats_.free_cells; else ++stats_.occupied_cells;
        } else if (it->second != value) {
            if (it->second == Cell::FREE) --stats_.free_cells; else --stats_.occupied_cells;
            it->second = value;
            if (value == Cell::FREE) ++stats_.free_cells; else ++stats_.occupied_cells;
        }
    }
    void clear_query_cache() const {
        for (auto& layer : safe_cells_) layer.clear();
    }
    bool safe_cell(const Key& k, int radius) const {
        auto& layer = safe_cells_[radius];
        const auto cached = layer.find(k);
        if (cached != layer.end()) return cached->second;
        bool safe = true;
        for (int dx=-radius; dx<=radius && safe; ++dx)
            for (int dy=-radius; dy<=radius && safe; ++dy)
                for (int dz=-radius; dz<=radius; ++dz)
                    if (state(Key(k.x+dx,k.y+dy,k.z+dz)) != Cell::FREE) { safe=false; break; }
        layer.emplace(k, safe);
        return safe;
    }
    static bool clip(double a, double b, double low, double high, double& enter, double& leave) {
        const double direction=b-a;
        if (direction==0) return a>=low && a<=high;
        double x=(low-a)/direction, y=(high-a)/direction;
        if (x>y) std::swap(x,y);
        enter=std::max(enter,x); leave=std::min(leave,y);
        return enter<=leave;
    }
    bool intersects_expanded_voxel(const Point& a, const Point& b, const Key& k, double margin) const {
        double enter=0, leave=1;
        return clip(a.x,b.x,k.x*resolution_-margin,(k.x+1)*resolution_+margin,enter,leave) &&
               clip(a.y,b.y,k.y*resolution_-margin,(k.y+1)*resolution_+margin,enter,leave) &&
               clip(a.z,b.z,k.z*resolution_-margin,(k.z+1)*resolution_+margin,enter,leave);
    }
    double face_distance(const Point& p, const Key& k, int radius) const {
        return std::min(std::min(p.x-(k.x-radius)*resolution_,(k.x+radius+1)*resolution_-p.x),
            std::min(std::min(p.y-(k.y-radius)*resolution_,(k.y+radius+1)*resolution_-p.y),
                     std::min(p.z-(k.z-radius)*resolution_,(k.z+radius+1)*resolution_-p.z)));
    }
    double resolution_;
    std::size_t max_cells_;
    std::unordered_map<Key,Cell,KeyHash> cells_;
    Stats stats_;
    std::uint64_t generation_ = 0;
    // safe_cell() certifies an entire integer-radius cube and therefore has
    // exactly the same result for all real margins inducing that radius. Only
    // this predicate is memoized by radius; expanded-voxel fallback tests retain
    // the original real-valued margin. At most 65 spatial layers (radii 0--64) can be used.
    mutable std::array<std::unordered_map<Key,bool,KeyHash>,65> safe_cells_;
};
}  // namespace sensor_occupancy
#endif
