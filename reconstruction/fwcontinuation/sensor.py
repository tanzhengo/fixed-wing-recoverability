"""Independent analytic sensor and timestamped pose reference utilities.

Geometry is physical sensor truth and must not be passed to the planner. This
is a new reproducible simulator, not recovered historical flight software.
Quaternion storage is explicitly (x, y, z, w), with world-from-body rotation.
"""
from dataclasses import dataclass
import math
import numpy as np
from .observed_map import Beam, _point


def _quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must contain four finite xyzw components")
    norm = float(np.linalg.norm(q))
    if norm <= 1e-15:
        raise ValueError("Zero quaternion")
    return q / norm


@dataclass(frozen=True)
class Pose:
    timestamp: float
    position: tuple
    quaternion_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self):
        if not math.isfinite(self.timestamp):
            raise ValueError("Pose timestamp must be finite")
        object.__setattr__(self, "position", _point(self.position))
        object.__setattr__(self, "quaternion_xyzw", tuple(_quaternion(self.quaternion_xyzw)))


def slerp(q0, q1, fraction):
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("Interpolation fraction must lie in [0,1]")
    a, b = _quaternion(q0), _quaternion(q1)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot  # quaternion double cover; choose shortest rotation
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return tuple(_quaternion((1 - fraction) * a + fraction * b))
    angle = math.acos(dot)
    return tuple((math.sin((1 - fraction) * angle) * a + math.sin(fraction * angle) * b)
                 / math.sin(angle))


def interpolate_pose(left, right, timestamp, max_gap=None):
    """Interpolate a bracketed pose; never silently extrapolate scans."""
    if not math.isfinite(timestamp) or right.timestamp < left.timestamp:
        raise ValueError("Invalid pose bracket")
    if not left.timestamp <= timestamp <= right.timestamp:
        raise ValueError("Scan timestamp is outside the pose bracket")
    gap = right.timestamp - left.timestamp
    if max_gap is not None and (not math.isfinite(max_gap) or max_gap < 0 or gap > max_gap):
        raise ValueError("Pose bracket exceeds the permitted gap")
    if gap == 0:
        if left.position != right.position or abs(float(np.dot(
                left.quaternion_xyzw, right.quaternion_xyzw))) < 1 - 1e-12:
            raise ValueError("Conflicting poses at the same timestamp")
        return Pose(timestamp, left.position, left.quaternion_xyzw)
    fraction = (timestamp - left.timestamp) / gap
    pos = tuple((1 - fraction) * np.array(left.position) + fraction * np.array(right.position))
    return Pose(timestamp, pos, slerp(left.quaternion_xyzw, right.quaternion_xyzw, fraction))


def rotation_matrix(quaternion_xyzw):
    x, y, z, w = _quaternion(quaternion_xyzw)
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


@dataclass(frozen=True)
class Sphere:
    center: tuple
    radius: float

    def __post_init__(self):
        object.__setattr__(self, "center", _point(self.center))
        if not math.isfinite(self.radius) or self.radius <= 0:
            raise ValueError("Sphere radius must be positive")


@dataclass(frozen=True)
class Cylinder:
    """Closed finite upright cylinder, centered at center including altitude."""
    center: tuple
    radius: float
    height: float

    def __post_init__(self):
        object.__setattr__(self, "center", _point(self.center))
        if not all(math.isfinite(v) and v > 0 for v in (self.radius, self.height)):
            raise ValueError("Cylinder dimensions must be positive")


def _intersections(origin, direction, obstacle):
    ox, oy, oz = (origin[j] - obstacle.center[j] for j in range(3))
    dx, dy, dz = direction
    if isinstance(obstacle, Sphere):
        half_b = ox*dx + oy*dy + oz*dz
        c = ox*ox + oy*oy + oz*oz - obstacle.radius**2
        disc = half_b*half_b - c
        if disc >= -1e-12:
            root = math.sqrt(max(0.0, disc))
            return (-half_b-root, -half_b+root)
        return ()
    if not isinstance(obstacle, Cylinder):
        raise TypeError("Only Sphere and upright Cylinder geometry is supported")
    hits = []
    a, half_b = dx*dx+dy*dy, ox*dx+oy*dy
    c = ox*ox+oy*oy-obstacle.radius**2
    disc = half_b*half_b-a*c
    if a > 1e-20 and disc >= -1e-12:
        root = math.sqrt(max(0.0, disc))
        for t in ((-half_b-root)/a, (-half_b+root)/a):
            if abs(oz+t*dz) <= obstacle.height/2+1e-12:
                hits.append(t)
    if abs(dz) > 1e-15:
        for cap in (-obstacle.height/2, obstacle.height/2):
            t = (cap-oz)/dz
            if (ox+t*dx)**2+(oy+t*dy)**2 <= obstacle.radius**2+1e-12:
                hits.append(t)
    return hits


def trace_ray(origin, direction, obstacles=(), min_range=1.0, max_range=120.0,
              ground_height=None):
    """Nearest-hit ideal ray; a blind-zone hit is invalid, never clear no-return."""
    origin, direction = _point(origin), _point(direction)
    if not all(math.isfinite(v) for v in (min_range, max_range)) or not 0 <= min_range < max_range:
        raise ValueError("Invalid sensor range")
    norm = math.sqrt(sum(x*x for x in direction))
    if norm <= 1e-15:
        raise ValueError("Zero ray direction")
    direction = tuple(x/norm for x in direction)
    closest = math.inf
    for obstacle in obstacles:
        for t in _intersections(origin, direction, obstacle):
            if t >= -1e-12:
                closest = min(closest, max(0.0, t))
    if ground_height is not None:
        if not math.isfinite(ground_height):
            raise ValueError("Nonfinite ground altitude")
        if abs(direction[2]) > 1e-15:
            t = (ground_height-origin[2])/direction[2]
            if t >= 0:
                closest = min(closest, t)
    has_return = closest <= max_range
    distance = closest if has_return else max_range
    end = tuple(origin[j] + distance*direction[j] for j in range(3))
    return Beam(origin, end, has_return, valid=(not has_return or distance >= min_range))


def lidar_directions(azimuth_count=512, elevation_count=32,
                     elevation_degrees=(-40.0, 20.0)):
    """Sensor-frame unit directions, full azimuth with no duplicate 2pi ray."""
    if azimuth_count < 1 or elevation_count < 1:
        raise ValueError("Ray counts must be positive")
    low, high = map(math.radians, elevation_degrees)
    if not -math.pi/2 <= low <= high <= math.pi/2:
        raise ValueError("Invalid elevation interval")
    az = np.arange(azimuth_count)*2*math.pi/azimuth_count
    elevations = np.linspace(low, high, elevation_count)
    return np.array([(math.cos(e)*math.cos(a), math.cos(e)*math.sin(a), math.sin(e))
                     for e in elevations for a in az])


def trace_scan(pose, directions, obstacles=(), sensor_offset=(0.0, 0.0, 0.15),
               min_range=1.0, max_range=120.0, ground_height=None):
    """Transform sensor rays using a scan-time pose and trace physical truth."""
    rotation = rotation_matrix(pose.quaternion_xyzw)
    origin = tuple(np.array(pose.position) + rotation @ np.array(_point(sensor_offset)))
    obstacles = tuple(obstacles)
    return [trace_ray(origin, rotation @ np.array(_point(direction)), obstacles,
                      min_range, max_range, ground_height) for direction in directions]


scan = trace_scan


def beams_from_returns(origin, points):
    """Return-only cloud adapter: absent beams provide no clearing evidence.

    Points and origin must already be time-aligned into the world frame. This
    adapter creates only return rays; it cannot recover valid no-return rays.
    """
    origin = _point(origin)
    return [Beam(origin, _point(point), True) for point in points]
