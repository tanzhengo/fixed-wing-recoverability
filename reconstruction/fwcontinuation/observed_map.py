"""Three-state voxel reference map reconstructed from the manuscript.

No historical code or experimental results are reproduced by this module.
Cells use half-open indexing for point lookup. Collision checks treat voxel
volumes as closed, so contact with a non-free face, edge or corner is rejected.
"""
from dataclasses import dataclass
from itertools import product
import math


STATES = frozenset(("free", "occupied", "unknown"))
_BOUNDARY_TOL = 1e-10  # in voxel coordinates; conservative floating-point guard


def _point(value):
    p = tuple(float(x) for x in value)
    if len(p) != 3 or not all(math.isfinite(x) for x in p):
        raise ValueError("Expected three finite coordinates")
    return p


@dataclass(frozen=True)
class Beam:
    """World-frame ray with explicit return and validity information.

    A valid no-return ray ends at its measured clearing range. Missing points
    in a return-only point cloud are NOT no-return rays.
    """
    origin: tuple
    end: tuple
    has_return: bool
    valid: bool = True


def _axis_cells(x):
    nearest = round(x)
    if abs(x - nearest) <= _BOUNDARY_TOL:
        return (nearest - 1, nearest)
    return (math.floor(x),)


def _segment_grid_events(a, b, voxel_size):
    if not math.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError("voxel_size must be positive")
    a, b = _point(a), _point(b)
    x = tuple(v / voxel_size for v in a)
    y = tuple(v / voxel_size for v in b)
    delta = tuple(y[j] - x[j] for j in range(3))
    events = {0.0, 1.0}
    for lo, hi, start, d in zip(map(min, x, y), map(max, x, y), x, delta):
        if d == 0.0:
            continue
        for plane in range(math.ceil(lo), math.floor(hi) + 1):
            t = (plane - start) / d
            if 0.0 < t < 1.0:
                events.add(t)
    return x, delta, sorted(events)


def supercover_segment(a, b, voxel_size=2.0):
    """Return every closed voxel touched by a segment, including its ends.

    Grid-plane events and interval midpoints also handle rays lying on grid
    planes/edges. Near-exact grid contacts are rounded conservatively.
    """
    x, delta, ts = _segment_grid_events(a, b, voxel_size)
    touched = set()
    for t in ts:
        p = tuple(x[j] + t * delta[j] for j in range(3))
        touched.update(product(*(_axis_cells(v) for v in p)))
    for left, right in zip(ts, ts[1:]):
        t = (left + right) * 0.5
        p = tuple(x[j] + t * delta[j] for j in range(3))
        touched.update(product(*(_axis_cells(v) for v in p)))
    return tuple(sorted(touched))


def _scan_clear_cells(a, b, voxel_size):
    """Cells intersected over positive ray length, without point-only contacts.

    Collision validation deliberately uses the closed supercover above. Free
    evidence must instead exclude cells touched only at an origin, crossing,
    or range endpoint: none of their interior ray interval was measured. A ray
    lying exactly along a grid face/edge intersects both sides over positive
    length. A zero-length beam supplies no clearing evidence.
    """
    x, delta, ts = _segment_grid_events(a, b, voxel_size)
    if not any(delta):
        return ()
    touched = set()
    for left, right in zip(ts, ts[1:]):
        t = left + (right - left) * 0.5
        cells = []
        for j in range(3):
            if delta[j] == 0.0 and x[j] == math.floor(x[j]):
                boundary = math.floor(x[j])
                cells.append((boundary - 1, boundary))
            else:
                cells.append((math.floor(x[j] + t * delta[j]),))
        touched.update(product(*cells))
    return tuple(sorted(touched))


def _intersects_expanded_cell(a, b, cell, size, margin):
    """Slab test for closed segment versus voxel expanded by an L-infinity box."""
    enter, leave = 0.0, 1.0
    for j in range(3):
        low = cell[j] * size - margin
        high = (cell[j] + 1) * size + margin
        direction = b[j] - a[j]
        if direction == 0.0:
            if a[j] < low or a[j] > high:
                return False
        else:
            t0, t1 = (low - a[j]) / direction, (high - a[j]) / direction
            if t0 > t1:
                t0, t1 = t1, t0
            enter, leave = max(enter, t0), min(leave, t1)
            if enter > leave + 1e-12:
                return False
    return True


class ObservedMap:
    """Sparse static map; absence means unknown, and occupied cells persist.

    Neighbor caching stores only candidate non-free cell indices, never a
    position-independent final collision answer. Exact coordinates and the
    real-valued margin are tested on every segment. Cache invalidation is tied
    to each content-version change. Not intended for concurrent mutation.
    """
    def __init__(self, voxel_size=2.0):
        voxel_size = float(voxel_size)
        if not math.isfinite(voxel_size) or voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        self.voxel_size = voxel_size
        self.version = 0
        self.timestamp = 0.0
        self._cells = {}
        self._neighbor_cache = {}

    def cell_index(self, point):
        return tuple(math.floor(x / self.voxel_size) for x in _point(point))

    def state(self, point):
        return self._cells.get(self.cell_index(point), "unknown")

    def cell_state(self, index):
        return self._cells.get(tuple(index), "unknown")

    def _changed(self):
        self.version += 1
        self._neighbor_cache.clear()

    def set_cell(self, index_tuple, state):
        if state not in STATES:
            raise ValueError("Unknown voxel state")
        index = tuple(index_tuple)
        if len(index) != 3 or any(isinstance(v, bool) or int(v) != v for v in index):
            raise ValueError("Cell index must contain three integers")
        index = tuple(int(v) for v in index)
        previous = self._cells.get(index, "unknown")
        if previous == state or previous == "occupied":
            return False
        if state == "unknown":
            self._cells.pop(index, None)
        else:
            self._cells[index] = state
        self._changed()
        return True

    def integrate_scan(self, beams, timestamp):
        """Commit a scan atomically, with occupied endpoints taking precedence.

        Invalid beams do not change cells or refresh the map timestamp. Callers
        must enforce sensor-freshness and pose-bracket policies before integration.
        """
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("Scan timestamp must be finite")
        free, occupied = set(), set()
        accepted_beam = False
        for beam in beams:
            if not beam.valid:
                continue
            try:
                origin, end = _point(beam.origin), _point(beam.end)
            except (TypeError, ValueError, OverflowError):
                continue  # malformed sensor measurements fail closed
            accepted_beam = True
            free.update(_scan_clear_cells(origin, end, self.voxel_size))
            if beam.has_return:
                occupied.add(self.cell_index(end))
        changed = False
        for index in free - occupied:
            if self._cells.get(index, "unknown") == "unknown":
                self._cells[index] = "free"
                changed = True
        for index in occupied:
            if self._cells.get(index) != "occupied":
                self._cells[index] = "occupied"
                changed = True
        if accepted_beam:
            self.timestamp = max(self.timestamp, timestamp)
        if changed:
            self._changed()
        return changed

    def _nonfree_neighbors(self, center, radius):
        key = (center, radius)
        cached = self._neighbor_cache.get(key)
        if cached is not None:
            return cached
        cells = tuple(index for index in product(*(
            range(center[j] - radius, center[j] + radius + 1) for j in range(3)))
            if self._cells.get(index, "unknown") != "free")
        if len(self._neighbor_cache) >= 32768:
            self._neighbor_cache.clear()
        self._neighbor_cache[key] = cells
        return cells

    def segment_free(self, a, b, margin=0.0):
        """Check the whole predicted segment with an exact per-face box margin.

        Occupied and unobserved closed voxel volumes are both inadmissible.
        This checks a numerical predictor's straight segment, not a physical
        nonlinear trajectory, a tracking tube, or the complete airframe volume.
        """
        margin = float(margin)
        if not math.isfinite(margin) or margin < 0:
            raise ValueError("Margin must be finite and nonnegative")
        try:
            a, b = _point(a), _point(b)
        except (TypeError, ValueError, OverflowError):
            return False
        # +1 includes closed contacts when margin is an exact cell multiple.
        radius = math.ceil(margin / self.voxel_size) + 1
        checked = set()
        for center in supercover_segment(a, b, self.voxel_size):
            for cell in self._nonfree_neighbors(center, radius):
                if cell in checked:
                    continue
                checked.add(cell)
                if _intersects_expanded_cell(a, b, cell, self.voxel_size, margin):
                    return False
        return True

    def snapshot(self):
        """JSON-compatible snapshot for provenance and deterministic replay."""
        return {"voxel_size": self.voxel_size, "version": self.version,
                "timestamp": self.timestamp,
                "cells": [[*index, state] for index, state in sorted(self._cells.items())]}

    @classmethod
    def from_snapshot(cls, snapshot):
        result = cls(snapshot["voxel_size"])
        for row in snapshot["cells"]:
            result.set_cell(row[:3], row[3])
        result.timestamp = float(snapshot["timestamp"])
        result.version = int(snapshot["version"])
        if not math.isfinite(result.timestamp) or result.version < 0:
            raise ValueError("Invalid snapshot metadata")
        result._neighbor_cache.clear()
        return result
