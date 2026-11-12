import itertools
import unittest
from fwcontinuation.observed_map import Beam, ObservedMap, supercover_segment


def free_box(voxel_size=2, low=-3, high=3):
    result = ObservedMap(voxel_size)
    for index in itertools.product(range(low, high+1), repeat=3):
        result.set_cell(index, "free")
    return result


class ObservedMapTests(unittest.TestCase):
    def test_default_unknown_and_negative_indices(self):
        m = ObservedMap()
        self.assertEqual(m.state((-.01, 0, 0)), "unknown")
        m.set_cell((-1, 0, 0), "free")
        self.assertEqual(m.state((-.01, 0, 0)), "free")
        self.assertEqual(m.state((0, 0, 0)), "unknown")

    def test_scan_endpoint_precedence_is_order_independent(self):
        beams = [Beam((.5, .5, .5), (4.5, .5, .5), True),
                 Beam((.5, .5, .5), (8.5, .5, .5), False)]
        maps = []
        for sequence in (beams, list(reversed(beams))):
            m = ObservedMap()
            m.integrate_scan(sequence, 1)
            self.assertEqual(m.state((4.5, .5, .5)), "occupied")
            self.assertEqual(m.state((8.5, .5, .5)), "free")
            m.integrate_scan([beams[1]], 2)
            self.assertEqual(m.state((4.5, .5, .5)), "occupied")
            maps.append(m.snapshot()["cells"])
        self.assertEqual(*maps)

    def test_invalid_beams_do_not_clear(self):
        m = ObservedMap()
        m.integrate_scan([Beam((0, 0, 0), (5, 0, 0), False, False),
                          Beam((float("nan"), 0, 0), (5, 0, 0), False)], 1)
        self.assertEqual(m.version, 0)
        self.assertEqual(m.timestamp, 0)
        self.assertEqual(m.state((1, 0, 0)), "unknown")

    def test_return_at_reverse_ray_endpoint_does_not_clear_behind_surface(self):
        m = ObservedMap()
        m.integrate_scan([Beam((3, 1, 1), (2, 1, 1), True)], 1)
        self.assertEqual(m.cell_state((1, 0, 0)), "occupied")
        self.assertEqual(m.cell_state((0, 0, 0)), "unknown")

    def test_scan_origin_face_only_contact_is_not_cleared(self):
        for end, free, unknown in [((3, 1, 1), (1, 0, 0), (0, 0, 0)),
                                   ((1, 1, 1), (0, 0, 0), (1, 0, 0))]:
            m = ObservedMap()
            m.integrate_scan([Beam((2, 1, 1), end, False)], 1)
            self.assertEqual(m.cell_state(free), "free")
            self.assertEqual(m.cell_state(unknown), "unknown")

    def test_no_return_range_endpoint_only_contact_is_not_cleared(self):
        for origin, free, unknown in [((1, 1, 1), (0, 0, 0), (1, 0, 0)),
                                      ((3, 1, 1), (1, 0, 0), (0, 0, 0))]:
            m = ObservedMap()
            m.integrate_scan([Beam(origin, (2, 1, 1), False)], 1)
            self.assertEqual(m.cell_state(free), "free")
            self.assertEqual(m.cell_state(unknown), "unknown")

    def test_zero_length_scan_has_no_free_evidence(self):
        m = ObservedMap()
        m.integrate_scan([Beam((2, 2, 2), (2, 2, 2), False)], 1)
        self.assertEqual(m.snapshot()['cells'], [])
        m.integrate_scan([Beam((2, 2, 2), (2, 2, 2), True)], 2)
        self.assertEqual(m.snapshot()['cells'], [[1, 1, 1, 'occupied']])

    def test_scan_crossing_corner_does_not_clear_point_only_neighbors(self):
        m = ObservedMap()
        m.integrate_scan([Beam((1, 1, 1), (3, 3, 3), False)], 1)
        self.assertEqual(m.snapshot()['cells'], [[0, 0, 0, 'free'], [1, 1, 1, 'free']])
        # Positive-length travel along a face still clears its two neighbors.
        face = ObservedMap()
        face.integrate_scan([Beam((2, 1, 1), (2, 3, 1), False)], 1)
        for cell in [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]:
            self.assertEqual(face.cell_state(cell), 'free')

    def test_supercover_face_edge_corner_and_zero_length(self):
        self.assertEqual(set(supercover_segment((0, 0, 0), (0, 0, 0))),
                         set(itertools.product((-1, 0), repeat=3)))
        cells = set(supercover_segment((1, 1, 1), (3, 3, 3)))
        self.assertTrue(set(itertools.product((0, 1), repeat=3)) <= cells)
        self.assertEqual(cells, set(supercover_segment((3, 3, 3), (1, 1, 1))))
        face = set(supercover_segment((0, .5, .5), (0, 5, .5)))
        self.assertTrue({(-1, 1, 0), (0, 1, 0)} <= face)

    def test_unknown_boundary_is_closed(self):
        m = ObservedMap()
        m.set_cell((0, 0, 0), "free")
        self.assertTrue(m.segment_free((1, 1, 1), (1.5, 1, 1), 0))
        self.assertFalse(m.segment_free((0, 1, 1), (1, 1, 1), 0))
        self.assertFalse(m.segment_free((1, 1, 1), (1, 1, 1), 1))

    def test_whole_segment_catches_obstacle_between_endpoints(self):
        m = free_box()
        m.set_cell((0, 0, 0), "occupied")
        self.assertEqual(m.state((-3, 1, 1)), "free")
        self.assertEqual(m.state((5, 1, 1)), "free")
        self.assertFalse(m.segment_free((-3, 1, 1), (5, 1, 1), 0))
        # Segment touches only an occupied corner at its midpoint.
        self.assertFalse(m.segment_free((-1, 1, 0), (1, -1, 0), 0))

    def test_fractional_margin_exact_test_and_cache_invalidation(self):
        m = free_box()
        a, b = (1, 1, 1), (1.5, 1, 1)
        self.assertTrue(m.segment_free(a, b, 1.5))
        m.set_cell((2, 0, 0), "occupied")  # occupied starts at x=4
        self.assertTrue(m.segment_free(a, b, 2.49))
        self.assertFalse(m.segment_free(a, b, 2.5))
        # Same integer neighborhood radius must not cache a final boolean.
        self.assertTrue(m.segment_free(a, b, 2.49))
        self.assertFalse(m.segment_free(a, b, 2.51))

    def test_unknown_to_free_invalidates_cached_neighborhood(self):
        m = free_box()
        m.set_cell((0, 0, 0), "unknown")
        self.assertFalse(m.segment_free((1, 1, 1), (1.5, 1, 1)))
        old_version = m.version
        m.set_cell((0, 0, 0), "free")
        self.assertGreater(m.version, old_version)
        self.assertTrue(m.segment_free((1, 1, 1), (1.5, 1, 1)))

    def test_snapshots_are_independent_and_deterministic(self):
        m = free_box()
        m.set_cell((0, 0, 0), "occupied")
        restored = ObservedMap.from_snapshot(m.snapshot())
        self.assertEqual(restored.snapshot(), m.snapshot())
        restored.set_cell((7, 0, 0), "free")
        self.assertEqual(m.state((15, 1, 1)), "unknown")

    def test_validation_rejects_invalid_geometry(self):
        m = free_box()
        self.assertFalse(m.segment_free((float("nan"), 0, 0), (0, 0, 0)))
        with self.assertRaises(ValueError):
            m.segment_free((0, 0, 0), (0, 0, 0), -.01)
        with self.assertRaises(ValueError):
            ObservedMap(0)


if __name__ == "__main__":
    unittest.main()
