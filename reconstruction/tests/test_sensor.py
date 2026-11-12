import math
import unittest
import numpy as np
from fwcontinuation.observed_map import ObservedMap
from fwcontinuation.sensor import (Pose, Sphere, Cylinder, interpolate_pose,
    slerp, rotation_matrix, trace_ray, trace_scan, lidar_directions, beams_from_returns)


class SensorTests(unittest.TestCase):
    def test_pose_interpolation_uses_shortest_quaternion_arc(self):
        left = Pose(1, (0, 0, 0))
        right = Pose(3, (2, 4, 6), (0, 0, 1, 0))
        mid = interpolate_pose(left, right, 2)
        np.testing.assert_allclose(mid.position, (1, 2, 3))
        np.testing.assert_allclose(rotation_matrix(mid.quaternion_xyzw) @ (1, 0, 0),
                                   (0, 1, 0), atol=1e-12)
        np.testing.assert_allclose(slerp((0, 0, 0, 1), (0, 0, 0, -1), .5), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            interpolate_pose(left, right, 4)
        with self.assertRaises(ValueError):
            interpolate_pose(left, right, 2, max_gap=.02)

    def test_nearest_sphere_hit_and_no_return(self):
        obstacles = [Sphere((10, 0, 0), 2), Sphere((5, 0, 0), 1)]
        beam = trace_ray((0, 0, 0), (2, 0, 0), obstacles)
        self.assertTrue(beam.has_return and beam.valid)
        np.testing.assert_allclose(beam.end, (4, 0, 0))
        empty = trace_ray((0, 0, 0), (0, 1, 0), obstacles, max_range=20)
        self.assertFalse(empty.has_return)
        self.assertTrue(empty.valid)
        np.testing.assert_allclose(empty.end, (0, 20, 0))

    def test_closed_cylinder_side_caps_and_tangent(self):
        cylinder = Cylinder((5, 0, 5), 1, 4)
        np.testing.assert_allclose(trace_ray((0, 0, 5), (1, 0, 0), [cylinder]).end, (4, 0, 5))
        np.testing.assert_allclose(trace_ray((5, 0, 10), (0, 0, -1), [cylinder]).end, (5, 0, 7))
        np.testing.assert_allclose(trace_ray((0, 1, 5), (1, 0, 0), [cylinder]).end, (5, 1, 5))
        self.assertFalse(trace_ray((0, 0, 8), (1, 0, 0), [cylinder]).has_return)

    def test_blind_zone_hit_never_becomes_free_space(self):
        beam = trace_ray((0, 0, 0), (1, 0, 0), [Sphere((.6, 0, 0), .1)], min_range=1)
        self.assertTrue(beam.has_return)
        self.assertFalse(beam.valid)
        m = ObservedMap()
        m.integrate_scan([beam], 1)
        self.assertEqual(m.version, 0)

    def test_ground_and_sensor_offset_use_world_frame(self):
        beam = trace_ray((1, 1, 5), (0, 0, -1), ground_height=0)
        np.testing.assert_allclose(beam.end, (1, 1, 0))
        pose = Pose(0, (10, 20, 30), (0, 0, math.sqrt(.5), math.sqrt(.5)))
        scan = trace_scan(pose, [(1, 0, 0)], [], sensor_offset=(0, 0, .15), max_range=10)
        np.testing.assert_allclose(scan[0].origin, (10, 20, 30.15))
        np.testing.assert_allclose(scan[0].end, (10, 30, 30.15), atol=1e-12)

    def test_return_only_input_does_not_invent_missing_rays(self):
        origin = (.5, .5, .5)
        beams = beams_from_returns(origin, [(4.5, .5, .5)])
        self.assertEqual(len(beams), 1)
        m = ObservedMap()
        m.integrate_scan(beams, 1)
        self.assertEqual(m.state((4.5, .5, .5)), "occupied")
        self.assertEqual(m.state((6.5, .5, .5)), "unknown")
        self.assertEqual(m.state((.5, 4.5, .5)), "unknown")

    def test_direction_grid_has_documented_shape_and_units(self):
        directions = lidar_directions(4, 3, (-20, 20))
        self.assertEqual(directions.shape, (12, 3))
        np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1)
        np.testing.assert_allclose(directions[4], (1, 0, 0), atol=1e-12)

    def test_invalid_pose_and_sensor_configuration(self):
        with self.assertRaises(ValueError):
            Pose(0, (0, 0, 0), (0, 0, 0, 0))
        with self.assertRaises(ValueError):
            trace_ray((0, 0, 0), (0, 0, 0))
        with self.assertRaises(ValueError):
            trace_ray((0, 0, 0), (1, 0, 0), min_range=10, max_range=5)


if __name__ == "__main__":
    unittest.main()
