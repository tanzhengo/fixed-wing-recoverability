#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np

MODULE=Path(__file__).resolve().parents[1]/'scripts/collision_mesh_truth.py'
spec=importlib.util.spec_from_file_location('collision_mesh_truth',MODULE)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

class TriangleTruthTests(unittest.TestCase):
    def setUp(self):
        self.face=np.array([[[0.,0,0],[4,0,0],[0,4,0]]])
    def test_face_edge_vertex_and_degenerate_point_distances(self):
        self.assertAlmostEqual(m.point_triangle_distances([1,1,3],self.face)[0],3)
        self.assertAlmostEqual(m.point_triangle_distances([3,3,0],self.face)[0],np.sqrt(2))
        self.assertAlmostEqual(m.point_triangle_distances([-1,-1,0],self.face)[0],np.sqrt(2))
        self.assertAlmostEqual(m.point_triangle_distances([0,2,0],np.array([[[0,0,0],[1,0,0],[2,0,0]]]))[0],2)
    def test_segment_crosses_face_between_free_endpoints(self):
        self.assertAlmostEqual(m.segment_triangle_distances([1,1,-2],[1,1,2],self.face)[0],0)
        self.assertAlmostEqual(m.segment_triangle_distances([1,1,2],[2,1,2],self.face)[0],2)
    def test_segment_edge_and_zero_length(self):
        self.assertAlmostEqual(m.segment_triangle_distances([3,3,-2],[3,3,2],self.face)[0],np.sqrt(2))
        self.assertAlmostEqual(m.segment_triangle_distances([1,1,3],[1,1,3],self.face)[0],3)
    def test_reported_garage_corner_has_real_mesh_clearance(self):
        triangle=np.array([[[55.38692464,-103.26725659,57.41081736],[90.9327028,-102.43306898,57.41081736],[90.9327028,-102.43306898,49.09774788]]])
        p=[95.70914375020091,-101.98810743022692,55.582292766495705]
        self.assertAlmostEqual(m.point_triangle_distances(p,triangle)[0],4.797121944966171,places=6)
    def test_random_segments_agree_with_dense_upper_bound(self):
        rng=np.random.RandomState(9203)
        for _ in range(30):
            tri=rng.uniform(-3,3,(5,3,3));a,b=rng.uniform(-4,4,(2,3));exact=m.segment_triangle_distances(a,b,tri)
            sampled=np.full(5,np.inf)
            for f in np.linspace(0,1,501):sampled=np.minimum(sampled,m.point_triangle_distances(a+(b-a)*f,tri))
            self.assertTrue(np.all(exact<=sampled+1e-8))
            self.assertTrue(np.all(sampled-exact<=np.linalg.norm(b-a)/1000+1e-8))
    def test_snapshot_solids_caps_and_unknown_shapes(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'box.npz';tri=m._box_triangles([2,2,2])
            np.savez(path,triangles=tri,object_offsets=[0,len(tri)],object_names=['box'],closed=[True],approximation_error=[0.])
            truth=m.MeshTruth(path)
            self.assertAlmostEqual(truth.point_clearance([0,0,0]),-1)
            self.assertAlmostEqual(truth.point_clearance([0,0,0],cap=.1),-1)
            self.assertAlmostEqual(truth.point_clearance([3,0,0]),2)
            self.assertEqual(truth.point_clearance([100,0,0],cap=5),5)
            self.assertLessEqual(truth.segment_clearance([-3,0,0],[3,0,0]),0)
            self.assertAlmostEqual(truth.segment_clearance([3,-3,0],[3,3,0]),2)
            with self.assertRaises(ValueError):truth.point_clearance([np.nan,0,0])
        with self.assertRaises(ValueError):m._primitive({'type':'unsupported'})

if __name__=='__main__':unittest.main()
