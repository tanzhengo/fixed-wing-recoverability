#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import unittest
import numpy as np

source = Path(__file__).resolve().parents[1]/'scripts/lidar_voxel_map.py'
spec = importlib.util.spec_from_file_location('lidar_voxel_map', source)
lidar = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lidar)


class LidarVoxelMapTests(unittest.TestCase):
    def test_full_pose_and_sensor_offset(self):
        # A forward return becomes world +Y at yaw=90 degrees; the mounted
        # sensor's 0.15m height is rotated with the actual body pose.
        points = lidar.transform_points([[2,0,0]], [10,20,30], lidar.rpy_quaternion([0,0,np.pi/2]))
        np.testing.assert_allclose(points, [[10,22,30.15]], atol=1e-10)
        pitched = lidar.transform_points([[2,0,0]], [0,0,0], lidar.rpy_quaternion([0,np.pi/2,0]))
        np.testing.assert_allclose(pitched, [[.15,0,-2]], atol=1e-10)

    def test_pose_interpolation_uses_scan_time(self):
        h = lidar.PoseHistory(max_gap=2.)
        h.add(0., [0,0,0], [0,0,0,1]);h.add(1., [10,0,0], lidar.rpy_quaternion([0,0,np.pi]))
        (p,q), reason = h.lookup(.5)
        self.assertIsNone(reason)
        np.testing.assert_allclose(p,[5,0,0])
        np.testing.assert_allclose(lidar.rotation_matrix(q)@np.array([1,0,0]),[0,1,0],atol=1e-10)
        self.assertEqual(h.lookup(1.1)[1], 'waiting_for_odometry')

    def test_future_scan_waits_for_bracketing_odometry(self):
        m=lidar.TemporalVoxelMap(max_pose_gap=2., sensor_offset=[0,0,0])
        m.add_pose(0.,[0,0,0],[0,0,0,1]);m.queue_cloud(.5,[[1,0,0]],0.)
        self.assertEqual(m.process_pending(.1),[])
        self.assertEqual(len(m.voxels),0)
        m.add_pose(1.,[10,0,0],[0,0,0,1]);done=m.process_pending(.2)
        np.testing.assert_allclose(done[0][1],[[6,0,0]])
        self.assertEqual(len(m.pending),0)

    def test_negative_voxel_indices_and_nonfinite_filtering(self):
        m=lidar.TemporalVoxelMap(sensor_offset=[0,0,0],min_range=0.)
        m.add_pose(1.,[0,0,0],[0,0,0,1])
        m.queue_cloud(1.,[[-.1,0,0],[-.9,.1,.1],[1.1,0,0],[np.nan,2,1],[np.inf,0,0]],0.)
        m.process_pending(0.)
        self.assertEqual(set(m.voxels),{(-1,0,0),(1,0,0)})
        self.assertEqual(m.stats['filtered_nonfinite_points'],2)
        self.assertEqual(m.stats['valid_points'],3)

    def test_reset_clears_old_epoch_map_and_pending_scans(self):
        m=lidar.TemporalVoxelMap(sensor_offset=[0,0,0])
        m.add_pose(100.,[0,0,0],[0,0,0,1]);m.queue_cloud(100.,[[2,0,0]],0.);m.process_pending(0.)
        m.queue_cloud(101.,[[3,0,0]],0.)
        self.assertTrue(m.add_pose(0.,[10,0,0],[0,0,0,1]))
        self.assertFalse(m.voxels);self.assertFalse(m.pending)
        self.assertEqual(m.stats['time_resets'],1)
        self.assertEqual(m.history.times,[0.])

    def test_gap_timeout_old_scan_and_capacity_are_bounded(self):
        m=lidar.TemporalVoxelMap(max_voxels=2,max_pose_gap=.1,queue_size=1,queue_timeout=.2,sensor_offset=[0,0,0])
        m.add_pose(1.,[0,0,0],[0,0,0,1]);m.add_pose(2.,[0,0,0],[0,0,0,1])
        m.queue_cloud(1.5,[[2,0,0]],0.);m.process_pending(.1)
        self.assertEqual(m.stats['dropped_odometry_gap'],1)
        m.queue_cloud(3.,[[2,0,0]],0.);m.process_pending(.3)
        self.assertEqual(m.stats['dropped_pose_timeout'],1)
        m.queue_cloud(0.,[[2,0,0]],0.);m.process_pending(.3)
        self.assertEqual(m.stats['dropped_older_than_odometry'],1)
        m.queue_cloud(2.,[[1,0,0],[2,0,0],[3,0,0]],1.);m.process_pending(1.)
        self.assertEqual(len(m.voxels),2);self.assertEqual(m.stats['evicted_voxels'],1)
        m.queue_cloud(4.,[[2,0,0]],1.);m.queue_cloud(5.,[[2,0,0]],1.)
        self.assertEqual(len(m.pending),1);self.assertEqual(m.stats['dropped_queue_full'],1)

    def test_quaternion_sign_does_not_change_interpolation(self):
        q=lidar.rpy_quaternion([.2,.3,.4])
        np.testing.assert_allclose(lidar.slerp(q,-q,.5),q)


if __name__ == '__main__':
    unittest.main()
