#!/usr/bin/env python3
"""Accumulate time-aligned lidar returns into occupied endpoint voxels.

The map is built only from received scans and odometry. It is a visualization
map for static scenes, not a free-space estimator or a SLAM backend. Pure
geometry/state classes below can be tested without importing ROS.
"""
import bisect
from collections import Counter, OrderedDict, deque
import json
import math
import threading
import time

import numpy as np


def unit_quaternion(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Quaternion must be finite and nonzero')
    return q / np.linalg.norm(q)


def rotation_matrix(quaternion):
    x, y, z, w = unit_quaternion(quaternion)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rpy_quaternion(rpy):
    r, p, y = np.asarray(rpy, dtype=float) / 2
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return unit_quaternion([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def slerp(a, b, fraction):
    a, b = unit_quaternion(a), unit_quaternion(b)
    dot = float(np.dot(a, b))
    if dot < 0:  # q and -q describe the same orientation.
        b, dot = -b, -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        return unit_quaternion(a + fraction*(b-a))
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


def transform_points(points, position, quaternion, sensor_offset=(0, 0, .15), sensor_rpy=(0, 0, 0)):
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    body = points @ rotation_matrix(rpy_quaternion(sensor_rpy)).T + np.asarray(sensor_offset)
    return body @ rotation_matrix(quaternion).T + np.asarray(position)


class PoseHistory:
    def __init__(self, duration=3., max_gap=.15, reset_threshold=.25):
        self.duration, self.max_gap, self.reset_threshold = duration, max_gap, reset_threshold
        self.times, self.poses = [], []

    def clear(self):
        self.times.clear()
        self.poses.clear()

    def add(self, stamp, position, quaternion):
        position = np.asarray(position, dtype=float)
        if not math.isfinite(stamp) or position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('Invalid odometry pose')
        quaternion = unit_quaternion(quaternion)
        reset = bool(self.times and stamp < self.times[-1]-self.reset_threshold)
        if reset:
            self.clear()
        index = bisect.bisect_left(self.times, stamp)
        if index < len(self.times) and self.times[index] == stamp:
            self.poses[index] = (position, quaternion)
        else:
            self.times.insert(index, stamp)
            self.poses.insert(index, (position, quaternion))
        cutoff = bisect.bisect_left(self.times, self.times[-1]-self.duration)
        if cutoff:
            del self.times[:cutoff]
            del self.poses[:cutoff]
        return reset

    def lookup(self, stamp):
        if not self.times or stamp > self.times[-1]:
            return None, 'waiting_for_odometry'
        if stamp < self.times[0]:
            return None, 'older_than_odometry'
        i = bisect.bisect_left(self.times, stamp)
        if self.times[i] == stamp:
            return self.poses[i], None
        a, b = self.times[i-1], self.times[i]
        if b-a > self.max_gap:
            return None, 'odometry_gap'
        alpha = (stamp-a)/(b-a)
        p0, q0 = self.poses[i-1]
        p1, q1 = self.poses[i]
        return (p0 + alpha*(p1-p0), slerp(q0, q1, alpha)), None


class TemporalVoxelMap:
    def __init__(self, resolution=1., max_voxels=100000, history_seconds=3.,
                 max_pose_gap=.15, queue_size=20, queue_timeout=3.,
                 min_range=.4, max_range=120., sensor_offset=(0, 0, .15), sensor_rpy=(0, 0, 0)):
        if not math.isfinite(resolution) or resolution <= 0 or max_voxels < 1 or queue_size < 1:
            raise ValueError('Resolution, map capacity and queue size must be positive')
        if min_range < 0 or max_range <= min_range or queue_timeout <= 0 or history_seconds <= 0 or max_pose_gap <= 0:
            raise ValueError('Invalid range/time configuration')
        self.resolution, self.max_voxels = resolution, int(max_voxels)
        self.queue_size, self.queue_timeout = int(queue_size), queue_timeout
        self.min_range, self.max_range = min_range, max_range
        self.sensor_offset, self.sensor_rpy = np.asarray(sensor_offset, dtype=float), np.asarray(sensor_rpy, dtype=float)
        if self.sensor_offset.shape != (3,) or self.sensor_rpy.shape != (3,) or not np.isfinite(np.r_[self.sensor_offset, self.sensor_rpy]).all():
            raise ValueError('Extrinsics must be finite 3-vectors')
        self.history = PoseHistory(history_seconds, max_pose_gap)
        self.pending, self.voxels = deque(), OrderedDict()
        self.stats = Counter()
        self.dirty = True
        self.latest_stamp = 0.
        self.latest_position = None

    def add_pose(self, stamp, position, quaternion):
        reset = self.history.add(stamp, position, quaternion)
        if reset:
            self.stats['dropped_time_reset'] += len(self.pending)
            self.pending.clear()
            self.voxels.clear()
            self.stats['time_resets'] += 1
            self.dirty = True
            self.latest_stamp = stamp
        self.latest_position = np.asarray(position)
        self.stats['odometry_received'] += 1
        return reset

    def queue_cloud(self, stamp, points, wall_time):
        if not math.isfinite(stamp):
            self.stats['dropped_invalid_stamp'] += 1
            return
        points = np.asarray(points, dtype=float).reshape((-1, 3))
        self.stats['scans_received'] += 1
        self.stats['input_points'] += len(points)
        finite = np.isfinite(points).all(axis=1)
        self.stats['filtered_nonfinite_points'] += int((~finite).sum())
        points = points[finite]
        radius = np.linalg.norm(points, axis=1)
        keep = (radius >= self.min_range) & (radius <= self.max_range)
        self.stats['filtered_range_points'] += int((~keep).sum())
        points = points[keep]
        if len(self.pending) >= self.queue_size:
            self.pending.popleft()
            self.stats['dropped_queue_full'] += 1
        self.pending.append((stamp, points, wall_time))

    def process_pending(self, wall_time):
        completed, waiting = [], deque()
        while self.pending:
            stamp, points, arrival = self.pending.popleft()
            pose, reason = self.history.lookup(stamp)
            if pose is None:
                if reason == 'waiting_for_odometry' and wall_time-arrival <= self.queue_timeout:
                    waiting.append((stamp, points, arrival))
                else:
                    self.stats['dropped_'+('pose_timeout' if reason == 'waiting_for_odometry' else reason)] += 1
                continue
            position, quaternion = pose
            world = transform_points(points, position, quaternion, self.sensor_offset, self.sensor_rpy)
            indices = np.unique(np.floor(world/self.resolution).astype(np.int64), axis=0)
            for key in map(tuple, indices.tolist()):
                self.voxels[key] = None
                self.voxels.move_to_end(key)
            while len(self.voxels) > self.max_voxels:
                self.voxels.popitem(last=False)
                self.stats['evicted_voxels'] += 1
            self.stats['scans_integrated'] += 1
            self.stats['valid_points'] += len(world)
            self.latest_stamp = max(stamp, self.latest_stamp)
            self.dirty = True
            completed.append((stamp, world))
        self.pending = waiting
        return completed

    def centers(self):
        keys = np.asarray(list(self.voxels), dtype=float).reshape((-1, 3))
        return (keys+.5)*self.resolution

    def status(self):
        return dict(self.stats, voxel_count=len(self.voxels), pending_scans=len(self.pending),
                    resolution_m=self.resolution, latest_scan_stamp=self.latest_stamp,
                    map_kind='accumulated_occupied_scan_endpoints', frame='world')


def main():
    import rospy
    from geometry_msgs.msg import Point
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud, PointCloud2
    from sensor_msgs import point_cloud2
    from std_msgs.msg import ColorRGBA, Header, String
    from visualization_msgs.msg import Marker

    rospy.init_node('lidar_voxel_map')
    get = rospy.get_param
    mapper = TemporalVoxelMap(
        resolution=float(get('~resolution', 1.)), max_voxels=int(get('~max_voxels', 100000)),
        history_seconds=float(get('~history_seconds', 3.)), max_pose_gap=float(get('~max_pose_gap', .15)),
        queue_size=int(get('~queue_size', 20)), queue_timeout=float(get('~queue_timeout', 3.)),
        min_range=float(get('~min_range', .4)), max_range=float(get('~max_range', 120.)),
        sensor_offset=get('~sensor_offset', [0., 0., .15]), sensor_rpy=get('~sensor_rpy', [0., 0., 0.]))
    expected_frame = str(get('~sensor_frame', 'suav1/lidar')).lstrip('/')
    world_frame = str(get('~world_frame', 'world'))
    map_hz = float(get('~map_publish_rate', 2.))
    color_min, color_max = float(get('~color_min_height', 0.)), float(get('~color_max_height', 150.))
    if not math.isfinite(map_hz) or map_hz <= 0 or color_max <= color_min:
        raise ValueError('Invalid publishing/color configuration')
    lock = threading.RLock()
    current_pub = rospy.Publisher('/suav1/lidar/points', PointCloud2, queue_size=1)
    map_pub = rospy.Publisher('/suav1/voxel_map', Marker, queue_size=1, latch=True)
    map_points_pub = rospy.Publisher('/suav1/voxel_map_points', PointCloud2, queue_size=1, latch=True)
    stats_pub = rospy.Publisher('/suav1/voxel_map/status', String, queue_size=1, latch=True)

    def publish_cloud(stamp, points):
        header = Header(stamp=rospy.Time.from_sec(stamp), frame_id=world_frame)
        current_pub.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32)))

    def process():
        for stamp, points in mapper.process_pending(time.monotonic()):
            publish_cloud(stamp, points)

    def odometry(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        try:
            with lock:
                reset = mapper.add_pose(message.header.stamp.to_sec(), [p.x,p.y,p.z], [q.x,q.y,q.z,q.w])
                if reset:
                    publish_cloud(message.header.stamp.to_sec(), np.empty((0,3)))
                process()
        except ValueError as exc:
            with lock:
                mapper.stats['dropped_invalid_odometry'] += 1
            rospy.logwarn_throttle(5., 'Invalid lidar odometry: %s', exc)

    def cloud(raw):
        message_type = raw._connection_header.get('type')
        try:
            if message_type == 'sensor_msgs/PointCloud':
                message = PointCloud().deserialize(raw._buff)
                points = np.array([(p.x,p.y,p.z) for p in message.points], dtype=float).reshape((-1,3))
            elif message_type == 'sensor_msgs/PointCloud2':
                message = PointCloud2().deserialize(raw._buff)
                points = np.array(list(point_cloud2.read_points(message, field_names=('x','y','z'), skip_nans=False)), dtype=float).reshape((-1,3))
            else:
                raise ValueError('Unsupported point cloud message: '+str(message_type))
            with lock:
                if message.header.frame_id.lstrip('/') != expected_frame:
                    mapper.stats['dropped_wrong_frame'] += 1
                    rospy.logwarn_throttle(5., 'Unexpected lidar frame: %s', message.header.frame_id)
                    return
                mapper.queue_cloud(message.header.stamp.to_sec(), points, time.monotonic())
                process()
        except (ValueError, TypeError) as exc:
            with lock:
                mapper.stats['dropped_invalid_cloud'] += 1
            rospy.logwarn_throttle(5., 'Invalid lidar cloud: %s', exc)

    subscribers = [rospy.Subscriber('/suav1/odom', Odometry, odometry, queue_size=500),
                   rospy.Subscriber('/suav1/lidar/points_raw', rospy.AnyMsg, cloud, queue_size=5, buff_size=8*1024*1024)]
    rospy.loginfo('Building %.2f m endpoint voxels from timestamp-aligned lidar returns', mapper.resolution)
    # Wall-clock scheduling stays responsive when Gazebo is paused or reset.
    try:
        while not rospy.is_shutdown():
            with lock:
                process()
                centers = mapper.centers() if mapper.dirty else None
                stamp = mapper.latest_stamp
                status = mapper.status()
                status['frame'] = world_frame
                mapper.dirty = False
            if centers is not None:
                header = Header(stamp=rospy.Time.from_sec(stamp), frame_id=world_frame)
                marker = Marker(header=header, ns='lidar_occupied_voxels', id=0, type=Marker.CUBE_LIST, action=Marker.ADD)
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = marker.scale.z = mapper.resolution
                marker.color.a = 1.
                marker.points = [Point(*xyz) for xyz in centers]
                heights = np.clip((centers[:,2]-color_min)/(color_max-color_min), 0., 1.)
                # Continuous blue -> cyan -> yellow -> red, with no plotting dependency.
                rgb = np.clip(np.array([1.5-abs(4*heights-3), 1.5-abs(4*heights-2), 1.5-abs(4*heights-1)]).T, 0., 1.)
                marker.colors = [ColorRGBA(r,g,b,.85) for r,g,b in rgb]
                map_pub.publish(marker)
                map_points_pub.publish(point_cloud2.create_cloud_xyz32(header, centers.astype(np.float32)))
            stats_pub.publish(String(data=json.dumps(status, sort_keys=True)))
            time.sleep(1./map_hz)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    main()
