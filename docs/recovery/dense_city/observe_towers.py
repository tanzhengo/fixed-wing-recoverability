#!/usr/bin/env python3
"""Read-only validation of loaded towers and measured lidar facade returns."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
import xmlrpc.client

import numpy as np


def xyz(message):
    """Decode XYZ without a per-point Python loop, including padded rows."""
    fields = {f.name: f for f in message.fields}
    formats, offsets = [], []
    endian = '>' if message.is_bigendian else '<'
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('XYZ fields must be scalar FLOAT32/FLOAT64')
        formats.append(endian + ('f4' if field.datatype == 7 else 'f8'))
        offsets.append(field.offset)
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                      'offsets': offsets, 'itemsize': message.point_step})
    if not message.width or not message.height:
        return np.empty((0, 3), dtype=float)
    values = np.ndarray((message.height, message.width), dtype=dtype,
                        buffer=message.data, strides=(message.row_step, message.point_step))
    return np.column_stack([values[name].ravel() for name in ('x', 'y', 'z')])


def surface_hits(points, center, half, tolerance):
    """Count finite points near the finite four facades or roof, not ground."""
    delta = points - center
    inside = np.all(np.abs(delta) <= half + tolerance, axis=1)
    inside &= points[:, 2] > tolerance
    local = delta[inside]
    if not len(local):
        return 0
    residual = np.minimum(np.minimum(np.abs(np.abs(local[:, 0]) - half[0]),
                                    np.abs(np.abs(local[:, 1]) - half[1])),
                          np.abs(local[:, 2] - half[2]))
    return int(np.count_nonzero(residual <= tolerance))


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--buildings', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--wall-seconds', type=float, default=420.)
    parser.add_argument('--sim-seconds', type=float, default=180.)
    parser.add_argument('--startup-seconds', type=float, default=30.)
    parser.add_argument('--surface-tolerance', type=float, default=.08)
    parser.add_argument('--pose-tolerance', type=float, default=1e-5)
    parser.add_argument('--waypoint-count', type=int, default=8)
    parser.add_argument('--stop-after-cycle', action='store_true', default=True)
    parser.add_argument('--no-stop-after-cycle', action='store_false', dest='stop_after_cycle')
    args = parser.parse_args()
    for name in ('wall_seconds', 'sim_seconds', 'startup_seconds', 'surface_tolerance', 'pose_tolerance'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(name + ' must be finite and positive')
    if args.waypoint_count < 2:
        parser.error('--waypoint-count must be at least two')
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error('--output-dir must be empty or new')
    buildings = json.loads(args.buildings.read_text())['buildings']
    if not buildings or len({b['name'] for b in buildings}) != len(buildings):
        parser.error('Building names must be nonempty and unique')
    centers = [np.array([b['x'], b['y'], b['height']/2], dtype=float) for b in buildings]
    halves = [np.array([b['size_x'], b['size_y'], b['height']], dtype=float)/2 for b in buildings]
    if not all(np.isfinite(c).all() and np.isfinite(h).all() and np.all(h > 0)
               for c, h in zip(centers, halves)):
        parser.error('Invalid building geometry')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = {b['name']: {'expected_position_m': c.tolist(), 'size_m': (2*h).tolist(),
                       'pose_samples': 0, 'maximum_position_error_m': 0.,
                       'maximum_identity_rotation_error': 0., 'surface_return_count': 0,
                       'scans_with_returns': 0, 'first_return_stamp': None, 'last_return_stamp': None}
            for b, c, h in zip(buildings, centers, halves)}
    lock = threading.RLock()
    interrupted = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: interrupted.set())
    stats = {'model_state_messages': 0, 'world_scan_messages': 0, 'finite_points': 0,
             'nonfinite_points': 0, 'first_scan_stamp': None, 'last_scan_stamp': None,
             'invalid_frames': 0}
    goals, errors, subscribers = [], [], []
    master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    start = time.monotonic()
    stop, error = 'startup_timeout', None
    try:
        while time.monotonic() - start < args.startup_seconds and not interrupted.is_set():
            try:
                if xmlrpc.client.ServerProxy(master, transport=TimeoutTransport()).getPid('/observe_towers_wait')[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error, socket.timeout):
                pass
            interrupted.wait(.2)
        else:
            raise TimeoutError('ROS master was not available')
        import rospy
        from gazebo_msgs.msg import ModelStates
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import Marker
        rospy.init_node('observe_dense_city_towers', anonymous=True, disable_signals=True)

        def models(message):
            indices = {name: i for i, name in enumerate(message.name)}
            with lock:
                stats['model_state_messages'] += 1
                for building, center in zip(buildings, centers):
                    name = building['name']
                    if name not in indices:
                        continue
                    pose = message.pose[indices[name]]
                    position = np.array([pose.position.x, pose.position.y, pose.position.z])
                    q = pose.orientation
                    row = rows[name]
                    row['pose_samples'] += 1
                    row['maximum_position_error_m'] = max(row['maximum_position_error_m'], float(np.linalg.norm(position-center)))
                    rotation_error = max(abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w)-1.))
                    row['maximum_identity_rotation_error'] = max(row['maximum_identity_rotation_error'], rotation_error)

        def scan(message):
            try:
                stamp = message.header.stamp.to_sec()
                points = xyz(message)
                finite = np.isfinite(points).all(axis=1)
                points = points[finite]
                valid_frame = message.header.frame_id.lstrip('/') == 'world'
                counts = [surface_hits(points, c, h, args.surface_tolerance) for c, h in zip(centers, halves)] if valid_frame else [0]*len(rows)
                with lock:
                    stats['world_scan_messages'] += 1
                    stats['finite_points'] += len(points)
                    stats['nonfinite_points'] += int((~finite).sum())
                    stats['invalid_frames'] += int(not valid_frame)
                    if stats['first_scan_stamp'] is None:
                        stats['first_scan_stamp'] = stamp
                    stats['last_scan_stamp'] = stamp
                    for building, count in zip(buildings, counts):
                        if not count:
                            continue
                        row = rows[building['name']]
                        row['surface_return_count'] += count
                        row['scans_with_returns'] += 1
                        if row['first_return_stamp'] is None:
                            row['first_return_stamp'] = stamp
                        row['last_return_stamp'] = stamp
            except Exception as exc:
                with lock:
                    errors.append({'topic': '/suav1/lidar/points', 'error': repr(exc)})

        def goal(message):
            p = message.pose.position
            target = [p.x, p.y, p.z]
            with lock:
                if not goals or goals[-1]['target'] != target:
                    goals.append({'stamp': message.header.stamp.to_sec(), 'target': target})

        subscribers = [rospy.Subscriber('/gazebo/model_states', ModelStates, models, queue_size=1),
                       rospy.Subscriber('/suav1/lidar/points', PointCloud2, scan, queue_size=4, buff_size=8*1024*1024),
                       rospy.Subscriber('/suav1/goal', Marker, goal, queue_size=100)]
        stop = 'wall_timeout'
        while time.monotonic()-start < args.wall_seconds and not interrupted.is_set():
            if rospy.is_shutdown():
                stop = 'ros_shutdown'; break
            with lock:
                complete = (len(goals) >= args.waypoint_count+1 and goals[0]['target'] == goals[-1]['target'])
                elapsed = ((stats['last_scan_stamp']-stats['first_scan_stamp'])
                           if stats['first_scan_stamp'] is not None else 0.)
            if args.stop_after_cycle and complete:
                stop = 'completed_waypoint_cycle'; break
            if elapsed >= args.sim_seconds:
                stop = 'simulation_limit'; break
            interrupted.wait(.1)
        if interrupted.is_set():
            stop = 'interrupted'
    except Exception as exc:
        error = repr(exc)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
    with lock:
        checks = {'all_models_loaded': all(r['pose_samples'] > 0 for r in rows.values()),
                  'all_model_poses_match': all(r['pose_samples'] > 0 and r['maximum_position_error_m'] <= args.pose_tolerance and r['maximum_identity_rotation_error'] <= args.pose_tolerance for r in rows.values()),
                  'all_towers_scanned': all(r['surface_return_count'] > 0 for r in rows.values()),
                  'world_frame': stats['world_scan_messages'] > 0 and stats['invalid_frames'] == 0,
                  'finite_clouds': stats['nonfinite_points'] == 0,
                  'no_errors': error is None and not errors}
        result = {'passed': all(checks.values()), 'checks': checks, 'stop_reason': stop, 'error': error,
                  'wall_seconds': time.monotonic()-start, 'ros_master_uri': master,
                  'buildings_sha256': hashlib.sha256(args.buildings.read_bytes()).hexdigest(),
                  'observer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  'surface_tolerance_m': args.surface_tolerance, 'pose_tolerance_m': args.pose_tolerance,
                  'surface_definition': 'Finite AABB four facades and roof; all three coordinates must lie within expanded bounds. Bottom face and z <= tolerance are excluded.',
                  'pose_definition': 'Gazebo model pose must equal (x, y, height/2) with identity rotation.',
                  'stats': stats, 'buildings': rows, 'goals': goals, 'callback_errors': errors}
    (args.output_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    sys.exit(main())
