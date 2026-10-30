#!/usr/bin/env python3
"""Read-only ROS flight observer for an imported box-mapped Gazebo world.

Run after sourcing ROS and the workspace. Select an isolated ROS_MASTER_URI
outside this script, then start the scene separately. This observer neither
commands the aircraft nor starts/stops Gazebo. It can wait for a new ROS master.
"""
import argparse
import bisect
from collections import Counter
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import threading
import time
import xmlrpc.client

import numpy as np
import yaml


def number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + ' must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(name + ' must be finite')
    return result


def load_task(path):
    task = yaml.safe_load(Path(path).read_text())
    boxes = []
    config = task.get('box_obstacles', {})
    count = config.get('num', 0)
    if type(count) is not int or count < 0:
        raise ValueError('box_obstacles/num must be a nonnegative integer')
    for index in range(1, count + 1):
        name = 'box' + str(index)
        source = config[name]
        box = {key: number(source[key], name + '/' + key)
               for key in ('x', 'y', 'z', 'size_x', 'size_y', 'size_z')}
        box['clearance'] = number(source.get('clearance', 1.0), name + '/clearance')
        if min(box[key] for key in ('size_x', 'size_y', 'size_z')) <= 0 or box['clearance'] < 0:
            raise ValueError(name + ' has invalid dimensions or clearance')
        box['name'] = name
        boxes.append(box)
    wp = task['waypoints']
    if type(wp['num']) is not int or wp['num'] < 1:
        raise ValueError('waypoints/num must be a positive integer')
    goals = [tuple(number(wp['wp' + str(i)][key], 'waypoint/' + key)
                   for key in ('x', 'y', 'h')) for i in range(1, wp['num'] + 1)]
    return task, boxes, goals


class BoxDistances:
    def __init__(self, boxes):
        self.centers = np.array([[b[k] for k in ('x', 'y', 'z')] for b in boxes], dtype=float).reshape((-1, 3))
        self.half = np.array([[b[k] / 2 for k in ('size_x', 'size_y', 'size_z')] for b in boxes], dtype=float).reshape((-1, 3))
        self.margins = np.array([b['clearance'] for b in boxes], dtype=float)[:, None]

    def query(self, point):
        if not len(self.centers):
            return None, None
        q = np.abs(np.asarray(point) - self.centers) - self.half
        def minimum(delta):
            values = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(np.max(delta, axis=1), 0)
            return float(np.min(values))
        return minimum(q), minimum(q - self.margins)


def quaternion_roll_pitch(q):
    x, y, z, w = q
    return (math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
            math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))))


def same_goal(a, b):
    return all(abs(x-y) <= 1e-5 for x, y in zip(a, b))


def complete_cycle(transitions, goals):
    if len(goals) < 2 or len(transitions) < len(goals) + 1:
        return False
    observed = [(row['x'], row['y'], row['z']) for row in transitions]
    expected = goals + [goals[0]]
    return all(same_goal(a, b) for a, b in zip(observed[:len(expected)], expected))


def associate_transitions(transitions, odometry, horizontal_tolerance, height_tolerance):
    """Use the preceding odometry sample, without expanding either goal gate."""
    times = [row['t'] for row in odometry]
    result = []
    for index, transition in enumerate(transitions):
        item = dict(transition)
        sample_index = bisect.bisect_right(times, transition['t']) - 1
        if sample_index >= 0:
            sample = odometry[sample_index]
            item['aircraft'] = sample
            item['odometry_age_s'] = transition['t'] - sample['t']
            if index:
                previous = transitions[index-1]
                item['previous_goal'] = [previous[k] for k in ('x', 'y', 'z')]
                item['previous_goal_horizontal_error'] = math.hypot(sample['x']-previous['x'], sample['y']-previous['y'])
                item['previous_goal_height_error'] = abs(sample['z']-previous['z'])
                item['previous_goal_gate_passed'] = (item['previous_goal_horizontal_error'] <= horizontal_tolerance and
                                                     item['previous_goal_height_error'] <= height_tolerance)
        result.append(item)
    return result


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection


def wait_for_master(uri, deadline, interrupted):
    while time.monotonic() < deadline and not interrupted.is_set():
        try:
            proxy = xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport())
            if proxy.getPid('/end_world_observer_wait')[0] == 1:
                return True
        except (OSError, xmlrpc.client.Error, socket.timeout):
            pass
        interrupted.wait(.2)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--sim-seconds', type=float, default=180)
    parser.add_argument('--wall-seconds', type=float, default=300)
    parser.add_argument('--startup-seconds', type=float, default=30)
    parser.add_argument('--stop-after-cycle', action='store_true')
    args = parser.parse_args()
    if min(args.sim_seconds, args.wall_seconds, args.startup_seconds) <= 0:
        parser.error('All time limits must be positive')
    output = args.output_dir.expanduser().resolve()
    if output.exists() and any(output.iterdir()):
        parser.error('--output-dir must be empty or new')
    output.mkdir(parents=True, exist_ok=True)
    task, boxes, goals = load_task(args.task_config)
    distances = BoxDistances(boxes)
    interrupted = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _signum, _frame: interrupted.set())
    records = {'odom': [], 'goal': [], 'status': [], 'parse_errors': []}
    transitions = []
    lock = threading.Lock()
    subscriptions = []
    planner_params, recovery_params = {}, {}
    start = time.monotonic()
    stop = 'startup_timeout'
    error = None
    horizontal_tolerance = height_tolerance = None
    master = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    try:
        if not wait_for_master(master, start + args.startup_seconds, interrupted):
            raise TimeoutError('ROS master did not become available before startup timeout')
        import rospy
        from nav_msgs.msg import Odometry
        from visualization_msgs.msg import Marker
        from std_msgs.msg import String
        rospy.init_node('verify_end_world', anonymous=True, disable_signals=True)

        def odom(message):
            pos = message.pose.pose.position
            q = message.pose.pose.orientation
            roll, pitch = quaternion_roll_pitch((q.x, q.y, q.z, q.w))
            physical, inflated = distances.query((pos.x, pos.y, pos.z))
            row = {'t': message.header.stamp.to_sec(), 'x': pos.x, 'y': pos.y, 'z': pos.z,
                   'V': message.twist.twist.linear.x, 'gamma': message.twist.twist.linear.y,
                   'sigma': message.twist.twist.linear.z, 'roll': -roll,
                   'attack': -pitch-message.twist.twist.linear.y,
                   'T': message.twist.twist.angular.x, 'd_sigma': message.twist.twist.angular.y,
                   'physical_box_clearance': physical, 'inflated_box_clearance': inflated}
            with lock:
                records['odom'].append(row)

        def goal(message):
            pos = message.pose.position
            row = {'t': message.header.stamp.to_sec(), 'x': pos.x, 'y': pos.y, 'z': pos.z}
            with lock:
                records['goal'].append(row)
                if not transitions or not same_goal(tuple(row[k] for k in ('x','y','z')),
                                                     tuple(transitions[-1][k] for k in ('x','y','z'))):
                    transitions.append(row)

        def status(message):
            try:
                row = json.loads(message.data)
                row['t'] = rospy.Time.now().to_sec()
                with lock:
                    records['status'].append(row)
            except (ValueError, TypeError) as exc:
                with lock:
                    records['parse_errors'].append({'raw': message.data, 'error': str(exc)})

        subscriptions = [rospy.Subscriber('/suav1/odom', Odometry, odom, queue_size=1000),
                         rospy.Subscriber('/suav1/goal', Marker, goal, queue_size=1000),
                         rospy.Subscriber('/suav1/planning_status', String, status, queue_size=1000)]
        stop = 'wall_timeout'
        last_progress = start
        while time.monotonic()-start < args.wall_seconds and not interrupted.is_set():
            if rospy.is_shutdown():
                stop = 'ros_shutdown'
                break
            with lock:
                last = dict(records['odom'][-1]) if records['odom'] else None
                first = records['odom'][0]['t'] if records['odom'] else None
                sequence = list(transitions)
            if last and not planner_params:
                planner_params = rospy.get_param('/suav1/motion_primitive', {})
                recovery_params = rospy.get_param('/suav1/recovery', {})
                if 'V' not in planner_params or 'horizon_time' not in planner_params:
                    raise RuntimeError('Planner gate parameters are unavailable')
                horizontal_tolerance = planner_params['V'] * (planner_params['horizon_time'] + .15)
                height_tolerance = planner_params.get('waypoint_altitude_tolerance', 3.0)
            if args.stop_after_cycle and complete_cycle(sequence, goals):
                stop = 'completed_first_waypoint_cycle'
                break
            if args.stop_after_cycle and len(goals) == 1 and last:
                target = goals[0]
                if math.hypot(last['x']-target[0], last['y']-target[1]) <= horizontal_tolerance and abs(last['z']-target[2]) <= height_tolerance:
                    stop = 'single_goal_attained'
                    break
            if last and last['t']-first >= args.sim_seconds:
                stop = 'simulation_limit'
                break
            if time.monotonic()-last_progress >= 20:
                print(json.dumps({'sim_time': last['t'] if last else None,
                                  'goal_transitions': len(sequence),
                                  'altitude': last['z'] if last else None}), flush=True)
                last_progress = time.monotonic()
            interrupted.wait(.1)
        if interrupted.is_set():
            stop = 'interrupted'
    except Exception as exc:
        error = repr(exc)
        if stop != 'startup_timeout':
            stop = 'observer_error'
    finally:
        for sub in subscriptions:
            sub.unregister()
        with lock:
            trace = {key: list(value) for key, value in records.items()}
            sequence = list(transitions)
        od = sorted(trace['odom'], key=lambda row: row['t'])
        transitions_final = associate_transitions(sequence, od, horizontal_tolerance, height_tolerance) if horizontal_tolerance is not None else sequence
        def minimum(key):
            return min((row[key] for row in od if row[key] is not None), default=None)
        def extent(key):
            return [min((row[key] for row in od), default=None), max((row[key] for row in od), default=None)]
        def max_abs_degrees(key):
            return max((abs(row[key])*180/math.pi for row in od), default=None)
        gate_checks = [row.get('previous_goal_gate_passed', False) for row in transitions_final[1:]]
        reached = stop in ('completed_first_waypoint_cycle', 'single_goal_attained')
        clearance = minimum('inflated_box_clearance')
        statuses = trace['status']
        result = {
            'stop_reason': stop, 'error': error, 'wall_seconds': time.monotonic()-start,
            'first_sim_time': od[0]['t'] if od else None, 'last_sim_time': od[-1]['t'] if od else None,
            'odom_count': len(od), 'status_count': len(statuses), 'last_odom': od[-1] if od else None,
            'goal_transitions': transitions_final, 'waypoints': goals,
            'waypoint_gate_horizontal_m': horizontal_tolerance, 'waypoint_gate_height_m': height_tolerance,
            'waypoint_gate_checks_passed': bool(gate_checks) and all(gate_checks) if len(goals)>1 else reached,
            'status_counts': dict(Counter(row.get('status', 'MISSING') for row in statuses)),
            'primary_infeasible_status_count': sum(row.get('primary_feasible') is False for row in statuses),
            'minimum_physical_box_clearance_m': minimum('physical_box_clearance'),
            'minimum_inflated_box_clearance_m': clearance,
            'physical_box_intrusion_samples': sum(row['physical_box_clearance'] is not None and row['physical_box_clearance']<=0 for row in od),
            'inflated_box_intrusion_samples': sum(row['inflated_box_clearance'] is not None and row['inflated_box_clearance']<=0 for row in od),
            'altitude_min_max_m': extent('z'), 'speed_min_max_m_s': extent('V'),
            'max_abs_gamma_deg': max_abs_degrees('gamma'), 'max_abs_roll_deg': max_abs_degrees('roll'),
            'planner_parameters': planner_params, 'recovery_parameters': recovery_params,
            'box_count': len(boxes), 'ros_master_uri': master,
            'task_config_sha256': hashlib.sha256(args.task_config.read_bytes()).hexdigest(),
            'observer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'point_clearance_note': 'Distances are sampled aircraft reference-point distances to the imported planning boxes, not swept aircraft mesh collision measurements.',
            'gate_sampling_note': 'Goal transition checks use the latest preceding recorded odometry without enlarging the planner gate; message timing can differ from the exact state consumed by the planner.',
            'cli': {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        }
        result['passed'] = bool(reached and od and statuses and not error and not trace['parse_errors'] and
                                (clearance is None or clearance > 0) and result['waypoint_gate_checks_passed'])
        raw = (json.dumps(trace, allow_nan=False)+'\n').encode()
        (output/'observations.json.gz').write_bytes(gzip.compress(raw, mtime=0))
        (output/'task_config.yaml').write_bytes(args.task_config.read_bytes())
        (output/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(result, indent=2, allow_nan=False), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
