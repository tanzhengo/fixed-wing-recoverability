#!/usr/bin/env python3
"""Run isolated sensor-map city trials with truth used only by the evaluator.

--generate-task PATH writes a geometry-free waypoint/limit snapshot without ROS.
Normal runs create their own snapshot and audit the planner's ROS parameters.
"""
import argparse
import copy
import gzip
import hashlib
import io
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import xmlrpc.client

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
TASK = REPO / 'motion_primitives_planning/launch/config/task_end_city_dense.yaml'
OBSTACLES = ('box_obstacles', 'static_obstacles', 'dynamic_obstacles',
             'sphere_obstacles', 'horizontal_obstacles')
WAITING = {'MAP_WAITING'}
GOOD = {'RECOVERY_VERIFIED', 'REVALIDATED_PLAN'}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sanitized_task(source):
    task = yaml.safe_load(Path(source).read_text())
    original = task['waypoints']
    if type(original['num']) is not int or original['num'] < 1:
        raise ValueError('Invalid waypoint count')
    result = {'motion_primitive': {}, 'waypoints': {'num': original['num']}}
    for name, value in task.get('motion_primitive', {}).items():
        if not isinstance(value, (int, float, bool)) or not math.isfinite(float(value)):
            raise ValueError('Only numeric primitive settings may enter the planner snapshot')
        result['motion_primitive'][name] = value
    for index in range(1, original['num'] + 1):
        name = 'wp' + str(index)
        row = {axis: original[name][axis] for axis in ('x', 'y', 'h')}
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(float(v)) for v in row.values()):
            raise ValueError('Invalid waypoint coordinate')
        result['waypoints'][name] = row
    result.update({name: {'num': 0} for name in OBSTACLES})
    return result


def assert_geometry_free(parameters):
    for name in OBSTACLES:
        values = parameters.get(name, {})
        if not isinstance(values, dict) or set(values) != {'num'} or type(values['num']) is not int or values['num'] != 0:
            raise ValueError('Planner received forbidden truth geometry: ' + name)
    if parameters.get('sensor_map', {}).get('enabled') is not True:
        raise ValueError('Sensor map is disabled in an unknown-map trial')


def active_statuses(statuses):
    result = []
    in_startup = True
    for status in statuses:
        if in_startup and status.get('status') in WAITING and not status.get('command_published'):
            continue
        in_startup = False
        result.append(status)
    return result


def json_scalar(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('Object of type ' + type(value).__name__ + ' is not JSON serializable')


def write_json(path, payload):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(payload, allow_nan=False, default=json_scalar) + '\n')
    temporary.replace(path)


def record_telemetry(output, truth, mode, record_rays=True, truth_mesh=None, mesh_margin=2.0, mesh_cap=5.0):
    from verify_end_world import BoxDistances, load_task
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    import rospy
    output.mkdir(parents=True, exist_ok=True)
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    deadline = time.monotonic() + 40
    while not stopped.is_set() and time.monotonic() < deadline:
        try:
            if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/unknown_trial_wait')[0] == 1:
                break
        except (OSError, xmlrpc.client.Error):
            pass
        stopped.wait(.2)
    rospy.init_node('unknown_trial_telemetry', anonymous=True, disable_signals=True)
    _, boxes, _ = load_task(truth)
    distance = BoxDistances(boxes)
    mesh = None
    if truth_mesh is not None:
        from collision_mesh_truth import MeshTruth
        mesh = MeshTruth(truth_mesh)
    previous_mesh_position = [None]
    rows = {'command': [], 'odom': [], 'status': [], 'sensor_map': [], 'rays': [], 'errors': []}
    latest = {'status': None, 'odom': None, 'first_command_time': None, 'first_sim_time': None,
              'parameter_audit_passed': False, 'parameter_audit_error': None}
    lock = threading.RLock()
    ray_file = gzip.open(output / 'rays.rosmsg.gz', 'wb', compresslevel=1) if record_rays else None

    def append(topic, row):
        row['receipt_sim_time'] = rospy.Time.now().to_sec()
        with lock:
            rows[topic].append(row)

    def flush_latest():
        with lock:
            write_json(output / 'latest.json', latest)

    def audit_parameters(name):
        parameters = rospy.get_param('/suav1', {})
        (output / name).write_text(yaml.safe_dump(parameters, sort_keys=True))
        try:
            if mode == 'unknown':
                assert_geometry_free(parameters)
            latest['parameter_audit_passed'] = True
        except ValueError as exc:
            latest['parameter_audit_error'] = str(exc)
            append('errors', {'error': str(exc), 'kind': 'parameter_audit'})

    def command(msg):
        append('command', {'roll_command': msg.linear.x, 'attack_command': msg.linear.y,
                           'thrust_command': msg.linear.z})

    def odom(msg):
        p = msg.pose.pose.position
        physical, expanded = distance.query((p.x, p.y, p.z))
        position = [p.x, p.y, p.z]
        mesh_interval = None
        if mesh is not None:
            mesh_interval = float(mesh.point_clearance(position, cap=mesh_cap) if previous_mesh_position[0] is None else
                                  mesh.segment_clearance(previous_mesh_position[0], position, cap=mesh_cap))
            previous_mesh_position[0] = position
        row = {'stamp': msg.header.stamp.to_sec(), 'x': p.x, 'y': p.y, 'z': p.z,
               'physical_clearance': physical, 'expanded_clearance': expanded,
               'mesh_interval_clearance': mesh_interval, 'mesh_margin': mesh_margin if mesh is not None else None,
               'mesh_clearance_cap': mesh_cap if mesh is not None else None}
        append('odom', row)
        with lock:
            latest['odom'] = row
            if latest['first_sim_time'] is None:
                latest['first_sim_time'] = row['stamp']
        flush_latest()

    def status(msg):
        try:
            payload = json.loads(msg.data)
            append('status', {'payload': payload})
            with lock:
                latest['status'] = payload
                if latest['first_command_time'] is None and payload.get('command_published'):
                    latest['first_command_time'] = rospy.Time.now().to_sec()
                if not latest['parameter_audit_passed'] and latest['parameter_audit_error'] is None:
                    audit_parameters('planner_parameters_initial.yaml')
            flush_latest()
        except (ValueError, TypeError) as exc:
            append('errors', {'kind': 'status_parse', 'error': str(exc)})

    def map_status(msg):
        try:
            append('sensor_map', {'payload': json.loads(msg.data)})
        except ValueError as exc:
            append('errors', {'kind': 'map_parse', 'error': str(exc)})

    def rays(msg):
        if stopped.is_set():
            return
        fields = {f.name: f for f in msg.fields}
        try:
            hit = np.ndarray((msg.height, msg.width), dtype=('>f4' if msg.is_bigendian else '<f4'),
                             buffer=msg.data, offset=fields['hit'].offset,
                             strides=(msg.row_step, msg.point_step))
            row = {'stamp': msg.header.stamp.to_sec(), 'frame': msg.header.frame_id,
                   'height': msg.height, 'width': msg.width, 'beam_count': msg.height * msg.width,
                   'hits': int(np.sum(hit == 1)), 'no_returns': int(np.sum(hit == 0)),
                   'invalid': int(np.sum(hit < 0)), 'sha256': hashlib.sha256(msg.data).hexdigest()}
            append('rays', row)
            if ray_file is not None:
                stream = io.BytesIO(); msg.serialize(stream); data = stream.getvalue()
                with lock:
                    if not stopped.is_set():
                        ray_file.write(struct.pack('<I', len(data))); ray_file.write(data)
        except (KeyError, ValueError, TypeError) as exc:
            append('errors', {'kind': 'ray_parse', 'error': str(exc)})

    subscribers = [rospy.Subscriber('/suav1/cmd_vel', Twist, command, queue_size=10000),
                   rospy.Subscriber('/suav1/odom', Odometry, odom, queue_size=10000),
                   rospy.Subscriber('/suav1/planning_status', String, status, queue_size=10000),
                   rospy.Subscriber('/suav1/sensor_map/status', String, map_status, queue_size=10000),
                   rospy.Subscriber('/suav1/lidar/rays', PointCloud2, rays, queue_size=100, buff_size=16*1024*1024)]
    while not stopped.is_set() and not rospy.is_shutdown():
        stopped.wait(.1)
    for sub in subscribers:
        sub.unregister()
    if ray_file is not None:
        with lock:
            ray_file.close()
    audit_parameters('planner_parameters_final.yaml')
    with lock:
        (output / 'telemetry.json.gz').write_bytes(gzip.compress(json.dumps(rows, allow_nan=False).encode(), mtime=0))
        flush_latest()


def audit_segments(output):
    trace = json.loads(gzip.decompress((output / 'flight/observations.json.gz').read_bytes()))
    task = yaml.safe_load((output / 'flight/task_config.yaml').read_text())
    config = task['box_obstacles']; names = ['box' + str(i) for i in range(1, config['num'] + 1)]
    centers = np.array([[config[name][a] for a in ('x', 'y', 'z')] for name in names])
    half = .5 * np.array([[config[name]['size_' + a] for a in ('x', 'y', 'z')] for name in names])
    margins = np.array([config[name].get('clearance', 1.0) for name in names])[:, None]
    result = {'method': 'slab intersection on linearly interpolated recorded odometry',
              'physical': {'count': 0, 'first': []}, 'expanded': {'count': 0, 'first': []}}
    rows = sorted(trace['odom'], key=lambda r: r['t'])
    for previous, current in zip(rows, rows[1:]):
        dt = current['t'] - previous['t']
        if dt <= 0:
            continue
        start = np.array([previous[a] for a in ('x', 'y', 'z')]); delta = np.array([current[a] for a in ('x', 'y', 'z')]) - start
        for label, sizes in [('physical', half), ('expanded', half + margins)]:
            low, high = centers-sizes, centers+sizes
            enter, leave = np.zeros(len(names)), np.ones(len(names)); valid = np.ones(len(names), bool)
            for axis in range(3):
                if abs(delta[axis]) < 1e-12:
                    valid &= (start[axis] >= low[:, axis]) & (start[axis] <= high[:, axis])
                else:
                    a, b = (low[:, axis]-start[axis])/delta[axis], (high[:, axis]-start[axis])/delta[axis]
                    enter = np.maximum(enter, np.minimum(a,b)); leave = np.minimum(leave, np.maximum(a,b))
            for i in np.flatnonzero(valid & (enter <= leave)):
                result[label]['count'] += 1
                if len(result[label]['first']) < 10:
                    result[label]['first'].append({'box': names[i], 'enter': previous['t'] + float(enter[i])*dt,
                                                   'leave': previous['t'] + float(leave[i])*dt})
    write_json(output / 'segment_audit.json', result)
    return result


def distribution(values):
    values = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(float(v))]
    return {'count': len(values), 'minimum': min(values) if values else None,
            'median': float(np.median(values)) if values else None,
            'p95': float(np.percentile(values, 95)) if values else None,
            'maximum': max(values) if values else None}


def convert_map_snapshot(output):
    source = output/'sensor_map.bin'
    if not source.exists():
        return None
    data = source.read_bytes()
    if len(data) < 40:
        raise ValueError('Truncated sensor map snapshot header')
    magic,resolution,stamp,nfree,noccupied = struct.unpack_from('<8sddQQ',data)
    if magic != b'SENMAP01' or len(data) != 40 + 12*(nfree+noccupied):
        raise ValueError('Sensor map snapshot signature or length mismatch')
    if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(stamp) or stamp <= 0:
        raise ValueError('Invalid map snapshot resolution/timestamp')
    points = np.frombuffer(data,dtype='<f4',offset=40).reshape((-1,3))
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite sensor map snapshot cells')
    np.savez_compressed(output/'map_snapshot.npz',free=points[:nfree],occupied=points[nfree:],
                        resolution=np.array([resolution]),stamp=np.array([stamp]))
    result={'free_cells':int(nfree),'occupied_cells':int(noccupied),'resolution_m':resolution,
            'stamp_s':stamp,'source_sha256':digest(source),'npz_sha256':digest(output/'map_snapshot.npz')}
    write_json(output/'map_snapshot.json',result)
    return result


def audit_mesh_segments(output, truth_mesh, margin, cap=30.0):
    from collision_mesh_truth import MeshTruth
    trace = json.loads(gzip.decompress((output/'flight/observations.json.gz').read_bytes()))
    rows = sorted(trace['odom'], key=lambda r: r['t'])
    mesh = MeshTruth(truth_mesh)
    minimum = float(cap)
    physical, margin_events = [], []
    physical_count = margin_count = 0
    segments = list(zip(rows, rows[1:])) if len(rows) > 1 else [(row, row) for row in rows]
    for previous, current in segments:
        a = [previous[k] for k in ('x','y','z')]; b = [current[k] for k in ('x','y','z')]
        distance = float(mesh.segment_clearance(a, b, cap=cap))
        minimum = min(minimum, distance)
        event = {'start_s': previous['t'], 'end_s': current['t'], 'clearance_m': distance}
        if distance <= 0:
            physical_count += 1
            if len(physical) < 20: physical.append(event)
        if distance <= margin:
            margin_count += 1
            if len(margin_events) < 20: margin_events.append(event)
    result = {'method':'exact recorded-position chord distance to exported physical collision triangles',
              'truth_mesh':str(truth_mesh),'truth_mesh_sha256':digest(truth_mesh),
              'margin_type':'isotropic distance to physical surfaces','margin_m':margin,
              'distance_cap_m':cap,'segment_count':len(segments),
              'minimum_surface_clearance_m':minimum if rows else None,
              'minimum_is_lower_bound':bool(rows and minimum >= cap - 1e-9),
              'physical_intersection_count':physical_count,'margin_violation_count':margin_count,
              'first_physical_intersections':physical,'first_margin_violations':margin_events}
    write_json(output/'mesh_audit.json',result)
    return result


def summarize(output, mode, stop_reason, truth_mesh=None, mesh_margin=2.0, mesh_cap=30.0):
    flight_path = output / 'flight/result.json'
    flight = json.loads(flight_path.read_text()) if flight_path.exists() else {}
    trace_path = output / 'flight/observations.json.gz'
    flight_trace = json.loads(gzip.decompress(trace_path.read_bytes())) if trace_path.exists() else {}
    telemetry_path = output / 'telemetry.json.gz'
    telemetry = json.loads(gzip.decompress(telemetry_path.read_bytes())) if telemetry_path.exists() else {}
    # Mission counts come from the flight observer, not the later shutdown grace.
    statuses = flight_trace.get('status', [])
    if not statuses:
        end = flight.get('last_sim_time', float('inf'))
        statuses = [r['payload'] for r in telemetry.get('status', []) if r['receipt_sim_time'] <= end]
    active = active_statuses(statuses)
    manifest_path = output/'run.json'
    expected_order = json.loads(manifest_path.read_text()).get('order') if manifest_path.exists() else None
    errors = list(telemetry.get('errors', []))
    parameter_audits = []
    for name in ['planner_parameters_initial.yaml', 'planner_parameters_final.yaml']:
        path = output / name
        if path.exists():
            params = yaml.safe_load(path.read_text())
            if mode == 'unknown':
                try:
                    assert_geometry_free(params)
                except ValueError as exc:
                    errors.append({'kind': 'parameter_audit', 'error': str(exc)})
            parameter_audits.append(name)
    execution_bad = [i for i, s in enumerate(active) if s.get('status') not in GOOD or
                     not s.get('primary_feasible') or not s.get('command_published') or
                     not (s.get('recovery_verified') or s.get('reused_verified_plan'))]
    map_bad = []
    if mode == 'unknown':
        map_bad = [i for i, s in enumerate(active) if s.get('sensor_map_enabled') is not True or
                   s.get('truth_obstacle_count') != 0 or s.get('map_ready') is not True or
                   s.get('sensor_map_current_state_free') is not True]
    order_bad = [i for i,s in enumerate(active) if expected_order is not None and s.get('candidate_order') != expected_order]
    invalid_updates = set(execution_bad) | set(map_bad) | set(order_bad)
    audit = audit_segments(output) if flight_path.exists() and trace_path.exists() else None
    mesh_audit = audit_mesh_segments(output, truth_mesh, mesh_margin, mesh_cap) if truth_mesh is not None and trace_path.exists() else None
    reached = flight.get('stop_reason') in {'completed_first_waypoint_cycle', 'single_goal_attained'}
    geometry_passed = bool(mesh_audit and mesh_audit['segment_count'] > 0 and mesh_audit['physical_intersection_count'] == 0 and mesh_audit['margin_violation_count'] == 0) if truth_mesh is not None else bool(
        flight.get('physical_box_intrusion_samples') == 0 and flight.get('inflated_box_intrusion_samples') == 0 and
        audit and audit['physical']['count'] == 0 and audit['expanded']['count'] == 0)
    mission_geometry_and_gate_passed = bool(reached and flight.get('odom_count', 0) > 0 and
        not flight.get('error') and not flight_trace.get('parse_errors') and
        flight.get('waypoint_gate_checks_passed') and geometry_passed)
    map_records = telemetry.get('sensor_map', [])
    result = {'mode': mode, 'supervisor_stop_reason': stop_reason, 'flight': flight,
              'raw_flight_observer_passed': flight.get('passed'),
              'mission_geometry_and_gate_passed': mission_geometry_and_gate_passed,
              'planning_updates': len(statuses), 'active_planning_updates': len(active),
              'telemetry_updates_including_shutdown_grace': len(telemetry.get('status', [])),
              'startup_wait_updates': len(statuses) - len(active),
              'active_rejection_count': len(invalid_updates),
              'active_execution_rejection_count': len(execution_bad),
              'active_map_contract_violation_count': len(map_bad),
              'requested_candidate_order': expected_order,
              'observed_candidate_orders': sorted({s.get('candidate_order','MISSING') for s in active}),
              'candidate_order_mismatch_count': len(order_bad),
              'map_ready_false_count': sum(s.get('map_ready') is not True for s in active) if mode == 'unknown' else 0,
              'current_state_not_free_count': sum(s.get('sensor_map_current_state_free') is not True for s in active) if mode == 'unknown' else 0,
              'truth_obstacle_nonzero_count': sum(s.get('truth_obstacle_count') != 0 for s in active) if mode == 'unknown' else 0,
              'map_age_s': distribution(s.get('map_age') for s in active) if mode == 'unknown' else None,
              'scan_to_map_receipt_latency_s': distribution(r['receipt_sim_time'] - r['payload']['last_stamp'] for r in map_records),
              'observation_to_command_s': distribution(s.get('command_stamp', 0) - s.get('observation_stamp', 0) for s in active if s.get('command_published')),
              'dispatch_validation_age_s': distribution(s.get('dispatch_validation_age') for s in active),
              'ray_scans': len(telemetry.get('rays', [])), 'map_status_updates': len(map_records),
              'telemetry_errors': errors, 'parameter_audits': parameter_audits, 'segment_audit': audit,
              'clearance_criterion': 'physical_collision_mesh_isotropic_margin' if truth_mesh is not None else 'axis_aligned_box_face_margin',
              'mesh_audit': mesh_audit}
    # Leading MAP_WAITING is a bounded initialization phase; it never grants a
    # command or excuses a physical/expanded-geometry incursion during startup.
    result['passed'] = bool(mission_geometry_and_gate_passed and active and not invalid_updates and
                            not errors and len(parameter_audits) == 2)
    write_json(output / 'summary.json', result)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path)
    p.add_argument('--truth-task', type=Path, default=TASK)
    p.add_argument('--generate-task', type=Path)
    p.add_argument('--mode', choices=['unknown', 'oracle'], default='unknown')
    p.add_argument('--world-name', type=Path, default=REPO/'simple-airplane/simple_airplane_gazebo/worlds/end_city_dense_lidar.world')
    p.add_argument('--truth-mesh', type=Path)
    p.add_argument('--mesh-margin', type=float, default=2.0)
    p.add_argument('--mesh-live-cap', type=float, default=5.0)
    p.add_argument('--mesh-audit-cap', type=float, default=30.0)
    p.add_argument('--summarize-only', type=Path)
    p.add_argument('--order', choices=['cost', 'diverse', 'learned', 'learned_diverse', 'learned_coverage', 'observed_coverage'], default='learned')
    p.add_argument('--model', type=Path, default=REPO/'docs/recovery/revision/training/state_joint/20261030/network_v1.txt')
    p.add_argument('--port', type=int, default=11618)
    p.add_argument('--primary-horizon', type=float, default=3.0)
    p.add_argument('--backup-horizon', type=float, default=3.0)
    p.add_argument('--resolution', type=float, default=2.0)
    p.add_argument('--clearance', type=float, default=2.0)
    p.add_argument('--map-max-age', type=float, default=1.0)
    p.add_argument('--map-max-cells', type=int, default=8000000)
    p.add_argument('--sim-seconds', type=float, default=180)
    p.add_argument('--wall-seconds', type=float, default=600)
    p.add_argument('--startup-sim-seconds', type=float, default=2.0)
    p.add_argument('--no-record-rays', action='store_true')
    p.add_argument('--continue-on-rejection', action='store_true')
    p.add_argument('--launch-arg', action='append', default=[])
    p.add_argument('--record-only', type=Path)
    args = p.parse_args()
    if args.summarize_only:
        manifest_path = args.summarize_only/'run.json'
        manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        truth_mesh = args.truth_mesh or (Path(manifest['truth_mesh']) if manifest.get('truth_mesh') else None)
        result = summarize(args.summarize_only, manifest.get('mode', args.mode), manifest.get('supervisor_stop_reason', 'posthoc_audit'), truth_mesh, args.mesh_margin, args.mesh_audit_cap)
        result['evaluation_script_sha256'] = digest(Path(__file__).resolve())
        result['source_unchanged'] = manifest.get('source_unchanged')
        result['passed'] = bool(result['passed'] and manifest.get('source_unchanged') is True)
        write_json(args.summarize_only/'summary.json', result)
        print(json.dumps(result, indent=2))
        return 0 if result['passed'] else 1
    if args.generate_task:
        args.generate_task.parent.mkdir(parents=True, exist_ok=True)
        args.generate_task.write_text(yaml.safe_dump(sanitized_task(args.truth_task), sort_keys=False))
        return 0
    if args.record_only:
        record_telemetry(args.record_only, args.truth_task, args.mode, not args.no_record_rays, args.truth_mesh, args.mesh_margin, args.mesh_live_cap)
        return 0
    if args.output is None:
        p.error('--output is required')
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        p.error('--output must be new or empty')
    output.mkdir(parents=True, exist_ok=True)
    for port in [args.port, args.port+1]:
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1', port)) == 0:
                p.error('Port already in use: ' + str(port))
    if args.truth_mesh is not None and not args.truth_mesh.is_file():
        p.error('--truth-mesh file is unavailable')
    if args.truth_mesh is not None and min(args.mesh_live_cap,args.mesh_audit_cap) <= args.mesh_margin:
        p.error('Mesh distance caps must exceed the evaluated margin')
    original = args.truth_task.read_bytes()
    planner_task = output / 'planner_task.yaml'
    task = sanitized_task(args.truth_task) if args.mode == 'unknown' else yaml.safe_load(original)
    planner_task.write_text(yaml.safe_dump(task, sort_keys=False))
    (output / 'truth_task_for_evaluation.yaml').write_bytes(original)
    env = os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:' + str(args.port),
        GAZEBO_MASTER_URI='http://127.0.0.1:' + str(args.port+1), GAZEBO_MODEL_DATABASE_URI='', ROS_LOG_DIR=str(output/'roslog'))
    common = ['--sim-seconds', str(args.sim_seconds), '--wall-seconds', str(args.wall_seconds), '--stop-after-cycle']
    commands = {
        'telemetry': ['python3', str(Path(__file__).resolve()), '--record-only', str(output), '--truth-task', str(args.truth_task), '--mode', args.mode] + (['--no-record-rays'] if args.no_record_rays else []),
        'mapping': ['python3', str(REPO/'docs/recovery/lidar_voxel_map/verify_lidar.py'), '--output-dir', str(output/'mapping'), '--resolution', '2.0'] + common,
        'flight': ['python3', str(REPO/'motion_primitives_planning/scripts/verify_end_world.py'), '--task-config', str(args.truth_task), '--output-dir', str(output/'flight')] + common,
        'launch': ['roslaunch', 'motion_primitives_planning', 'terminal_recovery_end_world_unknown.launch',
                   'task_config:=' + str(planner_task), 'world_name:=' + str(args.world_name.resolve()), 'gui:=false', 'rviz:=false', 'paused:=false',
                   'sensor_map_enabled:=' + str(args.mode == 'unknown').lower(), 'candidate_order:=' + args.order,
                   'model_path:=' + str(args.model), 'primary_horizon:=' + str(args.primary_horizon),
                   'backup_horizon:=' + str(args.backup_horizon), 'map_resolution:=' + str(args.resolution),
                   'map_clearance:=' + str(args.clearance), 'map_max_age:=' + str(args.map_max_age),
                   'map_max_cells:=' + str(args.map_max_cells),'map_snapshot_path:=' + str(output/'sensor_map.bin')] + args.launch_arg,
    }
    if args.truth_mesh is not None:
        commands['telemetry'] += ['--truth-mesh',str(args.truth_mesh),'--mesh-margin',str(args.mesh_margin),'--mesh-live-cap',str(args.mesh_live_cap)]
    sources = [args.truth_task, args.model, Path(__file__).resolve(),
               REPO/'motion_primitives_planning/launch/terminal_recovery_end_world_unknown.launch',
               REPO/'motion_primitives_planning/launch/terminal_recovery.launch',
               REPO/'motion_primitives_planning/launch/motion_primitives_test.launch',
               REPO/'simple-airplane/simple_airplane_description/urdf/simple_airplane_with_3d_lidar.xacro',
               args.world_name.resolve(),
               *sorted((REPO/'motion_primitives_planning/src').glob('*.h')),
               REPO/'simple-airplane/simple_airplane_plugins/src/gazebo_ros_3d_lidar.cpp',
               REPO/'simple-airplane/simple_airplane_plugins/src/lidar_ray_encoding.h']
    if args.truth_mesh is not None:
        sources += [args.truth_mesh,REPO/'motion_primitives_planning/scripts/collision_mesh_truth.py']
    hashes = {str(path): digest(path) for path in sources}
    manifest = {'commands': commands, 'mode': args.mode, 'order': args.order,
                'truth_available_to_planner': args.mode == 'oracle', 'source_sha256': hashes,
                'truth_mesh':str(args.truth_mesh) if args.truth_mesh is not None else None,
                'mesh_margin_m':args.mesh_margin if args.truth_mesh is not None else None,
                'planner_task_sha256': digest(planner_task), 'truth_task_sha256': digest(args.truth_task),
                'ray_record_format': 'gzip stream of little-endian uint32 length followed by serialized sensor_msgs/PointCloud2',
                'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
    binary_paths = [REPO.parents[1]/'devel/lib/motion_primitives_planning/motion_primitives_test',
                    REPO.parents[1]/'devel/lib/libgazebo_ros_3d_lidar.so']
    manifest['binary_sha256'] = {str(path): digest(path) for path in binary_paths if path.exists()}
    write_json(output/'run.json', manifest)
    processes, logs = {}, {};start = time.monotonic();stop_reason = 'wall_limit'
    try:
        for name, command in commands.items():
            logs[name] = (output/(name+'.log')).open('w')
            processes[name] = subprocess.Popen(command, env=env, stdout=logs[name], stderr=subprocess.STDOUT, start_new_session=True)
        while processes['flight'].poll() is None and time.monotonic()-start < args.wall_seconds+30:
            if processes['launch'].poll() is not None:
                stop_reason = 'launch_exit';break
            current = output/'latest.json'
            if current.exists():
                latest = json.loads(current.read_text());odom = latest.get('odom');status = latest.get('status')
                if latest.get('parameter_audit_error'):
                    stop_reason = 'truth_parameter_leak';break
                if odom and args.truth_mesh is not None and odom.get('mesh_interval_clearance') is not None and odom['mesh_interval_clearance'] <= args.mesh_margin:
                    stop_reason = 'physical_mesh_margin_incursion';break
                if odom and args.truth_mesh is None and odom['expanded_clearance'] is not None and odom['expanded_clearance'] <= 0:
                    stop_reason = 'expanded_geometry_incursion';break
                if status and not args.continue_on_rejection:
                    waiting = status.get('status') in WAITING and latest['first_command_time'] is None
                    if waiting and odom and odom['stamp']-latest['first_sim_time'] > args.startup_sim_seconds:
                        stop_reason = 'sensor_map_startup_timeout';break
                    if not waiting and status.get('primary_feasible') is False:
                        stop_reason = 'active_execution_rejection';break
                    if not waiting and status.get('command_published') and not (
                            status.get('status') in GOOD and status.get('primary_feasible') and
                            (status.get('recovery_verified') or status.get('reused_verified_plan'))):
                        stop_reason = 'unverified_control_publication';break
            time.sleep(.1)
        if processes['flight'].poll() is not None:
            stop_reason = 'flight_observer_complete'
        if stop_reason == 'flight_observer_complete' and processes['mapping'].poll() is None:
            try:processes['mapping'].wait(timeout=3)
            except subprocess.TimeoutExpired:pass
    finally:
        for name in ['telemetry','mapping','flight','launch']:
            process = processes.get(name)
            if process and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM);process.wait(timeout=8)
        for stream in logs.values():stream.close()
        manifest.update(ended_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()), elapsed_wall_s=time.monotonic()-start,
                        process_exit_codes={name:p.poll() for name,p in processes.items()},
                        source_unchanged=all(digest(path)==h for path,h in hashes.items()), supervisor_stop_reason=stop_reason)
        write_json(output/'run.json', manifest)
    snapshot = convert_map_snapshot(output)
    result = summarize(output, args.mode, stop_reason, args.truth_mesh, args.mesh_margin, args.mesh_audit_cap)
    result['map_snapshot'] = snapshot
    result['source_unchanged'] = manifest['source_unchanged'];result['passed'] = bool(result['passed'] and manifest['source_unchanged'])
    write_json(output/'summary.json', result)
    print(json.dumps({k:result[k] for k in ['passed','mode','supervisor_stop_reason','active_rejection_count','ray_scans']}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
