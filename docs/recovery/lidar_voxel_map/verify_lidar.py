#!/usr/bin/env python3
"""Read-only lidar/voxel integration observer. Does not command Gazebo or aircraft."""
import argparse
from collections import Counter
import gzip
import json
import math
from pathlib import Path
import signal
import threading
import time
import xmlrpc.client
import os

import numpy as np


def xyz(message):
    from sensor_msgs import point_cloud2
    return np.asarray(list(point_cloud2.read_points(message, field_names=('x','y','z'), skip_nans=False)), dtype=np.float32).reshape((-1,3))


def summary_rate(records):
    stamps = np.asarray([r['stamp'] for r in records], dtype=float)
    dt = np.diff(stamps)
    positive = dt[dt > 0]
    return {'messages':len(records), 'first_stamp':float(stamps[0]) if len(stamps) else None,
            'last_stamp':float(stamps[-1]) if len(stamps) else None,
            'mean_hz_sim':float(1/np.mean(positive)) if len(positive) else None,
            'median_hz_sim':float(1/np.median(positive)) if len(positive) else None,
            'nonpositive_stamp_steps':int(np.sum(dt <= 0))}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--sim-seconds', type=float, default=90.)
    p.add_argument('--wall-seconds', type=float, default=360.)
    p.add_argument('--startup-seconds', type=float, default=40.)
    p.add_argument('--resolution', type=float, default=1.)
    p.add_argument('--max-range', type=float, default=120.)
    p.add_argument('--stop-after-cycle', action='store_true')
    args=p.parse_args()
    if args.output_dir.exists() and any(args.output_dir.iterdir()): p.error('Output directory is not empty')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    lock=threading.RLock(); interrupted=threading.Event()
    for s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,lambda *_:interrupted.set())
    records={'raw':[],'world':[],'map':[],'marker':[],'odom':[],'goals':[],'status':[],'errors':[]}
    snapshots={}; start=time.monotonic(); stop='startup_timeout';error=None
    topic_names={'raw':'/suav1/lidar/points_raw','world':'/suav1/lidar/points','map':'/suav1/voxel_map_points',
                 'marker':'/suav1/voxel_map','status':'/suav1/voxel_map/status'}
    master=os.environ.get('ROS_MASTER_URI','http://localhost:11311')
    try:
        import socket
        class TimeoutTransport(xmlrpc.client.Transport):
            def make_connection(self, host):
                c=super().make_connection(host);c.timeout=1.;return c
        deadline=start+args.startup_seconds
        while time.monotonic()<deadline and not interrupted.is_set():
            try:
                if xmlrpc.client.ServerProxy(master,transport=TimeoutTransport()).getPid('/lidar_validation')[0]==1:break
            except (OSError,xmlrpc.client.Error,socket.timeout):pass
            interrupted.wait(.2)
        else:raise TimeoutError('ROS master not available')
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from visualization_msgs.msg import Marker
        from std_msgs.msg import String
        rospy.init_node('verify_lidar_voxels',anonymous=True,disable_signals=True)
        def cloud_callback(kind):
            def callback(msg):
                try:
                    points=xyz(msg);finite=np.isfinite(points).all(axis=1);valid=points[finite]
                    row={'stamp':msg.header.stamp.to_sec(),'frame':msg.header.frame_id,'count':len(points),'nonfinite':int((~finite).sum()),
                         'min':valid.min(0).tolist() if len(valid) else None,'max':valid.max(0).tolist() if len(valid) else None}
                    if kind=='raw' and len(valid):
                        radius=np.linalg.norm(valid,axis=1);elevation=np.arctan2(valid[:,2],np.hypot(valid[:,0],valid[:,1]))
                        row.update(min_range=float(radius.min()),max_range=float(radius.max()),outside_range=int(np.sum(radius>args.max_range+.001)),
                                   near_max_sphere=int(np.sum(np.abs(radius-args.max_range)<=.01)),vertical_span=float(np.ptp(valid[:,2])),
                                   elevation_bins=int(len(np.unique(np.round(elevation,5)))))
                    if kind=='world' and len(valid):
                        row.update(ground_near_zero_count=int(np.sum(np.abs(valid[:,2])<.3)),below_ground_count=int(np.sum(valid[:,2]<-1.)),
                                   closest_abs_ground_z=float(np.min(np.abs(valid[:,2]))))
                    if kind=='map' and len(valid):
                        scaled=valid/args.resolution-.5
                        row['max_grid_alignment_error']=float(np.max(np.abs(scaled-np.rint(scaled))))
                    with lock:
                        records[kind].append(row)
                        snapshots[kind]=valid.copy()
                except Exception as exc:
                    with lock:records['errors'].append({'topic':kind,'error':repr(exc)})
            return callback
        def marker(msg):
            with lock:
                records['marker'].append({'stamp':msg.header.stamp.to_sec(),'frame':msg.header.frame_id,'type':int(msg.type),
                    'count':len(msg.points),'color_count':len(msg.colors),'scale':[msg.scale.x,msg.scale.y,msg.scale.z],
                    'pose_identity':bool(abs(msg.pose.orientation.w-1)<1e-6 and abs(msg.pose.orientation.x)+abs(msg.pose.orientation.y)+abs(msg.pose.orientation.z)<1e-6 and abs(msg.pose.position.x)+abs(msg.pose.position.y)+abs(msg.pose.position.z)<1e-6)})
        def odom(msg):
            p=msg.pose.pose.position
            with lock:records['odom'].append({'stamp':msg.header.stamp.to_sec(),'x':p.x,'y':p.y,'z':p.z})
        def goal(msg):
            p=msg.pose.position;target=[p.x,p.y,p.z]
            with lock:
                if not records['goals'] or records['goals'][-1]['goal']!=target:
                    records['goals'].append({'stamp':msg.header.stamp.to_sec(),'goal':target})
        def status(msg):
            try:
                row=json.loads(msg.data);row['received_wall_seconds']=time.monotonic()-start
                with lock:records['status'].append(row)
            except (ValueError,TypeError) as exc:
                with lock:records['errors'].append({'topic':'status','error':repr(exc)})
        subscribers=[rospy.Subscriber(topic_names[k],PointCloud2,cloud_callback(k),queue_size=2,buff_size=16*1024*1024) for k in ('raw','world','map')]
        subscribers.extend([rospy.Subscriber(topic_names['marker'],Marker,marker,queue_size=2,buff_size=32*1024*1024),
            rospy.Subscriber(topic_names['status'],String,status,queue_size=20),rospy.Subscriber('/suav1/odom',Odometry,odom,queue_size=1000),
            rospy.Subscriber('/suav1/goal',Marker,goal,queue_size=100)])
        stop='wall_timeout';last_progress=start
        while time.monotonic()-start<args.wall_seconds and not interrupted.is_set():
            if rospy.is_shutdown():stop='ros_shutdown';break
            with lock:
                elapsed=(records['odom'][-1]['stamp']-records['odom'][0]['stamp']) if records['odom'] else 0
                cycle=(len(records['goals'])>=6 and records['goals'][0]['goal']==records['goals'][-1]['goal'])
                last_map=records['map'][-1]['count'] if records['map'] else 0
            if args.stop_after_cycle and cycle:stop='completed_waypoint_cycle';break
            if elapsed>=args.sim_seconds:stop='simulation_limit';break
            if time.monotonic()-last_progress>=15:
                print(json.dumps({'elapsed_sim_seconds':elapsed,'raw_messages':len(records['raw']),'world_messages':len(records['world']),
                                  'voxel_count':last_map,'goal_transitions':len(records['goals'])}),flush=True);last_progress=time.monotonic()
            interrupted.wait(.1)
        if interrupted.is_set():stop='interrupted'
        for sub in subscribers:sub.unregister()
    except Exception as exc:
        error=repr(exc)
        if stop!='startup_timeout':stop='observer_error'
    with lock:
        trace={k:list(v) for k,v in records.items()};snaps={k:v.copy() for k,v in snapshots.items()}
    raw=trace['raw'];world=trace['world'];maps=trace['map'];markers=trace['marker']
    raw_points=sum(r['count'] for r in raw);near_max=sum(r.get('near_max_sphere',0) for r in raw)
    checks={
        'raw_nonempty':raw_points>1000 and len(raw)>=5,
        'raw_frame':bool(raw) and all(r['frame'].lstrip('/')=='suav1/lidar' for r in raw),
        'raw_finite_in_range':bool(raw) and sum(r['nonfinite']+r.get('outside_range',0) for r in raw)==0,
        'raw_is_3d':bool(raw) and max(r.get('elevation_bins',0) for r in raw)>=4 and max(r.get('vertical_span',0) for r in raw)>2,
        'no_max_range_shell':bool(raw_points) and near_max/max(raw_points,1)<.02,
        'world_frame':bool(world) and all(r['frame']=='world' for r in world),
        'world_nonempty':sum(r['count'] for r in world)>1000,
        'voxel_map_nonempty':bool(maps) and maps[-1]['count']>100,
        'voxel_grid_alignment':bool(maps) and max((r.get('max_grid_alignment_error',0) for r in maps),default=1)<1e-4,
        'voxel_marker_valid':bool(markers) and all(r['type']==6 and r['frame']=='world' and r['count']==r['color_count'] and r['pose_identity'] and max(abs(v-args.resolution) for v in r['scale'])<1e-6 for r in markers),
        'map_grew':bool(maps) and max(r['count'] for r in maps)>min(r['count'] for r in maps),
        'no_observer_errors':not error and not trace['errors']}
    result={'stop_reason':stop,'error':error,'wall_seconds':time.monotonic()-start,'ros_master_uri':master,'topics':topic_names,
        'resolution_m':args.resolution,'range_max_m':args.max_range,'checks':checks,'passed':all(checks.values()),
        'raw':summary_rate(raw),'world':summary_rate(world),'map':summary_rate(maps),'marker':summary_rate(markers),
        'raw_point_total':raw_points,'raw_points_per_scan_median':float(np.median([r['count'] for r in raw])) if raw else None,
        'raw_elevation_bins_max':max((r.get('elevation_bins',0) for r in raw),default=0),
        'raw_vertical_span_max_m':max((r.get('vertical_span',0) for r in raw),default=0),
        'raw_near_max_range_fraction':near_max/max(raw_points,1),'world_ground_near_zero_points':sum(r.get('ground_near_zero_count',0) for r in world),
        'world_below_minus_one_m_points':sum(r.get('below_ground_count',0) for r in world),
        'final_voxel_count':maps[-1]['count'] if maps else 0,'maximum_voxel_count':max((r['count'] for r in maps),default=0),
        'map_counts_non_decreasing':all(b['count']>=a['count'] for a,b in zip(maps,maps[1:])),
        'goals':trace['goals'],'final_mapper_status':trace['status'][-1] if trace['status'] else None,
        'cloud_origin':'Gazebo ray sensor returns; this observer never reads the known obstacle map',
        'ground_note':'Near-zero and below-ground counts are diagnostic distributions; not all lidar returns are ground points.'}
    (args.output_dir/'observations.json.gz').write_bytes(gzip.compress(json.dumps(trace).encode(),mtime=0))
    np.savez_compressed(args.output_dir/'snapshots.npz',**snaps,
                        odometry=np.array([[r[k] for k in ('stamp','x','y','z')] for r in trace['odom']],dtype=float).reshape((-1,4)),
                        resolution=np.array([args.resolution]))
    (args.output_dir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
