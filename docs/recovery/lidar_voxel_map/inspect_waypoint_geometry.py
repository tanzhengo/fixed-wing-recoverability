#!/usr/bin/env python3
"""Inspect waypoint and straight-segment box distances, independent of flight traces."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--maximum-step',type=float,default=.25)
    args=p.parse_args()
    if not math.isfinite(args.maximum_step) or args.maximum_step<=0:p.error('--maximum-step must be positive')
    task=yaml.safe_load(args.config.read_text());config=task['box_obstacles']
    boxes=[config['box'+str(i)] for i in range(1,config['num']+1)]
    centers=np.asarray([[b[k] for k in ('x','y','z')] for b in boxes],dtype=float)
    half=np.asarray([[b[k]/2 for k in ('size_x','size_y','size_z')] for b in boxes],dtype=float)
    margins=np.asarray([b.get('clearance',1.) for b in boxes],dtype=float)[:,None]
    wp=task['waypoints'];goals=np.asarray([[wp['wp'+str(i)][k] for k in ('x','y','h')] for i in range(1,wp['num']+1)],dtype=float)
    def distances(point,inflated=True):
        q=np.abs(np.asarray(point)-centers)-half-(margins if inflated else 0.)
        values=np.linalg.norm(np.maximum(q,0.),axis=1)+np.minimum(np.max(q,axis=1),0.)
        closest=int(np.argmin(values))
        return float(values[closest]),closest
    points=[]
    for i,goal in enumerate(goals,1):
        value,index=distances(goal)
        physical,physical_index=distances(goal,False)
        points.append({'waypoint_index':i,'position_m':goal.tolist(),'new_middle_waypoint':i in (4,5,6),
                       'inflated_box_clearance_m':value,'nearest_inflated_box':'box'+str(index+1),
                       'nearest_inflated_box_source':boxes[index].get('source'),
                       'physical_box_clearance_m':physical,'nearest_physical_box_source':boxes[physical_index].get('source')})
    segments=[]
    for i in range(len(goals)):
        a,b=goals[i],goals[(i+1)%len(goals)];length=float(np.linalg.norm(b-a));count=max(1,int(math.ceil(length/args.maximum_step)))
        samples=np.linspace(a,b,count+1);measurements=[distances(point) for point in samples]
        best=min(range(len(measurements)),key=lambda n:measurements[n][0]);value,index=measurements[best];step=length/count
        segments.append({'from_waypoint':i+1,'to_waypoint':(i+1)%len(goals)+1,
                         'touches_inserted_middle_route':i in (2,3,4,5),
                         'length_3d_m':length,'horizontal_length_m':float(np.linalg.norm((b-a)[:2])),
                         'sample_count':len(samples),'actual_sample_spacing_m':step,
                         'minimum_sampled_inflated_clearance_m':value,'minimum_sample_position_m':samples[best].tolist(),
                         'nearest_box':'box'+str(index+1),'nearest_box_source':boxes[index].get('source'),
                         'straight_segment_clearance_lower_bound_m':value-step/2})
    result={'scope':'Waypoints and straight geometric centerlines only; these values are not measured flight or curved-trajectory clearances.',
            'config_path':str(args.config),'config_sha256':hashlib.sha256(args.config.read_bytes()).hexdigest(),
            'box_count':len(boxes),'configured_per_face_box_margins_m':sorted(set(float(b.get('clearance',1.)) for b in boxes)),
            'new_middle_waypoint_indices':[4,5,6],'initial_position_m':[-90.,-270.,40.],
            'loop_order':[i for i in range(1,len(goals)+1)]+[1],
            'maximum_sample_spacing_m':args.maximum_step,
            'algorithm':'At each sample, use exact Euclidean signed distance to each axis-aligned physical box expanded by its configured per-face clearance, then take the minimum. Each 3D segment includes both endpoints and is sampled uniformly with spacing <= maximum_sample_spacing_m. Altitude is linearly interpolated.',
            'continuous_straight_segment_bound':'Signed distance is 1-Lipschitz; subtract half the actual sample spacing from the minimum sampled value to bound every point on that straight segment.',
            'waypoints':points,'cycle_segments':segments}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'output':str(args.output),'config_sha256':result['config_sha256'],
                      'new_points':[{k:p[k] for k in ('waypoint_index','position_m','inflated_box_clearance_m')} for p in points if p['new_middle_waypoint']],
                      'new_segment_minimums':[s['minimum_sampled_inflated_clearance_m'] for s in segments if s['touches_inserted_middle_route']]}))


if __name__=='__main__':main()
