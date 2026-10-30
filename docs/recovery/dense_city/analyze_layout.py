from pathlib import Path
import json,gzip,yaml,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
root=Path(__file__).resolve().parents[2];r=root/'outputs/two_phase_ws/src/Two-phase-motion-planning';out=Path(__file__).parent
config=r/'motion_primitives_planning/launch/config/task_end_city.yaml';task=yaml.safe_load(config.read_text());bc=task['box_obstacles'];boxes=[dict(bc['box'+str(i)],index=i) for i in range(1,bc['num']+1)]
# Skip the terrain footprint only: all non-terrain boxes enter the checks,
# including small trees/props. The terrain overlaps every building foundation.
obstacles=[b for b in boxes if not(b['size_x']>500 and b['size_y']>800)]
rows=[(-55,0,24,24,100),(-115,75,20,24,115),(-55,240,26,26,95),(145,145,26,26,125),(80,-25,24,26,110),(-25,-140,24,22,105),(115,-190,24,26,100),(-60,-260,20,22,90)]
new=[dict(name='dense_highrise_%02d'%(i+1),x=v[0],y=v[1],size_x=v[2],size_y=v[3],height=v[4],style=['blue_glass','warm_concrete','silver_glass','blue_glass'][i%4]) for i,v in enumerate(rows)]
trace=json.loads(gzip.decompress((r/'docs/recovery/lidar_voxel_map/evidence/city_center_route/flight/observations.json.gz').read_bytes()))
xyz=np.array([[v[k] for k in ('x','y','z')] for v in trace['odom']]);times=np.array([v['t'] for v in trace['odom']]);wp=task['waypoints'];goals=np.array([[wp['wp'+str(i)][k] for k in ('x','y','h')] for i in range(1,wp['num']+1)])
def footprint_distance(a,b):
 q=np.abs(np.array([a['x']-b['x'],a['y']-b['y']]))-np.array([a['size_x']+b['size_x'],a['size_y']+b['size_y']])/2
 return float(np.linalg.norm(np.maximum(q,0))+min(max(q),0))
def sdf(points,b,margin=0):
 center=np.array([b['x'],b['y'],b['height']/2]);half=np.array([b['size_x'],b['size_y'],b['height']])/2+margin
 q=np.abs(points-center)-half
 return np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)
checks=[]
for b in new:
 distances=[footprint_distance(b,a) for a in obstacles];n=int(np.argmin(distances));d=sdf(xyz,b,2);g=sdf(goals,b,2)
 checks.append({'name':b['name'],'nearest_original_footprint_clearance_m':distances[n],'nearest_original_box':obstacles[n]['index'],'nearest_original_source':obstacles[n]['source'],'new_footprint_clearances_m':{a['name']:footprint_distance(b,a) for a in new if a is not b},'minimum_waypoint_clearance_to_inflated_box_m':float(g.min()),'minimum_prior_flight_clearance_to_inflated_box_m':float(d.min()),'nearest_prior_flight_time_s':float(times[d.argmin()]),'nearest_prior_flight_xyz_m':xyz[d.argmin()].tolist()})
report={'task_config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(),'new_building_count':len(new),'existing_boxes':len(boxes),'excluded_ground_boxes':[b['index'] for b in boxes if b not in obstacles],'all_original_footprints_disjoint':all(x['nearest_original_footprint_clearance_m']>0 for x in checks),'all_new_footprints_disjoint':all(v>0 for x in checks for v in x['new_footprint_clearances_m'].values()),'all_waypoints_outside_new_inflated_boxes':all(x['minimum_waypoint_clearance_to_inflated_box_m']>0 for x in checks),'all_prior_flight_samples_outside_new_inflated_boxes':all(x['minimum_prior_flight_clearance_to_inflated_box_m']>0 for x in checks),'inflation_per_face_m':2,'waypoints':goals.tolist(),'checks':checks,'note':'Prior-flight clearance is evaluated against newly proposed boxes only. It is not a new closed-loop flight result. Footprint checks exclude only the broad terrain box and include trees and small props.'}
(out/'buildings.json').write_text(json.dumps({'buildings':new},indent=2)+'\n');(out/'layout_checks.json').write_text(json.dumps(report,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(8,9),constrained_layout=True)
for b in obstacles:
 h=b['z']+b['size_z']/2
 if h<1:continue
 ax.add_patch(Rectangle((b['x']-b['size_x']/2,b['y']-b['size_y']/2),b['size_x'],b['size_y'],facecolor='0.6' if h>70 else '0.87',edgecolor='0.5',lw=.4))
for i,b in enumerate(new,1):
 ax.add_patch(Rectangle((b['x']-b['size_x']/2,b['y']-b['size_y']/2),b['size_x'],b['size_y'],facecolor='#4c9acb',edgecolor='#185b8b',lw=1.2))
 ax.annotate('N%d\n%dm'%(i,b['height']),(b['x'],b['y']),ha='center',va='center',fontsize=8)
ax.plot(xyz[:,0],xyz[:,1],color='#d94c35',lw=1.5,label='Previous measured flight (before additions)')
ax.scatter(goals[:,0],goals[:,1],marker='*',s=90,color='#192d48',label='Eight existing waypoints',zorder=5)
for i,g in enumerate(goals,1):ax.annotate(str(i),g[:2],xytext=(5,6),textcoords='offset points',color='#192d48',fontsize=10)
ax.set(xlim=(-190,220),ylim=(-370,330),xlabel='X (m)',ylabel='Y (m)',title='Proposed additional high-rises in end.world')
ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(loc='upper left',fontsize=8)
fig.savefig(out/'layout.png',dpi=170);plt.close(fig)
print(json.dumps({k:v for k,v in report.items() if k!='checks'},indent=2))
for c in checks:print(c['name'],c['nearest_original_footprint_clearance_m'],c['nearest_original_source'],'prior min',c['minimum_prior_flight_clearance_to_inflated_box_m'],'wp min',c['minimum_waypoint_clearance_to_inflated_box_m'])
