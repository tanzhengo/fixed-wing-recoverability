#!/usr/bin/env python3
"""Summarize the fifteen recorded known-map Gazebo configuration trials."""
import argparse,csv,hashlib,json
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2]/'docs/recovery/revision/city');a=p.parse_args()
 rows=[];hashes={}
 for method in ('cost','diverse','geometry','state_only','selected'):
  for relative,gate in ((False,48.825),(True,48.825),(True,15.)):
   tag=f"{method}_{'relative' if relative else 'coarse'}_gate{str(gate).replace('.','p')}";directory=a.root/tag
   data=json.loads((directory/'result.json').read_text());run=json.loads((directory/'run.json').read_text())
   assert data['error'] is None,(tag,data['error'])
   assert data['waypoint_gate_horizontal_m']==gate
   assert run['state_relative_altitudes']==relative
   transitions=data['goal_transitions'];completed=len([x for x in transitions if 'previous_goal' in x])
   assert completed<=8
   success=data['stop_reason']=='completed_first_waypoint_cycle'
   assert not success or completed==8
   rows.append(dict(case=tag,method=method,relative_altitudes=relative,max_primary_candidates=406 if relative else 203,horizontal_gate_m=gate,outcome=data['stop_reason'],completed_waypoints=completed,cycle_complete=success,last_sim_time_s=data['last_sim_time'],minimum_physical_clearance_m=data['minimum_physical_box_clearance_m'],minimum_inflated_clearance_m=data['minimum_inflated_box_clearance_m'],gate_checks_passed=data['waypoint_gate_checks_passed'],updates=data['status_count']))
   for f in directory.iterdir():
    if f.is_file():hashes[str(f.relative_to(a.root))]=hashlib.sha256(f.read_bytes()).hexdigest()
 summary={'trials':len(rows),'known_map':True,'sensor_map_feeds_planner':False,'prefix_check':False,'layout_count':1,'trials_per_configuration':1,'rows':rows,'source_sha256':hashes}
 (a.root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 with (a.root/'summary.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
 print(json.dumps({'trials':len(rows),'completed_cycles':sum(x['cycle_complete'] for x in rows)}))
if __name__=='__main__':main()
