#!/usr/bin/env python3
"""Reproduce supplementary city-block counts and recorded-path summaries."""
from pathlib import Path
import csv,json,hashlib,math
from collections import Counter,defaultdict

root=Path(__file__).resolve().parent
manifest=json.loads((root/'run_manifest.json').read_text())
assert manifest['status']=='complete' and manifest['exit_code']==0
assert manifest['source_unchanged'] and manifest['models_unchanged'] and manifest['executable_unchanged']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name,meta in manifest['outputs'].items():assert sha(root/name)==meta['sha256'],name
read=lambda name:list(csv.DictReader((root/name).open()))
episodes=read('urban_episodes.csv');paths=read('urban_paths.csv');plans=read('urban_plans.csv');geometry=read('urban_scenarios.csv');maps=read('urban_map_metadata.csv')
assert len(episodes)==80 and len(maps)==16
methods=['cost12','diverse12','learned12','state_only12','full203']
assert len(set(r['case_id'] for r in episodes))==80
assert len(set((r['scene'],r['method']) for r in episodes))==80
assert set(r['method'] for r in episodes)==set(methods)
case_paths=defaultdict(list);case_plans=defaultdict(list)
for r in paths:case_paths[r['case_id']].append(r)
for r in plans:case_plans[r['case_id']].append(r)
case_metrics=[]
for r in episodes:
 p=case_paths[r['case_id']];q=case_plans[r['case_id']];assert p and q
 coords=[tuple(float(x[k]) for k in ('x','y','z')) for x in p]
 assert all(math.isfinite(v) for point in coords for v in point)
 assert coords[0]==(0.,0.,20.)
 path_length=sum(math.sqrt(sum((a-b)**2 for a,b in zip(first,second))) for first,second in zip(coords,coords[1:]))
 states=[float(x['time']) for x in p]
 assert all(a<=b for a,b in zip(states,states[1:])) and abs(states[-1]-float(r['sim_seconds']))<1e-6
 limit=203 if r['method']=='full203' else 12
 assert all(int(x['recovery_checked'])<=limit and int(x['work_exhausted'])==0 for x in q)
 if r['outcome']=='goal':assert float(r['goal_distance'])<15 and abs(coords[-1][2]-20)<3
 case_metrics.append(dict(r,path_length_3d_m=path_length,min_height_m=min(c[2] for c in coords),max_height_m=max(c[2] for c in coords),path_rows=len(p),plan_rows=len(q)))
summary={}
for family in ['urban_blocks','urban_dense']:
 summary[family]={}
 for method in methods:
  rows=[r for r in case_metrics if r['family']==family and r['method']==method];assert len(rows)==8
  goals=[r for r in rows if r['outcome']=='goal']
  summary[family][method]={'episodes':8,'outcomes':dict(Counter(r['outcome'] for r in rows)),'goals':len(goals),'mean_path_length_success_m':sum(r['path_length_3d_m'] for r in goals)/len(goals) if goals else None,'mean_min_clearance_all_m':sum(float(r['min_obstacle_clearance']) for r in rows)/len(rows),'min_height_all_m':min(r['min_height_m'] for r in rows),'max_height_all_m':max(r['max_height_m'] for r in rows)}
result={'protocol':'supplementary_urban_v1','seed_count':8,'paired_map_instances':16,'episodes':80,'conditions':['nominal'],'model_adaptation':False,'display_seed':701000,'display_selection':'First seed fixed before execution, both densities, all five methods','building_counts':{'urban_blocks':20,'urban_dense':28},'box_counts':{'urban_blocks':40,'urban_dense':48},'view_rectangle_m2':45000,'box_inflation_m':1.0,'summary':summary,'displayed_episodes':[r for r in case_metrics if r['scene'].endswith('_0')],'all_episode_metrics':case_metrics,'metrics_note':'Path length is the 3D chord sum of stored 0.2s states. Clearance is checked on the RK4 state prefix until the first detected failure. Timing is diagnostic only. All counts retain failures.','provenance':{'run_manifest_sha256':sha(root/'run_manifest.json'),'analysis_source_sha256':sha(Path(__file__)),'inputs':{p.name:sha(p) for p in root.glob('urban_*.csv')}}}
(root/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'audit':'passed','episodes':80,'displayed_episodes':[(r['scene'],r['method'],r['outcome']) for r in result['displayed_episodes']],'summary':summary},indent=2))
