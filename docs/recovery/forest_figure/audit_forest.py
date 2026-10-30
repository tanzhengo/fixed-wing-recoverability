#!/usr/bin/env python3
"""Independently reconstruct forest geometry and validate all recorded episodes.

Use --figure to additionally verify the figure manifest, projection, and sampled
analytic surfaces. The script never reruns or changes the figure renderer.
"""
import csv,json,math,hashlib,sys
from pathlib import Path
from collections import defaultdict,Counter
ROOT=Path(__file__).resolve().parents[3]
D=ROOT/'docs/recovery/forest_figure'
def read(name):return list(csv.DictReader((D/name).open()))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def near(a,b):return math.isclose(float(a),float(b),rel_tol=5e-9,abs_tol=5e-8)
class RNG:
 def __init__(self,seed):self.state=seed
 def uniform(self,a,b):
  self.state=(1664525*self.state+1013904223)&0xffffffff
  return a+(b-a)*self.state/4294967296.
def geom_key(r):return tuple(r[x] for x in ('type','x','y','z','r','height'))
geo=read('forest_scenarios.csv');groups=defaultdict(list)
for r in geo:groups[r['scene']].append(r)
assert len(groups)==16
for index in range(8):
 seed=601000+index;trees=RNG(seed);bushes=RNG(seed^0x51ed270b);bank=[];bushbank=[]
 for j in range(80):
  x=trees.uniform(40,240);y=trees.uniform(-80,80);r=trees.uniform(.8,1.4);h=trees.uniform(24,36);c=trees.uniform(4,6)
  bank.extend([('cylinder',x,y,h/2,r,h),('sphere',x,y,h,c,0)])
 for j in range(12):
  x=bushes.uniform(40,240);y=bushes.uniform(-80,80);r=bushes.uniform(3,5);bushbank.append(('sphere',x,y,r,r,0))
 sparse=groups['forest_sparse_'+str(index)];dense=groups['forest_dense_'+str(index)]
 assert len(sparse)==92 and len(dense)==172
 assert [geom_key(r) for r in sparse[:80]]==[geom_key(r) for r in dense[:80]]
 assert [geom_key(r) for r in sparse[80:]]==[geom_key(r) for r in dense[160:]]
 for count,name in [(40,'forest_sparse_'),(80,'forest_dense_')]:
  actual=groups[name+str(index)];expected=bank[:2*count]+bushbank
  for row,values in zip(actual,expected):
   assert row['type']==values[0] and int(row['seed'])==seed
   assert row['low']=='5' and row['high']=='45'
   for k,v in zip(['x','y','z','r','height'],values[1:]):assert near(row[k],v),(row,k,v)
meta=read('forest_map_metadata.csv');assert len(meta)==16
for r in meta:
 count=40 if 'sparse' in r['scene'] else 80
 assert int(r['tree_count'])==count and int(r['bush_count'])==12 and float(r['area_m2'])==32000
 assert near(r['tree_density_per_m2'],count/32000)
 assert bool(int(r['display_seed']))==(int(r['seed'])==601000)
manifest=json.loads((D/'run_manifest.json').read_text());freeze=json.loads((ROOT/'docs/recovery/learning/MODEL_FREEZE.json').read_text())
for p,h in manifest['source_sha256'].items():assert sha(ROOT/p)==h,p
assert manifest['model_sha256']==freeze['models']
for p,h in freeze['source'].items():assert sha(ROOT/p)==h,p
report={'audit':'independent_forest_v1','geometry_passed':True,'maps':16,'paired_seeds':8,'tree_centers_sparse':40,'tree_centers_dense':80,'bushes':12,'independent_rng_reconstruction':True,'sparse_subset_of_dense':True,'same_bushes_in_each_pair':True,'frozen_sources_models_match':True,'display_seed':601000}
if '--geometry-only' in sys.argv:
 print(json.dumps(report,indent=2));sys.exit(0)
assert manifest['status']=='complete' and manifest['exit_code']==0
es=read('forest_episodes.csv');ps=read('forest_paths.csv');ds=read('forest_plans.csv')
methods=['cost12','diverse12','learned12','state_only12','full203'];assert len(es)==80
assert len({r['case_id'] for r in es})==80
assert {(r['scene'],r['method']) for r in es}=={(s,m) for s in groups for m in methods}
paths=defaultdict(list);plans=defaultdict(list)
for row in ps:paths[row['case_id']].append(row)
for row in ds:plans[row['case_id']].append(row)
assert set(paths)==set(plans)=={r['case_id'] for r in es}
initial=None;summary=defaultdict(lambda:defaultdict(Counter));statefields=['x','y','z','V','gamma','sigma','roll','attack','T','d_sigma']
allowed={'goal','no_feasible_primary','collision','envelope_violation','invalid_dynamics','timeout'}
for e in es:
 assert e['condition']=='nominal' and e['outcome'] in allowed
 key=e['case_id'];rows=paths[key];plan=plans[key];steps=int(e['steps'])
 assert int(rows[0]['step'])==0 and float(rows[0]['time'])==0
 init=tuple(rows[0][f] for f in statefields)
 if initial is None:initial=init
 assert init==initial
 assert int(rows[-1]['step'])==steps and near(rows[-1]['time'],steps*.2)
 assert near(e['sim_seconds'],steps*.2)
 last=rows[-1];assert math.isclose(float(e['goal_distance']),math.hypot(250-float(last['x']),float(last['y'])),rel_tol=1e-6,abs_tol=2e-5)
 if e['outcome']=='goal':assert float(e['goal_distance'])<15 and abs(float(last['z'])-20)<3
 previous=-1
 for row in rows:
  st=int(row['step']);assert previous<=st and st<=steps;assert near(row['time'],st*.2);previous=st
  assert all(math.isfinite(float(row[f])) for f in statefields)
 assert len({int(r['step']) for r in rows})==steps+1
 assert len({int(r['step']) for r in plan})==len(plan)
 for row in plan:
  assert int(row['recovery_checked'])<=(203 if e['method']=='full203' else 12)
  assert int(row['primary_count']) in (203,204)
  assert int(row['verified'])<=int(row['primary_feasible'])
  if int(row['verified']):assert int(row['backup_evaluated'])>0
  assert int(row['backup_steps'])>=0 and int(row['primary_steps'])>0
 if e['outcome']=='no_feasible_primary':assert int(plan[-1]['primary_feasible'])==0
 summary[e['family']][e['method']][e['outcome']]+=1
for family in summary:
 for method in methods:assert sum(summary[family][method].values())==8
report.update(episode_audit_passed=True,episodes=80,path_rows=len(ps),planning_rows=len(ds),all_outcomes_retained=True,identical_initial_states=True,candidate_budget_violations=0,outcomes={f:{m:dict(v) for m,v in methods_.items()} for f,methods_ in summary.items()},input_sha256={name:sha(D/name) for name in ['forest_episodes.csv','forest_paths.csv','forest_plans.csv','forest_scenarios.csv','forest_map_metadata.csv','run_manifest.json']},audit_script_sha256=sha(Path(__file__)))
if '--figure' in sys.argv:
 import importlib.util
 import numpy as np
 renderer=ROOT/'paper/create_forest_figure.py'
 figure_manifest=json.loads((D/'figure_manifest.json').read_text())
 assert figure_manifest['figure_seed']==601000
 assert figure_manifest['renderer_sha256']==sha(renderer)
 for name,value in figure_manifest['source_sha256'].items():assert sha(D/name)==value,name
 spec=importlib.util.spec_from_file_location('forest_renderer_under_audit',renderer)
 render=importlib.util.module_from_spec(spec);spec.loader.exec_module(render)
 basis=np.array([render.RIGHT,render.UP,render.DEPTH])
 assert np.allclose(basis@basis.T,np.eye(3),rtol=0,atol=1e-12)
 assert np.allclose([render.AZIMUTH,render.ELEVATION],np.deg2rad([-80,34]),rtol=0,atol=1e-12)
 assert np.allclose(render.project(np.eye(3)),basis[:2].T,rtol=0,atol=1e-12)
 expected_cases={e['case_id']:e for e in es if e['scene'] in ('forest_sparse_0','forest_dense_0')}
 displayed=figure_manifest['displayed_cases']
 assert len(displayed)==10 and {r['case_id'] for r in displayed}==set(expected_cases)
 for row in displayed:
  e=expected_cases[row['case_id']];recorded=paths[row['case_id']]
  assert all(row[k]==e[k] for k in ('scene','method','outcome'))
  assert row['stored_vertices']==len(recorded)
  assert row['endpoint_xyz']==[float(recorded[-1][k]) for k in ('x','y','z')]
  xyz=np.array([[float(r[k]) for k in ('x','y','z')] for r in recorded])
  assert np.isfinite(render.project(xyz)).all()
 surface_points=0
 for row in geo:
  if row['scene'] not in ('forest_sparse_0','forest_dense_0'):continue
  xyz=render.sample_surface(row);center=np.array([float(row[k]) for k in ('x','y','z')])
  delta=xyz-center;r=float(row['r']);h=float(row['height'])
  assert xyz.ndim==2 and xyz.shape[1]==3 and np.isfinite(xyz).all()
  if row['type']=='sphere':assert np.allclose(np.linalg.norm(delta,axis=1),r,rtol=0,atol=1e-10)
  else:
   assert np.allclose(np.linalg.norm(delta[:,:2],axis=1),r,rtol=0,atol=1e-10)
   assert np.all(np.abs(delta[:,2])<=h/2+1e-10)
  surface_points+=len(xyz)
 # A 50 m scale segment is projected by the same basis as the scene.
 scale=np.array([[5,-90,0],[55,-90,0]])
 assert np.linalg.norm(scale[1]-scale[0])==50
 assert np.allclose(np.diff(render.project(scale),axis=0)[0],basis[:2]@np.array([50.,0.,0.]))
 report['figure_audit']={'passed':True,'displayed_cases':10,'displayed_vertices':sum(r['stored_vertices'] for r in displayed),'all_endpoints_outcomes_match':True,'orthonormal_projection':True,'surface_points_on_actual_uninflated_geometry':surface_points,'figure_manifest_sha256':sha(D/'figure_manifest.json'),'renderer_sha256':sha(renderer),'output_sha256':{ext:sha(ROOT/('paper/figures/forest_comparison.'+ext)) for ext in ('png','pdf')}}
(D/'independent_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
