#!/usr/bin/env python3
"""Independently reconstruct forest geometry and validate all recorded episodes.

Use --figure to additionally verify the figure manifest, projection, occupied
voxel cells and visible faces. The script never reruns or changes the renderer.
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
def independent_voxelization(rows,side):
 # Scalar sphere/cylinder--AABB intersection, independently implemented rather
 # than inferred from surface samples or copied from the renderer's output.
 cells=set()
 for row in rows:
  center=[float(row[k]) for k in ('x','y','z')]
  radius=float(row['r']);height=float(row['height'])
  extent=[radius,radius,radius if row['type']=='sphere' else height/2]
  spans=[range(math.floor((center[d]-extent[d])/side),math.ceil((center[d]+extent[d])/side)) for d in range(3)]
  for i in spans[0]:
   for j in spans[1]:
    for k in spans[2]:
     index=(i,j,k);lo=[q*side for q in index];hi=[q+side for q in lo]
     distance=[max(lo[d]-center[d],center[d]-hi[d],0.) for d in range(3)]
     if row['type']=='sphere':overlap=sum(q*q for q in distance)<radius*radius
     else:overlap=distance[0]**2+distance[1]**2<radius*radius and lo[2]<center[2]+height/2 and hi[2]>center[2]-height/2
     if overlap:cells.add(index)
 return cells
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
 assert render.VOXEL_SIZE==1.5 and np.array_equal(render.GRID_ORIGIN,np.zeros(3))
 assert figure_manifest['voxel_size_m']==render.VOXEL_SIZE
 assert figure_manifest['grid_origin_m']==render.GRID_ORIGIN.tolist()
 voxel_counts={};face_counts={}
 def face_key(vertices):return tuple(sorted(tuple(int(q) for q in vertex) for vertex in vertices))
 for scene in ('forest_sparse_0','forest_dense_0'):
  objects=[r for r in geo if r['scene']==scene]
  expected=independent_voxelization(objects,1.5)
  cells=render.voxelize(objects)
  assert cells.ndim==2 and cells.shape[1]==3 and np.issubdtype(cells.dtype,np.integer)
  assert cells.tolist()==[list(q) for q in sorted(expected)]
  assert len(cells)==figure_manifest['voxel_counts'][scene]
  assert hashlib.sha256(cells.astype('<i8').tobytes()).hexdigest()==figure_manifest['occupied_indices_sha256'][scene]
  faces,heights,shades=render.visible_voxel_faces(cells)
  assert faces.ndim==3 and faces.shape[1:]==(4,3) and np.isfinite(faces).all()
  assert np.allclose(heights,faces[:,:,2].mean(axis=1),rtol=0,atol=1e-12)
  assert np.all((shades>0)&(shades<=1)) and len(shades)==len(faces)
  assert np.all(np.diff(faces.mean(axis=1)@render.DEPTH)>=-1e-10)
  lattice=faces/1.5
  assert np.allclose(lattice,np.rint(lattice),rtol=0,atol=1e-12)
  actual_faces=[face_key(v) for v in np.rint(lattice).astype(int)]
  assert len(set(actual_faces))==len(actual_faces)
  expected_faces=set()
  for cell in expected:
   for axis in range(3):
    sign=1 if render.DEPTH[axis]>0 else -1
    adjacent=list(cell);adjacent[axis]+=sign
    if tuple(adjacent) in expected:continue
    other=[d for d in range(3) if d!=axis];vertices=[]
    for a,b in ((0,0),(0,1),(1,0),(1,1)):
     vertex=list(cell);vertex[axis]+=(sign+1)//2
     vertex[other[0]]+=a;vertex[other[1]]+=b;vertices.append(vertex)
    expected_faces.add(face_key(vertices))
  assert set(actual_faces)==expected_faces
  assert len(faces)==figure_manifest['visible_face_counts'][scene]
  voxel_counts[scene]=len(cells);face_counts[scene]=len(faces)
 # A 50 m scale segment is projected by the same basis as the scene.
 scale=np.array([[5,-90,0],[55,-90,0]])
 assert np.linalg.norm(scale[1]-scale[0])==50
 assert np.allclose(np.diff(render.project(scale),axis=0)[0],basis[:2]@np.array([50.,0.,0.]))
 report['figure_audit']={'passed':True,'displayed_cases':10,'displayed_vertices':sum(r['stored_vertices'] for r in displayed),'all_endpoints_outcomes_match':True,'orthonormal_projection':True,'voxel_size_m':render.VOXEL_SIZE,'independent_positive_volume_overlap_check':True,'occupied_voxel_counts':voxel_counts,'exposed_camera_facing_face_counts':face_counts,'face_geometry_and_depth_sort_passed':True,'figure_manifest_sha256':sha(D/'figure_manifest.json'),'renderer_sha256':sha(renderer),'output_sha256':{ext:sha(ROOT/('paper/figures/forest_comparison.'+ext)) for ext in ('png','pdf')}}
(D/'independent_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
