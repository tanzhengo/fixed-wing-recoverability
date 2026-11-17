#!/usr/bin/env python3
"""Run the fixed nine-flight protocol with an explicitly supplied ROS runtime."""
from pathlib import Path
import argparse,json,hashlib,subprocess,time,os,platform

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--runtime-repo',type=Path,required=True,help='Restored and built original ROS repository; see ROS_RESTORATION_PLAN.json')
parser.add_argument('--catkin-workspace',type=Path,required=True)
parser.add_argument('--truth-mesh',type=Path,required=True,help='Independent physical collision mesh, used only for evaluation')
parser.add_argument('--output',type=Path,required=True,help='New campaign directory outside the checked-in evidence')
parser.add_argument('--model',type=Path,default=Path(__file__).resolve().parents[2]/'evidence_recovered/models/selected_obs_bce.txt')
parser.add_argument('--python',default='/usr/bin/python3',help='Python interpreter with ROS Noetic dependencies')
parser.add_argument('--port-base',type=int,default=12010)
parser.add_argument('--plan-only',action='store_true',help='Validate inputs and print commands without writing files or launching ROS')
parser.add_argument('--resume',action='store_true',help='Resume the same frozen protocol, retaining all existing outcomes')
args=parser.parse_args()
OUT=args.output.resolve()
REPO=args.runtime_repo.resolve()
PKG=REPO/'motion_primitives_planning'
if args.catkin_workspace.resolve()!=REPO.parents[1]:
 parser.error('--runtime-repo must be under --catkin-workspace/src, matching the runtime binary lookup.')
MODEL=args.model.resolve()
truth=args.truth_mesh.resolve()
configs={'Cost':('cost',.5),'Diverse':('diverse',.5),'Selected':('learned_coverage',.75)}
orders=[['Cost','Diverse','Selected'],['Diverse','Selected','Cost'],['Selected','Cost','Diverse']]
sequence=[{'block':b+1,'method':name} for b,order in enumerate(orders) for name in order]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
inputs=[MODEL,truth,PKG/'launch/config/task_end_city_dense.yaml',PKG/'scripts/run_unknown_city.py',PKG/'src/runtime_motion_primitives.h',PKG/'src/runtime_sensor_map.h',PKG/'src/sensor_occupancy_map.h',PKG/'src/learned_recovery.h',PKG/'src/observed_recovery_features.h',REPO/'simple-airplane/simple_airplane_gazebo/worlds/end_city_dense_lidar.world',args.catkin_workspace.resolve()/'devel/lib/motion_primitives_planning/motion_primitives_test']
missing=[str(p) for p in inputs if not p.is_file()]
if missing: parser.error('Missing runtime inputs: '+', '.join(missing))
if OUT==Path(__file__).resolve().parent or Path(__file__).resolve().parent in OUT.parents:
 parser.error('Use an output directory outside the checked-in urban-repeats evidence.')
def command(i,item):
 name=item['method'];dest=OUT/('block%d_%s'%(item['block'],name));order,alpha=configs[name]
 return [args.python,str(PKG/'scripts/run_unknown_city.py'),'--output',str(dest),'--order',order,'--model',str(MODEL),'--port',str(args.port_base+4*i),'--world-name',str(inputs[-2]),'--truth-mesh',str(truth),'--wall-seconds','900','--launch-arg','deferred_inference:=true','--launch-arg','prune_dominated:=true','--launch-arg','coverage_weight:='+str(alpha)]
payload={'design':'Three cyclic-order blocks, one asynchronous Gazebo flight per method per block; no parameter or model selection on these runs','sequence':sequence,'maps':'Same urban scene and prescribed eight-waypoint route; empty sensor map for every trial','candidate_budget':12,'primary_horizon_s':3,'successor_horizon_s':3,'deferred_inference':True,'incumbent_pruning':True,'selected_coverage_weight':.75,'supervisor':'Original bounded map initialization and termination on unresolved execution rejection','all_outcomes_retained':True,'hardware_flight':False,'old_acquisitions_not_pooled':True,'wall_limit_s':900,'files':{str(p):sha(p) for p in inputs},'platform':platform.platform(),'created_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
payload['runner']={'python':args.python,'port_base':args.port_base}
if args.plan_only:
 print(json.dumps({'protocol':payload,'commands':[command(i,item) for i,item in enumerate(sequence)]},indent=2))
 raise SystemExit(0)
try:
 resolved_package=Path(subprocess.check_output(['rospack','find','motion_primitives_planning'],text=True).strip()).resolve()
except (OSError,subprocess.CalledProcessError):
 parser.error('Source the restored catkin workspace before acquisition; rospack cannot find the planner.')
if resolved_package!=PKG.resolve():
 parser.error('ROS resolves a different planner package. Source the supplied runtime workspace first.')
prot=OUT/'protocol.json'
if OUT.exists() and any(OUT.iterdir()) and not args.resume:
 parser.error('Output is not empty; use a new directory or --resume for the identical protocol.')
OUT.mkdir(parents=True,exist_ok=True)
if prot.exists():
 old=json.loads(prot.read_text())
 if {k:v for k,v in old.items() if k!='created_utc'}!={k:v for k,v in payload.items() if k!='created_utc'}:
  parser.error('Existing protocol differs; retained without changes. Use a new output directory.')
else:prot.write_text(json.dumps(payload,indent=2)+'\n')
results=[]
for i,item in enumerate(sequence):
 name=item['method']; dest=OUT/('block%d_%s'%(item['block'],name)); order,alpha=configs[name]
 if (dest/'summary.json').is_file():
  result=json.loads((dest/'summary.json').read_text());results.append(dict(item,passed=result['passed'],resumed=True));continue
 if dest.exists() and any(dest.iterdir()):raise RuntimeError('Partial acquisition preserved; review before resuming: '+str(dest))
 if not all(sha(Path(p))==h for p,h in payload['files'].items()):
  raise RuntimeError('A frozen protocol input changed; acquisition stopped.')
 cmd=command(i,item)
 print('START',item,flush=True);t=time.monotonic()
 with (OUT/('block%d_%s.log'%(item['block'],name))).open('w') as log:
  proc=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
 report={'returncode':proc.returncode,'wall_s':time.monotonic()-t,**item}
 if (dest/'summary.json').is_file():
  result=json.loads((dest/'summary.json').read_text());report.update(passed=result['passed'],stop=result['supervisor_stop_reason'],flight_stop=result['flight'].get('stop_reason'))
 else:report['missing_summary']=True
 results.append(report);(OUT/'progress.json').write_text(json.dumps(results,indent=2)+'\n');print('END',report,flush=True)
 if report.get('missing_summary') or report.get('stop')=='launch_exit':raise RuntimeError('Infrastructure failure; retained and stopped for inspection')
(OUT/'complete.json').write_text(json.dumps({'runs':results,'all_runs_recorded':len(results)==9},indent=2)+'\n')
