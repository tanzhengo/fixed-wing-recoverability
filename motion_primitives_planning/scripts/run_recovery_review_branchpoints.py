from pathlib import Path
import datetime,hashlib,json,os,signal,subprocess,time,xmlrpc.client,shlex
repo=Path(__file__).resolve().parents[2];ws=repo.parents[1];out=repo/'docs/recovery/revision/diagnostics';out.mkdir(parents=True,exist_ok=True)
env=os.environ.copy();env['ROS_MASTER_URI']='http://localhost:11373';env['ROS_LOG_DIR']=str(out/'roslog')
setup='source /opt/ros/noetic/setup.bash; source '+shlex.quote(str(ws/'devel/setup.bash'))+'; '
masterlog=open(out/'master.log','w');master=subprocess.Popen(['bash','-c',setup+'exec roscore -p 11373'],env=env,stdout=masterlog,stderr=subprocess.STDOUT,start_new_session=True)
tracked=[repo/'motion_primitives_planning/src/motion_primitives.h',repo/'motion_primitives_planning/src/evaluation_plant.h',repo/'motion_primitives_planning/src/recovery_learning_benchmark.cpp',repo/'motion_primitives_planning/src/recovery_review_branchpoints.cpp',ws/'devel/lib/motion_primitives_planning/recovery_review_branchpoints',repo/'docs/recovery/learning/models/geometry/network.txt',repo/'docs/recovery/learning/models/state_only/network.txt',out/'models/geometry_yaw_mask.txt']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest={'start_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'ROS_MASTER_URI':env['ROS_MASTER_URI'],'cpu_affinity':[4],'status':'running','hashes_before':{str(p):sha(p) for p in tracked}}
try:
 for _ in range(100):
  try:
   if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/review_probe')[0]==1:break
  except Exception:time.sleep(.1)
 else:raise RuntimeError('ROS master did not start')
 exe=ws/'devel/lib/motion_primitives_planning/recovery_review_branchpoints'
 cmd=['taskset','-c','4',str(exe),'_output_prefix:='+str(out/'diagnostic'),'_model_path:='+str(repo/'docs/recovery/learning/models/geometry/network.txt'),'_state_only_model_path:='+str(repo/'docs/recovery/learning/models/state_only/network.txt')]
 manifest['command']=cmd;(out/'branch_run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 with open(out/'branch_run.log','w') as log:
  process=subprocess.Popen(['bash','-c',setup+'exec "$@"','diagnostic-run']+cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
  code=process.wait()
 manifest['exit_code']=code;manifest['status']='complete' if code==0 else 'failed';manifest['hashes_after']={str(p):sha(p) for p in tracked};manifest['sources_unchanged']=manifest['hashes_before']==manifest['hashes_after']
 manifest['end_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();(out/'branch_run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 print(json.dumps({'exit_code':code,'unchanged':manifest['sources_unchanged']},indent=2))
finally:
 os.killpg(master.pid,signal.SIGINT)
 try:master.wait(timeout=10)
 except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGTERM)
 masterlog.close()
