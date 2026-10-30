#!/usr/bin/env python3
"""Paired known-map Gazebo city configurations for the reviewer revision."""
from pathlib import Path
import argparse,hashlib,json,os,signal,socket,subprocess,time
import yaml

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--scratch',type=Path,required=True);p.add_argument('--only',default='');p.add_argument('--task-config',type=Path,default=Path(__file__).resolve().parents[2]/'docs/recovery/revision/city/geometry_relative_gate48p825/task_config.yaml');a=p.parse_args()
 repo=Path(__file__).resolve().parents[2];a.output.mkdir(parents=True,exist_ok=True);a.scratch.mkdir(parents=True,exist_ok=True)
 cases=[(method,relative,gate) for relative in (False,True) for method in ('cost','diverse','geometry','state_only','selected') for gate in (48.825,)]
 cases += [(m,True,15.) for m in ('cost','diverse','geometry','state_only','selected')]
 for method,relative,gate in cases:
  tag='%s_%s_gate%s'%(method,'relative' if relative else 'coarse',str(gate).replace('.','p'))
  if a.only and a.only!=tag:continue
  out=a.output/tag;work=a.scratch/tag
  if (out/'result.json').exists():print('EXISTING '+tag,flush=True);continue
  out.mkdir(exist_ok=True);work.mkdir(exist_ok=True)
  for port in (11371,11375):
   with socket.socket() as s: assert s.connect_ex(('127.0.0.1',port))!=0,'Port already active'
  task=yaml.safe_load(a.task_config.read_text())
  task['motion_primitive'].update(state_relative_altitudes=relative,waypoint_horizontal_tolerance=gate)
  config=work/'task.yaml';config.write_text(yaml.safe_dump(task,sort_keys=False))
  env=os.environ.copy();env.update(ROS_MASTER_URI='http://127.0.0.1:11371',GAZEBO_MASTER_URI='http://127.0.0.1:11375',GAZEBO_MODEL_DATABASE_URI='',ROS_LOG_DIR=str(work/'roslog'))
  model=(repo/'docs/recovery/revision/training/state_joint/20261030/network_v1.txt' if method=='selected' else repo/'motion_primitives_planning/models'/('recovery_mlp_no_geometry.txt' if method=='state_only' else 'recovery_mlp.txt'))
  order=method if method in ('cost','diverse') else 'learned'
  command=['taskset','-c','6,7','roslaunch','motion_primitives_planning','terminal_recovery.launch','gui:=false','rviz:=false','paused:=false','world_name:='+str(repo/'simple-airplane/simple_airplane_gazebo/worlds/end_city_dense_lidar.world'),'task_config:='+str(config),'description_file:='+str(repo/'simple-airplane/simple_airplane_description/urdf/simple_airplane_with_3d_lidar.xacro'),'candidate_order:='+order,'model_path:='+str(model),'max_candidates:=12','learned_work_power:=0.0','backup_horizon:=6.0','stop_on_rejection:=true','reuse_verified_plan:=false']
  observer=['python3',str(repo/'motion_primitives_planning/scripts/verify_end_world.py'),'--task-config',str(config),'--output-dir',str(out),'--sim-seconds','180','--wall-seconds','500','--stop-after-cycle','--stop-on-failure']
  manifest={'case':tag,'candidate_order':order,'model':str(model.relative_to(repo)),'model_sha256':hashlib.sha256(model.read_bytes()).hexdigest(),'state_relative_altitudes':relative,'horizontal_gate_m':gate,'command':command,'observer':observer,'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
  (work/'run.json').write_text(json.dumps(manifest,indent=2)+'\n')
  logs=[(work/name).open('w') for name in ('launch.log','observer.log')];processes=[];start=time.monotonic()
  print('START '+tag,flush=True)
  try:
   obs=subprocess.Popen(observer,env=env,stdout=logs[1],stderr=subprocess.STDOUT,start_new_session=True);processes.append(obs)
   launch=subprocess.Popen(command,env=env,stdout=logs[0],stderr=subprocess.STDOUT,start_new_session=True);processes.append(launch)
   while obs.poll() is None and time.monotonic()-start<530:
    if launch.poll() is not None:break
    time.sleep(1)
  finally:
   for proc in reversed(processes):
    if proc.poll() is None:
     os.killpg(proc.pid,signal.SIGINT)
     try:proc.wait(timeout=12)
     except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
   for log in logs:log.close()
  manifest['wall_seconds']=time.monotonic()-start;manifest['ended_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
  (out/'run.json').write_text(json.dumps(manifest,indent=2)+'\n')
  result=json.loads((out/'result.json').read_text()) if (out/'result.json').exists() else {'stop_reason':'missing_observer_result'}
  print(json.dumps({'case':tag,'stop_reason':result['stop_reason'],'transitions':len(result.get('goal_transitions',[])),'min_clearance':result.get('minimum_inflated_box_clearance_m'),'elapsed':manifest['wall_seconds']}),flush=True)
  time.sleep(2)
if __name__=='__main__':main()
