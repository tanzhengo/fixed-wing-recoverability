#!/usr/bin/env python3
"""Reproducible observed-map rollout collection; no test split is accepted."""
import argparse, concurrent.futures, hashlib, json, os, subprocess, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
SRC=ROOT/'outputs/two_phase_ws/src/Two-phase-motion-planning/motion_primitives_planning/src'

def digest(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,default=Path('/tmp/codex-observed-training'));p.add_argument('--workers',type=int,default=6);p.add_argument('--pilot',action='store_true');p.add_argument('--split',choices=['train','val','both'],default='both');a=p.parse_args();assert 1<=a.workers<=6
 a.raw.mkdir(parents=True,exist_ok=True)
 flags=subprocess.check_output(['pkg-config','--cflags','--libs','roscpp','tf'],env=dict(os.environ,PKG_CONFIG_PATH='/opt/ros/noetic/lib/pkgconfig'),text=True).split()
 exe=a.raw/'collector';subprocess.run(['g++','-std=c++14','-O2','-I'+str(SRC),'-I/usr/include/eigen3',str(HERE/'collect_observed.cpp'),'-o',str(exe)]+flags,check=True)
 protocol=dict(schema='OBS58',candidate_columns=115,metadata=['query','rank','cost','y','work_until_first_success','successors_until_first_success','rejected_collision','rejected_envelope','rejected_known_space','rejected_invalid','successor_library_size','primary_library_index'],feature_columns=[12,70],successor_columns=[70,115],successor_triples=['verified','steps','rejection'],features=dict(base=28,yaw_degrees=[-45,-22.5,0,22.5,45],pitch_degrees=[-20,0,20],distance_fractions=[.2,.4,.6,.8,1],scale='terminal airspeed * B',channels=['free_fraction','unknown_fraction']),training_scenes=64,validation_scenes=16,seeds=dict(train=1101000,val=1201000,pilot=9901000,test_reserved=1301000),forest_pairing='family 4 and 5 share base+40000+index',samplers=['Cost K12','Diverse K12','Full'],conditions=['nominal'],capture_seconds='0,0.2,...,2; 4,6,...,38; final planning failure if not already captured',map='empty initial three-state 2m voxels; nearest-hit 512x32 rays at 5Hz; 120m max; 360deg azimuth and [-40,20]deg elevation; 2m execution clearance + time-ramped 1m proposal reserve',horizon=3,continuation_horizon=3,primary_library_max=406,successor_library=15,planning_step=.02,execution_RK4_step=.005,execution_command_period=.1,experimental_prefix=False,strict_dispatch=True,cache='reuse validated remaining suffix; no forced refresh',failures='all cases and all prescribed startup snapshots retained; no-success snapshots included',oracle='all feasible primaries, all 15 successor outcomes, query work stops at first witness',maximum_task_seconds=40,hashes={name:digest(HERE/name) for name in ['run_collection.py','collect_observed.cpp','observed_scene.h','runtime_motion_primitives.h','learned_recovery.h','sensor_occupancy_map.h','runtime_sensor_map.h','evaluation_plant.h']},encoder_sha256=digest(SRC/'observed_recovery_features.h'))
 (HERE/'collection_protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
 cases=[]
 if a.pilot:
  cases=[('pilot',family,0,1,9901000,40) for family in range(6)]
 else:
  for split in (['train','val'] if a.split=='both' else [a.split]):
   for family in range(6):
    count=(12 if family<4 else 8) if split=='train' else (2 if family<4 else 4)
    for index in range(count):
     for method in range(3):cases.append((split,family,index,method,1101000 if split=='train' else 1201000,400))
 env=dict(os.environ,ROS_MASTER_URI='http://localhost:11858',ROS_LOG_DIR=str(a.raw/'roslogs'),LD_LIBRARY_PATH='/opt/ros/noetic/lib:'+os.environ.get('LD_LIBRARY_PATH',''))
 names=['staggered','wall','culdesac','spatial','forest_sparse','forest_dense']
 def run(c):
  split,family,index,method,base,steps=c;dest=a.raw/split;dest.mkdir(exist_ok=True);key=f'{split}_{names[family]}_{index}_m{method}'
  if (dest/(key+'_summary.csv')).exists():return {'case':key,'cached':True}
  begin=time.monotonic()
  with (dest/(key+'.log')).open('w') as log:r=subprocess.run([str(exe),str(dest),split,str(family),str(index),str(method),str(base),str(steps)],env=env,stdout=log,stderr=subprocess.STDOUT)
  return {'case':key,'returncode':r.returncode,'wall_s':time.monotonic()-begin}
 with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:
  for record in pool.map(run,cases):print(json.dumps(record),flush=True)
if __name__=='__main__':main()
