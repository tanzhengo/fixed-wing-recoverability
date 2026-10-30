#!/usr/bin/env python3
"""Run traceable revision experiments; new tests require an explicit freeze file."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,subprocess,time
from review_runtime_io import csv_exists

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--phase',choices=['matched','episodes'],required=True)
 p.add_argument('--split',choices=['val','test','review'],required=True);p.add_argument('--output-prefix',type=Path,required=True)
 p.add_argument('--selected-model',type=Path,required=True);p.add_argument('--freeze',type=Path)
 p.add_argument('--cpu',type=int,default=3);p.add_argument('--profile',action='store_true');p.add_argument('--repeats',type=int,default=3)
 p.add_argument('--timing-isolated',action='store_true');p.add_argument('--selection-only',action='store_true');p.add_argument('--prefix-check',action='store_true');a=p.parse_args()
 repo=Path(__file__).resolve().parents[2];ws=repo.parents[1];exe=ws/'devel/lib/motion_primitives_planning/recovery_review_benchmark'
 if a.split=='review' and (a.freeze is None or not a.freeze.is_file()):raise ValueError('New test requires a saved model-selection freeze')
 prefix=a.output_prefix.resolve();prefix.parent.mkdir(parents=True,exist_ok=True)
 if csv_exists(Path(str(prefix)+'_'+a.phase+'.csv')):raise ValueError('Refusing to overwrite experiment')
 train=repo/'docs/recovery/revision/training'
 models={'model_path':repo/'docs/recovery/learning/models/geometry/network.txt','state_only_model_path':repo/'docs/recovery/learning/models/state_only/network.txt','compact_legacy_model_path':repo/'docs/recovery/revision/runtime/controls/legacy_state_compact.txt','selected_model_path':a.selected_model.resolve(),
 'geometry_bce_model_path':train/'geometry_bce/20261030/network_compact.txt','geometry_aux_model_path':train/'geometry_joint/20261030/network_compact.txt','state_bce_model_path':train/'state_bce/20261030/network_compact.txt','state_aux_model_path':train/'state_joint/20261030/network_compact.txt'}
 for path in models.values():
  if not path.is_file():raise FileNotFoundError(path)
 cmd=['taskset','-c',str(a.cpu),str(exe),'_phase:='+a.phase,'_split:='+a.split,'_output_prefix:='+str(prefix),'_work_power:=0','_profile:='+str(a.profile).lower(),'_repeats:='+str(a.repeats),'_selection_only:='+str(a.selection_only).lower(),'_prefix_check:='+str(a.prefix_check).lower()]+['_'+k+':='+str(v) for k,v in models.items()]
 env=os.environ.copy();env['ROS_MASTER_URI']='http://localhost:11372'
 env['ROS_LOG_DIR']=str(ws.parents[1]/'work/review_runtime/roslog')
 sources=[repo/'motion_primitives_planning/src'/n for n in ['review_motion_primitives.h','learned_recovery_revision.h','recovery_review_benchmark.cpp','motion_primitives.h','evaluation_plant.h']]
 record={'start_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'command':cmd,'ros_master_uri':env['ROS_MASTER_URI'],'cpu':a.cpu,'isolated_timing':a.timing_isolated,'profile':a.profile,'prefix_check':a.prefix_check,'model_hashes':{k:sha(v) for k,v in models.items()},'source_hashes':{str(v.relative_to(repo)):sha(v) for v in sources},'executable_sha256':sha(exe),'selection_freeze_sha256':sha(a.freeze) if a.freeze else None}
 mf=Path(str(prefix)+'_'+a.phase+'_run_manifest.json');mf.write_text(json.dumps(record,indent=2)+'\n')
 start=time.monotonic()
 with Path(str(prefix)+'_'+a.phase+'_stdout.log').open('w') as log:
  proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT);record['pid']=proc.pid;mf.write_text(json.dumps(record,indent=2)+'\n');code=proc.wait()
 record.update(end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_wall_s=time.monotonic()-start,exit_code=code,source_unchanged=all(sha(repo/k)==v for k,v in record['source_hashes'].items()),models_unchanged=all(sha(models[k])==v for k,v in record['model_hashes'].items()),executable_unchanged=sha(exe)==record['executable_sha256'])
 mf.write_text(json.dumps(record,indent=2)+'\n');print(json.dumps({'phase':a.phase,'split':a.split,'exit_code':code,'wall_seconds':record['elapsed_wall_s']}))
 if code or not(record['source_unchanged'] and record['models_unchanged'] and record['executable_unchanged']):raise SystemExit(1)
if __name__=='__main__':main()
