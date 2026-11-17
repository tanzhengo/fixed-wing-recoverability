#!/usr/bin/env python3
from pathlib import Path
import argparse,sys,subprocess,json,time
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description='Replay the original paired maps and first prescribed urban query into fresh outputs.')
p.add_argument('--artifact-root',type=Path,required=True);p.add_argument('--build-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--port',type=int,default=11987);a=p.parse_args()
sys.path.insert(0,str(HERE.parent));from portable_support import REPO,fresh,copy_inputs,master,sha
OUT=fresh(a.output_dir);pair=OUT/'validation_pair';urban=OUT/'urban_trace';copy_inputs(HERE/'validation_pair',pair);urban.mkdir()
archive=a.artifact_root.resolve()/'work/restore_experiment_evidence/unknown_replay/learned3';models=REPO/'evidence_recovered/models'
rows=[('Selected','learned_coverage','selected_obs_bce.txt',.75,12),('Diverse','diverse','selected_obs_bce.txt',0,12),('ObsAux','learned','obs_aux.txt',0,12),('Cost','cost','-',0,12),('Full','cost','-',0,10000)]
spec=OUT/'methods.tsv'
spec.write_text('\n'.join('\t'.join(map(str,[name,mode,'-' if model=='-' else 'evidence_recovered/models/'+model,weight,k])) for name,mode,model,weight,k in rows)+'\n')
reports=[]
try:
    with master(OUT,a.port,{'REPLAY_METHODS_TSV':str(spec),'REPLAY_REPEATS':'1'}) as env:
        ns='/tro_mechanism/urban'
        subprocess.run(['rosparam','load',str(archive/'params.yaml'),ns],env=env,check=True)
        subprocess.run(['rosparam','set',ns+'/recovery/model_path',str(models/'legacy_state_aux_v1.txt')],env=env,check=True)
        jobs=[('validation_pair',[str(a.build_dir.resolve()/'pair'),str(pair)],pair),('urban_trace',[str(a.build_dir.resolve()/'urban'),str(archive),str(urban),ns,str(models/'legacy_state_aux_compact.txt'),'mechanism','1'],urban)]
        for name,command,folder in jobs:
            begin=time.monotonic()
            with (folder/'stdout.log').open('w') as stdout,(folder/'trace.log').open('w') as stderr:
                result=subprocess.run(command,env=env,cwd=REPO,stdout=stdout,stderr=stderr)
            reports.append({'name':name,'command':command,'returncode':result.returncode,'wall_seconds':time.monotonic()-begin})
            if result.returncode:raise RuntimeError(name+' failed; inspect fresh output logs')
        subprocess.run(['rosparam','delete',ns],env=env,check=True)
finally:(OUT/'execution.json').write_text(json.dumps(reports,indent=2)+'\n')
print(OUT)
