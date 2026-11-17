#!/usr/bin/env python3
import argparse,hashlib,json,subprocess
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
SRC=ROOT/'outputs/two_phase_ws/src/Two-phase-motion-planning/motion_primitives_planning/src'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--models',type=Path,required=True);p.add_argument('--data',type=Path,required=True);a=p.parse_args()
 raw=Path('/tmp/codex-observed-training/validation_features.f32');data=np.load(a.data/'val.npz');data['x'].astype('<f4').tofile(raw);exe=raw.with_name('predict_export');subprocess.run(['g++','-O2','-std=c++14','-I'+str(SRC),str(HERE/'predict_export.cpp'),'-o',str(exe)],check=True)
 reports=[]
 for cell in ['state_aux','observed','observed_aux']:
  for seed in [20261101,20261102,20261103]:
   folder=a.models/cell/str(seed);model=folder/'network_compact.txt';out=folder/'validation_cpp.f32';subprocess.run([str(exe),str(model),str(raw),str(out)],check=True,stdout=subprocess.DEVNULL)
   cpp=np.fromfile(out,dtype='<f4').reshape(-1,2);torch=np.load(folder/'validation_predictions.npz')['output'];assert cpp.shape[0]==len(torch);prob=(1/(1+np.exp(-np.clip(torch[:,0],-50,50)))).astype(np.float32)
   logit_error=np.abs(cpp[:,0]-torch[:,0]);prob_error=np.abs(cpp[:,1]-prob)
   assert float(logit_error.max())<1e-4 and float(prob_error.max())<5e-6
   np.savez_compressed(folder/'validation_deployed_predictions.npz',output=np.c_[cpp[:,0],torch[:,1],cpp[:,1]])
   reports.append(dict(cell=cell,seed=seed,candidates=len(cpp),max_logit_error=float(logit_error.max()),max_probability_error=float(prob_error.max()),mean_logit_error=float(logit_error.mean()),network_sha256=sha(model),cpp_predictions_sha256=sha(out)))
 report=dict(criterion='All validation candidate logits vs compact portable C++ inference; downstream schedule replay uses C++ probabilities',validation_feature_sha256=sha(raw),portable_header_sha256=sha(SRC/'learned_recovery.h'),source_sha256=sha(HERE/'predict_export.cpp'),models=reports)
 (a.models/'export_parity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
