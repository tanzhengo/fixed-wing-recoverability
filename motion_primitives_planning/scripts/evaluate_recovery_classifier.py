#!/usr/bin/env python3
"""Frozen held-out classifier evaluation; never fits weights or chooses settings."""
import argparse,json
from pathlib import Path
import numpy as np
import torch
from train_recovery_network import Network,load_data,predict,metrics,sha

def main():
 p=argparse.ArgumentParser();p.add_argument('--data',required=True);p.add_argument('--models',required=True);p.add_argument('--freeze',required=True);p.add_argument('--output',required=True);a=p.parse_args()
 torch.set_num_threads(1);data=load_data(a.data);freeze=json.loads(Path(a.freeze).read_text())
 assert all(str(s).startswith('test_') for s in data['scene'])
 result={'split':'frozen_test','candidate_rows':len(data['y']),'positive':int(data['y'].sum()),'maps':len(set(data['scene'])),'data_sha256':str(data['source_sha256']),'freeze_sha256':sha(a.freeze),'models':{}}
 output=Path(a.output);output.mkdir(parents=True,exist_ok=True)
 for name in ['geometry','state_only']:
  d=Path(a.models)/name
  assert sha(d/'network.txt')==freeze['models'][name]
  m=Network(name=='state_only');m.load_state_dict(torch.load(d/'checkpoint.pt',weights_only=True));prediction=predict(m,data['x']);r=metrics(prediction,data)
  positive=data['y']==1;chosen=prediction[:,0]>=0
  r['positive_recall']=float(chosen[positive].mean());r['negative_recall']=float((~chosen[~positive]).mean());r['balanced_accuracy']=(r['positive_recall']+r['negative_recall'])/2
  r['model_sha256']=sha(d/'network.txt');result['models'][name]=r
  np.savez_compressed(output/(name+'_test_predictions.npz'),output=prediction)
 (output/'classifier_metrics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
