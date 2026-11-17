#!/usr/bin/env python3
"""Pack complete observed-map oracle labels, retaining empty query metadata."""
import argparse,csv,hashlib,json
from pathlib import Path
import numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,default=Path('/tmp/codex-observed-training'));p.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'dataset');a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 summaries={}
 for split in ['train','val']:
  cases=sorted((a.raw/split).glob('*_summary.csv'));expected=192 if split=='train' else 48;assert len(cases)==expected,(split,len(cases),expected)
  blocks=[];allstates=[];casesummary=[];nextstate=0;sources={}
  for summaryfile in cases:
   key=summaryfile.name[:-len('_summary.csv')];prefix=summaryfile.parent/key
   case=list(csv.DictReader(summaryfile.open()))[0];casesummary.append(case)
   states=list(csv.DictReader(Path(str(prefix)+'_states.csv').open()));data=np.fromfile(Path(str(prefix)+'_candidates.f32'),dtype='<f4').reshape(-1,115)
   assert len(states)==int(case['queries']);n=len(data);sid=np.empty(n,dtype=np.int32)
   for row in states:
    idx=data[:,0].astype(int)==int(row['query']);assert idx.sum()==int(row['feasible']);assert int(data[idx,3].sum())==int(row['positive'])
    sid[idx]=nextstate;row.update(state_id=nextstate,case=key,split=split,family=case['family'],seed=case['seed'],sampler=case['sampler'],scene=f"{split}_{case['family']}_{case['index']}",outcome=case['outcome']);allstates.append(row);nextstate+=1
   assert np.isfinite(data).all();assert set(np.unique(data[:,3]))<={0.,1.};assert data[:,4].min(initial=0)>=0
   successors=data[:,70:].reshape(n,15,3);assert (np.max(successors[:,:,0],axis=1)==data[:,3]).all()
   blocks.append(dict(x=data[:,12:70],y=data[:,3],work=data[:,4].astype(np.int32),cost=data[:,2],rank=data[:,1].astype(np.int32),candidate=data[:,11].astype(np.int32),state=sid,successor_label=successors[:,:,0].astype(np.uint8),successor_work=successors[:,:,1].astype(np.int16),successor_reason=successors[:,:,2].astype(np.uint8)))
   for suffix in ['_candidates.f32','_states.csv','_path.csv','_geometry.csv','_summary.csv']:
    f=Path(str(prefix)+suffix);sources[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()
  dataset={name:np.concatenate([b[name] for b in blocks]) for name in blocks[0]};assert len(dataset['y'])<=1000000
  np.savez_compressed(a.output/(split+'.npz'),**dataset)
  for name,rows in [('states',allstates),('cases',casesummary)]:
   with (a.output/(split+'_'+name+'.csv')).open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
  report=dict(cases=len(cases),scenes=len(set(r['scene'] for r in allstates)),queries=len(allstates),candidates=len(dataset['y']),positive_candidates=int(dataset['y'].sum()),full_positive_queries=sum(int(r['positive'])>0 for r in allstates),full_negative_nonempty_queries=sum(int(r['positive'])==0 and int(r['feasible'])>0 for r in allstates),empty_queries=sum(int(r['feasible'])==0 for r in allstates),source_sha256=sources)
  summaries[split]=report;print(json.dumps({split:{k:v for k,v in report.items() if k!='source_sha256'}}),flush=True)
 (a.output/'manifest.json').write_text(json.dumps(summaries,indent=2)+'\n')
if __name__=='__main__':main()
