#!/usr/bin/env python3
"""Complete-label K12 replay and frozen validation-only model selection."""
import argparse,csv,hashlib,json,math,subprocess,sys
from pathlib import Path
import numpy as np
F=np.float32

def probability(logit):
 out=np.empty(len(logit),dtype=np.float32);pos=logit>=0;out[pos]=1/(1+np.exp(-logit[pos]));e=np.exp(logit[~pos]);out[~pos]=e/(1+e);return out

def schedule(features,costs,prob,mode,count=12,alpha=.5):
 n=len(costs);order=np.argsort(costs,kind='stable');limit=min(n,count)
 if mode=='cost':return order[:limit].tolist()
 if not n:return []
 headings=np.arctan2(features[:,15],features[:,16]);heights=features[:,14]*F(30);speed=np.maximum(F(3),features[0,6]*F(20));horizon=features[0,22]*F(10)
 hspan=np.minimum(F(np.pi),F(9.81)*np.abs(np.tan(F(35*np.pi/180)))*horizon/speed);zspan=speed*horizon*np.tan(F(20*np.pi/180))
 if mode=='observed_coverage':prob=np.max(features[:,28::2],axis=1)
 learned=order[np.argsort(-prob[order],kind='stable')];used=np.zeros(n,dtype=bool);closest=np.full(n,np.inf,dtype=np.float32);chosen=[int(order[0])];used[chosen[0]]=True
 for _ in range(1,limit):
  last=chosen[-1];difference=headings-headings[last];angle=np.abs(np.arctan2(np.sin(difference),np.cos(difference)))
  hd=angle/hspan if hspan>F(1e-5) else np.zeros(n,dtype=np.float32);zd=(heights-heights[last])/zspan if zspan>F(1e-5) else np.zeros(n,dtype=np.float32);closest=np.minimum(closest,hd*hd+zd*zd)
  if mode=='learned':current=int(next(i for i in learned if not used[i]))
  else:
   priority=closest
   if mode in ['learned_coverage','observed_coverage']:
    distance=np.sqrt(closest);priority=F(alpha)*prob+(F(1)-F(alpha))*distance/(F(1)+distance)
   best=F(-1);current=-1
   for i in order:
    if not used[i] and priority[i]>F(best+F(1e-7)):best=priority[i];current=int(i)
   assert current>=0
  used[current]=True;chosen.append(current)
 return chosen

def replay(features,costs,labels,work,prob,mode,count=12,alpha=.5):
 r=dict(found=False,checked=0,steps=0,pruned=0,selected_rank=-1,cost=None,regret=None,inference=False)
 if not len(costs):return r
 if labels[0]:r.update(found=True,checked=1,steps=int(work[0]),selected_rank=1,cost=float(costs[0]),regret=0.0);return r
 chosen=schedule(features,costs,prob,mode,count,alpha);r['inference']=mode in ['learned','learned_coverage'] and len(costs)>1
 for i in chosen:
  if r['found'] and (costs[i]>r['cost'] or (costs[i]==r['cost'] and i+1>=r['selected_rank'])):r['pruned']+=1;continue
  r['checked']+=1;r['steps']+=int(work[i])
  if not labels[i]:continue
  r.update(found=True,selected_rank=i+1,cost=float(costs[i]))
  if mode=='cost' or i==0:break
 if r['found']:r['regret']=max(0.,r['cost']-float(np.min(costs[labels.astype(bool)])))
 return r

def evaluate(data,states,output,cell,seed,mode,alpha,count=12):
 probs=(output[:,2] if output.shape[1]>=3 else probability(output[:,0])) if output is not None else np.zeros(len(data['y']),dtype=np.float32);rows=[]
 # Packed candidate rows are contiguous by query, including gaps for empty queries.
 ids=data['state'];starts=np.searchsorted(ids,np.arange(len(states)+1))
 for s in states:
  sid=int(s['state_id']);a,b=starts[sid],starts[sid+1];x=data['x'][a:b];cost=data['cost'][a:b];label=data['y'][a:b];work=data['work'][a:b];p=probs[a:b]
  assert b-a==int(s['feasible']);assert int(label.sum())==int(s['positive'])
  row={k:s[k] for k in ['state_id','scene','family','seed','sampler','t','active','feasible','positive','outcome']};row.update(model_cell=cell,model_seed=seed,mode=mode,alpha=alpha,budget=count,full_positive=bool(len(label) and np.any(label)),**replay(x,cost,label,work,p,mode,count,alpha));rows.append(row)
 positives=sum(r['full_positive'] for r in rows);found=sum(r['found'] for r in rows)
 summary=dict(cell=cell,seed=seed,mode=mode,alpha=alpha,budget=count,queries=len(rows),full_positive=positives,found=found,recall=found/positives if positives else None,mean_steps=float(np.mean([r['steps'] for r in rows])),mean_checks=float(np.mean([r['checked'] for r in rows])),mean_pruned=float(np.mean([r['pruned'] for r in rows])),mean_regret=float(np.mean([r['regret'] for r in rows if r['found']])),inference_calls=sum(r['inference'] for r in rows),empty_queries=sum(int(r['feasible'])==0 for r in rows))
 return rows,summary

def writecsv(path,rows):
 with path.open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def main():
 p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--models',type=Path,required=True);a=p.parse_args();
 if not (a.models/'export_parity.json').exists():subprocess.run([sys.executable,str(Path(__file__).with_name('check_export.py')),'--data',str(a.data),'--models',str(a.models)],check=True)
 data=dict(np.load(a.data/'val.npz',allow_pickle=False));states=list(csv.DictReader((a.data/'val_states.csv').open()));assert all(s['split']=='val' for s in states)
 out=a.models/'validation';out.mkdir(exist_ok=True);allrows=[];summaries=[]
 for name,mode,count,alpha in [('Cost','cost',12,.5),('Diverse','diverse',12,.5),('Full','cost',10000,.5)]+[('ObservedCoverage','observed_coverage',12,v) for v in [.25,.5,.75]]:
  rows,summary=evaluate(data,states,None,name,0,mode,alpha,count);allrows+=rows;summaries.append(summary)
 for cell in ['state_aux','observed','observed_aux']:
  for seed in [20261101,20261102,20261103]:
   folder=a.models/cell/str(seed);report=json.loads((folder/'training.json').read_text());output=np.load(folder/'validation_deployed_predictions.npz')['output']
   for mode,alpha in [('learned',0.)]+[('learned_coverage',v) for v in [.25,.5,.75]]:
    rows,summary=evaluate(data,states,output,cell,seed,mode,alpha);summary.update(bce=report['validation_metrics']['bce'],best_epoch=report['best_epoch'],model_path=str((folder/'network_compact.txt').resolve()),model_sha256=report['network_sha256']);allrows+=rows;summaries.append(summary)
 writecsv(out/'query_outcomes.csv',allrows)
 # CSV stable union columns permits deterministic baseline rows without model BCE.
 columns=list(dict.fromkeys(k for r in summaries for k in r))
 with (out/'summary.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=columns);w.writeheader();w.writerows(summaries)
 def key(r):return (-r['recall'],r['mean_steps'],r.get('bce',0),r['seed'],r['alpha'])
 selected={}
 for cell in ['state_aux','observed','observed_aux']:
  selected[cell]=min((r for r in summaries if r['cell']==cell and r['mode']=='learned'),key=key)
  selected['diagnostic_query_'+cell+'_coverage']=min((r for r in summaries if r['cell']==cell and r['mode']=='learned_coverage' and r['seed']==selected[cell]['seed']),key=key)
 selected['diagnostic_query_observed_coverage']=min((r for r in summaries if r['cell']=='ObservedCoverage'),key=key)
 selected['proposed_model']=selected['observed_aux']
 selected['coverage_status']='query alpha grid is diagnostic only; closed-loop validation chooses alpha for this fixed seed before test'
 temporary=out/'selected.tmp';temporary.write_text(json.dumps(selected,indent=2)+'\n');temporary.replace(out/'selected.json');print(json.dumps(selected,indent=2),flush=True)
if __name__=='__main__':main()
