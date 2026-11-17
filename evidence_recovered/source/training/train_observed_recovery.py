#!/usr/bin/env python3
"""OBS58 state/observation ablation using complete sensor-map oracle labels.

Only train/validation data are accepted. All cells share initialization, widths,
optimizer and checkpoint criterion. Export keeps only the feasibility output.
"""
import argparse,csv,hashlib,json,time,os
from pathlib import Path
import numpy as np
import torch
from torch import nn
import train_recovery_network as metrics_library

SEEDS=(20261101,20261102,20261103)
CELLS={'state_aux':(True,0.1),'observed':(False,0.0),'observed_aux':(False,0.1)}

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write_json(path,data):Path(path).write_text(json.dumps(data,indent=2)+'\n')
class Network(nn.Module):
 def __init__(self,state_only=False):
  super().__init__();self.state_only=state_only;self.layers=nn.Sequential(nn.Linear(58,64),nn.ReLU(),nn.Linear(64,32),nn.ReLU(),nn.Linear(32,2))
  with torch.no_grad():self.layers[4].bias[1]=6.0
 def forward(self,x):
  if self.state_only:x=torch.cat((x[:,:28],torch.zeros_like(x[:,28:])),dim=1)
  return self.layers(x)
def predict(model,x):
 model.eval()
 with torch.no_grad():return torch.cat([model(torch.from_numpy(v)) for v in np.array_split(x,max(1,(len(x)+4095)//4096))]).numpy()
def load(path,split):
 data=dict(np.load(path,allow_pickle=False));assert data['x'].shape[1]==58 and np.isfinite(data['x']).all();assert set(np.unique(data['y']))=={0,1}
 states=list(csv.DictReader(path.with_name(split+'_states.csv').open()));assert all(r['split']==split for r in states)
 assert set(np.unique(data['state'])).issubset({int(r['state_id']) for r in states});return data,states
def export(model,path,validation,output):
 inputs=28 if model.state_only else 58;arrays=[]
 for i in [0,2,4]:
  w=model.layers[i].weight.detach().numpy().copy();b=model.layers[i].bias.detach().numpy().copy()
  if i==0:w=w[:,:inputs]
  if i==4:w=w[:1];b=b[:1]
  arrays.extend((w.reshape(-1),b))
 with path.open('w') as f:
  f.write(f'RECOVERY_MLP_V2 {inputs} 64 32 1\n')
  for v in arrays:f.write(' '.join('%.9g'%a for a in v)+'\n')
 ids=np.linspace(0,len(validation['x'])-1,min(128,len(validation['x'])),dtype=int)
 np.savetxt(path.with_name('export_parity.csv'),np.c_[validation['x'][ids,:inputs],output[ids,0]],delimiter=',',header=','.join([f'f{i}' for i in range(inputs)]+['logit']),comments='')
 return inputs,sum(len(a) for a in arrays)
def train_one(name,seed,train,val,args,protocol_sha):
 state,aux=CELLS[name];dest=args.output/name/str(seed);dest.mkdir(parents=True,exist_ok=True)
 if (dest/'training.json').exists():print(json.dumps({'cached':name,'seed':seed}),flush=True);return
 torch.manual_seed(seed);np.random.seed(seed);model=Network(state);optimizer=torch.optim.Adam(model.parameters(),lr=.001,weight_decay=1e-5)
 x=torch.from_numpy(train['x']);y=torch.from_numpy(train['y']);work=torch.from_numpy(np.log1p(train['work']).astype(np.float32));best=float('inf');history=[];start=time.monotonic()
 for epoch in range(args.epochs+1):
  if epoch:
   model.train();total=np.zeros(3)
   for idx in torch.randperm(len(x)).split(args.batch_size):
    optimizer.zero_grad();output=model(x[idx]);bce=nn.functional.binary_cross_entropy_with_logits(output[:,0],y[idx]);huber=nn.functional.smooth_l1_loss(output[:,1],work[idx]);loss=bce+aux*huber;loss.backward();optimizer.step();total+=np.array([loss.item(),bce.item(),huber.item()])*len(idx)
   tm=dict(zip(['loss','bce','huber'],(total/len(x)).tolist()))
  else:tm=metrics_library.metrics(predict(model,train['x']),train)
  output=predict(model,val['x']);vm=metrics_library.metrics(output,val);history.append(dict(epoch=epoch,train=tm,validation=vm,elapsed_s=time.monotonic()-start))
  if vm['bce']<best:best=vm['bce'];bestepoch=epoch;torch.save(model.state_dict(),dest/'checkpoint.pt')
  if epoch%10==0 or epoch==args.epochs:print(json.dumps(dict(cell=name,seed=seed,epoch=epoch,val_bce=vm['bce'],elapsed_s=time.monotonic()-start)),flush=True)
 model.load_state_dict(torch.load(dest/'checkpoint.pt',weights_only=True));output=predict(model,val['x']);np.savez_compressed(dest/'validation_predictions.npz',output=output)
 inputs,parameters=export(model,dest/'network_compact.txt',val,output)
 report=dict(cell=name,seed=seed,auxiliary_weight=aux,training_architecture=[58,64,32,2],state_only=state,inference_architecture=[inputs,64,32,1],parameters_inference=parameters,epochs=args.epochs,best_epoch=bestepoch,checkpoint_rule='minimum validation BCE; earliest exact tie',validation_metrics=metrics_library.metrics(output,val),training_seconds=time.monotonic()-start,protocol_sha256=protocol_sha,network_sha256=sha(dest/'network_compact.txt'),history=history)
 write_json(dest/'training.json',report);print(json.dumps(dict(completed=name,seed=seed,best_epoch=bestepoch,seconds=report['training_seconds'])),flush=True)
def main():
 p=argparse.ArgumentParser();p.add_argument('--data',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--epochs',type=int,default=60);p.add_argument('--batch-size',type=int,default=2048);p.add_argument('--threads',type=int,default=2);p.add_argument('--cell',choices=['all']+list(CELLS),default='all');p.add_argument('--seed',type=int,choices=SEEDS);a=p.parse_args();torch.set_num_threads(a.threads);torch.use_deterministic_algorithms(True);a.output.mkdir(parents=True,exist_ok=True)
 train,ts=load(a.data/'train.npz','train');val,vs=load(a.data/'val.npz','val');assert not {r['seed'] for r in ts}&{r['seed'] for r in vs}
 protocol=dict(schema='OBS58',seeds=list(SEEDS),cells=CELLS,epochs=a.epochs,batch_size=a.batch_size,optimizer='Adam, lr0.001, weight_decay0.00001',checkpoint='minimum validation BCE; earliest exact tie',export='compact V2 feasibility head, 28 or58 input and64-32 hidden units',validation_schedule='K12; common cheapest-first seed, incumbent cost/rank pruning; no extra RK4 prefix; candidate set fixed before pruning; deferred inference',selection='per variant: maximum validation positive-query witness recall, minimum mean continuation steps, minimum BCE, minimum numeric seed; alpha for selected Obs+Aux seed chosen separately on16validation nominal closed-loop tasks: maximum completions, minimum per-update work, then lower alpha',data_sha256={f:sha(a.data/f) for f in ['train.npz','val.npz','train_states.csv','val_states.csv']},script_sha256=sha(__file__),torch_version=torch.__version__,train_scenes=len({r['scene'] for r in ts}),val_scenes=len({r['scene'] for r in vs}),train_queries=len(ts),val_queries=len(vs),train_candidates=len(train['y']),val_candidates=len(val['y']))
 protocol=json.loads(json.dumps(protocol))
 pp=a.output/'training_protocol.json'
 if pp.exists():assert json.loads(pp.read_text())==protocol,'Refuse changed training protocol within one run'
 else:
  temporary=pp.with_name('training_protocol.%d.json'%os.getpid());write_json(temporary,protocol);os.replace(temporary,pp)
 for name in (CELLS if a.cell=='all' else [a.cell]):
  for seed in (SEEDS if a.seed is None else [a.seed]):train_one(name,seed,train,val,a,sha(pp))
if __name__=='__main__':main()
