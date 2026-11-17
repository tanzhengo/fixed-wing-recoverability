#!/usr/bin/env python3
"""Train continuation feasibility/work heads on explicit validator labels.

No held-out test data is accepted by this program. Model selection minimizes
validation BCE + 0.1 Huber, then a separate validation replay selects scheduling
work exponents. Exported dense weights run in the ROS C++ planner.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
import torch
from torch import nn


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def load_data(path):
    path = Path(path)
    cache = path.with_suffix('.npz')
    digest = sha(path)
    if cache.exists():
        d = dict(np.load(cache, allow_pickle=False))
        if str(d['source_sha256']) == digest:
            return d
    x, y, work, state, scene, family, cost, candidate = [], [], [], [], [], [], [], []
    with path.open() as f:
        reader = csv.DictReader(f)
        assert [k for k in reader.fieldnames if k.startswith('f') and k[1:].isdigit()] == ['f%d' % i for i in range(103)]
        for row in reader:
            x.append([float(row['f%d' % i]) for i in range(103)])
            y.append(int(row['verified']))
            work.append(int(row['backup_steps']))
            state.append(int(row['state_id']))
            scene.append(row['scene']); family.append(row['family'])
            cost.append(float(row['cost'])); candidate.append(int(row['candidate_id']))
    d = dict(x=np.array(x, dtype=np.float32), y=np.array(y, dtype=np.float32),
             work=np.array(work, dtype=np.int32), state=np.array(state, dtype=np.int32),
             scene=np.array(scene), family=np.array(family), cost=np.array(cost),
             candidate=np.array(candidate), source_sha256=np.array(digest))
    assert np.isfinite(d['x']).all() and (d['work'] >= 0).all()
    assert set(np.unique(d['y'])) <= {0, 1}
    temporary = cache.with_name(cache.stem + '.%d.npz' % os.getpid())
    np.savez_compressed(temporary, **d)
    os.replace(temporary, cache)
    return d


class Network(nn.Module):
    def __init__(self, state_only=False):
        super().__init__()
        self.layers = nn.Sequential(nn.Linear(103, 64), nn.ReLU(), nn.Linear(64, 32), nn.ReLU(), nn.Linear(32, 2))
        self.state_only = state_only
        with torch.no_grad():
            self.layers[4].bias[1] = 6.0

    def forward(self, x):
        if self.state_only:
            x = torch.cat((x[:, :28], torch.zeros_like(x[:, 28:])), dim=1)
        return self.layers(x)


def predict(model, x):
    model.eval()
    with torch.no_grad():
        return torch.cat([model(torch.from_numpy(v)) for v in np.array_split(x, max(1, (len(x)+4095)//4096))]).numpy()


def losses(output, target, logwork):
    bce = nn.functional.binary_cross_entropy_with_logits(output[:, 0], target)
    huber = nn.functional.smooth_l1_loss(output[:, 1], logwork)
    return bce + .1 * huber, bce, huber


def metrics(output, data):
    p = 1/(1+np.exp(-np.clip(output[:, 0], -50, 50)))
    y = data['y']
    bce = float(np.mean(np.logaddexp(0, output[:, 0])-y*output[:, 0]))
    error = np.abs(output[:, 1]-np.log1p(data['work']))
    huber = float(np.mean(np.where(error < 1, .5*error**2, error-.5)))
    # Mann--Whitney AUROC with exact tie averaging.
    order = np.argsort(p, kind='stable'); sortedp = p[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sortedp))+1]; ends = np.r_[starts[1:],len(p)]
    ranks = np.empty(len(p), dtype=float)
    for a,b in zip(starts,ends): ranks[order[a:b]] = (a+1+b)/2
    positive = y.sum(); negative = len(y)-positive
    auc = float((ranks[y==1].sum()-positive*(positive+1)/2)/(positive*negative))
    return dict(loss=bce+.1*huber,bce=bce,huber=huber,brier=float(np.mean((p-y)**2)),
                auroc=auc,accuracy=float(np.mean((p>=.5)==y)),log_work_mae=float(error.mean()),
                count=len(y),positive=int(positive))


def export(model, path):
    values=[]
    for i in [0,2,4]:
        weight=model.layers[i].weight.detach().numpy().copy()
        if model.state_only and i==0: weight[:,28:]=0
        values.extend([weight.reshape(-1),model.layers[i].bias.detach().numpy()])
    with open(path,'w') as f:
        f.write('RECOVERY_MLP_V1 103 64 32 2\n')
        for v in values: f.write(' '.join('%.9g' % x for x in v)+'\n')


def train(args):
    torch.set_num_threads(args.threads)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(args.seed); np.random.seed(args.seed)
    out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    train=load_data(args.train); val=load_data(args.val)
    assert not set(train['scene']) & set(val['scene']), 'Map leakage'
    model=Network(args.state_only)
    optimizer=torch.optim.Adam(model.parameters(),lr=.001,weight_decay=1e-5)
    x=torch.from_numpy(train['x']); y=torch.from_numpy(train['y'])
    lw=torch.from_numpy(np.log1p(train['work']).astype(np.float32))
    history=[]; best=float('inf'); best_epoch=0; start=time.monotonic()
    for epoch in range(args.epochs+1):
        if epoch:
            model.train(); permutation=torch.randperm(len(x)); sums=np.zeros(3)
            for idx in permutation.split(args.batch_size):
                optimizer.zero_grad(); output=model(x[idx]); loss,bce,huber=losses(output,y[idx],lw[idx])
                loss.backward(); optimizer.step()
                sums += np.array([loss.item(),bce.item(),huber.item()])*len(idx)
            train_metrics=dict(zip(['loss','bce','huber'],(sums/len(x)).tolist()))
        else:
            train_metrics=metrics(predict(model,train['x']),train)
        val_output=predict(model,val['x']); vm=metrics(val_output,val)
        record=dict(epoch=epoch,train=train_metrics,val=vm,elapsed_s=time.monotonic()-start)
        history.append(record)
        if vm['loss']<best:
            best=vm['loss']; best_epoch=epoch
            torch.save(model.state_dict(),out/'checkpoint.pt')
        print(json.dumps(record),flush=True)
    model.load_state_dict(torch.load(out/'checkpoint.pt',weights_only=True))
    export(model,out/'network.txt')
    val_output=predict(model,val['x'])
    np.savez_compressed(out/'validation_predictions.npz',output=val_output)
    # Cross-language forward-parity fixture covers real validation features.
    indices=np.linspace(0,len(val['x'])-1,32,dtype=int)
    with (out/'parity.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(['f%d'%i for i in range(103)]+['logit','log_work'])
        writer.writerows(np.c_[val['x'][indices],val_output[indices]])
    report=dict(seed=args.seed,torch_version=torch.__version__,numpy_version=np.__version__,
                architecture=[103,64,32,2],parameters=sum(p.numel() for p in model.parameters()),
                state_only=args.state_only,epochs=args.epochs,best_epoch=best_epoch,
                optimizer='Adam',learning_rate=.001,weight_decay=1e-5,batch_size=args.batch_size,
                loss='BCEWithLogits + 0.1 SmoothL1(log1p(integration_steps))',
                train_samples=len(x),train_maps=len(set(train['scene'])),val_maps=len(set(val['scene'])),
                train_sha256=str(train['source_sha256']),val_sha256=str(val['source_sha256']),
                script_sha256=sha(__file__),model_sha256=sha(out/'network.txt'),
                val_metrics=metrics(val_output,val),training_seconds=time.monotonic()-start,history=history)
    (out/'training.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--train',required=True);p.add_argument('--val',required=True)
    p.add_argument('--output',required=True);p.add_argument('--state-only',action='store_true')
    p.add_argument('--seed',type=int,default=20261030);p.add_argument('--epochs',type=int,default=60)
    p.add_argument('--batch-size',type=int,default=2048);p.add_argument('--threads',type=int,default=4)
    train(p.parse_args())
