#!/usr/bin/env python3
"""Render training evidence and copy audited held-out figures into the paper."""
from pathlib import Path
import json,shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size':8.5,'axes.titlesize':9,'pdf.fonttype':42})
root=Path(__file__).resolve().parent; data=root.parent/'docs/recovery/learning'; out=root/'figures'
f,axes=plt.subplots(1,2,figsize=(7,2.65))
colors={'geometry':'#287a70','state_only':'#aa6a30'}
for name,label in [('geometry','Geometry'),('state_only','No geometry')]:
 r=json.loads((data/'models'/name/'training.json').read_text());h=r['history'];x=[v['epoch'] for v in h];color=colors[name]
 axes[0].plot(x,[v['train']['bce'] for v in h],color=color,ls='--',lw=1.25,label=label+' train')
 axes[0].plot(x,[v['val']['bce'] for v in h],color=color,lw=1.5,label=label+' validation')
 axes[1].plot(x,[v['val']['auroc'] for v in h],color=color,lw=1.5,label=label)
 axes[1].plot(r['best_epoch'],h[r['best_epoch']]['val']['auroc'],'o',color=color,ms=4)
axes[0].set(title='(a) Feasibility classification loss',xlabel='Epoch',ylabel='Binary cross-entropy',ylim=(0,.75))
axes[1].set(title='(b) Validation discrimination',xlabel='Epoch',ylabel='AUROC',ylim=(.45,1.02))
for ax in axes:
 ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False);ax.grid(axis='y',alpha=.18);ax.legend(frameon=False,fontsize=7.5)
f.tight_layout(pad=.7)
f.savefig(out/'training.pdf',bbox_inches='tight');f.savefig(out/'training.png',dpi=200,bbox_inches='tight')
for name in ['matched','outcomes']:
 for ext in ['pdf','png']:shutil.copyfile(data/(name+'.'+ext),out/('learning_'+name+'.'+ext))
(root/'tables/learning.tex').write_text((data/'table.tex').read_text())

# Predetermined first instance of every test family, nominal execution.
import csv
from matplotlib.patches import Circle
families=['staggered','wall','culdesac','spatial']
methods=['cost12','diverse12','learned12','no_geometry','full203']
labels={'cost12':'Cost','diverse12':'Diverse','learned12':'Learned','no_geometry':'No geometry','full203':'Full'}
colors={'cost12':'#9467bd','diverse12':'#d98b34','learned12':'#167c76','no_geometry':'#bf4b4b','full203':'#42699a'}
with (data/'test_episodes.csv').open() as stream:
 episodes={r['case_id']:r for r in csv.DictReader(stream) if r['condition']=='nominal' and r['scene'].endswith('_0') and r['method'] in methods}
with (data/'geometry_ablation/test_episodes.csv').open() as stream:
 for r in csv.DictReader(stream):
  if r['condition']=='nominal' and r['scene'].endswith('_0') and r['method']=='learned12':
   r=dict(r);r['method']='no_geometry';episodes['nogeo_'+r['case_id']]=r
paths={k:[] for k in episodes}
with (data/'test_paths.csv').open() as stream:
 for r in csv.DictReader(stream):
  if r['case_id'] in paths: paths[r['case_id']].append((float(r['x']),float(r['y']),float(r['z'])))
with (data/'geometry_ablation/test_paths.csv').open() as stream:
 for r in csv.DictReader(stream):
  key='nogeo_'+r['case_id']
  if key in paths:paths[key].append((float(r['x']),float(r['y']),float(r['z'])))
with (data/'test_scenarios.csv').open() as stream: obstacles=list(csv.DictReader(stream))
f,axes=plt.subplots(2,2,figsize=(7,4.55))
for ax,family in zip(axes.flat,families):
 scene='test_'+family+'_0'
 for o in obstacles:
  if o['scene']==scene: ax.add_patch(Circle((float(o['x']),float(o['y'])),float(o['r']),fc='#c6cbd0',ec='#969ea6',lw=.6,alpha=.65))
 for method in methods:
  case=next(k for k,v in episodes.items() if v['scene']==scene and v['method']==method)
  points=paths[case];x,y,z=zip(*points)
  ax.plot(x,y,color=colors[method],lw=1.2,label=labels[method]);ax.plot(x[-1],y[-1],marker='o' if episodes[case]['outcome']=='goal' else 'x',color=colors[method],ms=4,mew=1)
 ax.plot(0,0,'ks',ms=3);ax.plot(250,0,'k*',ms=6)
 ax.set(title=family.capitalize(),xlabel='x (m)',ylabel='y (m)',xlim=(-10,270));ax.set_aspect('equal',adjustable='datalim')
 ax.grid(alpha=.16);ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
f.legend(*axes[0,0].get_legend_handles_labels(),loc='upper center',ncol=5,frameon=False,bbox_to_anchor=(.52,1.025))
f.tight_layout(pad=.6,rect=(0,0,1,.96));f.savefig(out/'learning_paths.pdf',bbox_inches='tight');f.savefig(out/'learning_paths.png',dpi=200,bbox_inches='tight')
