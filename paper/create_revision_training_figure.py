#!/usr/bin/env python3
"""Render the three-seed controlled validation ablation from saved results."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


def main():
    root = Path(__file__).resolve().parent
    data = root.parent / 'docs/recovery/revision/training'
    source = data / 'summary.json'
    summary = json.loads(source.read_text())
    keys = ['geometry_bce', 'geometry_joint', 'state_bce', 'state_joint']
    cells = {r['cell']:r for r in summary['cells']}
    colors = ['#287a70', '#287a70', '#b36b31', '#b36b31']
    labels = ['Geom.\nBCE', 'Geom.\nAux', 'State\nBCE', 'State\nAux']
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':8,
                         'axes.titlesize':8.5, 'axes.labelsize':8,
                         'xtick.labelsize':7, 'ytick.labelsize':7,
                         'pdf.fonttype':42, 'ps.fonttype':42})
    fig, axes = plt.subplots(1,3,figsize=(7.15,2.6))
    for ax,metric,title in zip(axes,['bce','K12','W3600'],
                              ['(a) Validation BCE','(b) K = 12','(c) W = 3,600']):
        for i,key in enumerate(keys):
            if metric == 'bce':
                values = [r['validation_metrics']['bce'] for r in summary['runs'] if r['cell']==key]
            else:
                values = 100*np.asarray(cells[key][metric]['values'])
            mean,sd = float(np.mean(values)),float(np.std(values,ddof=1))
            ax.scatter(i+np.array([-.08,0,.08]),values,s=17,color=colors[i],
                       edgecolors='white',linewidths=.4,zorder=3)
            ax.errorbar(i,mean,yerr=sd,fmt='D',color='#252525',ms=3.5,
                        elinewidth=.9,capsize=3,zorder=4)
        ax.set(title=title,xticks=range(4),xticklabels=labels,xlim=(-.45,3.45))
        ax.set_ylabel('BCE loss' if metric=='bce' else 'Witness recall (%)')
        ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
        ax.grid(axis='y',color='#d9dfe4',linewidth=.5,zorder=0)
        ax.tick_params(axis='x',length=0,pad=5)
    axes[0].set_ylim(0,.32)
    axes[1].set_ylim(94.8,99.2)
    axes[2].set_ylim(83.8,89.5)
    fig.legend(handles=[Line2D([],[],ls='',marker='o',color='#287a70',ms=4,label='Individual seeds'),
                        Line2D([],[],ls='',marker='D',color='#252525',ms=4,label='Mean ± sample SD')],
               loc='lower center',ncol=2,frameon=False,fontsize=7.5,bbox_to_anchor=(.5,-.01))
    fig.subplots_adjust(left=.075,right=.99,top=.88,bottom=.28,wspace=.43)
    out=root/'figures';out.mkdir(exist_ok=True)
    fig.savefig(out/'revision_training.pdf',bbox_inches='tight')
    fig.savefig(out/'revision_training.png',dpi=220,bbox_inches='tight')
    plt.close(fig)
    freeze=json.loads((root.parent/'docs/recovery/revision/runtime/EXECUTION_MODEL_FREEZE.json').read_text())
    goals={r['configuration']:r['completed'] for r in freeze['ranked_validation_configurations']}
    lines=[r'\begin{tabular}{llrrrrr}',r'\toprule',
           r'Input & Loss & BCE & AUROC & K12 (\%) & W3600 (\%) & Val. goals \\',r'\midrule']
    for key in keys:
        d=cells[key]
        lines.append('%s & %s & %.4f & %.4f & %.2f & %.2f & %d/128 \\\\'%
                     ('Geometry' if key.startswith('geometry') else 'State',
                      'BCE' if key.endswith('bce') else 'Aux',
                      d['mean_validation_bce'],d['mean_validation_auroc'],
                      100*d['K12']['mean'],100*d['W3600']['mean'],goals[key]))
    lines += [r'\bottomrule',r'\end{tabular}',
              '% Prediction and query metrics are three-seed validation means.',
              '% Execution counts use fixed seed 20261030 on the same 128 validation tasks.',
              '% Aux denotes BCE + 0.1 Huber; all checkpoints use validation BCE.']
    (data/'paper_table.txt').write_text('\n'.join(lines)+'\n')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest={'source_sha256':sha(source),'script_sha256':sha(Path(__file__)),
              'figure_pdf_sha256':sha(out/'revision_training.pdf'),
              'seeds':[20261030,20261031,20261032],
              'data_split':'validation only','panels':['BCE','K12 recall','W3600 recall'],
              'display':'Three individual seed dots plus mean and sample standard deviation.'}
    (data/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(out/'revision_training.pdf')


if __name__=='__main__':
    main()
