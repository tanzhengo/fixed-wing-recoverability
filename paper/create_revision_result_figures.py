#!/usr/bin/env python3
"""Plot fresh held-out revision results; never synthesize missing experiments."""
import argparse
from collections import Counter, defaultdict
import csv
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch
import numpy as np


METHODS = ['cost12','diverse12','full203','geometry_eager12','state_legacy_eager12',
           'geometry_bce_lazy12','geometry_aux_lazy12','state_bce_lazy12','state_aux_lazy12']
LABELS = {'cost12':'Cost','diverse12':'Diverse','full203':'Full',
          'geometry_eager12':'G-Legacy','state_legacy_eager12':'S-Legacy',
          'geometry_bce_lazy12':'G+BCE','geometry_aux_lazy12':'G+Aux',
          'state_bce_lazy12':'S+BCE','state_aux_lazy12':'S+Aux'}
CONDITIONS = [('nominal','Nominal'),('model_mismatch','Model mismatch'),
              ('crosswind','Crosswind'),('delay100ms','100 ms delay')]
OUTCOMES = [('goal','Goal','#248a77'),('no_feasible_primary','No feasible primary','#a7b0b9'),
            ('collision','Margin incursion','#c74e56'),('timeout','Timeout','#d99439'),
            ('envelope_violation','Envelope violation','#8a68ad'),
            ('invalid_dynamics','Invalid dynamics','#5e6481')]


def source_path(path):
    if path.exists():
        return path
    compressed=path.with_suffix(path.suffix+'.gz')
    if path.suffix=='.csv' and compressed.exists():
        return compressed
    raise FileNotFoundError(str(path))


def read_csv(path):
    path=source_path(path)
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as stream:
        return list(csv.DictReader(stream))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(fig, out, name):
    for extension in ['pdf','png']:
        target=out/(name+'.'+extension)
        temporary=out/('.'+name+'.tmp.'+extension)
        fig.savefig(temporary,dpi=220,bbox_inches='tight')
        temporary.replace(target)
    plt.close(fig)


def episodes(data):
    result = {}
    for prefix,folder in [('off',data),('on',data/'prefix')]:
        rows = read_csv(folder/'heldout_episodes.csv')
        assert len(rows)==1152, 'Incomplete nine-method closed-loop matrix: '+str(folder)
        assert set(r['method'] for r in rows)==set(METHODS)
        assert len({r['scene'] for r in rows})==32
        assert set(r['outcome'] for r in rows) <= {o[0] for o in OUTCOMES}
        for method in METHODS:
            for condition,_ in CONDITIONS:
                assert sum(r['method']==method and r['condition']==condition for r in rows)==32
        result[prefix]=rows
    assert {(r['scene'],r['condition'],r['method']) for r in result['off']} == {
            (r['scene'],r['condition'],r['method']) for r in result['on']}
    return result


def plot_outcomes(data,out):
    rows=episodes(data)
    present={r['outcome'] for mode in rows.values() for r in mode}
    outcomes=[o for o in OUTCOMES if o[0] in present]
    fig,axes=plt.subplots(2,4,figsize=(7.25,4.75),sharex=True,sharey=True)
    y=np.arange(len(METHODS))
    counts={}
    for row,(prefix,rowtitle) in enumerate([('off','Prefix off'),('on','Prefix on')]):
        counts[prefix]={}
        for col,(condition,title) in enumerate(CONDITIONS):
            ax=axes[row,col]
            local={method:Counter(r['outcome'] for r in rows[prefix]
                                 if r['condition']==condition and r['method']==method)
                   for method in METHODS}
            counts[prefix][condition]={method:dict(local[method]) for method in METHODS}
            left=np.zeros(len(METHODS))
            for outcome,_,color in outcomes:
                values=np.array([local[m][outcome] for m in METHODS])
                ax.barh(y,values,left=left,height=.70,color=color,edgecolor='white',linewidth=.25)
                if outcome=='goal':
                    for i,n in enumerate(values):
                        if n>=4: ax.text(n/2,i,str(n),ha='center',va='center',color='white',fontsize=6)
                left+=values
            assert np.all(left==32)
            ax.set_xlim(0,32)
            ax.set_xticks([0,16,32]);ax.tick_params(axis='both',length=0,labelsize=6.5)
            ax.set_yticks(y)
            if col==0: ax.set_yticklabels([LABELS[m] for m in METHODS],fontsize=6.8)
            ax.grid(axis='x',color='#d7dde2',lw=.4,zorder=0)
            ax.set_axisbelow(True)
            for spine in ax.spines.values():spine.set_visible(False)
            for split in [2.5,4.5]:ax.axhline(split,color='#d0d5da',lw=.5)
            if row==0:ax.set_title(title,fontsize=8,pad=6)
            if row==1:ax.set_xlabel('Tasks',fontsize=7)
        axes[row,0].text(-.58,.5,rowtitle,transform=axes[row,0].transAxes,
                        rotation=90,va='center',ha='center',fontsize=8,fontweight='bold')
    axes[0,0].invert_yaxis()
    handles=[Patch(facecolor=color,label=label) for _,label,color in outcomes]
    fig.legend(handles=handles,loc='lower center',ncol=len(handles),frameon=False,
               fontsize=6.7,columnspacing=1.3,handlelength=1.3,bbox_to_anchor=(.5,-.005))
    fig.subplots_adjust(left=.15,right=.99,top=.92,bottom=.105,wspace=.20,hspace=.19)
    save(fig,out,'revision_outcomes')
    return {'counts':counts,'episodes_per_prefix':1152,'tasks_per_method_condition':32,
            'outcome_label_mapping':{o:l for o,l,_ in outcomes}}


def plot_paths(data,out):
    folder=data/'prefix'
    rows=episodes(data)['on']
    methods=['state_aux_lazy12','geometry_aux_lazy12','cost12','diverse12','full203']
    labels={'state_aux_lazy12':'State+Aux','geometry_aux_lazy12':'Geometry+Aux',
            'cost12':'Cost','diverse12':'Diverse','full203':'Full'}
    colors={'state_aux_lazy12':'#16816f','geometry_aux_lazy12':'#d07932',
            'cost12':'#a16bb1','diverse12':'#3778b5','full203':'#666b72'}
    styles={'state_aux_lazy12':'-','geometry_aux_lazy12':'-','cost12':'--',
            'diverse12':'-','full203':':'}
    families=['staggered','wall','culdesac','spatial']
    selected={r['case_id']:r for r in rows if r['condition']=='nominal'
              and r['method'] in methods and r['scene'].endswith('_0')}
    assert len(selected)==20
    points=defaultdict(list)
    for r in read_csv(folder/'heldout_paths.csv'):
        if r['case_id'] in selected:
            points[r['case_id']].append([float(r[k]) for k in ['x','y','z']])
    obstacles=read_csv(folder/'heldout_scenarios.csv')
    fig,axes=plt.subplots(2,2,figsize=(7.15,5.0))
    case_records=[]
    for index,(ax,family) in enumerate(zip(axes.flat,families)):
        scene='review_'+family+'_0'
        local=[o for o in obstacles if o['scene']==scene]
        assert local and {int(o['seed']) for o in local}=={801000+index*1000}
        for o in local:
            sphere=o['type']=='sphere'
            ax.add_patch(Circle((float(o['x']),float(o['y'])),float(o['r']),
                               facecolor='#d8d5df' if sphere else '#cbd0d5',
                               edgecolor='#91879d' if sphere else '#939da5',
                               linestyle='--' if sphere else '-',linewidth=.6,alpha=.80))
        # Draw the selected configuration last so coincident trajectories remain visible.
        for method in reversed(methods):
            cases=[(key,r) for key,r in selected.items() if r['scene']==scene and r['method']==method]
            assert len(cases)==1
            case,row=cases[0];xyz=np.asarray(points[case]);assert len(xyz)>1
            ax.plot(xyz[:,0],xyz[:,1],color=colors[method],ls=styles[method],lw=1.25)
            ax.plot(xyz[-1,0],xyz[-1,1],marker='o' if row['outcome']=='goal' else 'x',
                    color=colors[method],ms=4,mew=1.0)
            case_records.append({'case_id':case,'scene':scene,'seed':801000+index*1000,
                                 'method':method,'outcome':row['outcome'],
                                 'path_states':len(xyz),'min_altitude':float(xyz[:,2].min()),
                                 'max_altitude':float(xyz[:,2].max())})
        ax.plot(0,0,'ks',ms=3.2);ax.plot(250,0,'k*',ms=6)
        title=family.capitalize() if family!='culdesac' else 'Cul-de-sac'
        ax.set(title='(%s) %s'%('abcd'[index],title),xlabel='x (m)',ylabel='y (m)',
               xlim=(-10,270),ylim=(-85,85),xticks=[0,100,200],yticks=[-50,0,50])
        ax.set_aspect('equal',adjustable='box')
        ax.grid(alpha=.20,lw=.5);ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
    handles=[Line2D([],[],color=colors[m],ls=styles[m],lw=1.3,label=labels[m]) for m in methods]
    fig.legend(handles=handles,loc='upper center',ncol=5,frameon=False,fontsize=7.5,
               bbox_to_anchor=(.52,1.005),columnspacing=1.1,handlelength=2.0)
    fig.text(.5,.012,'Circle: goal attained    Cross: other termination    Star: target',
             ha='center',fontsize=7)
    fig.subplots_adjust(left=.085,right=.99,top=.89,bottom=.11,wspace=.25,hspace=.45)
    save(fig,out,'revision_paths')
    return {'selection_rule':'First map in each family, nominal execution, prefix on; chosen before inspection.',
            'projection':'XY; physical obstacle projections, without inflation. Dashed spheres have finite height.',
            'cases':case_records}


def collapse_matched(path):
    rows=read_csv(path)
    cells=defaultdict(list)
    for row in rows:cells[(row['method'],row['state_id'])].append(row)
    result=defaultdict(dict)
    for (method,state),records in cells.items():
        assert len(records)==3,'Expected three timing repeats in '+str(path)
        signature=lambda r:tuple(r[k] for k in ['verified','status','rank','cost','backup_steps'])
        assert len({signature(r) for r in records})==1,'Decision changes across repeats'
        result[method][state]=records[0]
    assert len(result['full203'])==576,'Incomplete matched-state matrix'
    for method in result:assert set(result[method])==set(result['full203'])
    return result


def plot_matched(data,out):
    settings={mode:collapse_matched(folder/'heldout_matched.csv')
              for mode,folder in [('off',data),('on',data/'prefix')]}
    stems=['cost','diverse','geometry_bce_lazy','geometry_aux_lazy','state_bce_lazy','state_aux_lazy']
    labels=['Cost','Diverse','G+BCE','G+Aux','S+BCE','S+Aux']
    fig,axes=plt.subplots(1,2,figsize=(7.15,2.65),sharey=True)
    colors={'off':'#adb8c1','on':'#248a77'}
    records={};denominators={}
    for mode,methods in settings.items():
        positives={key for key,row in methods['full203'].items() if int(row['verified'])}
        denominators[mode]=len(positives);records[mode]={}
        for panel,(suffix,title) in enumerate([('_k12','(a) K = 12'),('_w3600','(b) W = 3,600')]):
            ax=axes[panel];values=[];counts=[]
            for stem in stems:
                method=stem+suffix
                found={key for key,row in methods[method].items() if int(row['verified'])}
                assert found<=positives
                count=len(found);counts.append(count);values.append(100*count/len(positives))
                records[mode][method]={'found':count,'oracle_positive':len(positives),'recall_percent':values[-1]}
            x=np.arange(len(stems))+(-.18 if mode=='off' else .18)
            ax.bar(x,values,width=.34,color=colors[mode],label='Prefix '+mode)
            for i,(xx,value,count) in enumerate(zip(x,values,counts)):
                peer=settings['on' if mode=='off' else 'off'][stems[i]+suffix]
                same=count==sum(int(r['verified']) for r in peer.values())
                if same and mode=='off':continue
                ax.text(i if same else xx,value+1.0,str(count),ha='center',va='bottom',fontsize=6.2)
            ax.set(title=title,xticks=range(len(stems)),xticklabels=labels,xlim=(-.6,5.6),ylim=(0,107))
            ax.tick_params(axis='x',length=0,labelsize=6.8)
            ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
            ax.grid(axis='y',alpha=.20,lw=.5);ax.set_axisbelow(True)
    axes[0].set_ylabel('Witness recall (%)')
    fig.legend(*axes[0].get_legend_handles_labels(),loc='lower center',ncol=2,
               frameon=False,fontsize=7.5,bbox_to_anchor=(.5,.025))
    fig.subplots_adjust(left=.075,right=.995,top=.88,bottom=.23,wspace=.16)
    save(fig,out,'revision_matched')
    return {'matched_states':576,'repeat_policy':'Three repeats must have identical decisions; count each state once.',
            'oracle_positive':denominators,'settings':records}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only',choices=['all','matched','outcomes','paths'],default='all')
    parser.add_argument('--allow-pending',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parent;data=root.parent/'docs/recovery/revision/runtime';out=root/'figures'
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.titlesize':8.5,
                         'axes.labelsize':8,'xtick.labelsize':7,'ytick.labelsize':7,
                         'pdf.fonttype':42,'ps.fonttype':42})
    manifest_path=data/'paper_figures.json'
    manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    functions={'outcomes':plot_outcomes,'paths':plot_paths,'matched':plot_matched}
    for name,function in functions.items():
        if args.only not in ['all',name]:continue
        try:
            report=function(data,out)
        except FileNotFoundError:
            if args.allow_pending and name=='matched':
                print('Pending complete matched-state timing CSVs; no placeholder created.')
                continue
            raise
        inputs=[data/'heldout_episodes.csv',data/'prefix/heldout_episodes.csv']
        if name=='paths':inputs += [data/'prefix/heldout_paths.csv',data/'prefix/heldout_scenarios.csv']
        if name=='matched':inputs=[data/'heldout_matched.csv',data/'prefix/heldout_matched.csv']
        inputs=[source_path(p) for p in inputs]
        report.update(source_sha256={str(p.relative_to(data)):sha(p) for p in inputs},
                      script_sha256=sha(Path(__file__)),figure_sha256=sha(out/('revision_'+name+'.pdf')))
        manifest[name]=report;print(out/('revision_'+name+'.pdf'))
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':
    main()
