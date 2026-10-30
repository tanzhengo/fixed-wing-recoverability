#!/usr/bin/env python3
"""Summarize paired held-out review experiments without selecting models."""
from pathlib import Path
import argparse,csv,json,hashlib
from collections import defaultdict,Counter
import numpy as np
from review_runtime_io import csv_rows, csv_exists, raw_sha256, logical_csv_paths

def rows(path):
    return csv_rows(path)

def interval(a,b,keys):
    maps=sorted(set(k[0] for k in keys)); grouped=[]
    for m in maps:
        ks=[k for k in keys if k[0]==m]
        grouped.append([sum(a[k] for k in ks),sum(b[k] for k in ks),len(ks)])
    g=np.array(grouped,float);rng=np.random.default_rng(20261030)
    sample=g[rng.integers(0,len(g),size=(10000,len(g)))].sum(axis=1)
    delta=(sample[:,0]-sample[:,1])/sample[:,2]*100
    return {'difference_percentage_points':float((g[:,0].sum()-g[:,1].sum())/g[:,2].sum()*100),
            'ci95_map_block_bootstrap':np.percentile(delta,[2.5,97.5]).tolist(),'map_blocks':len(g)}

def main():
    p=argparse.ArgumentParser();p.add_argument('--directory',type=Path,required=True);args=p.parse_args();root=args.directory
    report={};grouped=defaultdict(list)
    if csv_exists(root/'heldout_matched.csv'):
        for r in rows(root/'heldout_matched.csv'):grouped[(r['state_id'],r['method'])].append(r)
        reduced={}
        decision_keys=['status', 'primary_feasible', 'verified', 'reused', 'primary_count', 'feasible_count', 'recovery_checked', 'backup_evaluated', 'primary_steps', 'backup_steps', 'cached_steps', 'rank', 'cost', 'reject_collision', 'reject_envelope', 'reject_known', 'reject_invalid', 'truncated', 'work_exhausted']
        for key,records in grouped.items():
            assert all(all(r[k]==records[0][k] for k in decision_keys) for r in records),key
            d=dict(records[0]);d['planning_ms']=float(np.median([float(r['planning_ms']) for r in records]));reduced[key]=d
        full={k[0]:r for k,r in reduced.items() if k[1]=='full203'}
        positives={k for k,r in full.items() if r['verified']=='1'}
        summary={}
        for method in sorted({k[1] for k in reduced}):
            rr=[r for (state,m),r in reduced.items() if m==method]
            ms=[r['planning_ms'] for r in rr]
            verified=sum(r['verified']=='1' for r in rr)
            summary[method]={'inputs':len(rr),'found':verified,'reference_positive':len(positives),'recall_percent':100*verified/len(positives),
                'median_ms':float(np.median(ms)),'p95_ms':float(np.percentile(ms,95)),'mean_ms':float(np.mean(ms)),
                'mean_backup_steps':float(np.mean([float(r['backup_steps']) for r in rr])),
                'mean_primary_steps':float(np.mean([float(r['primary_steps']) for r in rr]))}
        report['matched']=summary;report['matched_repeats']=len(next(iter(grouped.values())))
        pairs={}
        for a,b in [('geometry_eager','geometry_lazy'),('state_legacy_eager','state_compact_eager'),('state_legacy_eager','state_compact_lazy'),('selected_eager','selected_lazy')]:
            for suffix in ['_k12','_w3600']:
                if (next(iter(full)),a+suffix) not in reduced:continue
                equal=all(all(reduced[(k,a+suffix)][field]==reduced[(k,b+suffix)][field] for field in decision_keys) for k in full)
                pairs[a+suffix+' vs '+b+suffix]={'inputs':len(full),'exact_decision_and_budget_match':equal}
                assert equal,(a,b,suffix)
        report['exact_optimization_parity']=pairs
        recall_pairs={}
        for budget in ['k12','w3600']:
            a='selected_lazy_'+budget;b='diverse_'+budget
            if (next(iter(full)),a) not in reduced:continue
            blocks=[]
            for scene in sorted({r['scene'] for r in full.values()}):
                ids=[state for state,r in full.items() if r['scene']==scene and state in positives]
                blocks.append([sum(reduced[(i,a)]['verified']=='1' for i in ids),sum(reduced[(i,b)]['verified']=='1' for i in ids),len(ids)])
            g=np.asarray(blocks,float);rng=np.random.default_rng(20261030)
            draws=g[rng.integers(0,len(g),size=(10000,len(g)))].sum(axis=1);draws=draws[draws[:,2]>0]
            differences=100*(draws[:,0]-draws[:,1])/draws[:,2]
            recall_pairs[a+' minus '+b]={'difference_percentage_points':float(100*(g[:,0].sum()-g[:,1].sum())/g[:,2].sum()),'ci95_map_block_bootstrap':np.percentile(differences,[2.5,97.5]).tolist(),'map_blocks':len(g)}
        report['matched_recall_comparisons']=recall_pairs
    if csv_exists(root/'profile_matched.csv'):
        fields=['primary_ms','primary_integration_ms','primary_other_ms','prefix_ms','scalar_feature_ms','geometry_feature_ms','inference_ms','schedule_ms','continuation_ms','other_ms','profile_total_ms']
        prof=defaultdict(list)
        for r in rows(root/'profile_matched.csv'):prof[r['method']].append(r)
        report['time_breakdown_means_ms']={m:{f:float(np.mean([float(r[f]) for r in rr])) for f in fields} for m,rr in prof.items()}
        report['inference_skipped_fraction']={m:sum(r['feature_candidates']=='0' for r in rr)/len(rr) for m,rr in prof.items()}
        report['inference_skipped_among_feasible_fraction']={m:sum(r['feature_candidates']=='0' and r['primary_feasible']=='1' for r in rr)/max(1,sum(r['primary_feasible']=='1' for r in rr)) for m,rr in prof.items()}
        report['mean_prefix_rejections']={m:float(np.mean([float(r['prefix_rejected']) for r in rr])) for m,rr in prof.items()}
        if grouped:
            for rr in prof.values():
                for r in rr:
                    ref=grouped[(r['state_id'],r['method'])][0]
                    assert all(r[k]==ref[k] for k in decision_keys),(r['state_id'],r['method'])
            report['profile_preserves_all_decisions']=True
    if csv_exists(root/'heldout_episodes.csv'):
        ep=rows(root/'heldout_episodes.csv');by=defaultdict(list)
        for r in ep:by[r['method']].append(r)
        report['closed_loop']={m:{'episodes':len(rr),'goals':sum(r['outcome']=='goal' for r in rr),'outcomes':dict(Counter(r['outcome'] for r in rr)),
           'minimum_physical_clearance_m':min(float(r['min_obstacle_clearance']) for r in rr),'episodes_with_negative_physical_clearance':sum(float(r['min_obstacle_clearance'])<0 for r in rr),
           'condition_goals':{c:sum(r['outcome']=='goal' and r['condition']==c for r in rr) for c in sorted({r['condition'] for r in rr})}}
           for m,rr in sorted(by.items())}
        comparisons={}
        methods={m:{(r['scene'],r['condition']):int(r['outcome']=='goal') for r in rr} for m,rr in by.items()}
        pairs=[('geometry_bce_lazy12','geometry_aux_lazy12'),('state_bce_lazy12','state_aux_lazy12'),('state_bce_lazy12','geometry_bce_lazy12'),('state_aux_lazy12','geometry_aux_lazy12')]
        freeze=root/'EXECUTION_MODEL_FREEZE.json'
        if not freeze.exists():freeze=root.parent/'EXECUTION_MODEL_FREEZE.json'
        if freeze.exists():
            config=json.loads(freeze.read_text())['selected_configuration']
            selected={'geometry_bce':'geometry_bce_lazy12','geometry_joint':'geometry_aux_lazy12','state_bce':'state_bce_lazy12','state_joint':'state_aux_lazy12'}[config]
            pairs += [(selected,b) for b in ['cost12','diverse12','full203','geometry_eager12','state_legacy_eager12']]
        for a,b in pairs:
            if a in methods and b in methods:
                keys=list(methods[a]);assert set(keys)==set(methods[b]);comparisons[a+' minus '+b]=interval(methods[a],methods[b],keys)
        report['closed_loop_paired_comparisons']=comparisons
    report['data_sha256']={p.name:raw_sha256(p) for p in logical_csv_paths(root)}
    (root/'analysis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
