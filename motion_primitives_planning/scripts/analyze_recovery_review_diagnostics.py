#!/usr/bin/env python3
"""Analyze frozen replay, paired failure-state margins and numerical diagnostics."""
import argparse,csv,gzip,hashlib,json
from pathlib import Path
import numpy as np

def sha(p):
    p=Path(p);data=p.read_bytes() if p.exists() else gzip.decompress(Path(str(p)+'.gz').read_bytes())
    return hashlib.sha256(data).hexdigest()
def open_csv(p):
    p=Path(p);return p.open() if p.exists() else gzip.open(str(p)+'.gz','rt')
def rows(p):
    with open_csv(p) as f: return list(csv.DictReader(f))
def stats(a):
    a=np.asarray(a,dtype=float);a=a[np.isfinite(a)]
    if not len(a): return {'count':0}
    return {'count':len(a),'mean':float(a.mean()),'median':float(np.median(a)),'p95':float(np.quantile(a,.95)),'min':float(a.min()),'max':float(a.max())}
def subset(rows,**kw): return [r for r in rows if all(r[k]==str(v) for k,v in kw.items())]
def field(rs,key): return [float(r[key]) for r in rs]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,required=True);ap.add_argument('--prefix',type=Path,required=True);args=ap.parse_args()
    old=args.repo/'docs/recovery/learning';p=args.prefix
    episode_path=Path(str(p)+'_episodes.csv');pair_path=Path(str(p)+'_paired.csv');feature_path=Path(str(p)+'_features.csv')
    episodes=rows(episode_path);pairs=rows(pair_path)
    oldrows=rows(old/'test_episodes.csv');nogeo=rows(old/'geometry_ablation/test_episodes.csv')
    lookup={(r['scene'],r['condition'],r['method']):r for r in oldrows}
    no_lookup={(r['scene'],r['condition'],r['method']):r for r in nogeo}
    aliases={'geometry':'learned12','no_geometry':'learned12','diverse':'diverse12','full':'full203'}
    mismatches=[]
    for r in episodes:
        ref=(no_lookup if r['method']=='no_geometry' else lookup)[(r['scene'],r['condition'],aliases[r['method']])]
        if r['outcome']!=ref['outcome'] or r['steps']!=ref['steps'] or r['goal_distance']!=ref['goal_distance']:
            mismatches.append({'replay':r,'original':ref})
    if len(episodes)!=512 or mismatches:
        raise RuntimeError('Expected 512 matching replays, got %d episodes and %d mismatches' % (len(episodes),len(mismatches)))
    summary={'protocol_sha256':sha(p.parent/'PROTOCOL.json'),'replay':{'episodes':len(episodes),'exact_outcome_step_distance_matches':len(episodes)-len(mismatches),'mismatches':mismatches},'uncompressed_source_files_sha256':{str(f.relative_to(args.repo)):sha(f) for f in [episode_path,pair_path,feature_path]}}
    summary['episodes_by_method']={m:{c:len(subset(episodes,method=m,outcome=c)) for c in sorted(set(r['outcome'] for r in episodes))} for m in aliases}
    summary['paired_failure_states']={'unique_states':len(set(r['state_id'] for r in pairs)),'method_comparisons':len(pairs),'sampling':'Final attempted state and last executable predecessor of every failed episode; outcome-conditioned diagnostic set.'}
    keys=['primary_clearance','selected_backup_clearance','best_backup_clearance','feasible_backups','min_gamma_slack_deg','min_altitude_slack']
    summary['margin_by_method']={m:{k:stats(field(subset(pairs,method=m,primary_feasible=1),k)) for k in keys} for m in aliases}
    indexed={}
    for r in pairs:indexed.setdefault(r['state_id'],{})[r['method']]=r
    differences=[]
    for sid,group in indexed.items():
        g,n=group['geometry'],group['no_geometry']
        if g['verified']!='1' or n['verified']!='1':continue
        differences.append({'state_id':sid,**{k:float(g[k])-float(n[k]) for k in ['cost','primary_clearance','selected_backup_clearance','feasible_backups','min_gamma_slack_deg','min_altitude_slack']}})
    summary['geometry_minus_no_geometry_on_same_verified_states']={k:stats([r[k] for r in differences]) for k in ['cost','primary_clearance','selected_backup_clearance','feasible_backups','min_gamma_slack_deg','min_altitude_slack']}
    summary['geometry_minus_no_geometry_on_same_verified_states']['counts']={'pairs':len(differences),'geometry_lower_cost':sum(r['cost']< -1e-5 for r in differences),'geometry_lower_primary_clearance':sum(r['primary_clearance']< -1e-5 for r in differences),'geometry_higher_primary_clearance':sum(r['primary_clearance']> 1e-5 for r in differences),'geometry_fewer_feasible_backups':sum(r['feasible_backups']<0 for r in differences)}
    maskdiff=[]
    for sid,group in indexed.items():
        g,n=group['geometry'],group['geometry_yaw_mask']
        if g['primary_feasible']!='1' or n['primary_feasible']!='1':continue
        maskdiff.append({'state_id':sid,'geometry_verified':g['verified'],'masked_verified':n['verified'],**{k:float(n[k])-float(g[k]) for k in ['cost','rank','primary_clearance','feasible_backups']}})
    summary['yaw_rate_mask_posthoc_sensitivity']={'primary_feasible_paired_states':len(maskdiff),'different_cost_rank':sum(abs(r['rank'])>0 for r in maskdiff),'geometry_only_verified':sum(r['geometry_verified']=='1' and r['masked_verified']=='0' for r in maskdiff),'masked_only_verified':sum(r['geometry_verified']=='0' and r['masked_verified']=='1' for r in maskdiff),'masked_minus_geometry':{k:stats([r[k] for r in maskdiff]) for k in ['cost','primary_clearance','feasible_backups']},'note':'Only frozen network first-layer column11 masked at selected diagnostic states; true yaw rate and control generation unchanged. No retraining or closed-loop outcome claim.'}
    summary['full_minus_budgeted_on_same_verified_states']={}
    for method in ['geometry','no_geometry','diverse']:
        differences=[]
        for sid,group in indexed.items():
            full,budget=group['full'],group[method]
            if full['verified']!='1' or budget['verified']!='1':continue
            differences.append({k:float(full[k])-float(budget[k]) for k in ['cost','primary_clearance','feasible_backups']})
        summary['full_minus_budgeted_on_same_verified_states'][method]={'pairs':len(differences),'differences':{k:stats([r[k] for r in differences]) for k in ['cost','primary_clearance','feasible_backups']},'full_lower_clearance_count':sum(r['primary_clearance']< -1e-5 for r in differences),'full_higher_clearance_count':sum(r['primary_clearance']> 1e-5 for r in differences)}
    summary['counterfactuals']={}
    for condition in ['nominal','model_mismatch','crosswind','delay100ms']:
        rr=[r for r in subset(pairs,condition=condition,primary_feasible=1) if r['method']!='geometry_yaw_mask']
        summary['counterfactuals'][condition]={k:stats(field(rr,k)) for k in ['euler02_rk4_error','euler005_rk4_error','euler02_actual_error','rk4_actual_error','euler02_short_clearance','euler005_short_clearance','rk4_short_clearance','actual_short_clearance']}
        summary['counterfactuals'][condition]['counts']={k:sum(r[k]=='1' for r in rr) for k in ['euler02_short_collision','euler005_short_collision','rk4_short_collision','actual_short_collision']}
    # Only each source method's own final collision-producing command supports this attribution.
    collision=[]
    for r in pairs:
        if r['source_outcome']=='collision' and r['method']==r['source_method'] and r['source_role']=='final_attempt':collision.append(r)
    summary['actual_collision_predecessors']={'feasible_bank_count':stats(field(collision,'feasible_backups')),'by_method_counterfactual':{m:{'cases':len(subset(collision,method=m)),'euler02_crossings':sum(r['euler02_short_collision']=='1' for r in subset(collision,method=m)),'euler005_crossings':sum(r['euler005_short_collision']=='1' for r in subset(collision,method=m)),'rk4_nominal_crossings':sum(r['rk4_short_collision']=='1' for r in subset(collision,method=m)),'actual_condition_only_crossings':sum(r['rk4_short_collision']=='0' and r['actual_short_collision']=='1' for r in subset(collision,method=m))} for m in aliases},'count':len(collision),'rows':collision,'by_method':{m:len(subset(collision,method=m)) for m in aliases},'numerically_unsafe_without_disturbance':sum(r['rk4_short_collision']=='1' for r in collision),'euler005_detects':sum(r['euler005_short_collision']=='1' for r in collision),'euler02_detects':sum(r['euler02_short_collision']=='1' for r in collision)}
    train=np.load(old/'data/train_candidates.npz',allow_pickle=False)['x'];low=train.min(0);high=train.max(0);qlo,qhi=np.quantile(train,[.01,.99],axis=0)
    groups={'scalar':slice(0,28),'dynamic_state':slice(0,12),'relative_motion_goal':slice(12,20),'bounds_cost_context':slice(20,28),'clearance_grid':slice(28,103)}
    buckets={m:[] for m in aliases}
    with open_csv(feature_path) as f:
        for r in csv.DictReader(f):buckets[r['method']].append([float(r['f%d'%i]) for i in range(103)])
    feature_names={f['index']:f['name'] for f in json.loads((args.repo/'motion_primitives_planning/models/feature_schema.json').read_text())['features']}
    shift={}
    for m,vals in buckets.items():
        x=np.asarray(vals);shift[m]={'selected_candidates':len(x)}
        outside_all=(x<low-1e-6)|(x>high+1e-6)
        shift[m]['top_outside_range_channels']=[{'index':int(j),'name':feature_names[int(j)],'fraction':float(outside_all[:,j].mean()),'train_min':float(low[j]),'train_max':float(high[j]),'closed_loop_min':float(x[:,j].min()),'closed_loop_max':float(x[:,j].max())} for j in np.argsort(-outside_all.mean(0),kind='stable')[:10]]
        for name,s in groups.items():
            outside=(x[:,s]<low[s]-1e-6)|(x[:,s]>high[s]+1e-6)
            outsideq=(x[:,s]<qlo[s])|(x[:,s]>qhi[s])
            shift[m][name]={'any_channel_outside_train_range_fraction':float(outside.any(1).mean()),'mean_channel_fraction_outside_train_range':float(outside.mean()),'any_channel_outside_train_1_99_percentile_fraction':float(outsideq.any(1).mean())}
    summary['feature_shift']={'reference_samples':len(train),'note':'Closed-loop selected candidates vs training all feasible candidates: descriptive and selection-conditioned, not a test of causation. Rigid translation/heading are excluded from the representation.','methods':shift}
    branch_path=Path(str(p)+'_branchpoints.csv')
    if branch_path.exists() or Path(str(branch_path)+'.gz').exists():
        branches=rows(branch_path)
        if len(branches)!=128: raise RuntimeError('Incomplete shared-prefix probe')
        differences=[];nobranch=0
        for r in branches:
            gr=lookup[(r['scene'],r['condition'],'learned12')];nr=no_lookup[(r['scene'],r['condition'],'learned12')]
            if r['branch_found']=='0':
                nobranch+=1
                assert gr['outcome']==nr['outcome'], 'Identical-prefix outcome mismatch'
                continue
            dif={k:float(r['g_'+k])-float(r['n_'+k]) for k in ['cost','primary_clearance','backup_clearance','feasible','gamma_slack','altitude_slack']}
            differences.append({'scene':r['scene'],'condition':r['condition'],'step':int(r['step']),'geometry_outcome':gr['outcome'],'no_geometry_outcome':nr['outcome'],'both_verified':r['geometry_verified']=='1' and r['no_geometry_verified']=='1',**dif})
        def branch_stats(rr):
            return {'count':len(rr),'geometry_lower_cost':sum(r['cost']< -1e-5 for r in rr),'geometry_lower_primary_clearance':sum(r['primary_clearance']< -1e-5 for r in rr),'geometry_higher_primary_clearance':sum(r['primary_clearance']>1e-5 for r in rr),'geometry_fewer_feasible_continuations':sum(r['feasible']<0 for r in rr),'geometry_more_feasible_continuations':sum(r['feasible']>0 for r in rr),'geometry_minus_no_geometry':{k:stats([r[k] for r in rr]) for k in ['cost','primary_clearance','backup_clearance','feasible','gamma_slack','altitude_slack']}}
        summary['first_executed_control_divergence']={'tasks':128,'no_divergence':nobranch,'all_branches':branch_stats(differences),'both_verified_branches':branch_stats([r for r in differences if r['both_verified']]),'rows':differences,'note':'Posthoc analysis on original diagnostic maps. Controllers see exactly the same state and history until first differing executed command. This isolates selection difference at the branch, not its independent causal effect on downstream outcomes.'}
        summary['uncompressed_source_files_sha256'][str(branch_path.relative_to(args.repo))]=sha(branch_path)
    summary['interpretation_constraints']=['Failure states were selected by outcome, so margins here do not estimate population risk.','Alternative schedules share identical states/history but downstream closed-loop consequences are not replayed in this comparison.','Counterfactual numerical error holds state/command fixed; it does not imply corrected integrators alone improve mission outcomes.','Feature-range comparisons are descriptive, include selected-candidate bias, and do not establish covariate shift as failure cause.']
    (p.parent/'analysis.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'replay_matches':summary['replay']['exact_outcome_step_distance_matches'],'episodes':len(episodes),'paired_states':summary['paired_failure_states']['unique_states'],'source_collision_cases':len(collision),'numerical_nominal_crossings':summary['actual_collision_predecessors']['numerically_unsafe_without_disturbance']},indent=2))
    # No conclusions are selected based on statistical significance from this diagnostic set.
if __name__=='__main__':main()
