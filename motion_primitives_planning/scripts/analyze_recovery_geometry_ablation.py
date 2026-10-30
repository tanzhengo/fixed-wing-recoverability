#!/usr/bin/env python3
"""Audit the post-evaluation geometry-input mission ablation.

The 512 unaffected baseline replays must exactly match the primary experiment.
Only the 256 learned/hybrid cases compare different frozen models. Timing is
retained in raw files but excluded from comparisons in this additional study.
"""
import argparse
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import sys

HELPER=Path(__file__).with_name('analyze_recovery_learning.py')
spec=importlib.util.spec_from_file_location('learning_analysis',HELPER)
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
h=a.h
BASELINES=('primary3','cost12','diverse12','full203')
CHANGED=('learned12','hybrid12')


def validate_manifests(primary_prefix,ablation_prefix):
    primary_dir=Path(primary_prefix).parent;ablation_dir=Path(ablation_prefix).parent
    freeze_path=primary_dir/'MODEL_FREEZE.json';primary_manifest=primary_dir/'run_manifest.json';ablation_manifest=ablation_dir/'run_manifest.json'
    freeze=json.loads(freeze_path.read_text());main=json.loads(primary_manifest.read_text());extra=json.loads(ablation_manifest.read_text())
    h.require(main['status']==extra['status']=='complete' and main['exit_code']==extra['exit_code']==0,'Both runs must complete successfully')
    h.require(extra['primary_run_manifest_sha256']==a.file_hash(primary_manifest)['sha256'],'Primary provenance changed')
    h.require(extra['freeze_sha256']==a.file_hash(freeze_path)['sha256'],'Model freeze changed')
    h.require(extra['main_model']['sha256']==freeze['models']['state_only'],'Ablation did not use frozen no-geometry model')
    h.require(extra['executable']['sha256']==main['executable']['sha256'],'Ablation changed benchmark executable')
    h.require(extra['frozen_source_hashes']==main['frozen_source_hashes'],'Ablation source hash set differs')
    h.require(extra['source_unchanged'] and extra['model_unchanged'] and extra['executable_unchanged'],'Ablation implementation changed during run')
    h.require(extra['protocol_sha256']==a.file_hash(ablation_dir/'PROTOCOL.md')['sha256'],'Post-evaluation protocol changed during run')
    h.require(a.file_hash(Path(str(primary_prefix)+'_scenarios.csv'))['sha256']==a.file_hash(Path(str(ablation_prefix)+'_scenarios.csv'))['sha256'],'Map geometry differs')
    return [freeze_path,primary_manifest,ablation_manifest,ablation_dir/'PROTOCOL.md']


def audit_replays(primary,ablation):
    baseline_cases=[];path_rows=plan_rows=0
    fields=('case_id','scene','family','condition','method','outcome','steps','sim_seconds','goal_distance','min_obstacle_clearance')
    primary_paths=h.group_by(primary['paths'],('case_id',));ablation_paths=h.group_by(ablation['paths'],('case_id',))
    for key,p in primary['indexed'].items():
        if key[2] not in BASELINES:continue
        q=ablation['indexed'][key];baseline_cases.append(key)
        h.require(all(p[f]==q[f] for f in fields),'Baseline episode changed '+str(key))
        x,y=primary_paths[(p['case_id'],)],ablation_paths[(q['case_id'],)]
        h.require(len(x)==len(y),'Baseline path row count changed '+str(key))
        h.require(all(all(xx[f]==yy[f] for f in ('case_id','step','time')+h.STATE_FIELDS) for xx,yy in zip(x,y)),'Baseline path changed '+str(key));path_rows+=len(x)
        x,y=primary['plans_by_case'][(p['case_id'],)],ablation['plans_by_case'][(q['case_id'],)]
        deterministic=('case_id','step','status','cost','rank')+h.BOOL_FIELDS+h.COUNT_FIELDS
        h.require(len(x)==len(y),'Baseline plan row count changed '+str(key))
        h.require(all(all(xx[f]==yy[f] for f in deterministic) for xx,yy in zip(x,y)),'Baseline plan decision/work changed '+str(key));plan_rows+=len(x)
    h.require(len(baseline_cases)==512,'Expected 512 baseline replay episodes')
    return {'passed':True,'baseline_replays':512,'changed_model_cases':256,'independent_maps':32,'baseline_path_rows':path_rows,'baseline_plan_rows':plan_rows,
            'outcome_mismatches':0,'path_mismatches':0,'decision_mismatches':0,'work_mismatches':0,'allowed_varying_field':'planning_ms'}


def common_success_trajectory_summary(primary,ablation,method):
    primary_paths=h.group_by(primary['paths'],('case_id',));ablation_paths=h.group_by(ablation['paths'],('case_id',))
    records=[]
    for scene in a.SCENES:
        for condition in h.CONDITIONS:
            p=primary['indexed'][(scene,condition,method)];q=ablation['indexed'][(scene,condition,method)]
            if p['outcome']!='goal' or q['outcome']!='goal':continue
            def metrics(row,paths):
                points=paths[(row['case_id'],)]
                length=sum(math.sqrt(sum((b[f]-a[f])**2 for f in ('x','y','z'))) for a,b in zip(points,points[1:]))
                return {'sampled_path_length_m':length,'maximum_absolute_y_m':max(abs(x['y']) for x in points),'minimum_uninflated_clearance_m':row['min_obstacle_clearance']}
            records.append({'scene':scene,'condition':condition,'geometry':metrics(p,primary_paths),'no_geometry':metrics(q,ablation_paths)})
    metrics=('sampled_path_length_m','maximum_absolute_y_m','minimum_uninflated_clearance_m')
    conditions={}
    for condition in ('all',)+h.CONDITIONS:
        rows=[r for r in records if condition=='all' or r['condition']==condition]
        summary={'common_goal_cases':len(rows),'metrics':{}}
        for metric in metrics:
            numerator=[];denominator=[]
            for scene in a.SCENES:
                pairs=[r for r in rows if r['scene']==scene]
                numerator.append(sum(r['no_geometry'][metric]-r['geometry'][metric] for r in pairs));denominator.append(len(pairs))
            summary['metrics'][metric]={'geometry':h.distribution(r['geometry'][metric] for r in rows),
                'no_geometry':h.distribution(r['no_geometry'][metric] for r in rows),
                'paired_difference_no_geometry_minus_geometry':h.distribution(r['no_geometry'][metric]-r['geometry'][metric] for r in rows),
                'mean_difference_map_bootstrap':a.bootstrap(numerator,denominator)}
        conditions[condition]=summary
    return {'interpretation':'Exploratory paired description conditional on both frozen models reaching the goal; not a causal estimate or all-task comparison.',
            'path_length_definition':'Sum of 3D chord lengths between stored 0.2 s trajectory samples; identical repeated terminal samples contribute zero.',
            'lateral_excursion_definition':'Maximum absolute world-y coordinate; the nominal start-goal line is y=0.',
            'clearance_definition':'Minimum uninflated obstacle clearance recorded by the independently integrated plant over the complete successful episode.',
            'conditions':conditions,'cases':records}


def summary(primary,ablation):
    results={}
    for method in CHANGED:
        name='no_geometry_'+('learned' if method=='learned12' else 'hybrid')
        by_condition={}
        for condition in ('all',)+h.CONDITIONS:
            conditions=h.CONDITIONS if condition=='all' else (condition,)
            original=[];modified=[];numerators=[];denominators=[];by_map=[]
            for scene in a.SCENES:
                delta=0
                for c in conditions:
                    p=primary['indexed'][(scene,c,method)];q=ablation['indexed'][(scene,c,method)]
                    original.append(p);modified.append(q);delta+=int(p['outcome']=='goal')-int(q['outcome']=='goal')
                numerators.append(delta);denominators.append(len(conditions));by_map.append({'scene':scene,'goal_difference_geometry_minus_no_geometry':delta})
            pc=Counter(x['outcome'] for x in original);qc=Counter(x['outcome'] for x in modified)
            by_condition[condition]={'episodes_per_model':len(original),'geometry_outcomes':{k:pc[k] for k in h.OUTCOMES},'no_geometry_outcomes':{k:qc[k] for k in h.OUTCOMES},
                'geometry_only_goals':sum(p['outcome']=='goal' and q['outcome']!='goal' for p,q in zip(original,modified)),
                'no_geometry_only_goals':sum(q['outcome']=='goal' and p['outcome']!='goal' for p,q in zip(original,modified)),
                'goal_fraction_difference_geometry_minus_no_geometry':a.bootstrap(numerators,denominators),'by_map':by_map}
        incursions=[]
        for row in ablation['episodes']:
            if row['method']==method and row['outcome']=='collision':
                v={k:row[k] for k in ('case_id','scene','family','condition','sim_seconds','min_obstacle_clearance')}
                v['last_command_verified']=ablation['plans_by_case'][(row['case_id'],)][-1]['verified'];incursions.append(v)
        reference_comparisons={}
        for baseline in ('diverse12','full203'):
            numerator=[];denominator=[];treatment_only=baseline_only=0
            for scene in a.SCENES:
                delta=0
                for condition in h.CONDITIONS:
                    p=primary['indexed'][(scene,condition,baseline)]['outcome']=='goal'
                    q=ablation['indexed'][(scene,condition,method)]['outcome']=='goal'
                    delta+=int(q)-int(p);treatment_only+=q and not p;baseline_only+=p and not q
                numerator.append(delta);denominator.append(4)
            reference_comparisons[baseline]={'baseline':baseline,'no_geometry_only_goals':treatment_only,'baseline_only_goals':baseline_only,
                'goal_fraction_difference_no_geometry_minus_baseline':a.bootstrap(numerator,denominator)}
        results[name]={'original_method':method,'conditions':by_condition,'margin_incursions_no_geometry':incursions,'reference_comparisons':reference_comparisons,'common_success_trajectories':common_success_trajectory_summary(primary,ablation,method)}
    return results


def table(results,directory):
    lines=[r'\begin{tabular}{lrrrr}',r'\toprule',r'Schedule & Input & Goals & Margin incursions & No primary \\',r'\midrule']
    for key,label in (('no_geometry_learned','Learned'),('no_geometry_hybrid','Hybrid')):
        row=results[key]['conditions']['all']
        for role,text in (('geometry','Geometry'),('no_geometry','No geometry')):
            counts=row[role+'_outcomes']
            lines.append('{} & {} & {} & {} & {} \\\\'.format(label,text,counts['goal'],counts['collision'],counts['no_feasible_primary']))
    lines.extend((r'\bottomrule',r'\end{tabular}'));(directory/'table.tex').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('primary_prefix');parser.add_argument('ablation_prefix');parser.add_argument('--output-dir',required=True,type=Path);args=parser.parse_args()
    provenance=validate_manifests(args.primary_prefix,args.ablation_prefix)
    primary=a.audit_episodes(args.primary_prefix);ablation=a.audit_episodes(args.ablation_prefix)
    report={'design':'Post-evaluation geometry-input ablation on the same 32 test maps; both models were frozen before primary evaluation. Baseline replays are not additional independent observations.',
            'timing':'Raw timings preserved but not compared in this post-evaluation study.',
            'audit':audit_replays(primary,ablation),'comparisons':summary(primary,ablation)}
    files=provenance+[Path(__file__),HELPER]
    for prefix in (args.primary_prefix,args.ablation_prefix):files.extend(Path(str(prefix)+'_'+suffix+'.csv') for suffix in ('episodes','plans','paths','scenarios'))
    report['provenance']=[a.file_hash(path) for path in files]
    args.output_dir.mkdir(parents=True,exist_ok=True);(args.output_dir/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');table(report['comparisons'],args.output_dir)
    print(json.dumps({'audit':report['audit'],'comparisons':{k:v['conditions']['all'] for k,v in report['comparisons'].items()}},indent=2))


if __name__=='__main__':
    try:main()
    except (h.AuditError,ValueError,OSError) as error:print('Geometry ablation audit failed: '+str(error),file=sys.stderr);sys.exit(2)
