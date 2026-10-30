#!/usr/bin/env python3
"""Audit and summarize the frozen, held-out learning experiment.

Example:
  analyze_recovery_learning.py TEST_PREFIX --output-dir OUT \
    --freeze-json MODEL_FREEZE.json --model learned.txt --state-only-model state.txt
The default requires all 576 matched states and all 768 closed-loop episodes.
--matched-only is an explicitly incomplete intermediate report.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np

HELPER = Path(__file__).with_name('analyze_recovery_v3.py')
spec = importlib.util.spec_from_file_location('recovery_audit_helpers', HELPER)
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)
FAMILIES = ('staggered', 'wall', 'culdesac', 'spatial')
SCENES = tuple('test_{}_{}'.format(f, s) for f in FAMILIES for s in range(8))
METHODS = ('cost12', 'diverse12', 'learned12', 'hybrid12', 'full203',
           'cost_w3600', 'diverse_w3600', 'learned_w3600', 'hybrid_w3600',
           'state_only12', 'state_only_w3600')
EPISODE_METHODS = ('primary3', 'cost12', 'diverse12', 'learned12', 'hybrid12', 'full203')
LABELS = {'primary3':'Primary', 'cost12':'Cost', 'diverse12':'Diverse', 'learned12':'Learned',
          'hybrid12':'Hybrid', 'full203':'Full', 'state_only12':'No geometry'}
COLORS = {'primary3':'#999999', 'cost12':'#0072B2', 'diverse12':'#D55E00',
          'learned12':'#6F4E7C', 'hybrid12':'#009E73', 'full203':'#333333', 'state_only12':'#888888'}


def file_hash(path):
    sha = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda:handle.read(1048576), b''):
            sha.update(block)
    return {'file': str(path.resolve()), 'sha256': sha.hexdigest(), 'bytes': path.stat().st_size}


def bootstrap(numerator, denominator):
    numerator, denominator = np.asarray(numerator, dtype=np.float64), np.asarray(denominator, dtype=np.float64)
    h.require(numerator.shape == denominator.shape == (32,), 'Bootstrap requires all 32 map clusters')
    h.require(denominator.sum() > 0, 'Bootstrap denominator is zero')
    indices = np.random.default_rng(12345).integers(0, 32, size=(10000, 32))
    den = denominator[indices].sum(axis=1)
    valid = den > 0
    values = numerator[indices].sum(axis=1)[valid] / den[valid]
    return {'estimate': float(numerator.sum()/denominator.sum()),
            'ci95': [float(x) for x in np.quantile(values, [.025,.975])],
            'clusters':32,'resamples':10000,'valid_resamples':int(valid.sum()),'seed':12345}


def audit_decision(row):
    h.prepare_decision(row)
    h.require(not row['reused'] and row['cached_steps']==0, row['_source']+': cache must be disabled')
    h.require(row['primary_feasible']==(row['feasible_count']>0), row['_source']+': inconsistent primary feasibility')
    h.require(row['primary_count'] in (203,204), row['_source']+': primary library size changed')
    if row['primary_feasible']:
        h.require(1<=row['rank']<=row['feasible_count'], row['_source']+': invalid primary rank')
    else:
        h.require(row['rank']==-1 and not row['verified'], row['_source']+': invalid infeasible decision')
    if row['verified']:
        h.require(row['backup_steps']>0 and row['backup_evaluated']>0, row['_source']+': verification without explicit work')
    if row.get('method','').endswith('12'):
        h.require(row['recovery_checked']<=12, row['_source']+': candidate budget exceeded')


def audit_matched(prefix):
    path = Path(str(prefix)+'_matched.csv')
    rows = h.read_csv(path, ('state_id','scene','family','method','repeat')+h.STATE_FIELDS+h.DECISION_FIELDS)
    h.require(len(rows)==576*11*2, 'Incomplete matched design: expected 12672 rows')
    for row in rows:
        for field in ('state_id','repeat'):
            h.number(row, field, integer=True, nonnegative=True)
        h.require(row['scene'] in SCENES and row['method'] in METHODS, row['_source']+': unexpected scene or method')
        h.require(row['family']==row['scene'].split('_')[1], row['_source']+': family mismatch')
        h.prepare_state(row); audit_decision(row)
    indexed = h.unique_index(rows, ('state_id','method','repeat'))
    groups = h.group_by(rows, ('state_id',))
    h.require(set(groups)=={(i,) for i in range(1,577)}, 'Expected matched state IDs 1..576')
    collapsed, positions = {}, defaultdict(set)
    for (state_id,), group in groups.items():
        h.require({(r['method'],r['repeat']) for r in group}=={(m,r) for m in METHODS for r in (0,1)}, 'Incomplete comparator grid')
        first = group[0]
        for row in group:
            h.require((row['scene'],row['family'])==(first['scene'],first['family']), 'State map identity differs')
            h.require(all(row[f]==first[f] for f in h.STATE_FIELDS), 'Complete matched state differs')
            h.require(all(row[f]==first[f] for f in ('primary_count','feasible_count','primary_steps','primary_feasible')), 'Common primary evaluation differs')
        position = (first['x'],first['y'],first['sigma'])
        h.require(position not in positions[first['scene']], 'Duplicate matched position')
        positions[first['scene']].add(position)
        for method in METHODS:
            a,b = (indexed[(state_id,method,r)] for r in (0,1))
            fields = ('status','rank','cost')+h.BOOL_FIELDS+h.COUNT_FIELDS
            h.require(all(a[f]==b[f] for f in fields), 'Timing repeat changes decision at state {} method {}'.format(state_id,method))
            collapsed[(state_id,method)] = dict(a, planning_ms=float(np.median([a['planning_ms'],b['planning_ms']])))
        reference = collapsed[(state_id,'full203')]
        for method in METHODS:
            row = collapsed[(state_id,method)]
            h.require(not row['verified'] or reference['verified'], 'Limited search contradicts full reference')
            if row['verified']:
                h.require(row['cost']>=reference['cost'] or h.near(row['cost'],reference['cost']), 'Verified cost below full reference')
                h.require(row['rank']>=reference['rank'], 'Verified rank below full reference')
            if method=='cost12' and row['verified']:
                h.require(row['cost']==reference['cost'] and row['rank']==reference['rank'], 'Cost-12 did not select reference-minimum cost')
    expected = {(x,y,psi) for x in (0.,25.,50.) for y in (-5.,0.,5.) for psi in (-.15,.15)}
    h.require(set(positions)==set(SCENES), 'Incorrect matched map set')
    for scene, grid in positions.items():
        h.require(len(grid)==18 and all(any(all(h.near(a,b) for a,b in zip(x,y)) for y in expected) for x in grid), scene+': incorrect matched input grid')
    return {'rows':rows,'collapsed':collapsed,'path':path}


def summarize_matched(data):
    collapsed = data['collapsed']
    reference = {i:collapsed[(i,'full203')] for i in range(1,577)}
    positive = sum(r['verified'] for r in reference.values())
    h.require(positive>0,'No positive reference states')
    methods = {}
    for method in METHODS:
        rows = [collapsed[(i,method)] for i in range(1,577)]
        verified = [r for r in rows if r['verified']]
        methods[method] = {
            'states':576,'verified':len(verified),'reference_positive':positive,'discovery_recall':len(verified)/positive,
            'primary_steps':h.distribution(r['primary_steps'] for r in rows),
            'backup_steps':h.distribution(r['backup_steps'] for r in rows),
            'total_steps':h.distribution(r['primary_steps']+r['backup_steps'] for r in rows),
            'candidates_checked':h.distribution(r['recovery_checked'] for r in rows),
            'planning_ms_state_median':h.distribution(r['planning_ms'] for r in rows),
            'planning_ms_all_timing_repeats':h.distribution(r['planning_ms'] for r in data['rows'] if r['method']==method),
            'verified_cost_excess_over_full':h.distribution(r['cost']-reference[r['state_id']]['cost'] for r in verified),
            'work_exhausted_states':sum(r['work_exhausted'] for r in rows),
            'status_counts':dict(Counter(r['status'] for r in rows)),
        }
    pairs = {}
    for budget, baseline, treatments in [('K12','diverse12',('learned12','hybrid12','state_only12')),
                                         ('W3600','diverse_w3600',('learned_w3600','hybrid_w3600','state_only_w3600'))]:
        for treatment in treatments:
            numerators,denominators,by_map = [],[],[]
            a = [collapsed[(i,baseline)] for i in range(1,577)]
            b = [collapsed[(i,treatment)] for i in range(1,577)]
            for scene in SCENES:
                ids=[i for i in range(1,577) if reference[i]['scene']==scene]
                delta=sum(int(collapsed[(i,treatment)]['verified'])-int(collapsed[(i,baseline)]['verified']) for i in ids)
                denominator=sum(reference[i]['verified'] for i in ids)
                numerators.append(delta);denominators.append(denominator)
                by_map.append({'scene':scene,'reference_positive':denominator,'verified_difference':delta})
            joint = [(x,y) for x,y in zip(a,b) if x['verified'] and y['verified']]
            pairs[treatment+'_minus_'+baseline] = {
                'budget':budget,'baseline':baseline,'treatment':treatment,
                'treatment_only':sum(y['verified'] and not x['verified'] for x,y in zip(a,b)),
                'baseline_only':sum(x['verified'] and not y['verified'] for x,y in zip(a,b)),
                'both_verified':len(joint),'recall_difference':bootstrap(numerators,denominators),'by_map':by_map,
                'common_positive_cost_difference':h.distribution(y['cost']-x['cost'] for x,y in joint),
                'all_state_backup_step_difference':h.distribution(y['backup_steps']-x['backup_steps'] for x,y in zip(a,b)),
                'all_state_timing_difference_ms':h.distribution(y['planning_ms']-x['planning_ms'] for x,y in zip(a,b)),
            }
    return {'reference_positive':positive,'reference_negative':576-positive,
            'no_feasible_primary':sum(not r['primary_feasible'] for r in reference.values()),
            'primary_feasible_no_continuation':sum(r['primary_feasible'] and not r['verified'] for r in reference.values()),
            'methods':methods,'paired_comparisons':pairs}


def audit_episodes(prefix):
    fields = {'episodes':('case_id','scene','family','condition','method','outcome','steps','sim_seconds','goal_distance','min_obstacle_clearance'),
              'plans':('case_id','step')+h.DECISION_FIELDS,
              'paths':('case_id','step','time')+h.STATE_FIELDS}
    data = {name:h.read_csv(Path(str(prefix)+'_'+name+'.csv'),cols) for name,cols in fields.items()}
    episodes=data['episodes'];h.require(len(episodes)==768,'Incomplete episode design: expected 768')
    cases=h.unique_index(episodes,('case_id',));grid=h.unique_index(episodes,('scene','condition','method'))
    h.require(set(grid)=={(s,c,m) for s in SCENES for c in h.CONDITIONS for m in EPISODE_METHODS},'Episode grid differs from frozen design')
    for row in episodes:
        h.require(row['family']==row['scene'].split('_')[1],'Episode family mismatch')
        h.require(row['outcome'] in h.OUTCOMES,'Unknown episode outcome')
        h.number(row,'steps',integer=True,nonnegative=True)
        for field in ('sim_seconds','goal_distance','min_obstacle_clearance'):h.number(row,field)
        h.require(row['steps']<=200 and h.near(row['sim_seconds'],row['steps']*.2),'Invalid episode duration')
        if row['outcome']=='timeout':h.require(row['steps']==200,'Premature timeout')
        if row['outcome']=='goal':h.require(row['goal_distance']<15,'Goal horizontal tolerance violated')
    for row in data['plans']:
        h.require((row['case_id'],) in cases,'Unknown plan case')
        row['method']=cases[(row['case_id'],)]['method'];h.number(row,'step',integer=True,nonnegative=True);audit_decision(row)
        if row['method']=='primary3':h.require(row['backup_steps']==0 and not row['verified'],'Primary-only condition performs continuation search')
    h.unique_index(data['plans'],('case_id','step'))
    for row in data['paths']:
        h.require((row['case_id'],) in cases,'Unknown path case');h.number(row,'step',integer=True,nonnegative=True);h.number(row,'time',nonnegative=True);h.prepare_state(row)
        h.require(h.near(row['time'],row['step']*.2),'Path timestamp differs from step')
    plans=h.group_by(data['plans'],('case_id',));paths=h.group_by(data['paths'],('case_id',))
    for key,episode in cases.items():
        pp=plans.get(key,[]);xx=paths.get(key,[]);count=episode['steps']+(episode['outcome']=='no_feasible_primary')
        h.require(len(pp)==count and [p['step'] for p in pp]==list(range(count)),'Incomplete plan sequence '+str(key))
        h.require(xx and xx[0]['step']==0 and xx[-1]['step']==episode['steps'],'Missing initial/final trajectory state')
        index={}
        for point in xx:
            if point['step'] in index:h.require(all(point[f]==index[point['step']][f] for f in h.STATE_FIELDS),'Conflicting duplicate trajectory state')
            index[point['step']]=point
        h.require(set(index)==set(range(episode['steps']+1)),'Missing trajectory steps')
        h.require(h.near(math.hypot(250-xx[-1]['x'],xx[-1]['y']),episode['goal_distance'],5e-5,2e-7),'Final goal distance differs from trajectory')
        if episode['outcome']=='goal':h.require(abs(xx[-1]['z']-20)<3,'Goal altitude tolerance violated')
        if episode['outcome']=='no_feasible_primary':h.require(pp and not pp[-1]['primary_feasible'],'Wrong no-primary termination')
    data['indexed']=grid;data['plans_by_case']=plans
    return data


def summarize_episodes(data):
    episodes=data['episodes'];by_method={}
    for method in EPISODE_METHODS:
        rows=[r for r in episodes if r['method']==method];counts=Counter(r['outcome'] for r in rows)
        by_method[method]={'episodes':128,'outcomes':{o:counts[o] for o in h.OUTCOMES},'goal_fraction':counts['goal']/128,
                           'min_obstacle_clearance':h.distribution(r['min_obstacle_clearance'] for r in rows),
                           'planning_ms':h.distribution(r['planning_ms'] for r in data['plans'] if r['method']==method)}
    by_condition={c:{} for c in h.CONDITIONS}
    for condition in h.CONDITIONS:
        for method in EPISODE_METHODS:
            rows=[r for r in episodes if r['method']==method and r['condition']==condition];counts=Counter(r['outcome'] for r in rows)
            by_condition[condition][method]={'episodes':32,'outcomes':{o:counts[o] for o in h.OUTCOMES},'goal_fraction':counts['goal']/32}
    pairs={}
    for treatment in ('learned12','hybrid12'):
        for condition in ('all',)+h.CONDITIONS:
            conditions=h.CONDITIONS if condition=='all' else (condition,)
            deltas=[];denominators=[];treatment_only=baseline_only=0
            for scene in SCENES:
                delta=0
                for c in conditions:
                    a=data['indexed'][(scene,c,'diverse12')]['outcome']=='goal';b=data['indexed'][(scene,c,treatment)]['outcome']=='goal'
                    delta+=int(b)-int(a);treatment_only+=b and not a;baseline_only+=a and not b
                deltas.append(delta);denominators.append(len(conditions))
            pairs[treatment+'_minus_diverse12_'+condition]={'condition':condition,'treatment':treatment,'baseline':'diverse12',
                'treatment_only_goals':treatment_only,'baseline_only_goals':baseline_only,'goal_fraction_difference':bootstrap(deltas,denominators)}
    incursions=[]
    for row in episodes:
        if row['outcome']=='collision':
            plans=data['plans_by_case'][(row['case_id'],)]
            incursions.append({k:row[k] for k in ('case_id','scene','family','condition','method','sim_seconds','min_obstacle_clearance')})
            incursions[-1]['last_command_verified']=plans[-1]['verified']
    return {'methods':by_method,'by_condition':by_condition,'paired_comparisons':pairs,'margin_incursions':incursions,
            'margin_interpretation':'Collision outcomes cross the configured inflated obstacle boundary. Logged clearance is to the uninflated geometry through the first detected violation.'}


def audit_geometry(prefix):
    path=Path(str(prefix)+'_scenarios.csv')
    rows=h.read_csv(path,('scene','family','seed','low','high','type','x','y','z','r','height'))
    h.require({r['scene'] for r in rows}==set(SCENES),'Incomplete held-out geometry')
    for row in rows:
        family=FAMILIES.index(row['family']);index=int(row['scene'].split('_')[-1])
        h.require(row['scene']=='test_{}_{}'.format(row['family'],index),'Geometry identity mismatch')
        h.number(row,'seed',integer=True);h.require(row['seed']==501000+family*1000+index,'Test map seed mismatch')
        for field in ('low','high','x','y','z','r','height'):h.number(row,field)
        h.require(row['high']>row['low'] and row['r']>0 and row['type'] in ('cylinder','sphere'),'Invalid geometry')
    return path


def figures(summary, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':8,'axes.titlesize':9,'axes.labelsize':8,'xtick.labelsize':8,'ytick.labelsize':8,
                         'legend.fontsize':7,'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,2,figsize=(7,2.65))
    for ax,suffix,title in zip(axes,('12','_w3600'),('12 candidate limit','3600 integration-step limit')):
        stems=('cost','diverse','state_only','learned','hybrid');names=[x+suffix for x in stems]
        values=[100*summary['matched']['methods'][m]['discovery_recall'] for m in names]
        bars=ax.bar(range(5),values,color=[COLORS[x+'12'] for x in stems],width=.68)
        ax.set_xticks(range(5));ax.set_xticklabels([LABELS[x+'12'] for x in stems],rotation=15)
        ax.set_ylim(0,108);ax.set_title(title);ax.set_ylabel('Continuation discovery (%)');ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
        for bar,value,m in zip(bars,values,names):ax.text(bar.get_x()+bar.get_width()/2,value+1.6,str(summary['matched']['methods'][m]['verified']),ha='center',fontsize=7)
    fig.tight_layout(pad=.6);fig.savefig(directory/'matched.pdf',bbox_inches='tight');fig.savefig(directory/'matched.png',dpi=180,bbox_inches='tight');plt.close(fig)
    if 'episodes' not in summary:return
    fig,axes=plt.subplots(2,2,figsize=(7,4.15),sharey=True)
    outcome_labels={'goal':'Goal','no_feasible_primary':'No primary','collision':'Margin incursion','envelope_violation':'Envelope','invalid_dynamics':'Invalid dynamics','timeout':'Timeout'}
    outcome_colors={'goal':'#009E73','no_feasible_primary':'#999999','collision':'#D55E00','envelope_violation':'#CC79A7','invalid_dynamics':'#6F4E7C','timeout':'#E6AB02'}
    for ax,condition in zip(axes.ravel(),h.CONDITIONS):
        bottom=np.zeros(6)
        for outcome in h.OUTCOMES:
            counts=np.array([summary['episodes']['by_condition'][condition][m]['outcomes'][outcome] for m in EPISODE_METHODS])
            ax.bar(range(6),counts,bottom=bottom,color=outcome_colors[outcome],label=outcome_labels[outcome],width=.72);bottom+=counts
        ax.set_xticks(range(6));ax.set_xticklabels([LABELS[m] for m in EPISODE_METHODS],rotation=22,ha='right');ax.set_ylim(0,32);ax.set_yticks((0,8,16,24,32));ax.set_title(h.LABELS[condition]);ax.set_ylabel('Episodes')
    handles,labels=axes[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(.5,1.01))
    fig.tight_layout(rect=(0,0,1,.92),pad=.7);fig.savefig(directory/'outcomes.pdf',bbox_inches='tight');fig.savefig(directory/'outcomes.png',dpi=180,bbox_inches='tight');plt.close(fig)


def table(summary,directory):
    lines=[r'\begin{tabular}{llrrrrr}',r'\toprule',r'Schedule & Budget & Found & Recall (\%) & Backup steps & Median (ms) & P95 (ms) \\',r'\midrule']
    for method in ('cost12','diverse12','state_only12','learned12','hybrid12','cost_w3600','diverse_w3600','state_only_w3600','learned_w3600','hybrid_w3600','full203'):
        row=summary['matched']['methods'][method]
        label=LABELS[method.replace('_w3600','12')];budget='3600 steps' if '_w' in method else ('203 candidates' if method=='full203' else '12 candidates')
        lines.append('{} & {} & {} & {:.1f} & {:.0f} & {:.2f} & {:.2f} \\\\'.format(label,budget,row['verified'],100*row['discovery_recall'],row['backup_steps']['mean'],row['planning_ms_state_median']['median'],row['planning_ms_state_median']['p95']))
    lines.extend((r'\bottomrule',r'\end{tabular}'))
    (directory/'table.tex').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('prefix');parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--freeze-json',required=True,type=Path);parser.add_argument('--model',required=True,type=Path);parser.add_argument('--state-only-model',required=True,type=Path)
    parser.add_argument('--matched-only',action='store_true');parser.add_argument('--json-only',action='store_true');args=parser.parse_args()
    for path in (args.freeze_json,args.model,args.state_only_model):h.require(path.is_file(),'Missing frozen model/protocol '+str(path))
    freeze=json.loads(args.freeze_json.read_text());h.require(isinstance(freeze,dict) and freeze,'Empty model freeze record')
    h.require(file_hash(args.model)['sha256']==freeze['models']['geometry'],'Geometry model differs from frozen weights')
    h.require(file_hash(args.state_only_model)['sha256']==freeze['models']['state_only'],'State-only model differs from frozen weights')
    manifest_path=Path(args.prefix).parent/'run_manifest.json'
    h.require(manifest_path.is_file(),'Missing before-run provenance manifest')
    manifest=json.loads(manifest_path.read_text())
    h.require(manifest['freeze_sha256']==file_hash(args.freeze_json)['sha256'],'Freeze record changed after benchmark launch')
    h.require(manifest['model_hashes']==freeze['models'],'Run used different model hashes')
    if not args.matched_only:
        h.require(manifest['status']=='complete' and manifest['exit_code']==0,'Benchmark run did not complete')
        h.require(manifest['source_unchanged'] and manifest['models_unchanged'] and manifest['executable_unchanged'],'Frozen implementation changed during benchmark')
    repository=Path(__file__).resolve().parents[2]
    for relative,digest in freeze['source'].items():
        h.require(file_hash(repository/relative)['sha256']==digest,'Frozen source changed: '+relative)
    geometry=audit_geometry(args.prefix);matched=audit_matched(args.prefix)
    summary={'schema_version':'learning_v1','audit':{'passed':True,'maps':32,'matched_states':576,'matched_rows':12672,'timing_repeats':2,
            'repeat_decision_mismatches':0,'input_mismatches':0,'budget_violations':0,'reference_contradictions':0,'episode_phase_complete':not args.matched_only},
            'design':{'timing':'Percentiles of 576 within-state medians from two rotated timing repeats; all repeat timing distribution also exported.',
                      'bootstrap':'10000 paired resamples of 32 map clusters, seed12345; all states and conditions in each sampled map retained.',
                      'budget':'Hard backup Euler-step cap; equal caps do not imply equal consumed work. Feature/inference runtime is included in planning_ms.',
                      'source_split':'Independent test maps with seed base501000. Training base301000; validation base401000.'},
            'matched':summarize_matched(matched),'model_freeze':freeze}
    inputs=[geometry,matched['path']]
    if not args.matched_only:
        episodes=audit_episodes(args.prefix);summary['episodes']=summarize_episodes(episodes)
        summary['audit'].update({'episodes':768,'plan_rows':len(episodes['plans']),'path_rows':len(episodes['paths'])})
        inputs.extend(Path(str(args.prefix)+'_'+name+'.csv') for name in ('episodes','plans','paths'))
    source=Path(__file__).resolve().parents[1]/'src'
    provenance=[args.freeze_json,manifest_path,args.model,args.state_only_model,Path(__file__),HELPER,
                source/'recovery_learning_benchmark.cpp',source/'motion_primitives.h',source/'learned_recovery.h',source/'evaluation_plant.h']
    summary['provenance']={'inputs':[file_hash(p) for p in inputs],'source_and_models':[file_hash(p) for p in provenance]}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    (args.output_dir/'analysis.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    table(summary,args.output_dir)
    if not args.json_only:figures(summary,args.output_dir)
    print(json.dumps({'audit':summary['audit'],'reference_positive':summary['matched']['reference_positive'],
                      'verified':{k:v['verified'] for k,v in summary['matched']['methods'].items()},
                      'goals':{k:v['outcomes']['goal'] for k,v in summary.get('episodes',{}).get('methods',{}).items()}},indent=2))


if __name__=='__main__':
    try:main()
    except (h.AuditError,ValueError,OSError) as error:
        print('Learning evaluation audit failed: '+str(error),file=sys.stderr);sys.exit(2)
