#!/usr/bin/env python3
"""Summarize the preregistered three-block urban acquisitions without pooling old flights.

Definitions follow reviewer_revision_20261101/paper/analyze_city.py: mission
observer rows define the cutoff; leading noncommand MAP_WAITING rows and
shutdown-grace telemetry are excluded from active-update metrics. Independent
physical collision-mesh chord clearance is the geometric outcome criterion.
"""
from pathlib import Path
import argparse, csv, gzip, hashlib, json, math, sys
import numpy as np

BASE=Path(__file__).resolve().parent
METHODS=('Cost','Diverse','Selected')
ORDERS={'Cost':'cost','Diverse':'diverse','Selected':'learned_coverage'}

def read(p): return json.loads(Path(p).read_text())
def compressed(p): return json.loads(gzip.decompress(Path(p).read_bytes()))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stats(v):
    a=np.asarray([x for x in v if x is not None and math.isfinite(float(x))],float)
    return {'n':len(a),'mean':float(a.mean()) if len(a) else None,
            'sample_sd':float(a.std(ddof=1)) if len(a)>1 else None,
            'minimum':float(a.min()) if len(a) else None,
            'maximum':float(a.max()) if len(a) else None}
def dist(v):
    a=np.asarray([x for x in v if x is not None and math.isfinite(float(x))],float)
    return {'count':len(a),'mean':float(a.mean()) if len(a) else None,
            'p95':float(np.percentile(a,95)) if len(a) else None,
            'maximum':float(a.max()) if len(a) else None}
def active_rows(rows):
    n=0
    while n<len(rows) and rows[n].get('status')=='MAP_WAITING' and not rows[n].get('command_published'): n+=1
    return n,rows[n:]
def check(cond,msg):
    if not cond: raise AssertionError(msg)
def near(a,b,atol=1e-9): return abs(float(a)-float(b))<=atol

def analyze_run(base,item,protocol):
    name='block%d_%s'%(item['block'],item['method']);p=base/name
    for rel in ('summary.json','run.json','flight/result.json','flight/observations.json.gz','telemetry.json.gz','map_snapshot.json'):
        check((p/rel).is_file(),name+': missing '+rel)
    s,m,r,d,tel,mp=[read(p/'summary.json'),read(p/'run.json'),read(p/'flight/result.json'),compressed(p/'flight/observations.json.gz'),compressed(p/'telemetry.json.gz'),read(p/'map_snapshot.json')]
    check(s['mode']==m['mode']=='unknown',name+': mode')
    check(not m['truth_available_to_planner'] and s['truth_obstacle_nonzero_count']==0,name+': truth leakage')
    check(not d['parse_errors'] and not tel['errors'] and not s['telemetry_errors'],name+': parse/telemetry errors')
    check(m['source_unchanged'] and s['source_unchanged'],name+': modified source')
    recorded={**m['source_sha256'],**m['binary_sha256']}
    for path,h in protocol['files'].items(): check(recorded.get(path)==h,name+': frozen file '+path)
    check(m['order']==s['requested_candidate_order']==ORDERS[item['method']],name+': order')
    check(s['candidate_order_mismatch_count']==0,name+': order mismatch')
    rec=r['recovery_parameters'];prim=r['planner_parameters']
    check(rec['max_candidates']==protocol['candidate_budget'] and rec['horizon_time']==protocol['successor_horizon_s'] and prim['horizon_time']==protocol['primary_horizon_s'],name+': K/H/B')
    check(rec['deferred_inference'] and rec['prune_dominated'] and rec['strict_execution'] and rec['stop_on_rejection'],name+': execution configuration')
    if item['method']=='Selected':check(near(rec['coverage_weight'],protocol['selected_coverage_weight']),name+': selected weight')
    check(len(r['waypoints'])==8,name+': waypoint count')
    trans=r['goal_transitions'];route_complete=r['stop_reason']=='completed_first_waypoint_cycle'
    completed=bool(route_complete and s['passed'])
    if route_complete:
        check(len(trans)==9,name+': completed transition count')
        check(r['waypoint_gate_checks_passed'],name+': waypoint gate')
        check(all(v.get('previous_goal_gate_passed',True) for v in trans),name+': per-waypoint gate')
    odom=d['odom'];clock=np.asarray([v['t'] for v in odom]);xyz=np.asarray([[v[k] for k in ('x','y','z')] for v in odom])
    check(len(clock)>0 and np.all(np.diff(clock)>0) and np.isfinite(xyz).all(),name+': odometry')
    startup,active=active_rows(d['status']);published=[v for v in active if v.get('command_published')]
    check(len(active)==s['active_planning_updates'],name+': active cutoff/count')
    check(len(d['status'])==s['planning_updates'],name+': mission cutoff/count')
    check(len(tel['status'])==s['telemetry_updates_including_shutdown_grace'],name+': shutdown telemetry count')
    check(all(v.get('truth_obstacle_count',0)==0 for v in d['status']),name+': mission truth count')
    check(all(v['payload'].get('truth_obstacle_count',0)==0 for v in tel['status']),name+': raw truth count')
    check(all(v.get('candidate_order',ORDERS[item['method']])==ORDERS[item['method']] for v in active),name+': per-update order')
    check(all(v.get('sensor_map_enabled',False) for v in active),name+': disabled sensor map')
    check(all(v.get('primary_evaluated',0) in (0,406) for v in active),name+': primary bank mismatch')
    check(all(v.get('scheduled_count',0)<=12 for v in active),name+': scheduled budget exceeded')
    check(tel['sensor_map'][0]['payload']['integrated_scans']==1,name+': map not reset')
    cutoff=float(trans[-1]['t']) if route_complete else float(clock[-1])
    if route_complete:
        check(float(clock[-1])<=cutoff+.1,name+': postmission odometry')
        check(max(v['t'] for v in d['status'])<=cutoff+.1,name+': postmission statuses')
    mesh=s['mesh_audit'];check(mesh['truth_mesh_sha256']==protocol['files'][m['truth_mesh']],name+': truth audit hash')
    check(mesh['segment_count']==len(odom)-1 and mesh['margin_m']==2.0,name+': mesh audit population')
    check(read(p/'mesh_audit.json')==mesh,name+': archived mesh audit differs')
    if s['passed']:
        check(route_complete and mesh['physical_intersection_count']==0 and mesh['margin_violation_count']==0,name+': success consistency')
        check(s['active_execution_rejection_count']==0 and s['active_map_contract_violation_count']==0,name+': success execution contract')
    steps=[int(v.get('backup_integration_steps',0)) for v in active]
    generation=dist(v.get('generation_duration') for v in active)
    latency=dist(v['command_stamp']-v['observation_stamp'] for v in published)
    validation=dist(v.get('dispatch_validation_age') for v in active)
    mapage=dist(v.get('map_age') for v in active)
    # The source summary uses the same observer population. Its percentiles are
    # checked, not pooled with the additional shutdown-grace telemetry rows.
    for observed,archived in ((latency,s['observation_to_command_s']),(validation,s['dispatch_validation_age_s']),(mapage,s['map_age_s'])):
        check(observed['count']==archived['count'],name+': latency population')
        if observed['count']:check(near(observed['p95'],archived['p95']),name+': latency percentile')
    row={**item,'run':name,'success':completed,'route_completed':route_complete,'passed':s['passed'],
        'completed_waypoints':max(0,len(trans)-1),'mission_waypoints':8,
        'stop_reason':r['stop_reason'],'supervisor_stop_reason':s['supervisor_stop_reason'],
        'completion_sim_s':float(trans[-1]['t']) if completed else None,
        'route_completion_sim_s':float(trans[-1]['t']) if route_complete else None,
        'observed_end_sim_s':float(clock[-1]),'mission_cutoff_sim_s':cutoff,
        'min_chord_clearance_m':mesh['minimum_surface_clearance_m'],
        'physical_intersections':mesh['physical_intersection_count'],'margin_violations':mesh['margin_violation_count'],
        'execution_rejections':s['active_execution_rejection_count'],'map_contract_violations':s['active_map_contract_violation_count'],
        'current_state_not_free_count':s['current_state_not_free_count'],'map_ready_false_count':s['map_ready_false_count'],
        'flight_observer_error':r.get('error'),
        'startup_wait_updates':startup,'active_updates':len(active),'command_updates':len(published),
        'fresh_updates':sum(v['status']=='RECOVERY_VERIFIED' for v in active),
        'cached_updates':sum(v['status']=='REVALIDATED_PLAN' for v in active),
        'maximum_primary_evaluated':max([v.get('primary_evaluated',0) for v in active],default=0),
        'maximum_scheduled_count':max([v.get('scheduled_count',0) for v in active],default=0),
        'cumulative_successor_steps':sum(steps),'mean_successor_steps':float(np.mean(steps)) if steps else None,
        'cumulative_cache_revalidation_steps':sum(int(v.get('cached_integration_steps',0)) for v in active),
        'mean_queries':float(np.mean([v.get('recovery_checked',0) for v in active])) if active else None,
        'pruned_candidates':sum(v.get('pruned_candidates',0) for v in active),
        'inference_batches':sum(v.get('inference_batches',0) for v in active),
        'inference_candidates':sum(v.get('inference_candidates',0) for v in active),
        'generation_mean_ms':None if generation['mean'] is None else 1000*generation['mean'],
        'generation_p95_ms':None if generation['p95'] is None else 1000*generation['p95'],
        'observation_command_mean_ms':None if latency['mean'] is None else 1000*latency['mean'],
        'observation_command_p95_ms':None if latency['p95'] is None else 1000*latency['p95'],
        'dispatch_validation_age_mean_ms':None if validation['mean'] is None else 1000*validation['mean'],
        'dispatch_validation_age_p95_ms':None if validation['p95'] is None else 1000*validation['p95'],
        'dispatch_validation_age_max_ms':None if validation['maximum'] is None else 1000*validation['maximum'],
        'map_age_p95_ms':None if mapage['p95'] is None else 1000*mapage['p95'],
        'free_voxels':mp['free_cells'],'occupied_voxels':mp['occupied_cells'],
        'truth_obstacle_nonzero_count':0,'source_unchanged':True,
        'legacy_box_observer_passed':r['passed'],
        'legacy_box_physical_intrusion_samples':r['physical_box_intrusion_samples'],
        'legacy_box_inflated_intrusion_samples':r['inflated_box_intrusion_samples'],
        'elapsed_wall_s':m['elapsed_wall_s']}
    details={'source_sha256':m['source_sha256'],'binary_sha256':m['binary_sha256'],
        'waypoints':r['waypoints'],'recovery_parameters':rec,'planner_parameters':prim,
        'generation_sim_s':generation,'observation_to_command_sim_s':latency,
        'dispatch_validation_age_sim_s':validation,'map_age_sim_s':mapage,
        'input_hashes':{rel:sha(p/rel) for rel in ('summary.json','run.json','flight/result.json','flight/observations.json.gz','telemetry.json.gz','mesh_audit.json','map_snapshot.json')}}
    return row,details

METRICS=('completion_sim_s','cumulative_successor_steps','mean_successor_steps','min_chord_clearance_m','generation_mean_ms','generation_p95_ms','observation_command_mean_ms','observation_command_p95_ms','dispatch_validation_age_mean_ms','dispatch_validation_age_p95_ms','dispatch_validation_age_max_ms','map_age_p95_ms','active_updates','fresh_updates','cached_updates','cumulative_cache_revalidation_steps','elapsed_wall_s')
def aggregate(rows):
    return {method:{'runs':len(group),'successes':sum(v['success'] for v in group),
            'waypoint_counts':[v['completed_waypoints'] for v in group],
            'physical_intersections':sum(v['physical_intersections'] for v in group),
            'margin_violations':sum(v['margin_violations'] for v in group),
            'execution_rejections':sum(v['execution_rejections'] for v in group),
            **{k:stats(v[k] for v in group) for k in METRICS}}
        for method in METHODS for group in [[v for v in rows if v['method']==method]]}
def pm(v,d=2,scale=1):
    if v['mean'] is None:return '--'
    if v['sample_sd'] is None:return ('%.*f'%(d,v['mean']/scale))
    return '%.*f $\\pm$ %.*f'%(d,v['mean']/scale,d,v['sample_sd']/scale)
def fragments(base,a):
    lines=[r'\begin{table*}[t]',r'\centering',
        r"\caption{Repeated unknown-city flights: three new acquisitions per schedule, each initialized with an empty sensor map. Entries are means $\pm$ sample standard deviations across flights. $T$ is the completion time; $C_{\min}$ is each flight's minimum reference-point chord clearance to the physical collision mesh. Work counts successor integration steps. Latencies use the ROS simulation clock; P95 is computed within each flight.}",
        r'\label{tab:urban_repeats}',r'\footnotesize',r'\setlength{\tabcolsep}{3pt}',r'\begin{tabular}{lcccccccc}',r'\hline',
        r'Method & Complete & $T$ (s) & \shortstack{Work/flight\\($10^3$)} & Work/update & $C_{\min}$ (m) & \shortstack{Generation\\mean (ms)} & \shortstack{Generation\\P95 (ms)} & \shortstack{Obs.--command\\P95 (ms)} \\',r'\hline']
    for m in METHODS:
        x=a[m];lines.append(m+' & '+str(x['successes'])+'/'+str(x['runs'])+' & '+' & '.join((pm(x['completion_sim_s']),pm(x['cumulative_successor_steps'],1,1000),pm(x['mean_successor_steps'],0),pm(x['min_chord_clearance_m']),pm(x['generation_mean_ms'],1),pm(x['generation_p95_ms'],1),pm(x['observation_command_p95_ms'],1)))+r' \\')
    lines.extend([r'\hline',r'\end{tabular}',r'\end{table*}'])
    (base/'manuscript_fragment.tex').write_text('\n'.join(lines)+'\n')
    allsuccess=sum(x['successes'] for x in a.values());physical=sum(x['physical_intersections'] for x in a.values());margin=sum(x['margin_violations'] for x in a.values())
    protocol=(r'We evaluate asynchronous urban navigation in three cyclic-order blocks, each containing one Cost, Diverse, and Selected flight. The order is Cost--Diverse--Selected, Diverse--Selected--Cost, and Selected--Cost--Diverse. Every flight starts with an empty sensor map and follows the same eight-waypoint route. All schedules use $H=B=3$ s and $K=12$, with compact deferred inference and incumbent-cost pruning; Selected uses $\lambda_c=0.75$. The model, scene, execution policy, and software configuration are held fixed across the nine flights. These repeated-flight statistics form a separate sample from the three prescribed acquisitions in Table~\ref{tab:city}. Table~\ref{tab:urban_repeats} reports means and sample standard deviations across the three acquisitions for each schedule. Planning statistics exclude the initial map-initialization interval and end at the mission cutoff. Completion times include only successfully completed missions. Generation and observation-to-command intervals use the ROS simulation clock; the reported P95 values are summarized across the three per-flight percentiles.'+'\n\n')
    protocol+=('The nine flights yield %d successful eight-waypoint completions, %d physical-mesh intersections, and %d violations of the 2 m clearance margin.\n'%(allsuccess,physical,margin))
    (base/'protocol_fragment.tex').write_text(protocol)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True,help='External raw campaign directory containing protocol.json and the nine block directories')
    parser.add_argument('--output',type=Path,required=True,help='New analysis output directory; archived records are not overwritten')
    parser.add_argument('--path-map',type=Path,help='JSON object mapping recorded absolute protocol paths to relocated files; hashes must still match')
    parser.add_argument('--partial',action='store_true')
    args=parser.parse_args();base=args.base.resolve();out=args.output.resolve()
    if out==base or out==BASE.resolve():parser.error('Choose a separate analysis output directory.')
    if out.exists() and any(out.iterdir()):parser.error('Analysis output must be new or empty.')
    relocated=read(args.path_map) if args.path_map else {}
    prot=read(base/'protocol.json');complete=(base/'complete.json').is_file()
    if not args.partial:check(complete,'Campaign has not written complete.json; use --partial for progress only.')
    current={path:sha(relocated.get(path,path)) for path in prot['files']};check(current==prot['files'],'Frozen source/model/config/binary changed')
    rows=[];details={};pending=[]
    for item in prot['sequence']:
        name='block%d_%s'%(item['block'],item['method'])
        if not (base/name/'summary.json').is_file():pending.append(name);continue
        row,detail=analyze_run(base,item,prot);rows.append(row);details[name]=detail
        print(name,'success',row['success'],'T',row['completion_sim_s'],'work',row['cumulative_successor_steps'],'clearance',row['min_chord_clearance_m'],flush=True)
    if not args.partial:
        check(not pending and len(rows)==9,'Incomplete final run population')
        campaign=read(base/'complete.json')
        check(campaign['all_runs_recorded'] and len(campaign['runs'])==9,'Campaign completion record incomplete')
        check([(x['block'],x['method']) for x in campaign['runs']]==[(x['block'],x['method']) for x in prot['sequence']],'Campaign sequence differs')
        for recorded_run,analysis_row in zip(campaign['runs'],rows):
            check(recorded_run['passed']==analysis_row['passed'],'Campaign/analysis outcome mismatch')
    if rows:
        reference=details[rows[0]['run']]
        for name,d in details.items():
            check(d['source_sha256']==reference['source_sha256'],name+': source differs among repetitions')
            check(d['binary_sha256']==reference['binary_sha256'],name+': binary differs among repetitions')
            check(d['waypoints']==reference['waypoints'],name+': mission route differs among repetitions')
    agg=aggregate(rows)
    result={'complete':complete and len(rows)==9,'pending':pending,'protocol_sha256':sha(base/'protocol.json'),
        'analysis_sha256':sha(__file__),'metric_reference':'work/reviewer_revision_20261101/paper/analyze_city.py',
        'population':'Nine newly acquired flights only; all outcomes retained; mission-observer cutoff excludes shutdown grace.',
        'success_definition':'Completed eight-waypoint cycle and passed independent physical-mesh clearance/execution checks.',
        'latency_units':'Milliseconds of ROS simulation time; not CPU wall time.',
        'dispersion':'Sample standard deviation (ddof=1) of per-flight values; no pooled-update dispersion.',
        'failed_completion_time':'null; observed_end_sim_s records the termination horizon.',
        'clearance_definition':'Recorded-reference-position chord distance to independent physical collision mesh, isotropic 2 m margin, 30 m cap; legacy box-envelope diagnostics retained separately.',
        'protocol_files_unchanged':True,'all_run_sources_identical':True,'rows':rows,'aggregate':agg,'details':details}
    out.mkdir(parents=True,exist_ok=True)
    suffix='partial' if args.partial else 'summary'
    (out/('analysis_'+suffix+'.json')).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    if rows:
        with (out/('per_run_'+suffix+'.csv')).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    if not args.partial:
        fragments(out,agg)
        flat=[]
        for method,a in agg.items():
            v={'method':method,'runs':a['runs'],'successes':a['successes']}
            for metric in METRICS:
                v[metric+'_mean']=a[metric]['mean'];v[metric+'_sample_sd']=a[metric]['sample_sd']
            flat.append(v)
        with (out/'aggregate.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
        notes=['Repeated unknown-city flight analysis',
               'All nine preregistered runs are included; no archived single acquisition is pooled.',
               'Source/config/model/binary hashes match protocol.json and remain identical across all runs.',
               'Completion follows the eight-waypoint cycle and independent physical collision-mesh assessment.',
               'The legacy rectangular planning-box observer can report intrusions in its enclosing boxes. These fields are retained in per_run_summary.csv; the original analysis uses exact physical collision-surface clearance for this mesh scene.',
               'Successor work is summed over active mission-observer statuses. Cache revalidation integration is a separate metric.',
               'Generation and latency use ROS simulation time. Dispatch-validation age is the command minus dispatch-state stamp, not CPU validation duration.',
               'Each dispersion is a sample SD across three flight-level observations; percentile rows summarize per-flight P95 values.',
               'Mean completion time contains successful missions only; failed outcomes retain null completion time and a separate termination horizon.', '']
        for method,a in agg.items():
            notes.append('%s: %d/%d complete; T %s s; successor work/flight %s; work/update %s; minimum clearance %s m; generation %s ms; generation P95 %s ms; observation-command P95 %s ms.'%(method,a['successes'],a['runs'],pm(a['completion_sim_s']),pm(a['cumulative_successor_steps'],0),pm(a['mean_successor_steps'],1),pm(a['min_chord_clearance_m'],3),pm(a['generation_mean_ms']),pm(a['generation_p95_ms']),pm(a['observation_command_p95_ms'])))
        notes.append('All-flight minimum physical clearance: %.6f m.'%min(v['min_chord_clearance_m'] for v in rows))
        notes.append('All-flight maximum dispatch-validation age: %.3f ms.'%max(v['dispatch_validation_age_max_ms'] for v in rows))
        (out/'ANALYSIS_NOTES.txt').write_text('\n'.join(notes)+'\n')
    print(json.dumps({'complete':result['complete'],'runs':len(rows),'pending':pending,'successes':{k:v['successes'] for k,v in agg.items()}},indent=2))
if __name__=='__main__':main()
