from pathlib import Path
import argparse
p=argparse.ArgumentParser(description='Compare a fresh C++ replay against original validation arrays and archived outcomes.');p.add_argument('--artifact-root',type=Path,required=True);p.add_argument('--run-dir',type=Path,required=True);args=p.parse_args();ROOT=args.artifact_root.resolve();OUT=args.run_dir.resolve()
from pathlib import Path
import numpy as np,csv,json,hashlib,collections
DATA=ROOT/'work/reviewer_revision_20261101/training/dataset'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return list(csv.DictReader(p.open()))
d=dict(np.load(DATA/'val.npz',allow_pickle=False));states=read(DATA/'val_states.csv');report={};pair=[]
for scene,name in enumerate(['val_culdesac_0_m0','val_staggered_0_m0']):
 st=next(a for a in states if a['case']==name and a['query']=='4');sid=int(st['state_id']);idx=np.where(d['state']==sid)[0];a=np.fromfile(OUT/'validation_pair'/(name+'_rerun.f32'),'<f4').reshape(-1,115);succ=a[:,70:].reshape(-1,15,3)
 match={'features':np.array_equal(a[:,12:70],d['x'][idx]),'labels':np.array_equal(a[:,3],d['y'][idx]),'work':np.array_equal(a[:,4],d['work'][idx]),'candidate_order':np.array_equal(a[:,11],d['candidate'][idx]),'successor_labels':np.array_equal(succ[:,:,0],d['successor_label'][idx]),'successor_work':np.array_equal(succ[:,:,1],d['successor_work'][idx]),'successor_reasons':np.array_equal(succ[:,:,2],d['successor_reason'][idx])};assert all(match.values())
 row=a[a[:,11]==69][0];gi=int(idx[np.flatnonzero(a[:,11]==69)[0]]);pair.append(row)
 report[name]={'parity':{k:bool(v) for k,v in match.items()},'candidate_count':len(a),'raw_state':{k:float(st[k]) for k in ['x','y','h','V','gamma','heading','roll','attack','thrust']},'goal':[250,0,20],'query_time':float(st['t']),'candidate_id':69,'rank':int(row[1]),'label':int(row[3]),'work_until_first_witness':int(row[4]),'full_valid_controls_1based':(np.flatnonzero(row[70:].reshape(15,3)[:,0])+1).tolist(),'encoded_state28':row[12:40].tolist(),'observation30':row[40:70].tolist(),'recorded_network_predictions':{}}
 for model in ['state_aux/20261102','observed/20261103','observed_aux/20261101']:
  file=ROOT/'work/reviewer_revision_20261101/training/models'/model/'validation_deployed_predictions.npz';v=np.load(file)['output'][gi];report[name]['recorded_network_predictions'][model]={'logit':float(v[0]),'probability':float(v[2]),'source_sha256':sha(file)}
assert report['val_culdesac_0_m0']['raw_state']==report['val_staggered_0_m0']['raw_state'];assert np.array_equal(pair[0][12:40],pair[1][12:40]);assert not np.array_equal(pair[0][40:70],pair[1][40:70])
paths=read(OUT/'validation_pair/paths.csv');controls=read(OUT/'validation_pair/controls.csv')
pa=[[{k:v for k,v in x.items() if k!='scene'} for x in paths if x['scene']==str(s) and x['candidate']=='69' and x['successor']=='-1'] for s in [0,1]];co=[[{k:v for k,v in x.items() if k!='scene'} for x in controls if x['scene']==str(s) and x['candidate']=='69'] for s in [0,1]];assert pa[0]==pa[1];assert co[0]==co[1];assert all(float(x['t'])<=6 for x in paths)
report['pair_equalities']={'raw_state_identical':True,'initial_yaw_rate':[0,0],'goal_identical':True,'full_primary_state_sequence_identical':True,'primary_and_all_15_successor_commands_identical':True,'encoded_state28_identical':True,'observation_different_components':int(np.count_nonzero(pair[0][40:70]!=pair[1][40:70])),'observation_l1':float(np.abs(pair[0][40:70]-pair[1][40:70]).sum()),'observation_max_difference':float(np.max(np.abs(pair[0][40:70]-pair[1][40:70]))),'primary_control':co[0][0],'complete_control_bank':co[0][1:]}
checks=read(OUT/'validation_pair/candidate_checks.csv');report['successor_rejections']={}
for scene,cid in [(0,69),(0,107),(1,69)]:
 a=[x for x in checks if x['scene']==str(scene) and x['candidate']==str(cid)];counts=collections.Counter()
 for q in a:
  reason=int(q['rejection'])
  if reason==0:cat='valid'
  elif reason==1:cat='envelope'
  elif reason==2:
   u,o=int(q['unknown_blockers']),int(q['occupied_blockers']);cat='unknown_and_occupied' if u and o else 'unknown_only' if u else 'occupied_only' if o else 'unclassified';assert cat!='unclassified'
  else:cat='other'
  counts[cat]+=1
 assert sum(counts.values())==15
 report['successor_rejections'][str(scene)+'_P'+str(cid)]=dict(counts)
new=read(OUT/'urban_trace/query_results_mechanism.csv');prior=read(ROOT/'work/reviewer_revision_20261101/urban_replay/learned3/query_results_budget_grid.csv');urban=[]
for a in new:
 name={'ObsAux':'ObsAux','Full':'Full'}.get(a['method'],a['method']+'_K12');b=next(x for x in prior if x['method']==name and x['query_index']=='0');keys=[k for k in a if k not in ['method','ms','repeat','order'] and not k.endswith('_ms')];m={k:[a[k],b[k]] for k in keys if a[k]!=b[k]};assert not m
 urban.append({'method':a['method'],'prior_method':name,'mismatches':m,'work':int(a['backup_steps']),'queries':int(a['queries']),'pruned':int(a['pruned_candidates']),'selected_cost_rank':int(a['cost_rank']),'cost':float(a['cost']),'signature':a['signature']})
report['urban_parity']=urban
report['case_selection']={'pair':'First conflicting 28-feature group in the original packed validation order with exactly matching recorded raw state and primary library ID; first negative and first positive members. Alternative primary: first continuation-positive candidate in stable task-cost order on Map A.','urban':'First common-seed-failing input in the original acquisition order learned3, cost1, diverse1 and increasing prescribed query time. This is learned3 query index 0 at 2 s. No selection based on model advantage.'}
report['source_hashes']={str(p):sha(p) for p in [DATA/'val.npz',DATA/'val_states.csv',ROOT/'work/reviewer_revision_20261101/urban_replay/learned3/query_results_budget_grid.csv',OUT/'validation_pair/paths.csv',OUT/'validation_pair/candidate_checks.csv',OUT/'urban_trace/trace.log',OUT/'urban_trace/query_results_mechanism.csv']}
(OUT/'parity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'pair':report['pair_equalities'],'rejections':report['successor_rejections'],'urban':urban},indent=2))
