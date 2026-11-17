from pathlib import Path
import argparse
p=argparse.ArgumentParser(description='Compare a fresh C++ replay against original validation arrays and archived outcomes.');p.add_argument('--artifact-root',type=Path,required=True);p.add_argument('--run-dir',type=Path,required=True);args=p.parse_args();ROOT=args.artifact_root.resolve();OUT=args.run_dir.resolve()
from pathlib import Path
import csv,json,hashlib,collections
import numpy as np
DATA=ROOT/'work/reviewer_revision_20261101/training/dataset'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return list(csv.DictReader(p.open()))
frozen=json.loads((OUT/'frozen_pairs_final.json').read_text());d=dict(np.load(DATA/'val.npz',allow_pickle=False));states=read(DATA/'val_states.csv');paths=read(OUT/'raw/paths.csv');controls=read(OUT/'raw/controls.csv');checks=read(OUT/'raw/candidate_checks.csv');maps=read(OUT/'raw/map_summary.csv');report={'groups':[],'parity':[],'selection_protocol_sha256':sha(OUT/'selection_protocol.json'),'selection_amendment_sha256':sha(OUT/'selection_amendment.json'),'frozen_pairs_sha256':sha(OUT/'frozen_pairs_final.json'),'validation_population_unchanged':{'inputs':767,'full_positive':517}}
for gi,group in enumerate(frozen['groups']):
 paired=[]
 for side,item in enumerate(group['pair']):
  sc=2*gi+side;name=item['state']['case'];cid=item['candidate'];sid=int(item['state']['state_id']);idx=np.where(d['state']==sid)[0];a=np.fromfile(OUT/'raw'/(name+'_rerun.f32'),'<f4').reshape(-1,115);succ=a[:,70:].reshape(-1,15,3)
  equal={'features58':np.array_equal(a[:,12:70],d['x'][idx]),'labels':np.array_equal(a[:,3],d['y'][idx]),'work':np.array_equal(a[:,4],d['work'][idx]),'candidate_order':np.array_equal(a[:,11],d['candidate'][idx]),'successor_labels':np.array_equal(succ[:,:,0],d['successor_label'][idx]),'successor_work':np.array_equal(succ[:,:,1],d['successor_work'][idx]),'successor_reasons':np.array_equal(succ[:,:,2],d['successor_reason'][idx])};assert all(equal.values()),(name,equal)
  rr=a[a[:,11]==cid][0];row=int(item['packed_row']);assert d['candidate'][row]==cid and d['state'][row]==sid
  cp=[{k:v for k,v in r.items() if k!='scene'} for r in controls if int(r['scene'])==sc and int(r['candidate'])==cid];pp=[{k:v for k,v in r.items() if k!='scene'} for r in paths if int(r['scene'])==sc and int(r['candidate'])==cid and r['successor']=='-1'];cc=[r for r in checks if int(r['scene'])==sc and int(r['candidate'])==cid];assert len(cp)==16 and len(cc)==15
  reasoncounts=collections.Counter();outcomes=[]
  for c in cc:
   reason=int(c['rejection']);u,o=int(c['unknown_blockers']),int(c['occupied_blockers'])
   cat='valid' if reason==0 else 'envelope' if reason==1 else ('unknown_and_occupied' if u and o else 'unknown_only' if u else 'occupied_only' if o else 'unclassified') if reason==2 else 'other';assert cat not in ['unclassified','other'];reasoncounts[cat]+=1;outcomes.append({'control_index_1based':int(c['successor'])+1,'verified':int(c['verified']),'steps':int(c['work']),'rejection':int(c['rejection']),'category':cat,'unknown_blockers':u,'occupied_blockers':o})
  rawstate={k:float(item['state'][k]) for k in ['x','y','h','V','gamma','heading','roll','attack','thrust']};rawstate['d_sigma']=0
  paired.append({'scene_index':sc,'case':name,'family':item['state']['family'],'seed':int(item['state']['seed']),'state':rawstate,'time_s':float(item['state']['t']),'goal':[250,0,20],'candidate_id':cid,'cost':float(rr[2]),'rank':int(rr[1]),'primary_feasible':True,'label':int(rr[3]),'work_until_first_witness':int(rr[4]),'full_bank_work':int(rr[70:].reshape(15,3)[:,1].sum()),'verified_controls_1based':(np.flatnonzero(rr[70:].reshape(15,3)[:,0])+1).tolist(),'feature28':rr[12:40].tolist(),'observation30':rr[40:70].tolist(),'commands':cp,'primary_state_sequence':pp,'successor_outcomes':outcomes,'rejection_counts':dict(reasoncounts),'map_counts':maps[sc],'map_slice_sha256':sha(OUT/'raw'/(name+'_slice.csv')),'geometry_sha256':sha(OUT/'raw'/(name+'_geometry.csv'))})
  report['parity'].append({'case':name,'candidates':len(a),'exact_original_dataset_parity':{k:bool(v) for k,v in equal.items()}})
 left,right=paired;assert left['state']==right['state'];assert left['goal']==right['goal'];assert left['primary_state_sequence']==right['primary_state_sequence'];assert left['commands']==right['commands'];assert left['feature28']==right['feature28'];assert left['label']!=right['label'];assert left['geometry_sha256']!=right['geometry_sha256'];diff=np.array(left['observation30'])-right['observation30'];assert np.any(diff)
 grouprecord={'group':gi+1,'equalities':{'raw_state_goal_yaw_rate':True,'primary_state_sequence':True,'primary_and_15_successor_commands':True,'state_task_features28':True,'all15_controls_checked':True},'observation_changed_components':int(np.count_nonzero(diff)),'observation_l1_difference':float(np.abs(diff).sum()),'observation_max_difference':float(np.max(np.abs(diff))),'members':paired}
 report['groups'].append(grouprecord)
 print('Group',gi+1,[(m['case'],m['label'],len(m['verified_controls_1based']),m['work_until_first_witness']) for m in paired], 'obs-different',grouprecord['observation_changed_components'])
 for m in paired:
  pp=[p for p in paths if int(p['scene'])==m['scene_index'] and int(p['candidate'])==m['candidate_id']]
  print(' path range',m['case'],[(k,min(float(p[k]) for p in pp),max(float(p[k]) for p in pp)) for k in ['x','y','h']])
assert len({m['geometry_sha256'] for g in report['groups'] for m in g['members']})==6
report['six_distinct_geometries_and_observed_maps']=True
report['source_hashes']={str(p.relative_to(OUT)):sha(p) for p in (OUT/'raw').glob('*') if p.is_file()};report['source_dataset_sha256']=sha(DATA/'val.npz')
(OUT/'pair_evidence.json').write_text(json.dumps(report,indent=2)+'\n')
# Small data summary for figure labels and reviewer inspection.
with (OUT/'paired_summary.csv').open('w') as f:
 fields=['group','case','seed','candidate','label','valid_controls','work_until_first_witness','full_bank_work','changed_observation_features'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
 for g in report['groups']:
  for m in g['members']:w.writerow(dict(group=g['group'],case=m['case'],seed=m['seed'],candidate=m['candidate_id'],label=m['label'],valid_controls=len(m['verified_controls_1based']),work_until_first_witness=m['work_until_first_witness'],full_bank_work=m['full_bank_work'],changed_observation_features=g['observation_changed_components']))
