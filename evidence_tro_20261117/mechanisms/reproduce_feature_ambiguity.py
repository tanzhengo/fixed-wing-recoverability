from pathlib import Path
import argparse
p=argparse.ArgumentParser(description='Reproduce state-feature ambiguity from original validation arrays.');p.add_argument('--artifact-root',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);args=p.parse_args();ROOT=args.artifact_root.resolve();OUT=args.output_dir.resolve();OUT.mkdir(parents=True,exist_ok=True)
"""Reproduce the fixed validation pair and ambiguous state-feature group count."""
from pathlib import Path
import csv,json,hashlib,collections
import numpy as np
DATA=ROOT/'work/reviewer_revision_20261101/training/dataset'
d=dict(np.load(DATA/'val.npz',allow_pickle=False))
states={int(x['state_id']):x for x in csv.DictReader((DATA/'val_states.csv').open())}
fields=['x','y','h','V','gamma','heading','roll','attack','thrust']
groups={}
for i,x in enumerate(d['x'][:,:28]):groups.setdefault(x.tobytes(),[]).append(i)
conflicting=matched=0;first=None
for inds in groups.values():
 if not (any(d['y'][i]==0 for i in inds) and any(d['y'][i]==1 for i in inds)):continue
 conflicting+=1;raw={}
 for i in inds:
  state=states[int(d['state'][i])]
  key=tuple(float(state[k]) for k in fields)+(int(d['candidate'][i]),)
  raw.setdefault(key,[]).append(i)
 pairs=[]
 for rr in raw.values():
  neg=next((i for i in rr if d['y'][i]==0),None)
  pos=next((i for i in rr if d['y'][i]==1),None)
  if neg is not None and pos is not None:pairs.append((neg,pos))
 if pairs:
  matched+=1
  if first is None:first=pairs[0]
assert (len(groups),conflicting,matched)==(60863,1186,1186)
assert first==(48,66005)
result={'distinct_state_task_vectors':len(groups),'conflicting_groups':conflicting,'conflicting_groups_with_matching_recorded_raw_state_and_candidate':matched,'first_pair_packed_rows':list(first),'raw_fields_compared':fields,'dataset_sha256':hashlib.sha256((DATA/'val.npz').read_bytes()).hexdigest(),'states_sha256':hashlib.sha256((DATA/'val_states.csv').read_bytes()).hexdigest()}
(OUT/'validation_group_count_reproduced.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
