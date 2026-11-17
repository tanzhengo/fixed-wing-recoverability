#!/usr/bin/env python3
"""Reproduce frozen pairs from the declared family traversal; no figure inspection."""
from pathlib import Path
import argparse,csv,json,itertools,collections
import numpy as np
OUT=Path(__file__).resolve().parent

def main():
 p=argparse.ArgumentParser();p.add_argument('--artifact-root',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);args=p.parse_args();args.dataset=args.artifact_root.resolve()/'work/reviewer_revision_20261101/training/dataset';args.output_dir.mkdir(parents=True,exist_ok=True)
 d=dict(np.load(args.dataset/'val.npz',allow_pickle=False));s={int(x['state_id']):x for x in csv.DictReader((args.dataset/'val_states.csv').open())}
 protocol=json.loads((OUT/'selection_protocol.json').read_text());frozen=json.loads((OUT/'frozen_pairs_final.json').read_text());fields=protocol['raw_state_fields']+['t']
 def key(i):
  st=s[int(d['state'][i])];return tuple(float(st[k]) for k in fields)+(int(d['candidate'][i]),d['x'][i,:28].tobytes())
 def find(fa,fb,used):
  index=collections.defaultdict(list)
  for i in range(len(d['y'])):
   st=s[int(d['state'][i])]
   if st['family']==fb and st['scene'] not in used:index[key(i)].append(i)
  count=0;first=None
  for i in range(len(d['y'])):
   st=s[int(d['state'][i])]
   if st['family']!=fa or st['scene'] in used:continue
   j=next((j for j in index.get(key(i),[]) if d['y'][i]!=d['y'][j]),None)
   if j is not None:
    count+=1
    if first is None:first=[i,j]
  return count,first
 pair1=protocol['preserved_group']['packed_rows'];used={s[int(d['state'][i])]['scene'] for i in pair1}
 count2,pair2=find('wall','spatial',used);assert count2==0 and pair2 is None
 _,pair3=find('forest_sparse','forest_dense',used);used|={s[int(d['state'][i])]['scene'] for i in pair3}
 families=sorted({v['family'] for v in s.values()});order=sorted(itertools.combinations(families,2),key=lambda x:(not any(v in ['wall','spatial'] for v in x),x));counts=[]
 for fa,fb in order:
  count,pair=find(fa,fb,used);counts.append({'families':[fa,fb],'eligible_packed_rows':count})
  if pair2 is None and pair is not None:pair2=pair
 actual=[pair1,pair2,pair3];expected=[[v['packed_row'] for v in g['pair']] for g in frozen['groups']]
 assert actual==expected,(actual,expected);assert counts==frozen['fallback_family_pair_counts']
 report={'packed_rows':actual,'all_match_frozen_pairs':True,'wall_spatial_original_match_count':count2,'fallback_counts':counts}
 (args.output_dir/'selection_reproduced.json').write_text(json.dumps(report,indent=2)+'\n');print('Frozen pair selection reproduced:',actual)
if __name__=='__main__':main()
