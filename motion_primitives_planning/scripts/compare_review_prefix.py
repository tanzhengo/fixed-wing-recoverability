#!/usr/bin/env python3
"""Paired comparison of the frozen prefix-off and prefix-on test matrices."""
from pathlib import Path
import csv,json
from collections import Counter
from analyze_review_runtime import interval
from review_runtime_io import csv_rows

def read(p):
 return {(r['scene'],r['condition'],r['method']):r for r in csv_rows(p)}

def main():
 repo=Path(__file__).resolve().parents[2];root=repo/'docs/recovery/revision/runtime'
 off=read(root/'heldout_episodes.csv');on=read(root/'prefix/heldout_episodes.csv');assert set(off)==set(on)
 result={}
 for method in sorted({k[2] for k in off}):
  keys=[k for k in off if k[2]==method];assert len(keys)==128
  a={(k[0],k[1]):int(on[k]['outcome']=='goal') for k in keys};b={(k[0],k[1]):int(off[k]['outcome']=='goal') for k in keys}
  result[method]={'off_outcomes':dict(Counter(off[k]['outcome'] for k in keys)),'on_outcomes':dict(Counter(on[k]['outcome'] for k in keys)),
    'on_minus_off_completion':interval(a,b,list(a)),'on_only_completions':sum(a[k] and not b[k] for k in a),'off_only_completions':sum(b[k] and not a[k] for k in a),
    'condition_goals':{c:{'off':sum(off[k]['outcome']=='goal' for k in keys if k[1]==c),'on':sum(on[k]['outcome']=='goal' for k in keys if k[1]==c)} for c in sorted({k[1] for k in keys})}}
 output={'paired_episodes_per_layer':len(off),'comparison':result,'interpretation':'Same models and scenes; only common nominal RK4 execution-prefix filter changes. Candidate scheduling is not credited with the shared numerical-filter effect.'}
 (root/'prefix_comparison.json').write_text(json.dumps(output,indent=2)+'\n');print(json.dumps(output,indent=2))
if __name__=='__main__':main()
