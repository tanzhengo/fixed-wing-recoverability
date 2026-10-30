#!/usr/bin/env python3
"""Freeze deployment configuration from predeclared validation execution only."""
from pathlib import Path
import csv,datetime,hashlib,json
from review_runtime_io import csv_rows, raw_sha256

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 repo=Path(__file__).resolve().parents[2];root=repo/'docs/recovery/revision/runtime';training=repo/'docs/recovery/revision/training'
 result_path=root/'validation_episodes.csv';manifest_path=root/'validation_episodes_run_manifest.json'
 if not manifest_path.exists():manifest_path=root/'validation_run_manifest.json'
 manifest=json.loads(manifest_path.read_text())
 assert manifest.get('exit_code')==0 and manifest.get('source_unchanged') and manifest.get('models_unchanged')
 rows=csv_rows(result_path);assert len(rows)==512
 lookup={'geometry_bce_lazy12':'geometry_bce','geometry_aux_lazy12':'geometry_joint','state_bce_lazy12':'state_bce','state_aux_lazy12':'state_joint'}
 cells={c['cell']:c for c in json.loads((training/'summary.json').read_text())['cells']}
 results=[]
 for method,cell in lookup.items():
  rr=[r for r in rows if r['method']==method];assert len(rr)==128
  assert len(set(r['scene'] for r in rr))==32 and all(r['scene'].startswith('val_') for r in rr)
  results.append({'configuration':cell,'method':method,'completed':sum(r['outcome']=='goal' for r in rr),'margin_incursions':sum(r['outcome']=='collision' for r in rr),'validation_query_mean_recall':cells[cell]['mean_validation_recall'],'validation_mean_bce':cells[cell]['mean_validation_bce'],'condition_completions':{c:sum(r['condition']==c and r['outcome']=='goal' for r in rr) for c in sorted(set(r['condition'] for r in rr))}})
 results.sort(key=lambda r:(-r['completed'],r['margin_incursions'],-r['validation_query_mean_recall'],r['validation_mean_bce'],r['configuration']))
 winner=results[0]['configuration'];model=training/winner/'20261030/network_compact.txt'
 freeze={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'selected_configuration':winner,'deployment_seed':20261030,'model_path':str(model.relative_to(repo)),'model_sha256':sha(model),'new_test_accessed':False,'selection_protocol_sha256':sha(root/'VALIDATION_SELECTION_PROTOCOL.json'),'validation_episodes_sha256':raw_sha256(result_path),'validation_run_manifest_sha256':sha(manifest_path),'training_summary_sha256':sha(training/'summary.json'),'ranked_validation_configurations':results,'interpretation':'Execution configuration selected only on existing validation maps. Original query-only freeze remains separate.'}
 target=root/'EXECUTION_MODEL_FREEZE.json';assert not target.exists();target.write_text(json.dumps(freeze,indent=2)+'\n');print(json.dumps(freeze,indent=2))
if __name__=='__main__':main()
