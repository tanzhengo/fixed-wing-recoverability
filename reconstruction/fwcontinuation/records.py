"""Replayable records with explicit reconstruction provenance."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from . import __version__
from .types import Config, State, Control
from .observed_map import ObservedMap

def canonical_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

def source_hashes():
    root=Path(__file__).parent
    return {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.py'))}

def parse_frame(record):
    required=('state','goal','map','scene_id','query_id')
    if any(key not in record for key in required):raise ValueError('Frame requires '+', '.join(required))
    state=State(**record['state']);config=Config(**record.get('config',{}))
    goal=tuple(float(x) for x in record['goal'])
    if len(goal)!=3 or any(not __import__('math').isfinite(x) for x in goal):raise ValueError('Invalid goal')
    observed_map=ObservedMap.from_snapshot(record['map'])
    if observed_map.voxel_size!=config.voxel_size:raise ValueError('Map/config voxel-size mismatch')
    previous=Control(**record['previous']) if record.get('previous') else None
    return state,goal,observed_map,config,previous

def result_record(frame,result,mode,model_hash=None):
    witness=result.witness
    return {'schema':'fwcontinuation.reconstruction.query.v1','implementation':__version__,
            'evidence_kind':'new_reconstructed_numerical_replay','historical_result_reproduced':False,
            'frame_sha256':canonical_hash(frame),'source_sha256':source_hashes(),
            'model_sha256':model_hash,'mode':mode,'scene_id':frame['scene_id'],'query_id':frame['query_id'],
            'status':result.status,'selected_candidate_id':witness.candidate.index if witness else None,
            'witness_successor_id':witness.successor_index if witness else None,
            'primary_steps':result.primary_steps,'successor_steps':result.steps,
            'inference_calls':result.inference_calls,'map_version':result.map_version,
            'fixed_schedule':result.schedule,'query_trace':result.queries,'scores':result.recorded_scores,
            'candidates':[asdict(c) for c in result.candidates],
            'witness':asdict(witness) if witness else None}

def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
