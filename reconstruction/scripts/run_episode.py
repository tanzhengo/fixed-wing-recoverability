#!/usr/bin/env python3
"""Run one explicitly supplied scene as a synchronous numerical diagnostic.

Scene JSON requires scene_id, start (all nine State physical fields), goal
[x,y,h], and obstacles (sphere/cylinder dictionaries). Optional ground_height,
planner_config and simulation_config are supported. No scenes, success counts,
trained weights or historical experiment records are generated implicitly.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fwcontinuation.types import Config
from fwcontinuation.simulation import SimulationConfig,run_episode
from fwcontinuation.learning import ContinuationMLP
from fwcontinuation.planner import MODES


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('scene',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--mode',choices=MODES,default='diverse')
    p.add_argument('--model',type=Path)
    p.add_argument('--duration',type=float)
    p.add_argument('--condition',choices=['nominal','crosswind','model_mismatch','delay100ms'])
    p.add_argument('--azimuth-count',type=int,help='Explicit diagnostic override; manuscript setting is 512')
    p.add_argument('--elevation-count',type=int,help='Explicit diagnostic override; manuscript setting is 32')
    p.add_argument('--max-range',type=float,help='Explicit diagnostic override; manuscript setting is 120 m')
    args=p.parse_args(argv)
    if args.mode in ('learned','combined') and args.model is None:
        p.error('Learned schedules require an explicitly supplied trained model')
    scene=json.loads(args.scene.read_text(encoding='utf-8'))
    config=Config(**scene.get('planner_config',{}))
    settings=dict(scene.get('simulation_config',{}))
    for key in ['duration','condition','azimuth_count','elevation_count','max_range']:
        value=getattr(args,key)
        if value is not None:settings[key]=value
    simulation=SimulationConfig(**settings)
    model=ContinuationMLP.load(args.model) if args.model else None
    model_hash=hashlib.sha256(args.model.read_bytes()).hexdigest() if args.model else None
    summary=run_episode(scene,args.output_dir,config,simulation,args.mode,model,model_hash)
    print(json.dumps({k:summary[k] for k in ['evidence_kind','clock_policy','outcome',
          'elapsed_simulation_seconds','query_count','scan_count','fresh_sequences','cached_sequences']}))


if __name__=='__main__':main()
