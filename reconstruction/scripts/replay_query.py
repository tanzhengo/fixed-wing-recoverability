#!/usr/bin/env python3
"""Replay one timestamped observed-map frame; never executes an aircraft."""
import argparse,hashlib,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fwcontinuation.planner import plan,MODES
from fwcontinuation.records import parse_frame,result_record,write_json
from fwcontinuation.learning import ContinuationMLP

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frame');parser.add_argument('--mode',choices=MODES,default='diverse')
    parser.add_argument('--model');parser.add_argument('--output',required=True)
    parser.add_argument('--no-pruning',action='store_true');parser.add_argument('--eager',action='store_true')
    args=parser.parse_args()
    frame=json.loads(Path(args.frame).read_text(encoding='utf-8'))
    state,goal,observed_map,config,previous=parse_frame(frame)
    model=ContinuationMLP.load(args.model) if args.model else None
    model_hash=hashlib.sha256(Path(args.model).read_bytes()).hexdigest() if args.model else None
    result=plan(state,goal,observed_map,config,args.mode,model,previous,not args.no_pruning,not args.eager)
    write_json(args.output,result_record(frame,result,args.mode,model_hash))
    print(json.dumps({'status':result.status,'successor_steps':result.steps,'output':args.output}))

if __name__=='__main__':main()
