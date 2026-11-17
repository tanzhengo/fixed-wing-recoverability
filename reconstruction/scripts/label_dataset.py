#!/usr/bin/env python3
"""Label actual supplied observed-map snapshots using the reconstructed oracle.

Input is JSONL, one frame per line using records.parse_frame's schema. These
new labels must never be mixed with the manuscript's unavailable old labels.
"""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fwcontinuation.records import parse_frame,canonical_hash,write_json,source_hashes
from fwcontinuation.features import encode_features
from fwcontinuation.planner import feasible_candidates,continuation_query

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames');parser.add_argument('--output',required=True)
    parser.add_argument('--split',choices=['train','validation','test'],required=True)
    args=parser.parse_args();output=Path(args.output)
    if output.suffix!='.npz':raise ValueError('Dataset output must end in .npz')
    if output.exists():raise FileExistsError('Refusing to overwrite dataset')
    output.parent.mkdir(parents=True,exist_ok=True)
    features=[];labels=[];work=[];groups=[];query_ids=[];accounting=[];ids=set()
    with open(args.frames,encoding='utf-8') as stream:
        for line in stream:
            if not line.strip():continue
            frame=json.loads(line);key=(str(frame['scene_id']),str(frame['query_id']))
            if key in ids:raise ValueError('Duplicate scene/query ID')
            ids.add(key);state,goal,observed_map,config,previous=parse_frame(frame)
            candidates,primary_steps=feasible_candidates(state,goal,observed_map,config,previous)
            positive=0
            for candidate in candidates:
                witness,steps,trace=continuation_query(candidate,observed_map,config)
                features.append(encode_features(state,candidate,goal,observed_map,config))
                labels.append(int(witness is not None));work.append(steps);groups.append(key[0]);query_ids.append(key[1])
                positive+=int(witness is not None)
            accounting.append({'scene_id':key[0],'query_id':key[1],'frame_sha256':canonical_hash(frame),
                'candidate_count':len(candidates),'positive_candidates':positive,'full_positive':positive>0,
                'primary_steps':primary_steps})
    X=np.asarray(features,dtype=np.float64).reshape(-1,58)
    np.savez_compressed(output,X=X,y=np.asarray(labels,dtype=np.float64),work=np.asarray(work,dtype=np.int64),
                        scene_id=np.asarray(groups),query_id=np.asarray(query_ids))
    write_json(output.with_suffix('.manifest.json'),{'schema':'fwcontinuation.reconstruction.labels.v1',
        'split':args.split,'historical_labels_reproduced':False,'source_sha256':source_hashes(),
        'input_sha256':hashlib.sha256(Path(args.frames).read_bytes()).hexdigest(),
        'dataset_sha256':hashlib.sha256(output.read_bytes()).hexdigest(),
        'examples':len(labels),'positive_examples':sum(labels),'query_accounting':accounting})
    print(json.dumps({'examples':len(labels),'queries':len(accounting),'output':str(output)}))

if __name__=='__main__':main()
