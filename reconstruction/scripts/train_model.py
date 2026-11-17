#!/usr/bin/env python3
"""Train reconstructed models only on explicitly supplied 58-feature datasets.

NPZ files must contain X (or x) [N,58], y [N], work [N], and scene_id (or scene)
identifiers for a train/validation scene-overlap check. This script creates no dataset
and does not recover manuscript weights, select deployment seeds, or validate
closed-loop schedules. New weights/results are labelled reconstructed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fwcontinuation.learning import ContinuationMLP


def load_dataset(path):
    with np.load(path,allow_pickle=False) as data:
        if not {"y","work"}.issubset(data.files) or not ({"x","X"}&set(data.files)):
            raise ValueError("Dataset requires X (or x), y and work arrays")
        if "x" in data.files and "X" in data.files and not np.array_equal(data['x'],data['X']):
            raise ValueError("Conflicting x and X feature arrays")
        x=np.asarray(data["X" if "X" in data.files else "x"],dtype=np.float64)
        y=np.asarray(data["y"],dtype=np.float64)
        work=np.asarray(data["work"],dtype=np.float64)
        if "scene_id" in data.files and "scene" in data.files and not np.array_equal(data['scene_id'],data['scene']):
            raise ValueError("Conflicting scene and scene_id arrays")
        scene=data["scene_id"].copy() if "scene_id" in data.files else (data["scene"].copy() if "scene" in data.files else None)
    if x.ndim!=2 or x.shape[1]!=58 or len(x)==0:
        raise ValueError("Expected nonempty reconstructed 58-feature dataset; legacy103 data cannot be reinterpreted")
    if y.shape!=(len(x),) or work.shape!=(len(x),):
        raise ValueError("Dataset label/work dimensions do not match x")
    if not all(np.isfinite(a).all() for a in [x,y,work]):
        raise ValueError("Dataset contains NaN/Inf")
    if not np.isin(y,[0,1]).all() or np.any(work<0):
        raise ValueError("Expected binary feasibility and nonnegative integration work")
    if scene is not None and scene.shape!=(len(x),):
        raise ValueError("Scene identifiers must have one entry per candidate")
    return x,y,work,scene


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--train",type=Path,required=True)
    p.add_argument("--validation",type=Path,required=True)
    p.add_argument("--output-dir",type=Path,required=True)
    p.add_argument("--dataset-description",required=True,help="Origin and label-generation protocol of the supplied data")
    p.add_argument("--variants",nargs="+",choices=["state_aux","obs_bce","obs_aux"],default=["state_aux","obs_bce","obs_aux"])
    p.add_argument("--seeds",nargs="+",type=int,default=[20261101,20261102,20261103])
    p.add_argument("--epochs",type=int,default=60)
    p.add_argument("--batch-size",type=int,default=2048)
    p.add_argument("--learning-rate",type=float,default=1e-3)
    p.add_argument("--weight-decay",type=float,default=1e-5)
    args=p.parse_args(argv)
    if args.train.resolve()==args.validation.resolve():
        p.error("Training and validation must be separate input files")
    train=load_dataset(args.train)
    validation=load_dataset(args.validation)
    if train[3] is None or validation[3] is None:
        p.error("Both datasets require scene_id (or scene) identifiers to check the scene split")
    if set(train[3].tolist()) & set(validation[3].tolist()):
        p.error("Training/validation scene IDs overlap; provide independently split scenes")
    split_check="provided scene IDs are disjoint; geometry uniqueness not independently established"
    paths=[args.output_dir/f"{v}_{s}" for v in args.variants for s in args.seeds]
    if len(set(paths))!=len(paths):
        p.error("Duplicate variant/seed combinations")
    if any(d.exists() for d in paths):
        p.error("An output run directory already exists; choose a new output directory")
    schema=Path(__file__).resolve().parents[1]/'configs/feature_schema.json'
    provenance={"dataset_description":args.dataset_description,"split_check":split_check,
                "feature_schema_sha256":hashlib.sha256(schema.read_bytes()).hexdigest(),
                "train_sha256":hashlib.sha256(args.train.read_bytes()).hexdigest(),
                "validation_sha256":hashlib.sha256(args.validation.read_bytes()).hexdigest(),
                "result_status":"NEW_RECONSTRUCTED_TRAINING; not manuscript reproduction"}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for variant in args.variants:
        for seed in args.seeds:
            model=ContinuationMLP.for_training(seed=seed,state_only=variant=="state_aux",
                                               auxiliary_weight=0.0 if variant=="obs_bce" else 0.1)
            model.fit(*train[:3],validation[0],validation[1],epochs=args.epochs,
                      batch_size=args.batch_size,learning_rate=args.learning_rate,
                      weight_decay=args.weight_decay)
            model.provenance.update(provenance)
            run=args.output_dir/f"{variant}_{seed}"
            run.mkdir()
            model.save(run/"training_model.npz")
            compact=model.export_compact()
            compact.save(run/"compact_model.json")
            compact.save(run/"compact_model.npz")
            summary={"variant":variant,"seed":seed,"best_epoch":model.best_epoch,
                     "best_validation_bce":model.best_validation_bce,
                     "history":model.history,**provenance}
            (run/"training.json").write_text(json.dumps(summary,indent=2,allow_nan=False),encoding="utf-8")
            summaries.append({k:v for k,v in summary.items() if k!="history"})
            print(json.dumps({k:summary[k] for k in ["variant","seed","best_epoch","best_validation_bce"]}),flush=True)
    (args.output_dir/"summary.json").write_text(json.dumps({"runs":summaries,
        "deployment_selection":"not performed; requires validation witness recall/work and closed-loop validation"},indent=2,allow_nan=False),encoding="utf-8")


if __name__=="__main__":
    main()
