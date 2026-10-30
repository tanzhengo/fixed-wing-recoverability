# Geometry and auxiliary-loss revision ablation

This experiment addresses the missing independent BCE-only versus BCE+Huber comparison. It uses only the archived training and validation maps; no test data enters this script.

## Protocol

The matrix has geometry/state-only inputs × BCE-only/BCE+0.1 Huber loss, with seeds 20261030, 20261031 and 20261032. All 12 runs use the same 103–64–32–2 training architecture, initialization, batches, Adam settings and 60-epoch budget. Every checkpoint is selected by validation BCE, so the checkpoint criterion is comparable across loss variants. State-only training masks the 75 geometry channels. All priorities use feasibility alone (beta=0).

The original query-only selection averages K12 and W3600 validation discovery over all three seeds; tied discovery is resolved by mean validation BCE. Deployment seed is fixed at 20261030. `PROTOCOL.json` was written before training began.

The parent revision subsequently introduced a separate closed-loop-validation selection stage before opening the new test maps. Consequently, `SELECTION_FREEZE.json` records the **query-only comparator**, not the final deployment decision. That later protocol and its outcomes are recorded separately by the revision benchmark.

## Validation results

32 maps supply 768 sampled states, including 477 states with a finite-library witness. Values below are mean ± sample standard deviation across three training seeds; they are validation results, not independent held-out execution evidence.

| Input / loss | BCE | AUROC | K12 recall (%) | W3600 recall (%) |
| --- | ---: | ---: | ---: | ---: |
| geometry_bce | 0.07645 | 0.99602 | 97.554 ± 0.121 | 87.841 ± 0.363 |
| geometry_joint | 0.07548 | 0.99615 | 98.183 ± 0.320 | 87.212 ± 0.000 |
| state_bce | 0.28208 | 0.94799 | 96.087 ± 0.121 | 86.233 ± 0.640 |
| state_joint | 0.28205 | 0.94838 | 95.947 ± 0.121 | 85.465 ± 1.396 |

For geometry inputs, auxiliary supervision raises K12 discovery by about 0.629 percentage points and lowers W3600 discovery by the same amount. The two-budget mean ties; the lower BCE selects geometry_joint as the query-only comparator. With state inputs, the auxiliary loss does not improve either recall metric. These data support a budget-dependent tradeoff rather than a universal benefit from the work target.

## Actual compact inference

The V1 export preserves the original two-head format for comparisons. The V2 export removes the unused work output; state-only exports additionally remove all 75 inactive input columns. The resulting state-only inference network has 28–64–32–1 layers and 3,969 parameters. Geometry inference has 103–64–32–1 layers and 8,769 parameters. These are exact weight extractions, with no retraining.

The header is `RECOVERY_MLP_V2 <28 or 103> 64 32 1`, followed by six whitespace-separated rows: W1, b1, W2, b2, W3, b3. Weights are output-major, matching PyTorch Linear. `compact_parity.csv` has 64 real validation feature vectors plus the PyTorch feasibility logit.

`check_exports.py` verifies identical retained weights, zero discarded state-only columns, dimensions, hashes and sequential-float32 parity against all 768 fixture rows across 12 models. Maximum absolute logit error is 1.1444091796875e-5. The C++ deployment loader is tested separately in the revision planner.

## Data support

The archived datasets contain 298,418 training candidates from 160 maps and 53,117 validation candidates from 32 maps. `feature_support.json` records candidate-wise ranges and 1st/99th percentiles. In particular, initial yaw-rate feature11 is identically zero in both datasets. This limitation is retained for the controlled ablation and evaluated separately as a distribution-shift diagnostic; this experiment does not expand the state sampling domain.

## Files and reproduction

- `PROTOCOL.json`: pre-training design and dataset/helper/script hashes.
- `execution_manifest.json`, `execution.jsonl`: process command, CPU affinity, progress and completed status.
- `summary.json`: aggregate results and each seed outcome.
- `SELECTION_FREEZE.json`: query-only selected model and hashes.
- `<cell>/<seed>/training.json`: full epoch history and selected checkpoint.
- `validation_schedule_states.csv`: all per-state replay decisions.
- `network_v1.txt`, `network_compact.txt`, `checkpoint.pt`: model artifacts.
- `export_validation.json`: independent compact-format verification.

From the repository root, using the learning Python environment:

```bash
python motion_primitives_planning/scripts/train_recovery_revision.py \
  --train docs/recovery/learning/data/train_candidates.npz \
  --val docs/recovery/learning/data/val_candidates.npz \
  --states docs/recovery/learning/val_states.csv \
  --output /absolute/new_training_directory --threads 2
python docs/recovery/revision/training/check_exports.py
```

The recorded run was pinned to CPU 8,9 with two PyTorch threads. Training elapsed times are not planner runtime benchmarks. The original training script, original model weights and original evidence files remain unchanged.
