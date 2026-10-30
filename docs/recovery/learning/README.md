# Learned terminal continuation search

This extension trains and deploys a neural continuation-query scheduler. It retains the existing primary control library and explicitly validates every accepted continuation. The trained MLP has 103 inputs, hidden widths 64 and 32, two outputs and 8,802 parameters.

The first output predicts whether a terminal state has a feasible successor in the fixed bank. The second regresses the log of the incremental validation work. Validation selected a work exponent of zero, so the released online priority uses feasibility probability; the work head remains an auxiliary training target. `learned_diverse` alternates learned ranking and maneuver coverage after the common cheapest-first query.

## Evidence

- `MODEL_FREEZE.json`: parameters and model/source hashes recorded before test access.
- `run_manifest.json`: exact timed commands, executable/source/model hashes and run timestamps.
- `dataset_audit.json`, `train_states.csv`, `val_states.csv`: all sampled states, including states with no feasible generated primary.
- `data/*_candidates.npz`: compressed training/validation candidates, features and explicit-validator labels. Each archive embeds the source CSV hash. Loading with `numpy.load(..., allow_pickle=False)` yields `x`, `y`, `work`, `state`, `scene`, `family`, `cost` and `candidate` arrays.
- `models/geometry/` and `models/state_only/`: checkpoint, portable dense weights, training histories, validation predictions and C++ parity evidence. `state_only` is the obstacle-geometry ablation; goal, flight-band and cost inputs remain present.
- `validation_schedule.json`: every tried work exponent, both learned schedules, two budgets and the frozen shared choice.
- `test_summary.json`: 65 ROS tests and ten Python replay tests.
- `reproduction_audit.json`: an independently rerun subset matches the original oracle labels and features exactly.

The canonical runtime model is `motion_primitives_planning/models/recovery_mlp.txt`; it has the same hash as `models/geometry/network.txt`. The full feature schema is beside the runtime model.

## Reproduction

Build the ROS workspace using the instructions in `../FINAL_RESULTS.md`. Start a dedicated ROS master and source the workspace environment. Create output directories before running commands. All output prefixes below denote local files, and the data generator does not contact external services.

```bash
rosrun motion_primitives_planning recovery_learning_benchmark \
  _phase:=dataset _split:=train _output_prefix:=/absolute/data/train
rosrun motion_primitives_planning recovery_learning_benchmark \
  _phase:=dataset _split:=val _output_prefix:=/absolute/data/val
```

Install `requirements-learning.txt` into an isolated Python environment. The released run used Python 3.8, PyTorch 2.4.1+cpu and NumPy 1.24.4. From the repository root:

```bash
python motion_primitives_planning/scripts/train_recovery_network.py \
  --train /absolute/data/train_candidates.csv \
  --val /absolute/data/val_candidates.csv --output /absolute/models/geometry --threads 3
python motion_primitives_planning/scripts/train_recovery_network.py \
  --train /absolute/data/train_candidates.csv \
  --val /absolute/data/val_candidates.csv --output /absolute/models/state_only --state-only --threads 3
python motion_primitives_planning/scripts/select_recovery_schedule.py \
  --val /absolute/data/val_candidates.csv --states /absolute/data/val_states.csv \
  --geometry /absolute/models/geometry --state-only /absolute/models/state_only \
  --output /absolute/results/validation_schedule.json
```

The network remains fixed during test evaluation. The original test command is recorded in `run_manifest.json`; its important arguments are:

```bash
rosrun motion_primitives_planning recovery_learning_benchmark \
  _phase:=all _split:=test _output_prefix:=/absolute/results/test \
  _model_path:=/absolute/models/geometry/network.txt \
  _state_only_model_path:=/absolute/models/state_only/network.txt \
  _work_power:=0
```

Online simulation loads the bundled weights:

```bash
roslaunch motion_primitives_planning terminal_recovery_learned.launch \
  candidate_order:=learned_diverse learned_work_power:=0.0
```

Run `roslaunch motion_primitives_planning terminal_recovery_no_geometry.launch` for the frozen no-geometry model; use `candidate_order:=learned_diverse` to interleave coverage. The no-geometry closed-loop comparison is a post-evaluation ablation archived in `geometry_ablation/`, with unchanged weights and parameters. The general launch retains existing defaults; the learned launch explicitly enables the model. Missing or malformed weights produce a warning and diversity fallback in the live planner. The benchmark refuses such models before generating results.

Training and test datasets are separate procedural map instances from the same four geometry families. Closed-loop evaluation additionally varies execution dynamics, wind and latency. The protocol does not claim transfer to unseen geometry families or physical flight.
