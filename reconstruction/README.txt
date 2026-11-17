Fixed-wing terminal continuation: reconstructed reference implementation
======================================================================

Status: new manuscript-based Python/NumPy implementation, not recovered source.
Ubuntu setup and staged validation: UBUNTU_GUIDE.md
Task text for continuing with Codex on Ubuntu: CODEX_HANDOFF.md
Development context for agents: AGENTS.md
Specification: the user-supplied LaTeX main11.tex (unknown environment,
58 inputs, H=B=3 seconds, K=12, up to 406 primaries, 15 successors).
The older GitHub archive is a source of inherited dynamics and library-order
conventions, NOT the current manuscript's implementation or experiment data.

Original evidence recovered on Ubuntu, 2026-11-17
------------------------------------------------
The original 58-input checkpoints, C++/ROS sources, training records and raw
experiment archives are available again. Their file hashes and the manuscript's
main-result aggregates have been checked independently. See
../evidence_recovered/USAGE.txt for the separate original-source snapshots,
compact models, portable analysis/build tools and large-archive checksums.
This recovery does not turn the new NumPy implementation into the original
runtime; their arithmetic, software checks and experiments remain identified.

Ubuntu validation of sections 3--7 in UBUNTU_GUIDE.md is complete: 76 software
checks pass; labeling, smoke training, learned replay and the short synchronous
numerical episode ran successfully. The episode outcome was time_limit, not
goal attainment. See validation/ubuntu_20261117_summary.json. The original
Windows software report remains unchanged.

From the repository root, first run: cd reconstruction
Then use Python 3.10+:
  python -m pip install -r requirements.txt
  python scripts/run_checks.py --output checks.json
  python scripts/run_diagnostics.py --help
  python scripts/replay_query.py --help
  python scripts/label_dataset.py --help
  python scripts/train_model.py --help
  python scripts/run_episode.py --help

Keep newly generated records separate from the checked-in software test report:
  python scripts/run_checks.py --output runs/software_checks.json
  python scripts/run_diagnostics.py --output runs/diagnostics
  python scripts/run_episode.py configs/open_scene_diagnostic.json --output-dir runs/open_episode
The diagnostic and open-scene commands generate new numerical fixtures only.
Learned diagnostics additionally require an explicitly trained --model checkpoint.

Core API
--------
types.py: explicit state, control, limits and result records (SI, radians).
observed_map.py: three-state sparse voxel map and swept segment validation.
sensor.py: nearest-hit analytic sensor, no-return semantics, pose interpolation.
dynamics.py: inherited point-mass equations, actuators, Euler/RK4, control banks.
features.py: 28 state/task + 30 observed-space fractions with an indexed schema.
learning.py: 58-64-32-2 BCE/auxiliary Adam training and compact 28/58-input export.
planner.py: Cost, Diverse, pure learned, combined, observed coverage and Full;
            deferred scoring, stable selection, fixed-schedule cost pruning.
execution.py: timestamp/history alignment, freshness checks, cached sequence
              trimming, revalidation and at most one validation-age retry.
records.py: immutable-input hashes, full candidate paths, query order and work.
simulation.py: explicitly synchronous numerical diagnostic episodes.

Integration boundary
--------------------
The planner receives only an ObservedMap snapshot, state, target and optional
previous command. Physical scene geometry is confined to the sensor simulator
and independent trajectory evaluator. A search result is only a proposal.
Dispatch must revalidate it; a missing witness never authorizes primary-only
execution. Rejection is a supervisor event, not a physically possible stop.

Map mutation and planning must be separated using a stable snapshot, e.g.
ObservedMap.from_snapshot(live_map.snapshot()). Time values passed to execution
must share a clock. Recorded controls must cover the entire alignment interval.
The Python reference is not claimed to satisfy the manuscript's CPU timing or
20 ms dispatch deadline. The episode runner explicitly models zero computation
latency and must not be presented as an asynchronous real-time validation.

Learning and provenance
-----------------------
No pretrained deployment model is bundled. Learned modes require an explicitly
trained checkpoint. The 103-input archived models are deliberately rejected.
If diagnostic_model is included with a validation bundle, it is a smoke-training
artifact on constructed maps, NOT Selected, NOT the recovered original model, and NOT a model
validated for navigation or deployment.

Dataset frames are JSONL with keys state, goal, map, config, previous (optional),
scene_id and query_id. map is an embedded ObservedMap.snapshot(). The label
export preserves no-primary queries in its manifest. Candidate negatives mean
the complete 15-control bank was rejected, never that no physical motion exists.
Training and validation scene IDs must be disjoint; IDs alone do not prove that
geometries or original scene seeds differ, so retain and inspect scene hashes.
Use training/validation only for checkpoint and schedule selection; freeze a
new model/config/source manifest before opening the final test split.

Unspecified or intentionally reconstructed choices
------------------------------------------------
Float64 arithmetic, NumPy He initialization and Adam are new choices; bitwise
parity with the lost C++/PyTorch implementation is not asserted. Feature probe
order is beta-major then eta, with [free,unknown] pairs. Stable primary ordering
and relative height offsets follow the available old runtime header. Collision
checks conservatively use the larger endpoint reserve over each linear segment.
The alpha-command denominator 0.03+T/L is retained exactly as printed; it is an
inherited construction and not an exact inverse of the prediction equations.
The current reconstruction has no integration-work cap or CPU deadline inside
query search. Adding either changes the pruning equivalence conditions.

Further experiments
-------------------
Use the recovered original records and frozen selection before repeating formal
experiments. New runs must use fresh output directories and retain configuration,
model and source hashes. The reference implementation is suitable for software
checks and controlled mechanism diagnostics; it has not replaced the original
C++/ROS timing or asynchronous deployment evaluation. Paired uncertainty and
recorded-query diagnostics are supplied with the recovered evidence. Sensor
noise, mapping capacity, timestamp delay and independent aircraft dynamics
require distinct evaluations with their own declared protocols.
