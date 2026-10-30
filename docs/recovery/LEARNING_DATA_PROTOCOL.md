# Learning data and independent evaluation protocol

The learning extension uses `recovery_learning_benchmark`, with geometric distributions inherited from the earlier benchmark and new independent map seeds. Existing V3/V4 maps are not training, validation, or test maps for this extension.

## Frozen splits

| Split | LCG map seed base | Maps per family | States per map | Purpose |
|---|---:|---:|---:|---|
| train | 301000 | 40 | 24 | Network fitting |
| val | 401000 | 8 | 24 | Model/ordering selection |
| test | 501000 | 8 | 18 matched; four closed-loop conditions | Held-out evaluation after model freeze |

Map seed is `base + 1000 * family_index + map_index`, where family order is staggered, wall, culdesac, spatial. All geometry is exported; LCG state advances as `1664525 * state + 1013904223` modulo 2^32. The generator uses explicit sequential draws. Training and validation states use a separate LCG initialized by `map_seed XOR 0x9e3779b9`.

For each training/validation state, independent uniform draws are taken in this exact order: speed [14.8,16.2] m/s, x [0,65] m, y [-12,12] m, heading [-0.3,0.3] rad, flight-path angle [-0.04,0.04] rad, roll [-0.18,0.18] rad, attack-angle perturbation [-0.015,0.015] rad, thrust multiplier [0.9,1.1]. Altitude is 20 m. Attack and thrust start from the nominal zero-climb trim solution at the sampled speed, before these perturbations. Yaw rate is initialized to zero. The previous command is initialized to hold the sampled roll, attack angle and thrust for three seconds, defining the smooth-command cost consistently.

The 203 primary candidates are propagated for three seconds, subjected to the existing numerical, flight-envelope, collision and known-space checks, then scored by the unmodified primary objective. Each feasible candidate receives an oracle label by explicitly checking the fixed bank of 15 six-second terminal continuations until the first feasible witness or exhaustion of the bank. Incremental rejection is enabled. The labels contain binary continuation feasibility, actual Euler integration steps and number of examined backups. No network is used during label generation, and no trajectory teacher provides the labels.

All states, including those with no feasible generated primary, appear in the state summary. Dataset labels concern the 203 generated primitives; the online last-command fallback used when none of these passes is not added as a training example.

Candidate features are computed by the same public `recovery_features` method used online. Candidate CSV records include state, scene, family and original candidate identifiers; objective cost; binary oracle label; validation steps; examined backup count; and features f0--f102. State and geometry CSV files support independent reconstruction. Raw training files belong in `work/learning_data`, while released evidence records their hashes, counts and generation configuration.

## Held-out matched-state evaluation

The test grid consists of x in {0,25,50} m, y in {-5,0,5} m, and heading in {-0.15,0.15} rad at 15.5 m/s trim, giving 18 states per map and 576 states. Each state is evaluated twice with rotated method order. Methods are cost-12, diverse-12, learned-12, hybrid-12, full-cost-203, and cost/diverse/learned/hybrid with 203 candidates and an exact 3600-backup-integration-step cap. Supplying `state_only_model_path` also evaluates state-only learned models under the 12-candidate and 3600-step caps. The same frozen weights and work-power setting are used across matched states and episodes. Timing includes feature extraction and network inference for learned methods.

Primary propagation is common to all terminal methods. The work cap counts backup Euler integration steps, rather than CPU time. Actual consumed work and measured wall-clock time are both reported. Every accepted continuation is checked by the original explicit validator; predicted feasibility does not bypass validation. Repeated timing rows and multiple states from one map are not independent map samples.

## Closed-loop evaluation

All 32 held-out maps are evaluated under nominal, actuator/aerodynamic mismatch, crosswind, and 100 ms command-delay conditions. The methods are primary-only with a three-second horizon, cost-12, diverse-12, learned-12, hybrid-12, and full-cost-203 (768 episodes). The independently integrated RK4 plant and termination rules are unchanged from V3/V4. Planning occurs every 0.2 s for at most 40 s. Model mismatch uses actuator scale 1.25, lift scale 0.97 and drag scale 1.10; crosswind is 1 m/s along world y. The delay condition executes the preceding command for 0.1 s and the new command for the remaining 0.1 s.

An optional test dataset pass exports candidate oracle labels on the same 576 matched states for classifier evaluation; it uses the fixed test grid, not the training-state distribution. Test geometry, labels and evaluation results are not inspected until network architecture, training checkpoint selection and ordering parameters have been frozen on training/validation data. Any further adaptation requires a new held-out split or explicit disclosure as test-set development.

## Reproduction commands

Build `recovery_learning_benchmark` in the existing catkin workspace. Start a dedicated ROS master and source the workspace. Generate each development split with:

```bash
rosrun motion_primitives_planning recovery_learning_benchmark _phase:=dataset _split:=train _output_prefix:=/absolute/path/train
rosrun motion_primitives_planning recovery_learning_benchmark _phase:=dataset _split:=val _output_prefix:=/absolute/path/val
```

Train the geometry and state-only networks using `train_recovery_network.py`; select the shared work exponent using `select_recovery_schedule.py`. The released `MODEL_FREEZE.json` records the selected network hashes and shared exponent before the first test run. The benchmark's `all` phase runs matched states followed by closed-loop episodes. Supply both `model_path` and `state_only_model_path` to reproduce all eleven matched comparators. The state-only model is an input ablation and does not alter the explicit validator.

`run_manifest.json` stores the actual timed command, source/model/executable hashes, CPU affinity, start/end times, and exit status. `analyze_recovery_learning.py` checks this manifest against the model freeze, requires the complete experiment grid, verifies repeated decisions and budget limits, and then generates statistics, figures, and the TeX table. A `--matched-only` report is marked as an intermediate result and does not satisfy the closed-loop audit.
