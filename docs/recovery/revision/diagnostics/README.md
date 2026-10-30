# Frozen-controller failure diagnostics

These analyses address the reviewer's numerical-error, boundary-selection and training-support hypotheses using the original 32 diagnostic maps. They do not change the frozen controller, networks, simulator or original result files.

## Reproduction and sampling

`recovery_review_diagnostics` repeats Geometry, No-geometry, Diverse and Full under the four original execution conditions: 512 episodes. All 512 outcomes, update counts and final horizontal goal distances match the archived runs. No runtime comparisons are made while these diagnostic processes run.

For every unsuccessful episode, the final attempted planning state and the last primary-feasible planning state are retained, without duplicates within an episode. The resulting 257 states are supplied to all four methods with identical command histories. A fifth, post-hoc inference probe sets only input column 11 of the Geometry network's first layer to zero; the true yaw rate and primitive generator are unchanged. The complete 15-control continuation bank is then inspected for each selected primary.

`recovery_review_branchpoints` additionally runs the Geometry and No-geometry controllers along their identical state/control prefix in each of the 128 map-condition tasks. It stops at the first differing executed control and measures both choices at that shared state. This was added after the late failure-state analysis showed mostly identical choices.

## Measured results

The replay preserves the original completed-task counts: Geometry 99/128, No-geometry 117/128, Diverse 70/128 and Full 87/128. The corresponding margin-incursion counts are 9, 2, 6 and 4. The archived outcome label `collision` denotes entry into the inflated planning boundary, not necessarily physical obstacle contact.

### Prediction of the actual incursion-producing prefix

Each entry below integrates the same initial state and selected control for 0.2 s. RK4 nominal removes wind, parameter mismatch and command delay while retaining the recorded initial state. The final column counts cases that cross only under the recorded execution condition; all three are crosswind cases.

| Frozen method | Actual margin incursions | Euler 0.02 s detects | Euler 0.005 s detects | RK4 nominal detects | Actual-condition-only |
| --- | ---: | ---: | ---: | ---: | ---: |
| Geometry | 9 | 0 | 8 | 8 | 1 |
| No-geometry | 2 | 0 | 2 | 2 | 0 |
| Diverse | 6 | 0 | 3 | 4 | 2 |
| Full | 4 | 0 | 2 | 4 | 0 |
| Total | 21 | 0 | 15 | 18 | 3 |

All 21 selected primaries admit **3–10 feasible continuations out of 15**. The Geometry cases admit 5–10. Thus the observed failures are not instances of having only one feasible successor. Many selected backup segments have substantial clearance while the primary execution prefix is close to an inflated boundary. Numerical consistency of the executed prefix and the size of the continuation bank address different effects.

The clearance function uses the frozen planner's signed distance to inflated obstacles and caps positive distances at 30 m. All reported primary/backup clearances use this same cap. The counterfactual results identify local prediction discrepancies; they do not by themselves measure the closed-loop performance of a modified planner.

### First executed-control divergence

The two learned controllers diverge in 124 of 128 shared-prefix tasks; in all 124 cases both choices have verified continuations. Geometry chooses a lower task-cost primary in 99 cases, and fewer feasible continuation controls in 88 cases (mean difference −1.185 out of 15). However, its primary clearance is lower in 42 cases, higher in 48, and tied in 34 (mean difference +1.118 m, with the 30 m clearance cap). This supports a cost/continuation-option tradeoff, but not a universal reduction in primary clearance.

Among the 97 late failure-state pairs where both methods find a witness, Geometry has lower primary clearance in three, higher clearance in two, and equal clearance in 92. Such late-state agreement does not explain earlier trajectory divergence; the shared-prefix probe measures the first selection difference explicitly.

### Feature support

Input 11, initial yaw rate, is identically zero in both archived training and validation data. It falls outside that training support (tolerance 1e-6) in 92.67% of the Geometry controller's 9,668 selected-candidate observations, and 93.04% of the No-geometry controller's 10,195 observations. Initial roll also exceeds its training range frequently (46.30% and 46.39%, respectively).

Masking only the unsupported yaw-rate input at the 139 primary-feasible late diagnostic states changes **zero** selected ranks, costs or verified statuses. The feature-support mismatch is therefore measurable, but this probe does not establish it as the cause of the Geometry controller's lower completion rate. The range comparison uses closed-loop selected candidates against all feasible training candidates and includes selection effects.

## Paper-ready interpretation

A controlled replay reproduces all 512 archived method-condition episodes. At the 21 margin-incursion predecessor states, the selected primaries admit between three and ten feasible continuations. Reintegrating the same 0.2 s command detects 15 boundary crossings with refined Euler integration and 18 with the nominal RK4 model; the remaining three require the recorded crosswind. This distinguishes near-boundary execution error from scarcity of continuation witnesses and motivates a separate numerical check of the executed prefix.

At the first divergent control along 124 otherwise identical Geometry/No-geometry prefixes, the geometry-conditioned network selects a lower task-cost primary in 99 cases and fewer feasible continuations in 88. Primary clearance does not decrease consistently: it is lower in 42 cases and higher in 48. The results expose a tradeoff between task cost and the number of available continuations, rather than establishing a general tendency of the geometry representation to select lower-clearance primaries.

## Files and commands

- `PROTOCOL.json`: original replay plan and clearly identified post-hoc probes.
- `diagnostic_episodes.csv.gz`: exact replay outcome comparison.
- `diagnostic_paired.csv.gz`: all 1,285 paired-state method choices.
- `diagnostic_features.csv.gz`: 35,861 selected-candidate feature observations.
- `diagnostic_branchpoints.csv.gz`: 128 shared-prefix tasks.
- `analysis.json`: complete scalar, pairwise and counterfactual summaries.
- `run_manifest.json`, `branch_run_manifest.json`: exact commands and before/after source, executable and model hashes.
- `models/geometry_yaw_mask.txt`, `models/provenance.json`: the paired-state-only column-11 probe.
- `MANIFEST.json`: final code and compressed evidence hashes.

Build the two review targets in the catkin workspace, then run from the repository root:

```bash
python3 motion_primitives_planning/scripts/run_recovery_review_diagnostics.py
python3 motion_primitives_planning/scripts/run_recovery_review_branchpoints.py
python3 motion_primitives_planning/scripts/analyze_recovery_review_diagnostics.py \
  --repo . --prefix docs/recovery/revision/diagnostics/diagnostic
```

The runners start and stop their dedicated ROS master on port 11373 and pin diagnostic execution to CPU 4. The analyzer accepts either raw CSV or its `.csv.gz` archive and rejects incomplete or mismatching episode replay. Original test maps are used for mechanism diagnosis; new performance claims require the separate revision test protocol.
