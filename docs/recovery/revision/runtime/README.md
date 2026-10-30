# Review runtime and held-out evaluation

This experiment isolates implementation overhead from continuation validation.
The original planner, model files, and published evidence remain unchanged.

`review_motion_primitives.h` contains the same dynamics, candidate libraries,
constraints, costs, and continuation validator as the frozen header. Its two
optional changes are:

1. A V2 network evaluates only the feasibility output. The legacy state-only
   model can be converted exactly by deleting its zero input columns, reducing
   the input to 28 scalars and omitting the 75 obstacle probes.
2. Learned ordering is deferred until the common cheapest candidate fails its
   continuation check. After failure, the original remaining schedule and
   remaining budgets are used. Early acceptance of that cheapest candidate
   requires no feature extraction or network inference.

`old_state_equivalence.json` checks all 576 historical matched states under
both candidate and integration-step budgets. These correctness runs overlap
other work and are not timing evidence. `compact_model_parity.json` compares
compact C++ outputs with padded C++ and Python on real validation examples.

## Selection and held-out protocol

`PROTOCOL.json` records the new 801000 seed range before its evaluation.
`VALIDATION_SELECTION_PROTOCOL.json` documents the amendment to select the
execution configuration using validation closed-loop completion. The amendment
was recorded after some validation-query training runs completed but before
any added validation execution or new test evaluation. The original
query-recall selection remains a separate comparator.

The execution selection compares four fixed-seed configurations on 32 existing
validation maps under nominal dynamics, model mismatch, 1 m/s crosswind, and
100 ms command delay (512 episodes). Selection first maximizes goal completion,
then minimizes margin incursions, then compares across-seed validation-query
recall and BCE. The deployment seed is fixed at 20261030.

Only after writing the selection freeze are the 32 new test maps opened.
Held-out execution compares Cost, Diverse, Full-203, both original networks,
and all four revised configurations on the same 128 map-condition tasks.
The same 203 primaries, 15 continuations, 15 m horizontal goal tolerance and
3 m altitude tolerance apply to every method.

## Timing

Each new test map has 18 matched states. Three repeats rotate method order on
CPU 2, after training, simulations, diagnostics and compilation finish.
Uninstrumented total planning times (three repeats) and the instrumented
breakdown (one traversal of all matched states) are separate runs. The latter reports primary integration; remaining primary
cost/check work; scalar features; geometric probes; neural inference;
schedule work; continuation integration/checks; and residual work.

Profiling records do not replace the uninstrumented timing comparison. All
method decisions are checked for equality between the two runs. Counts of
integration steps are reported separately from elapsed planning time.

## Files and reproduction

- `controls/legacy_state_compact.txt`: exact compact conversion of the original
  state-only network, with a source hash in `conversion.json`.
- `*_run_manifest.json`: full command, CPU, timestamps, source/executable/model
  hashes and post-run unchanged checks.
- `*_episodes.csv`, `*_plans.csv`, `*_paths.csv`: execution outcomes and traces.
- `*_matched.csv`: matched-state decisions, runtime and optional decomposition.
- `analysis.json`: summary derived by `analyze_review_runtime.py`.

Large CSVs are stored as `.csv.gz`. The analysis, comparison and selection
scripts automatically read either representation; the launcher refuses to
overwrite either one. `CSV_ARCHIVE_MANIFEST.json` preserves the original CSV
SHA256 and the archive SHA256. All 12 archives were decompressed and checked
before removing their raw copies. Recomputing both analysis reports and the
prefix comparison from the archives produced byte-identical JSON files.

Build `recovery_review_benchmark` and `recovery_review_parity` in the ROS
workspace. Source the workspace and run a dedicated ROS master on port 11372.
Use `motion_primitives_planning/scripts/run_review_runtime.py --help` for the
traceable launcher; the `review` split requires an explicit saved freeze file.
No benchmark node commands the simulated aircraft.

## Common numerical-prefix filter

Historical failure diagnostics motivated the separately declared intervention
in `PREFIX_PROTOCOL.json`, after deployment model selection and before opening
the new test maps. Prefix-off and prefix-on each run the same 1,152 held-out
episodes. The enabled version rejects an otherwise feasible primary if a
nominal RK4 rollout of its next 0.2 s (0.005 s steps) violates the shared
sampled envelope or inflated collision constraints. It receives no true wind,
perturbed coefficients or command-delay information.

This common filter is evaluated for every scheduling baseline. It adds a
short numerical check without modifying the long-horizon Euler labels, task
cost, continuation bank or learned weights. `prefix_unit.json` reports seven
checks of the callback and its collision/envelope behavior. The prefix timer
is nested within the primary phase and is reported as a separate component;
`primary_other_ms` excludes this component to avoid double counting.

`EXECUTION_MODEL_FREEZE.json` contains the fixed deployment choice and all
validation evidence hashes. `prefix_comparison.json` separates the common
filter's effect from the query-scheduling comparisons.

## Completed held-out execution results

The frozen execution choice is **State + auxiliary work supervision**, exported
as a 28–64–32–1 feasibility network. Validation completions were 108/128 for
State+Aux, 102 for State+BCE, 94 for Geometry+Aux and 89 for Geometry+BCE.
This differs from the geometry model chosen by validation query recall.

| Schedule / model | Prefix off: goals / margin | Prefix on: goals / margin |
| --- | ---: | ---: |
| Cost | 43 / 2 | 44 / 0 |
| Diverse | 88 / 1 | 89 / 0 |
| Full-203 | 94 / 4 | 96 / 0 |
| Original geometry | 90 / 4 | 92 / 1 |
| Original state-only | 100 / 10 | 108 / 0 |
| Geometry + BCE | 90 / 13 | 100 / 1 |
| Geometry + auxiliary | 95 / 5 | 100 / 0 |
| State + BCE | 103 / 14 | 113 / 3 |
| **State + auxiliary (selected)** | **107 / 12** | **118 / 0** |

Every entry covers the same 128 tasks on the new 32 maps. “Margin” is an
incursion into inflated obstacle geometry. The selected configuration gains
11 completions and loses none when adding the common prefix check; one other
former margin-incursion case terminates with no feasible primary. The paired
completion difference is +8.59 percentage points, with a 32-map-block bootstrap
95% interval [4.69, 12.50]. With the same prefix filter, its difference from
Diverse is +22.66 points [14.84, 30.47]. The ten remaining selected-method
failures are no-feasible-primary outcomes, so these results do not establish
recursive feasibility.

## Completed isolated runtime results

The accepted four runs used an i5-12400F, `-O2`, and CPU affinity 2 after
training, simulation, diagnostic execution, compilation and figure rendering
finished. Clock frequency was not locked. `TIMING_ENVIRONMENT.json` records
these conditions. A first interrupted run overlapped the end of figure
rendering; its partial data remain in `timing_aborted_before_isolation/` and
are excluded from every timing summary.

Each table value aggregates 576 matched states after taking the median of
three uninstrumented repeats per state. All timing columns are milliseconds.

| Method, K=12 unless stated | Recall (%) | Prefix off median / P95 | Prefix on median / P95 |
| --- | ---: | ---: | ---: |
| Cost | 66.19 | 2.540 / 6.053 | 3.809 / 7.733 |
| Diverse | 95.98 | 2.699 / 4.292 | 3.855 / 5.825 |
| Full-203 | 100.00 | 2.545 / 11.641 | 3.812 / 13.212 |
| Original geometry, eager | 98.11 | 3.986 / 6.364 | 5.275 / 7.843 |
| Original state-only, padded/eager | 98.11 | 3.716 / 5.933 | 5.005 / 7.487 |
| Original state-only, compact/deferred | 98.11 | 3.019 / 4.804 | 3.830 / 6.383 |
| Selected state+auxiliary, compact/deferred | 98.11 | 3.023 / 4.799 | 3.840 / 6.362 |

The compact/deferred implementation preserves every selected decision and
budget count of the original state-only model. With the prefix check enabled,
it lowers that model's median runtime by 23.48% and P95 by 14.75%. The selected
retrained model has a median comparable to Diverse, but its mean (4.120 vs
3.853 ms) and P95 remain higher. The network does not remove the common primary
propagation cost.

The separate instrumented run decomposes selected-model mean runtime as
follows (prefix enabled): primary integration 2.378 ms, remaining primary
cost/check work 0.234 ms, prefix validation 0.910 ms, scalar feature extraction
0.00264 ms, geometric probes 0, neural forward pass 0.253 ms, schedule work
0.0190 ms, continuation integration/checks 0.310 ms, and residual work
0.00963 ms. The uninstrumented comparison supplies the speed measurements;
the instrumented run supplies the component attribution. All instrumented
and uninstrumented decisions agree.

Deferral additionally avoids inference in 28.30% of all matched states, or
35.21% of those having a feasible primary. Including no-primary inputs that
already skipped inference in the eager path, 47.92% require no neural pass. Under the 3,600-step continuation
budget, the selected model finds 389/423 available witnesses (91.96%), versus
312/423 (73.76%) for Diverse: +18.20 percentage points, with a 32-map-block
bootstrap interval [10.83, 26.41]. Under K=12, the corresponding difference is
415/423 versus 406/423, or +2.13 points [0.24, 4.55].

The nominal prefix check rejects no candidate at these matched initial states;
its closed-loop benefit emerges at states reached during execution. Thus the
off/on matched recall and choices are identical, while prefix validation adds
runtime to both the learned and nonlearned schedules.
