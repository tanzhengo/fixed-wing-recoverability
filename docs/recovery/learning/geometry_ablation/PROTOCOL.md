# Post-evaluation geometry-input ablation

This additional experiment was specified after inspecting the primary held-out results. The state-only and geometry networks discovered the same number of continuations at K=12, motivating the question of whether obstacle-clearance inputs affect closed-loop mission outcomes. The experiment does not select or modify network weights, architecture, ordering parameters, or test maps.

The unchanged frozen benchmark executable is rerun with `phase=episodes`, `split=test`, and the already frozen state-only model supplied as the main model. Work exponent remains zero. All 768 raw episode records are retained. The 128 learned and 128 hybrid tasks use changed network inputs/weights; the other 512 primary-only, cost, diverse, and full-search tasks repeat the original implementations. These baseline replay episodes must reproduce the original outcomes, state paths, selected costs/ranks, and validation counts exactly. Only wall-clock planning time may differ.

The 256 changed method-map-condition combinations are compared against the corresponding geometry-network cases from the primary experiment. Both models were trained and selected on the original training/validation data before any held-out evaluation. Paired success differences use 10,000 bootstrap resamples of the same 32 map clusters, retaining all four conditions within each map. No new independent maps are introduced, and replayed baselines are not pooled as additional observations.

The additional run is used to interpret mission completion and failure outcomes. Its timing columns are preserved as raw records but are not used for a new latency comparison. Main-paper timing results remain those of the primary isolated evaluation.
