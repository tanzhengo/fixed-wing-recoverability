# Forest-density trajectory comparison

The supplementary experiment supplies actual closed-loop trajectories for a sparse/dense three-dimensional forest figure. It uses both previously trained networks and unchanged planner dynamics; no retraining or parameter selection is performed.

- Eight paired seeds, 16 map instances and five methods: 80 nominal closed-loop episodes.
- Sparse 40 and dense 80 tree centers over 32,000 m². Each center has an actual cylinder trunk and spherical crown; 12 shared bush spheres complete each map.
- First seed 601000 was fixed for both displayed panels before execution.
- All methods complete 8/8 sparse tasks. In the dense maps, the geometry Learned model completes 7/8 and incurs one inflated-obstacle-margin violation; all other methods complete 8/8.
- The unsuccessful geometry Learned path belongs to the displayed dense panel and is retained as recorded.

These results provide a qualitative new-family comparison. They do not establish a systematic ranking among algorithms under increasing density.

## Files

- `PROTOCOL.md`: the design declared before execution.
- `run_manifest.json`: exact command, before-run hashes, completion and unchanged-source checks.
- `forest_scenarios.csv`: exact physical collision geometry.
- `forest_map_metadata.csv`: paired seeds, tree densities and display selection.
- `forest_paths.csv`, `forest_plans.csv`, `forest_episodes.csv`: complete recorded execution and all outcomes.
- `analysis.json`: per-density counts and recorded-path summaries.
- `analyze.py`: rechecks source/run integrity and regenerates the summary.
- `audit_forest.py`: independently reconstructs the procedural geometry, verifies paired maps and checks all recorded episodes; `--figure` also checks the figure's input hashes, displayed endpoints, projection basis and sampled surfaces.
- `independent_audit.json`: the reproducible independent checks and input hashes.
- `figure_manifest.json`: the ten displayed cases, recorded vertex counts, endpoints and renderer/input hashes.

From the repository root, reproduce the summary, render the figure, then audit both data and figure:

```bash
python3 docs/recovery/forest_figure/analyze.py
python3 paper/create_forest_figure.py
python3 docs/recovery/forest_figure/audit_forest.py --figure
```

The renderer and optional figure audit require NumPy and Matplotlib. The data-only audit uses the Python standard library and can be run without `--figure`. All paths are resolved relative to the scripts, so the audit also works from another working directory. The ROS launch command is given in the protocol.

The rendered surface samples lie on the recorded, uninflated analytic obstacle geometry; they are not simulated sensor measurements. The same orthographic projection is used in both panels. Trajectories use the stored execution vertices directly, and terminal symbols retain goal attainment or other termination without extending a path to the target. Cost and Full coincide in the displayed pair.
