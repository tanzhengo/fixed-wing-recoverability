# City-block trajectory comparison (archived alternative)

The active manuscript has returned to the forest voxel figure. This city-block experiment is retained as an evaluated alternative, with its geometry, trajectories and all outcomes preserved.

- Eight paired seeds, 16 city maps and five methods: **80 nominal closed-loop episodes**.
- Base layout: 20 buildings composed of 40 podium/tower boxes. Dense layout: the same base buildings plus eight infill buildings, giving 28 buildings and 48 boxes.
- The generated urban extension adds analytic axis-aligned building boxes to collision checks and network clearance features. Each face uses a 1 m expansion. The original planner header, training code and both network weight files remain unchanged.
- Display seed **701000** was fixed for both panels before execution. All recorded outcomes, including failed prefixes, are retained.

| Method | Base goals / 8 | Dense goals / 8 |
| --- | ---: | ---: |
| Cost-12 | 7 | 6 |
| Diverse-12 | 6 | 1 |
| Learned-12, geometry | 4 | 1 |
| Learned-12, no geometry | 8 | 1 |
| Full-203 | 7 | 4 |

The displayed base map has five goal attainments. In the displayed dense map, Cost reaches the goal, while Diverse, Learned, No geometry and Full terminate at an inflated obstacle margin. Other seeds also contain no-feasible-primary terminations. `margin_incursion` records entry into the expanded collision boundary, and does not by itself denote physical building contact.

The denser building layout exposes a substantial loss of completion for the frozen learned schedules. The maps and models are retained as evaluated; no retraining or map-based parameter selection is performed. These supplementary outcomes are reported separately from the original learning test split. Timing columns are diagnostic records rather than isolated performance measurements.

## Reproduction and evidence

- `PROTOCOL.md`: fixed design, generator order, box margins and display selection.
- `run_manifest.json`: before-run source/model/executable hashes, exact command and completion checks.
- `urban_scenarios.csv`: the exact box components exported from the simulator; sizes are full side lengths.
- `urban_map_metadata.csv`: paired scene identities, seed, building count and box count.
- `urban_episodes.csv`, `urban_paths.csv`, `urban_plans.csv`: all outcomes, recorded states and decisions.
- `analyze.py`, `analysis.json`: reproducible counts and path summaries.
- `audit_urban.py`, `independent_audit.json`: independent scene reconstruction, episode checks and optional figure verification.
- `geometry_tests.json`: tests for the building extension, provided with the geometry implementation.

Check the generated extension before building:

```bash
python3 motion_primitives_planning/scripts/generate_urban_planner.py --check
```

Build the catkin workspace, source it, then run the launch command documented in `PROTOCOL.md`. From the repository root, summarize the exported evidence with:

```bash
python3 docs/recovery/urban_figure/analyze.py
python3 paper/create_urban_figure.py
python3 docs/recovery/urban_figure/audit_urban.py --figure
```

The voxel figure is a visualization of these analytic boxes. It uses the actual recorded trajectories and retains the initial state, target, outcomes and all path vertices.
