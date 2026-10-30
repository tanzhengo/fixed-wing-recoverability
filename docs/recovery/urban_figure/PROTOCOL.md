# Supplementary city-block trajectory experiment

The experiment replaces the paper's forest visualization with original urban-building geometry and newly executed paths. It evaluates the already frozen scheduling networks without retraining. This design is fixed before the first city episode.

## Geometry and paired layout

- Eight paired seeds: 701000 through 701007. The displayed pair is seed **701000**, fixed before execution regardless of outcome.
- Nominal view rectangle: x in [0,250] m and y in [-90,90] m, 45,000 m². This rectangle is a display reference, not an extra flight boundary.
- The base layout (`urban_blocks`) has 20 buildings in four columns and five rows. Column centers are 60,105,150,195 m; row centers are -68,-34,0,34,68 m. Each center receives independent x/y jitter uniform in [-3,3] m.
- Each building consists of two axis-aligned boxes: a podium and an offset tower. Podium footprint side lengths are independently uniform in [16,24] m and its height is uniform in [8,14] m. Tower footprint side lengths are 0.6 times those of its podium; its height is uniform in [24,58] m. Both components extend upward from z=0. Tower center offsets are `sign * 0.2 * side`, with `sign=+1` when (column+row) is odd and -1 otherwise, giving stepped roofs and asymmetric tower placement.
- Generation loops over columns first, then rows. The six LCG draws for each base building are x jitter, y jitter, x side, y side, podium height, tower height, in that order. The recurrence is `state = 1664525 * state + 1013904223` modulo 2³²; uniform fractions use `state / 4294967296`.
- The dense layout (`urban_dense`) retains all base boxes and adds eight infill buildings. Their nominal column centers are 82.5,127.5,172.5,217.5 m, with two rows at -17 and +17 m. The additional generator uses `seed XOR 0x51ed270b`, looping over columns then rows. Each building draws x/y jitter in [-2,2] m, x/y side lengths in [10,16] m, then height in [18,42] m. It consists of one grounded box.
- The paired maps therefore contain 20/28 buildings and 40/48 box components. Overlapping components form a union. No routes are cleared, difficult maps rejected or display seeds selected after seeing results.

## Planner and execution

- Five methods: Cost-12, Diverse-12, Learned-12 (geometry), No-geometry Learned-12 and Full-203. Both neural networks retain their published weights and feasibility-only priority exponent zero.
- Initial trim: position (0,0,20) m, heading zero, airspeed 15.5 m/s. Target (250,0,20) m, horizontal tolerance 15 m, altitude tolerance 3 m. Flight altitude bounds are 5–45 m; no additional horizontal map boundary is imposed.
- Primary and continuation horizons are 3 and 6 s, using the inherited 203-primary and 15-continuation libraries and 0.02 s Euler prediction. The independent nominal RK4 plant uses 0.005 s integration and 0.2 s held commands, for at most 40 s.
- Building collision tests use exact axis-aligned boxes expanded by **1.0 m on every face**. The same inflated boxes supply the 75 clearance-grid input channels. The execution monitor uses the same geometric acceptance boundary; recorded physical clearance is relative to uninflated boxes. `margin_incursion` means entry into the expanded boundary, not necessarily physical contact.
- A generated, isolated `urban_motion_primitives.h` adds only box geometry to the frozen planner source. The generator checks the original source SHA, and the original header and learning benchmark remain unchanged. Geometry tests cover face/edge/corner distance, thin walls, union queries, rollout rejection and network feature visibility.
- Total: 8 paired seeds × 2 layouts × 5 methods = 80 nominal episodes, with all outcomes retained. No new parameters or weights are selected from these results.
- Timing fields are diagnostic; these runs do not replace the original isolated latency study.

## Recorded evidence and display

The CSV records each box's full side lengths, center, building ID and component. These are exactly the boxes installed into the planner. Paths contain stored execution states and retain unsuccessful prefixes. A voxel renderer may discretize these boxes for visualization, while collision checking remains analytic. Flight trajectories are not shifted, repaired, smoothed or extended for the figure.

## Reproduction

From the repository, first check the generated extension:

```bash
python3 motion_primitives_planning/scripts/generate_urban_planner.py --check
```

Build the catkin workspace, source it, then run:

```bash
roslaunch motion_primitives_planning recovery_urban_benchmark.launch \
  output_prefix:=/absolute/path/urban
```

The run manifest records the source/model/executable hashes before execution and verifies them afterward. Analysis and independent audits use the exported files without changing recorded outcomes.
