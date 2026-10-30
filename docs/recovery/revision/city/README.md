# Controlled known-map urban trials

Fifteen trials compare Cost, Diverse, original Geometry, original State, and
validation-selected State+Aux in the same Gazebo city with eight waypoints.
Three settings are coarse altitude levels/48.825 m horizontal gate, relative
altitude levels/48.825 m gate, and relative levels/15 m gate. Vertical tolerance
is 3 m; H=3 s, B=6 s and K=12 are common. Prefix revalidation is disabled.

All five coarse trials fail before the first waypoint. Every relative/48.825 m
trial completes the cycle. At 15 m only original Geometry completes; Cost,
Diverse and Selected lose the primary set after three waypoints, while original
State enters inflated geometry. Each configuration has one recorded run in one
layout. See `summary.json`/`summary.csv` for timestamps and clearances.

The planner uses supplied building boxes. The world contains the actual lidar
sensor, but the batch matrix does not run the voxel accumulation node. The
archived acquisition flight and Gazebo/RViz screenshots in the paper are
separate records with the original Geometry model. The runtime adapter uses
padded V1 networks for compatibility; it is not the compact benchmark path.

Each case directory preserves the exact task YAML, run command/model hash,
result summary and compressed observations. The observer stops at the first
cycle completion, primary failure, inflated-box incursion or 180 s simulation
limit. The first target publication is not counted as a completed waypoint;
completion is counted from subsequent target switches and verified gate errors.

From a sourced ROS/catkin environment, run:

```bash
python3 motion_primitives_planning/scripts/run_review_city.py \
  --output docs/recovery/revision/city \
  --scratch /absolute/path/to/work/city-rerun
python3 motion_primitives_planning/scripts/analyze_review_city.py
```

Use a fresh output path to rerun; existing result directories are skipped.
The runner uses ROS port11371, Gazebo port11375 and CPUs6,7. No GUI is required.
The imported city meshes must be available in Gazebo's model path. Clearances
refer to the aircraft reference point, not a swept airframe volume. The
`collision`/incursion outcome denotes inflated boxes, distinct from physical
box penetration.
