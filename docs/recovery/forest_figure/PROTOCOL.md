# Supplementary forest-density trajectory comparison

This experiment supplies original simulated trajectories for a three-dimensional comparison inspired by the presentation in Fig. 8 of *You Only Plan Once*. It is supplementary to the previously frozen learning evaluation. It introduces a new procedural map family without retraining or selecting new models.

## Fixed design (declared before execution)

- Seeds: 601000 through 601007, giving eight paired sparse/dense instances.
- Tree-placement region: x in [40,240] m and y in [-80,80] m; area 32,000 m².
- Sparse density: 40 tree centers, 1/800 tree/m².
- Dense density: 80 tree centers, 1/400 tree/m². The first 40 have exactly the sparse trunk and crown geometry; dense adds another 40.
- Every tree draws x, y, trunk radius, trunk height and crown radius in that order. Trunk radius is uniform on [0.8,1.4] m; height on [24,36] m; crown radius on [4,6] m. The trunk is a vertical cylinder centered at half its height. The crown is a sphere at the same x,y with center altitude equal to the trunk height. Both volumes enter the collision model. The LCG is the same explicit integer recurrence used by the learning experiment.
- Twelve low bushes are spheres with radius in [3,5] m and center altitude equal to radius. Bush x, y and radius are drawn in that order from a separate generator initialized by `seed XOR 0x51ed270b`. Bush geometry is identical in the paired maps.
- Placement is unconditioned on route feasibility. There is no path clearing, rejection of difficult maps or result-based seed selection. Overlapping obstacle volumes are treated as a geometric union by the collision checks.
- Flight altitude bounds: 5–45 m. Initial state is the nominal 15.5 m/s trim at (0,0,20) m, heading zero. Target and tolerance remain (250,0,20) m, horizontal 15 m and vertical 3 m.
- Five methods: Cost-12, Diverse-12, Learned-12 (geometry), No-geometry Learned-12, and Full-203. Both networks retain the previously frozen weights and work exponent zero. No execution cache is used.
- Prediction: 3 s primary, 6 s continuation, 0.02 s Euler steps, incremental rejection. Execution: independent nominal RK4 model with 0.005 s steps and 0.2 s control updates, at most 40 s.
- Total: 8 paired seeds × 2 densities × 5 methods = 80 closed-loop episodes. All outcomes are retained.
- The displayed pair is **seed 601000** for both densities, regardless of whether any method succeeds. The figure therefore does not select favorable results from this experiment.

The densities are chosen for the scale and maneuver envelope of this fixed-wing model, and differ from the quadrotor densities in YOPO. This experiment does not implement YOPO, TopoTraj, MPPI or Agile Autonomy as comparison algorithms. All five compared methods are explicit variants of this repository's fixed-wing local planner.

## Geometry and visualization contract

Each tree consists of one trunk cylinder and one crown sphere. Density counts tree centers, not the number of collision volumes. The exported CSV contains the exact cylinders and spheres supplied to the planner and execution collision checker. Rendering uses these physical volumes; any decorative vegetation must not be represented as additional collision evidence. The renderer may show uninflated physical obstacles while identifying the planner's inflated-margin termination. Trajectories are the actual recorded closed-loop states, including unsuccessful prefixes. They must not be extended, smoothed or repaired for presentation.

This is a qualitative density comparison with eight seeds per density, not a new model-selection set. Outcomes and raw paths support the figure; timing columns are retained as diagnostic records without replacing the isolated primary timing experiment.

## Reproduction

Build the catkin workspace, start a ROS master, source the workspace, and create the chosen output directory. Then run:

```bash
roslaunch motion_primitives_planning recovery_forest_benchmark.launch \
  output_prefix:=/absolute/path/forest
```

The supplementary source reuses the unchanged learning benchmark's parameter, state and logging helpers, and supplies its own paired forest generator and nominal episode loop. The before-run manifest records both source files, model hashes and the new executable hash. Existing learning source files and previously released data remain unchanged.
