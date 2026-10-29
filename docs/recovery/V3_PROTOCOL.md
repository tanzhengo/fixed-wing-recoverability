# V3 evaluation protocol

Protocol frozen before running V3 (2026-10-30).

The study tests whether maneuver coverage improves terminal-continuation
discovery under finite validation budgets. No learned model is used.

## Methods and common settings

All V3 methods apply identical collision, airspeed, attitude, altitude and
observation-range checks. The primary-only ablations omit only continuation
validation. The main comparison uses 203 primary primitives (29 headings and
7 heights), 15 continuation controls, 3 s primary and 6 s continuation horizons,
0.02 s prediction steps, and 0.2 s control updates. Cost and diversity schedules
use K=12. Full-library search uses K=203. A 6 s primary-only ablation tests
whether a longer single control explains the effect. Reuse is disabled.

## Frozen maps and execution conditions

Twenty-four maps: six seeded instances each of staggered pillars, lateral
walls, offset cul-de-sacs, and spatial obstacles. Geometry is generated with
a fixed integer pseudo-random sequence and exported before simulation.
No map or parameter is removed after observing an outcome. Initial speed is
15.5 m/s at nominal level-flight trim; the goal is (250,0,20) m.

Each map is evaluated under four execution conditions: nominal RK4 dynamics;
actuator time constants multiplied by 1.25, lift by 0.97 and drag by 1.10;
1 m/s crosswind; and 0.1 s command delay. RK4 uses 0.005 s steps and is
implemented separately from planner prediction. The wind condition changes
ground velocity; sensing supplies the actual aircraft state. This is a
point-mass aircraft experiment, without a six-degree-of-freedom airframe model.

The five methods yield 24 x 4 x 5 = 480 episodes. The common timeout is 40 s.
Goal acceptance requires horizontal distance <15 m and altitude error <3 m.
Outcomes distinguish goal, no feasible primary, collision, envelope violation,
invalid dynamics and timeout. Every outcome is retained. Collision denotes entry
into the same inflated obstacle geometry used by planning; the reported minimum
clearance is measured to the uninflated geometric obstacle surface.

## Matched-input evaluation

Use 24 maps x 3 longitudinal positions x 3 lateral positions x 2 heading
offsets = 432 input states at level trim, with identical previous control.
Compare cost-K12, diverse-K12, full-K203, and cost/diverse schedules with
backup integration limits of 3600 and 10800 steps (K=203). Two timing repeats
rotate method order; repeats are not independent scenes. Use the full library
as the reference for existence of a continuation in that library.

Report discovery recall, paired verification disagreements, primary and backup
integration counts, elapsed median/P95/P99 and selected cost. Hard work budgets
must never be exceeded; unfinished rollouts do not count as verified.
Inference uses maps as the independent units; paired map bootstrap intervals
describe uncertainty within this generated suite.

## Failure diagnosis

Replay the previous high-speed, 120 m observation-radius cache case, logging
complete states and rejection counts. At the same states, compare cache-free
K12 and full-library searches, both with and without the observation radius.
Record finite-library failures separately from budget misses. Extending the
timeout is a diagnostic and does not change the original V2 result.

## Reproducibility

Archive source hashes, compiler and machine versions, all raw observations,
generated figures and the analysis program. Run timed benchmarks separately
from Gazebo and compilation.
