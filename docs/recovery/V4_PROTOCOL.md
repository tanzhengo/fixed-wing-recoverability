# Incremental rejection evaluation

This supplemental protocol was fixed after observing V3 and before running V4.
V3 showed a K=12 discovery gain without a gain under smaller integration limits.
The refinement stops a continuation as soon as a sampled constraint violation
proves its rejection. The Euler update, complete-witness criterion, primary
library, cost and scheduling rule remain the same.

Repeat the exact 432 matched inputs, seven methods, two timing repeats and work
limits from V3 with `stop_on_rejection=true`. No horizon, geometry, input or
budget is tuned. Require full-library and K=12 decisions, ranks and selected
costs to agree with V3; any disagreement requires an implementation audit.
Work-limited decisions can change because earlier rejection releases budget.
Report both the original complete-rollout and incremental-rejection results.
The original closed-loop suite remains the mission-level evidence.

Rejection reason counts in V4 classify the first observed violation; V3 applied
its classification after a complete rollout. Their per-reason totals therefore
have different meanings, while complete-witness feasibility is unchanged.

After the matched-input equivalence audit passed, repeat the 480 closed-loop
episodes with incremental rejection enabled. Compare every episode outcome,
step count, final state and complete recorded path with V3. This is an
implementation-equivalence check on the same tasks, not 480 additional
independent scenarios. All original task parameters remain fixed.
