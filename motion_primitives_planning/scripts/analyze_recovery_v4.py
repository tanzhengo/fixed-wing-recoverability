#!/usr/bin/env python3
"""Compare complete-rollout V3 and incremental-rejection V4 matched inputs.

Usage: analyze_recovery_v4.py V3_PREFIX V4_PREFIX [--output-dir DIRECTORY]

Reads PREFIX_matched.csv from each experiment. If V4 episode CSV exists, also
requires complete episode/path/plan CSVs and audits exact replay equivalence. Both inputs must contain
432 states x 7 methods x 2 repeats. Requires exact input-state agreement and
unchanged full-library/K12 decisions, ranks and selected costs. Outputs one
four-panel PDF/PNG figure, JSON statistics, two TeX tables and numeric macros.
The incremental-rejection refinement was specified after V3; mission outcomes
remain those of the original V3 closed-loop experiment.
"""

import argparse
from collections import Counter, defaultdict
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

HELPER_PATH = Path(__file__).with_name("analyze_recovery_v3.py")
_spec = importlib.util.spec_from_file_location("recovery_v3_analysis_helpers", HELPER_PATH)
v3 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(v3)

UNCHANGED_METHODS = ("cost12", "diverse12", "full203")
VERSIONS = ("v3", "v4")
VERSION_LABELS = {"v3": "Complete rollout (V3)", "v4": "Incremental rejection (V4)"}
PAIRS = (("K12", "cost12", "diverse12"), ("W3600", "cost_w3600", "diverse_w3600"), ("W10800", "cost_w10800", "diverse_w10800"))


def read_matched(prefix):
    path = Path(f"{prefix}_matched.csv")
    required = ("state_id", "scene", "family", "method", "repeat") + v3.STATE_FIELDS + v3.DECISION_FIELDS
    rows = v3.read_csv(path, required)
    v3.require(len(rows) == 6048, f"{path}: found {len(rows)} rows, expected 6048; matched phase is incomplete")
    for row in rows:
        for field in ("state_id", "repeat"):
            v3.number(row, field, integer=True, nonnegative=True)
        v3.require(row["method"] in v3.MATCHED_METHODS and row["scene"] in v3.SCENES, f"{row['_source']}: unknown method or map")
        v3.require(row["family"] == row["scene"].rsplit("_", 1)[0], f"{row['_source']}: scene/family mismatch")
        v3.prepare_state(row)
        v3.prepare_decision(row)
        v3.require(not row["reused"] and row["cached_steps"] == 0, f"{row['_source']}: cache must be disabled in matched evaluation")
    index = v3.unique_index(rows, ("state_id", "method", "repeat"))
    groups = v3.group_by(rows, ("state_id",))
    v3.require(set(groups) == {(state_id,) for state_id in range(1, 433)}, f"{path}: expected state IDs 1 through 432")
    grid = {(method, repeat) for method in v3.MATCHED_METHODS for repeat in (0, 1)}
    scene_positions = defaultdict(set)
    collapsed = {}
    for (state_id,), repeated in sorted(groups.items()):
        first = repeated[0]
        v3.require({(r["method"], r["repeat"]) for r in repeated} == grid, f"State {state_id}: incomplete method/repeat grid")
        for row in repeated:
            v3.require((row["scene"], row["family"]) == (first["scene"], first["family"]), f"State {state_id}: inconsistent map identity")
            v3.require(all(row[field] == first[field] for field in v3.STATE_FIELDS), f"State {state_id}: complete input state differs across comparators")
        position = (first["x"], first["y"], first["sigma"])
        v3.require(position not in scene_positions[first["scene"]], f"{first['scene']}: duplicate position/heading")
        scene_positions[first["scene"]].add(position)
        for method in v3.MATCHED_METHODS:
            a, b = (index[(state_id, method, repeat)] for repeat in (0, 1))
            fields = ("status", "rank") + v3.BOOL_FIELDS + v3.COUNT_FIELDS
            v3.require(all(a[field] == b[field] for field in fields) and v3.near(a["cost"], b["cost"]), f"State {state_id}, {method}: timing-repeat decisions or work counts differ")
            collapsed[(state_id, method)] = dict(a, planning_ms=float(np.median([a["planning_ms"], b["planning_ms"]])))
        reference = collapsed[(state_id, "full203")]
        for method in v3.MATCHED_METHODS:
            row = collapsed[(state_id, method)]
            v3.require(not row["verified"] or reference["verified"], f"State {state_id}, {method}: limited search contradicts full-library reference")
            if row["verified"]:
                v3.require(row["cost"] >= reference["cost"] or v3.near(row["cost"], reference["cost"]), f"State {state_id}, {method}: verified selected cost is below full reference")
    expected_positions = {(x, y, heading) for x in (0., 25., 50.) for y in (-5., 0., 5.) for heading in (-.15, .15)}
    v3.require(set(scene_positions) == set(v3.SCENES), "Matched map set differs from frozen design")
    for scene, positions in scene_positions.items():
        v3.require(len(positions) == 18 and all(any(all(v3.near(a, b) for a, b in zip(position, expected)) for expected in expected_positions) for position in positions), f"{scene}: expected the frozen 3 x 3 x 2 state grid")
    positives = sum(collapsed[(state_id, "full203")]["verified"] for state_id in range(1, 433))
    v3.require(positives == 335, f"{path}: full-library reference has {positives} positive states; frozen V3 has 335")
    return {"path": path, "rows": rows, "collapsed": collapsed}


def audit_comparison(baseline, refined):
    old, new = baseline["collapsed"], refined["collapsed"]
    v3.require(set(old) == set(new), "V3 and V4 method/state keys differ")
    comparison_fields = ("status", "primary_feasible", "verified", "rank")
    for key, a in old.items():
        b = new[key]
        v3.require((a["scene"], a["family"]) == (b["scene"], b["family"]), f"{key}: V3/V4 map identity differs")
        v3.require(all(a[field] == b[field] for field in v3.STATE_FIELDS), f"{key}: V3/V4 complete input state differs")
        v3.require(all(a[field] == b[field] for field in ("primary_count", "feasible_count", "primary_steps", "primary_feasible")), f"{key}: primary evaluation changed with backup rejection mode")
        if key[1] in UNCHANGED_METHODS:
            v3.require(all(a[field] == b[field] for field in comparison_fields) and a["cost"] == b["cost"], f"{key}: V4 changed a K-count/full-library decision, rank or selected cost; implementation audit required")
            v3.require(a["recovery_checked"] == b["recovery_checked"] and a["backup_evaluated"] == b["backup_evaluated"], f"{key}: V4 changed the unbounded-work candidate/backup search sequence")
            v3.require(b["backup_steps"] <= a["backup_steps"], f"{key}: incremental rejection increased unbounded-work integration count")
    return {"passed": True, "observations_per_version": 6048, "states": 432, "maps": 24, "timing_repeats": 2, "reference_positive": 335, "reference_negative": 97, "input_mismatches": 0, "primary_evaluation_mismatches": 0, "unchanged_K_count_methods": list(UNCHANGED_METHODS), "K_count_decision_rank_cost_mismatches": 0, "repeat_decision_mismatches": 0, "work_limit_violations": 0, "reference_contradictions": 0, "cross_version_cost_tolerance": {"absolute": 0, "relative": 0}, "repeat_cost_tolerance": {"absolute": 1e-7, "relative": 1e-8}}


def audit_implementation_equivalence(v3_prefix, v4_prefix):
    """Optionally audit duplicate closed-loop tasks; never pool them as new tasks.

    A V4 episode file announces the replay phase. Once it exists, all replay
    inputs must be complete; a partially written phase is a hard audit failure.
    Timing and backup rejection/work counters may change. Aircraft states,
    decisions, selected costs and primary evaluation must remain identical.
    """
    if not Path(f"{v4_prefix}_episodes.csv").exists():
        return None, []
    episode_fields = ("case_id", "scene", "family", "condition", "method", "outcome", "steps", "sim_seconds", "goal_distance", "min_obstacle_clearance")
    path_fields = ("case_id", "step", "time") + v3.STATE_FIELDS
    plan_fields = ("case_id", "step") + v3.DECISION_FIELDS
    datasets, inputs = {}, []
    for version, prefix in (("v3", v3_prefix), ("v4", v4_prefix)):
        tables = {}
        for name, fields in (("episodes", episode_fields), ("paths", path_fields), ("plans", plan_fields)):
            path = Path(f"{prefix}_{name}.csv")
            tables[name] = v3.read_csv(path, fields)
            inputs.append({"version": version, "role": "implementation_equivalence", "file": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
        v3.require(len(tables["episodes"]) == 480, f"{version}: incomplete implementation-equivalence phase: {len(tables['episodes'])} episodes, expected 480; wait for replay completion")
        expected = {(scene, condition, method) for scene in v3.SCENES for condition in v3.CONDITIONS for method in v3.EPISODE_METHODS}
        v3.require(set(v3.unique_index(tables["episodes"], ("scene", "condition", "method"))) == expected, f"{version}: implementation-equivalence episode grid is incomplete")
        case_index = v3.unique_index(tables["episodes"], ("case_id",))
        for row in tables["episodes"]:
            v3.require(row["outcome"] in v3.OUTCOMES, f"{row['_source']}: unknown episode outcome")
            v3.number(row, "steps", integer=True, nonnegative=True)
            for field in ("sim_seconds", "goal_distance", "min_obstacle_clearance"):
                v3.number(row, field)
            v3.require(row["steps"] <= 200 and v3.near(row["sim_seconds"], .2 * row["steps"]), f"{row['_source']}: invalid episode step/time count")
        for row in tables["paths"]:
            v3.require((row["case_id"],) in case_index, f"{row['_source']}: unknown trajectory case")
            v3.number(row, "step", integer=True, nonnegative=True)
            v3.number(row, "time", nonnegative=True)
            v3.prepare_state(row)
        for row in tables["plans"]:
            v3.require((row["case_id"],) in case_index, f"{row['_source']}: unknown planner case")
            v3.number(row, "step", integer=True, nonnegative=True)
            v3.prepare_decision(row)
            v3.require(not row["reused"] and row["cached_steps"] == 0, f"{row['_source']}: unexpected cache in replay")
        v3.unique_index(tables["plans"], ("case_id", "step"))
        path_cases = v3.group_by(tables["paths"], ("case_id",))
        plan_cases = v3.group_by(tables["plans"], ("case_id",))
        for key, episode in case_index.items():
            paths, plans = path_cases.get(key, []), plan_cases.get(key, [])
            expected_plans = episode["steps"] + (episode["outcome"] == "no_feasible_primary")
            v3.require(len(plans) == expected_plans and [r["step"] for r in plans] == list(range(expected_plans)), f"{version}, {key}: incomplete or inconsistent planner decision sequence")
            v3.require(paths and paths[0]["step"] == 0 and paths[-1]["step"] == episode["steps"] and {r["step"] for r in paths} == set(range(episode["steps"] + 1)), f"{version}, {key}: incomplete trajectory sequence")
        datasets[version] = tables
    a, b = datasets["v3"], datasets["v4"]
    for name in ("episodes", "paths", "plans"):
        v3.require(len(a[name]) == len(b[name]), f"Implementation-equivalence {name} row counts differ: V3={len(a[name])}, V4={len(b[name])}; replay may be incomplete")
    for index, (old, new) in enumerate(zip(a["episodes"], b["episodes"]), 1):
        changed = [field for field in episode_fields if old[field] != new[field]]
        v3.require(not changed, f"Episode row {index}: V3/V4 outcomes, counts or final metrics differ: {changed}")
    for index, (old, new) in enumerate(zip(a["paths"], b["paths"]), 1):
        changed = [field for field in path_fields if old[field] != new[field]]
        v3.require(not changed, f"Trajectory row {index}: V3/V4 aircraft state or time differs: {changed}")
    allowed = ("planning_ms", "backup_steps", "reject_collision", "reject_envelope", "reject_known", "reject_invalid", "truncated", "work_exhausted")
    preserved = tuple(field for field in plan_fields if field not in allowed)
    for index, (old, new) in enumerate(zip(a["plans"], b["plans"]), 1):
        changed = [field for field in preserved if old[field] != new[field]]
        v3.require(not changed, f"Planner row {index}: V3/V4 decision, cost/rank or primary evaluation differs: {changed}")
        v3.require(new["backup_steps"] <= old["backup_steps"], f"Planner row {index}: incremental rejection increased backup integration work")
    return {"passed": True, "episodes": 480, "path_rows_per_version": len(a["paths"]), "decision_rows_per_version": len(a["plans"]), "episode_mismatches": 0, "trajectory_mismatches": 0, "decision_mismatches": 0, "numeric_comparison": "exact equality of parsed CSV numeric values; no tolerance", "preserved_episode_fields": list(episode_fields), "preserved_path_fields": list(path_fields), "preserved_decision_fields": list(preserved), "allowed_varying_decision_fields": list(allowed), "interpretation": "Duplicate executions of the original 480 tasks verify implementation equivalence; they are not additional independent scenarios and are not pooled with V3 outcomes."}, inputs


def method_summary(rows, reference):
    good = [row for row in rows if row["verified"]]
    return {"states": len(rows), "verified": len(good), "reference_positive": 335, "discovery_recall": len(good) / 335, "planning_ms_state_median": v3.distribution(row["planning_ms"] for row in rows), "primary_steps": v3.distribution(row["primary_steps"] for row in rows), "backup_steps": v3.distribution(row["backup_steps"] for row in rows), "total_steps": v3.distribution(row["primary_steps"] + row["backup_steps"] for row in rows), "candidates_checked": v3.distribution(row["recovery_checked"] for row in rows), "backups_evaluated": v3.distribution(row["backup_evaluated"] for row in rows), "selected_cost_verified": v3.distribution(row["cost"] for row in good), "selected_cost_excess_over_reference": v3.distribution(row["cost"] - reference[row["state_id"]]["cost"] for row in good), "work_exhausted_states": sum(row["work_exhausted"] for row in rows), "truncated_rollouts": sum(row["truncated"] for row in rows), "status_counts": dict(Counter(row["status"] for row in rows))}


def paired_map_recall(a, b, reference):
    numerators, denominators, maps = [], [], []
    for scene in v3.SCENES:
        indices = [i for i, row in enumerate(reference) if row["scene"] == scene]
        delta = sum(int(b[i]["verified"]) - int(a[i]["verified"]) for i in indices)
        positive = sum(reference[i]["verified"] for i in indices)
        numerators.append(delta)
        denominators.append(positive)
        maps.append({"scene": scene, "reference_positive": positive, "paired_verified_difference": delta})
    return v3.bootstrap_paired_map(numerators, denominators), maps


def summarize(baseline, refined, audit):
    sets = {"v3": baseline, "v4": refined}
    versions = {}
    for version, dataset in sets.items():
        collapsed = dataset["collapsed"]
        reference = {state_id: collapsed[(state_id, "full203")] for state_id in range(1, 433)}
        methods = {method: method_summary([collapsed[(state_id, method)] for state_id in range(1, 433)], reference) for method in v3.MATCHED_METHODS}
        pairs = {}
        for label, cost, diverse in PAIRS:
            a = [collapsed[(state_id, cost)] for state_id in range(1, 433)]
            b = [collapsed[(state_id, diverse)] for state_id in range(1, 433)]
            refs = [reference[state_id] for state_id in range(1, 433)]
            pairs[label] = v3.paired_summary(a, b, refs)
            pairs[label]["map_bootstrap"], pairs[label]["by_map"] = paired_map_recall(a, b, refs)
        versions[version] = {"label": VERSION_LABELS[version], "methods": methods, "paired_schedule_comparisons": pairs}
    changes = {}
    old, new = baseline["collapsed"], refined["collapsed"]
    reference = [new[(state_id, "full203")] for state_id in range(1, 433)]
    for method in v3.MATCHED_METHODS:
        a = [old[(state_id, method)] for state_id in range(1, 433)]
        b = [new[(state_id, method)] for state_id in range(1, 433)]
        old_mean = versions["v3"]["methods"][method]["backup_steps"]["mean"]
        new_mean = versions["v4"]["methods"][method]["backup_steps"]["mean"]
        joint = [(x, y) for x, y in zip(a, b) if x["verified"] and y["verified"]]
        boot, maps = paired_map_recall(a, b, reference)
        changes[method] = {"v4_only_verified": sum(y["verified"] and not x["verified"] for x, y in zip(a, b)), "v3_only_verified": sum(x["verified"] and not y["verified"] for x, y in zip(a, b)), "both_verified": len(joint), "recall_difference_v4_minus_v3": boot["estimate"], "map_bootstrap": boot, "by_map": maps, "mean_backup_steps_v3": old_mean, "mean_backup_steps_v4": new_mean, "mean_backup_work_reduction_fraction": 1 - new_mean / old_mean if old_mean > 0 else None, "paired_backup_step_difference_v4_minus_v3": v3.distribution(y["backup_steps"] - x["backup_steps"] for x, y in zip(a, b)), "paired_timing_difference_ms_v4_minus_v3": v3.distribution(y["planning_ms"] - x["planning_ms"] for x, y in zip(a, b)), "paired_speedup_v3_over_v4": v3.distribution(x["planning_ms"] / y["planning_ms"] for x, y in zip(a, b) if y["planning_ms"] > 0), "paired_selected_cost_difference_v4_minus_v3_jointly_verified": v3.distribution(y["cost"] - x["cost"] for x, y in joint), "changed_selected_rank_states": sum(x["rank"] != y["rank"] for x, y in zip(a, b)), "changed_selected_cost_states": sum(not v3.near(x["cost"], y["cost"]) for x, y in zip(a, b))}
    return {"schema_version": 4, "audit": audit, "study_design": {"timing": "Median of two timings within each state/method; reported percentiles summarize 432 state medians. V3 and V4 were timed in separate runs.", "refinement": "Incremental rejection was specified after observing V3 and before running V4. A backup terminates on the first sampled explicit constraint violation; the Euler update and complete-witness criterion remain unchanged.", "comparison": "Same frozen maps, 432 states, seven methods, two repeats and 3,600/10,800-step hard limits. No new closed-loop episodes are included.", "bootstrap": "10,000 paired resamples of 24 map clusters, seed 12345; each map retains all its states. Percentile intervals describe this generated suite.", "rejection_counts": "V4 classifies the first observed violation; V3 classifies after a complete rollout. Per-reason rejection counts have different meanings and are not compared.", "budget": "Equal hard integration limits do not imply equal consumed work. Unfinished rollouts never count as verified."}, "versions": versions, "v4_minus_v3": changes}


def plot_comparison(summary, output, stem):
    plt = v3.style_figures()
    from matplotlib.patches import Patch
    from matplotlib.ticker import FuncFormatter
    fig, axes = plt.subplots(2, 2, figsize=(9.3, 6.5))
    colors = {"cost": v3.COLORS["cost12"], "diverse": v3.COLORS["diverse12"], "full": v3.COLORS["full203"]}
    ax = axes[0, 0]
    for vi, version in enumerate(VERSIONS):
        for mi, method in enumerate(("cost", "diverse")):
            offset = (vi * 2 + mi - 1.5) * .19
            values = [100 * summary["versions"][version]["methods"][f"{method}_w{budget}"]["discovery_recall"] for budget in (3600, 10800)]
            bars = ax.bar(np.arange(2) + offset, values, width=.18, color=colors[method], alpha=.45 if vi == 0 else 1, hatch="///" if vi == 0 else None, edgecolor=colors[method], linewidth=.5)
            for bar, value in zip(bars, values):
                ax.annotate(f"{value:.1f}", (bar.get_x() + bar.get_width() / 2, value), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=6.8)
    ax.set(xticks=[0, 1], xticklabels=["3,600 steps", "10,800 steps"], ylabel="Discovery recall (%)", ylim=(0, 110), title="a  Equal integration limits, 335 reference positives")
    ax.grid(axis="y", alpha=.17)
    ax = axes[0, 1]
    for vi, version in enumerate(VERSIONS):
        vals = [summary["versions"][version]["methods"][method]["backup_steps"]["mean"] for method in UNCHANGED_METHODS]
        ax.bar(np.arange(3) + (vi - .5) * .32, vals, width=.3, color=[colors[m] for m in ("cost", "diverse", "full")], alpha=.45 if vi == 0 else 1, hatch="///" if vi == 0 else None, edgecolor=[colors[m] for m in ("cost", "diverse", "full")], linewidth=.5)
    ax.set(xticks=range(3), xticklabels=["Cost, K=12", "Diverse, K=12", "Full, K=203"], ylabel="Mean backup integration steps", title="b  Backup integration with identical decisions")
    ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x / 1000:g}k"))
    ax.grid(axis="y", alpha=.17)
    ax = axes[1, 0]
    for vi, version in enumerate(VERSIONS):
        methods = summary["versions"][version]["methods"]
        values = [methods[m]["planning_ms_state_median"]["median"] for m in UNCHANGED_METHODS]
        p95 = [methods[m]["planning_ms_state_median"]["p95"] for m in UNCHANGED_METHODS]
        x = np.arange(3) + (vi - .5) * .32
        ax.bar(x, values, width=.3, color=[colors[m] for m in ("cost", "diverse", "full")], alpha=.45 if vi == 0 else 1, hatch="///" if vi == 0 else None, edgecolor=[colors[m] for m in ("cost", "diverse", "full")], linewidth=.5)
        ax.vlines(x, values, p95, color="#454545", linewidth=.9)
        ax.scatter(x, p95, color="#454545", marker="_", s=45, linewidth=.9)
    ax.set(xticks=range(3), xticklabels=["Cost, K=12", "Diverse, K=12", "Full, K=203"], ylabel="Planning latency (ms)", title="c  State-median latency: bars median, whiskers P95")
    ax.grid(axis="y", alpha=.17)
    ax = axes[1, 1]
    for vi, version in enumerate(VERSIONS):
        for j, budget in enumerate((3600, 10800)):
            boot = summary["versions"][version]["paired_schedule_comparisons"][f"W{budget}"]["map_bootstrap"]
            estimate, low, high = [100 * x for x in (boot["estimate"], *boot["ci95"])]
            y = j + (vi - .5) * .22
            ax.errorbar(estimate, y, xerr=[[estimate - low], [high - estimate]], color="#555555" if vi == 0 else "#D55E00", marker="o" if vi else "s", mfc="white" if vi == 0 else "#D55E00", ms=5, capsize=3, lw=1.4, label=VERSION_LABELS[version] if j == 0 else None)
    ax.axvline(0, color="#777777", ls="--", lw=.8)
    ax.set(yticks=[0, 1], yticklabels=["3,600 steps", "10,800 steps"], ylim=(-.45, 1.45), xlabel="Diversity − cost recall (percentage points)", title="d  Paired differences with 95% map intervals")
    ax.invert_yaxis()
    ax.grid(axis="x", alpha=.17)
    ax.legend(frameon=False, fontsize=7, loc="best")
    handles = [Patch(facecolor=colors[method], label=label) for method, label in (("cost", "Cost schedule"), ("diverse", "Diversity schedule"), ("full", "Full-library reference"))]
    handles += [Patch(facecolor="#888888", alpha=.45, hatch="///", edgecolor="#555555", label="Complete rollout (V3)"), Patch(facecolor="#888888", label="Incremental rejection (V4)")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .005), ncol=3, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, .11, 1, 1))
    files = []
    v3.save_figure(fig, output, f"{stem}_comparison", files)
    plt.close(fig)
    return files


def write_tables(summary, output, stem):
    files = []
    lines = [r"\begin{tabular}{llrrrrrr}", r"\toprule", r"Evaluation & Method & Verified & Recall (\%) & Median (ms) & P95 (ms) & P99 (ms) & Mean backup steps \\", r"\midrule"]
    for vi, version in enumerate(VERSIONS):
        if vi:
            lines.append(r"\midrule")
        for i, method in enumerate(v3.MATCHED_METHODS):
            row = summary["versions"][version]["methods"][method]
            timing = row["planning_ms_state_median"]
            values = [VERSION_LABELS[version] if i == 0 else "", v3.LABELS[method], str(row["verified"]), f"{100 * row['discovery_recall']:.1f}", *[f"{timing[key]:.2f}" for key in ("median", "p95", "p99")], f"{row['backup_steps']['mean']:.0f}"]
            lines.append(" & ".join(values) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}"]
    path = output / f"{stem}_comparison_table.tex"
    path.write_text("% Supplemental matched-input experiment; V4 was specified after V3.\n" + "\n".join(lines) + "\n")
    files.append(path.name)
    lines = [r"\begin{tabular}{llrrr}", r"\toprule", r"Evaluation & Budget & Diversity-only & Cost-only & Difference, pp [95\% map interval] \\", r"\midrule"]
    for vi, version in enumerate(VERSIONS):
        if vi:
            lines.append(r"\midrule")
        for i, (label, _, _) in enumerate(PAIRS):
            pair = summary["versions"][version]["paired_schedule_comparisons"][label]
            boot = pair["map_bootstrap"]
            low, high = boot["ci95"]
            lines.append(f"{VERSION_LABELS[version] if i == 0 else ''} & {label} & {pair['diverse_only']} & {pair['cost_only']} & {100 * boot['estimate']:.2f} [{100 * low:.2f}, {100 * high:.2f}]" + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}"]
    path = output / f"{stem}_paired_table.tex"
    path.write_text("% Percentile intervals resample 24 paired maps, 10,000 repetitions, seed 12345.\n" + "\n".join(lines) + "\n")
    files.append(path.name)
    macros = {"VFourMatchedStates": 432, "VFourReferencePositive": 335}
    for label, method in (("Cost", "cost12"), ("Diverse", "diverse12"), ("Full", "full203")):
        row = summary["versions"]["v4"]["methods"][method]
        macros[f"VFour{label}Verified"] = row["verified"]
        macros[f"VFour{label}MedianMs"] = f"{row['planning_ms_state_median']['median']:.2f}"
        macros[f"VFour{label}MeanBackupSteps"] = f"{row['backup_steps']['mean']:.0f}"
    path = output / f"{stem}_numbers.tex"
    path.write_text("% Generated incremental-rejection matched-input values.\n" + "\n".join("\\newcommand{\\" + key + "}{" + str(value) + "}" for key, value in macros.items()) + "\n")
    files.append(path.name)
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("v3_prefix", type=Path)
    parser.add_argument("v4_prefix", type=Path)
    parser.add_argument("--output-dir", type=Path, help="Defaults to V4 prefix parent")
    parser.add_argument("--audit-only", action="store_true", help="Audit all available inputs without writing artifacts")
    parser.add_argument("--json-only", action="store_true", help="Update JSON and provenance while preserving existing figures and TeX artifacts")
    args = parser.parse_args()
    try:
        baseline, refined = read_matched(args.v3_prefix), read_matched(args.v4_prefix)
        audit = audit_comparison(baseline, refined)
        equivalence, equivalence_inputs = audit_implementation_equivalence(args.v3_prefix, args.v4_prefix)
        if args.audit_only:
            print("PASS: both 6048-row matched designs complete; exact states, primary work and all K12/full203 decisions preserved; repeats, work limits and references consistent.")
            if equivalence is not None:
                print(f"PASS: implementation equivalence across {equivalence['episodes']} episodes, {equivalence['path_rows_per_version']} trajectory rows and {equivalence['decision_rows_per_version']} planner decisions.")
            return 0
        summary = summarize(baseline, refined, audit)
        if equivalence is not None:
            summary["implementation_equivalence"] = equivalence
            summary["study_design"]["comparison"] = "Same frozen maps, 432 states, seven methods, two repeats and 3,600/10,800-step hard limits. A separate replay of the same 480 closed-loop tasks verifies implementation equivalence and is not counted as additional independent evidence."
        output = args.output_dir or args.v4_prefix.parent
        output.mkdir(parents=True, exist_ok=True)
        stem = args.v4_prefix.name
        if args.json_only:
            files = [f"{stem}_{suffix}" for suffix in ("comparison.pdf", "comparison.png", "comparison_table.tex", "paired_table.tex", "numbers.tex")]
            v3.require(all((output / name).is_file() for name in files), "--json-only requires the existing figures and TeX artifacts; run without it to create the full report")
        else:
            files = plot_comparison(summary, output, stem)
            files += write_tables(summary, output, stem)
        summary["provenance"] = {"inputs": [{"version": version, "file": str(dataset["path"].resolve()), "sha256": hashlib.sha256(dataset["path"].read_bytes()).hexdigest()} for version, dataset in (("v3", baseline), ("v4", refined))], "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "v3_helper_sha256": hashlib.sha256(HELPER_PATH.read_bytes()).hexdigest(), "numpy_version": np.__version__, "matplotlib_version": __import__("matplotlib").__version__}
        summary["provenance"]["inputs"].extend(equivalence_inputs)
        path = output / f"{stem}_analysis.json"
        summary["artifacts"] = files + [path.name]
        path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(f"PASS: wrote {len(summary['artifacts'])} supplementary artifacts to {output.resolve()}")
        if equivalence is not None:
            print(f"Implementation equivalence: {equivalence['episodes']} episodes, {equivalence['path_rows_per_version']} trajectory rows, {equivalence['decision_rows_per_version']} planner decisions; all identical.")
        for method in v3.MATCHED_METHODS:
            row = summary["versions"]["v4"]["methods"][method]
            print(f"{method}: verified {row['verified']}/335, recall {100 * row['discovery_recall']:.2f}%, median {row['planning_ms_state_median']['median']:.3f} ms, mean backup steps {row['backup_steps']['mean']:.1f}")
        return 0
    except (v3.AuditError, OSError, ValueError) as exc:
        print(f"V4 comparison refused: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
