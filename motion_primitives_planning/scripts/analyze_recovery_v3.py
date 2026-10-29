#!/usr/bin/env python3
"""Audit and analyze the frozen V3 finite-continuation experiment.

Usage: analyze_recovery_v3.py /path/to/prefix [--output-dir /path/to/report]

Reads prefix_{episodes,plans,matched,paths,scenarios,diagnostic}.csv.  All
480 episodes and 6,048 matched observations must be present before any report
is written. Timing repeats are collapsed within state/method; uncertainty is
estimated by resampling the 24 maps, never the timing observations. Requires
NumPy and Matplotlib. Outputs JSON, publication figures (PDF/PNG), and TeX
fragments; the JSON manifest names every generated artifact.
"""

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

EPISODE_METHODS = ("primary3", "primary6", "cost12", "diverse12", "full203")
MATCHED_METHODS = ("cost12", "diverse12", "full203", "cost_w3600", "diverse_w3600", "cost_w10800", "diverse_w10800")
DIAGNOSTIC_METHODS = ("cache120", "diverse120", "full120", "diverse_unlimited", "full_unlimited")
CONDITIONS = ("nominal", "model_mismatch", "crosswind", "delay100ms")
FAMILIES = ("staggered", "wall", "culdesac", "spatial")
SCENES = tuple(f"{family}_{seed}" for family in FAMILIES for seed in range(6))
OUTCOMES = ("goal", "no_feasible_primary", "collision", "envelope_violation", "invalid_dynamics", "timeout")
STATE_FIELDS = ("x", "y", "z", "V", "gamma", "sigma", "roll", "attack", "T", "d_sigma")
BOOL_FIELDS = ("primary_feasible", "verified", "reused", "work_exhausted")
COUNT_FIELDS = ("primary_count", "feasible_count", "recovery_checked", "backup_evaluated", "primary_steps", "backup_steps", "cached_steps", "reject_collision", "reject_envelope", "reject_known", "reject_invalid", "truncated")
DECISION_FIELDS = ("planning_ms", "status", "cost", "rank") + BOOL_FIELDS + COUNT_FIELDS
LABELS = {"primary3": "Primary, 3 s", "primary6": "Primary, 6 s", "cost12": "Cost, K=12", "diverse12": "Diverse, K=12", "full203": "Full, K=203", "cost_w3600": "Cost, W=3,600", "diverse_w3600": "Diverse, W=3,600", "cost_w10800": "Cost, W=10,800", "diverse_w10800": "Diverse, W=10,800", "nominal": "Nominal", "model_mismatch": "Model mismatch", "crosswind": "Crosswind", "delay100ms": "100 ms delay", "staggered": "Staggered pillars", "wall": "Lateral wall", "culdesac": "Offset cul-de-sac", "spatial": "Spatial obstacles", "cache120": "Cache, 120 m", "diverse120": "Diverse, 120 m", "full120": "Full, 120 m", "diverse_unlimited": "Diverse, unrestricted", "full_unlimited": "Full, unrestricted"}
COLORS = {"primary3": "#999999", "primary6": "#6F4E7C", "cost12": "#0072B2", "diverse12": "#D55E00", "full203": "#009E73"}


class AuditError(ValueError):
    """Input is missing, incomplete, or inconsistent with the frozen design."""


def require(condition, message):
    if not condition:
        raise AuditError(message)


def near(a, b, absolute=1e-7, relative=1e-8):
    return a == b or (math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, abs_tol=absolute, rel_tol=relative))


def read_csv(path, required):
    require(path.is_file(), f"Missing input: {path}; finish every benchmark phase before analysis.")
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        columns = reader.fieldnames or []
        require(len(columns) == len(set(columns)), f"{path}: duplicate column names")
        require(set(required) <= set(columns), f"{path}: missing columns {sorted(set(required) - set(columns))}")
        rows = []
        for line, row in enumerate(reader, 2):
            require(None not in row and all(row.get(key) not in (None, "") for key in required), f"{path}:{line}: missing/extra fields (possibly an unfinished write)")
            row["_source"] = f"{path.name}:{line}"
            rows.append(row)
    require(rows, f"{path}: no observations; benchmark is incomplete")
    return rows


def number(row, field, integer=False, nonnegative=False, finite=True):
    try:
        value = float(row[field])
    except (ValueError, TypeError, KeyError) as exc:
        raise AuditError(f"{row['_source']}: invalid {field}") from exc
    require(not math.isnan(value) and (not finite or math.isfinite(value)), f"{row['_source']}: nonfinite {field}")
    require(not nonnegative or value >= 0, f"{row['_source']}: negative {field}")
    if integer:
        require(math.isfinite(value) and value == int(value), f"{row['_source']}: noninteger {field}")
        value = int(value)
    row[field] = value
    return value


def prepare_state(row):
    for field in STATE_FIELDS:
        number(row, field)


def prepare_decision(row):
    number(row, "planning_ms", nonnegative=True)
    for field in BOOL_FIELDS:
        require(row[field] in ("0", "1"), f"{row['_source']}: {field} must be 0 or 1")
        row[field] = row[field] == "1"
    for field in COUNT_FIELDS:
        number(row, field, integer=True, nonnegative=True)
    number(row, "rank", integer=True)
    number(row, "cost", finite=row["primary_feasible"])
    require(row["rank"] >= -1, f"{row['_source']}: invalid selected rank")
    require(not row["verified"] or row["primary_feasible"], f"{row['_source']}: verified without a feasible primary")
    require(row["feasible_count"] <= row["primary_count"], f"{row['_source']}: feasible count exceeds primary count")
    if "method" in row and "_w" in row["method"]:
        budget = int(row["method"].split("_w")[1])
        require(row["backup_steps"] <= budget, f"{row['_source']}: backup work {row['backup_steps']} exceeds hard limit {budget}")


def group_by(rows, fields):
    grouped = defaultdict(list)
    for row in rows:
        grouped[tuple(row[field] for field in fields)].append(row)
    return grouped


def unique_index(rows, fields):
    index = {}
    for row in rows:
        key = tuple(row[field] for field in fields)
        require(key not in index, f"{row['_source']}: duplicate key {key}")
        index[key] = row
    return index


def audit(prefix):
    required = {
        "episodes": ("case_id", "scene", "family", "condition", "method", "outcome", "steps", "sim_seconds", "goal_distance", "min_obstacle_clearance"),
        "plans": ("case_id", "step") + DECISION_FIELDS,
        "matched": ("state_id", "scene", "family", "method", "repeat") + STATE_FIELDS + DECISION_FIELDS,
        "paths": ("case_id", "step", "time") + STATE_FIELDS,
        "scenarios": ("scene", "family", "low", "high", "type", "x", "y", "z", "r", "height"),
        "diagnostic": ("step", "time", "method") + STATE_FIELDS + DECISION_FIELDS,
        "diagnostic_summary": ("outcome", "steps", "time", "goal_distance") + STATE_FIELDS,
    }
    inputs = {name: Path(f"{prefix}_{name}.csv") for name in required}
    data = {name: read_csv(inputs[name], columns) for name, columns in required.items()}
    episodes, matched = data["episodes"], data["matched"]
    require(len(episodes) == 480, f"Incomplete episode design: found {len(episodes)}, expected 480 (24 maps × 4 conditions × 5 methods).")
    require(len(matched) == 6048, f"Incomplete matched design: found {len(matched)}, expected 6048 (432 states × 7 methods × 2 repeats).")
    cases = unique_index(episodes, ("case_id",))
    episode_grid = unique_index(episodes, ("scene", "condition", "method"))
    expected = {(scene, condition, method) for scene in SCENES for condition in CONDITIONS for method in EPISODE_METHODS}
    require(set(episode_grid) == expected, "Episode design does not match the frozen 24-map factorial grid.")
    for row in episodes:
        require(row["family"] == row["scene"].rsplit("_", 1)[0], f"{row['_source']}: scene/family mismatch")
        require(row["outcome"] in OUTCOMES, f"{row['_source']}: unknown outcome {row['outcome']}")
        number(row, "steps", integer=True, nonnegative=True)
        for field in ("sim_seconds", "goal_distance"):
            number(row, field, nonnegative=True)
        number(row, "min_obstacle_clearance")
        require(row["steps"] <= 200 and row["sim_seconds"] <= 40 + 1e-7, f"{row['_source']}: exceeds the 40 s episode horizon")
        require(near(row["sim_seconds"], .2 * row["steps"]), f"{row['_source']}: time and step count differ")
        if row["outcome"] == "goal":
            require(row["goal_distance"] < 15, f"{row['_source']}: goal outcome violates horizontal tolerance")
        if row["outcome"] == "timeout":
            require(row["steps"] == 200, f"{row['_source']}: premature timeout")
    for row in data["plans"]:
        require((row["case_id"],) in cases, f"{row['_source']}: unknown case")
        row["method"] = cases[(row["case_id"],)]["method"]
        number(row, "step", integer=True, nonnegative=True)
        prepare_decision(row)
        require(not row["reused"], f"{row['_source']}: reuse is disabled in the main experiment")
    unique_index(data["plans"], ("case_id", "step"))
    plans_by_case = group_by(data["plans"], ("case_id",))
    for (case_id,), episode in cases.items():
        plans = plans_by_case.get((case_id,), [])
        expected_count = episode["steps"] + (episode["outcome"] == "no_feasible_primary")
        require(len(plans) == expected_count, f"{case_id}: {len(plans)} plan rows, expected {expected_count}")
        require([r["step"] for r in plans] == list(range(expected_count)), f"{case_id}: plan sequence is incomplete or out of order")
    for row in matched:
        require(row["method"] in MATCHED_METHODS and row["scene"] in SCENES, f"{row['_source']}: unknown method or map")
        require(row["family"] == row["scene"].rsplit("_", 1)[0], f"{row['_source']}: scene/family mismatch")
        for field in ("state_id", "repeat"):
            number(row, field, integer=True, nonnegative=True)
        prepare_state(row)
        prepare_decision(row)
        require(not row["reused"], f"{row['_source']}: reuse is disabled for matched states")
    matched_index = unique_index(matched, ("state_id", "method", "repeat"))
    state_groups = group_by(matched, ("state_id",))
    require(len(state_groups) == 432, f"Expected 432 unique matched states; found {len(state_groups)}")
    grid = {(method, repeat) for method in MATCHED_METHODS for repeat in (0, 1)}
    scene_positions = defaultdict(set)
    collapsed = {}
    for (state_id,), rows in sorted(state_groups.items()):
        require({(r["method"], r["repeat"]) for r in rows} == grid, f"State {state_id}: incomplete comparator/repeat grid")
        first = rows[0]
        for row in rows:
            require((row["scene"], row["family"]) == (first["scene"], first["family"]), f"State {state_id}: map metadata differ across comparators")
            require(all(near(row[field], first[field], 1e-10, 1e-10) for field in STATE_FIELDS), f"State {state_id}: complete aircraft state differs across comparators")
        position = (first["x"], first["y"], first["sigma"])
        require(position not in scene_positions[first["scene"]], f"{first['scene']}: duplicate matched aircraft position/heading")
        scene_positions[first["scene"]].add(position)
        for method in MATCHED_METHODS:
            a, b = (matched_index[(state_id, method, repeat)] for repeat in (0, 1))
            deterministic = ("status", "rank") + BOOL_FIELDS + COUNT_FIELDS
            require(all(a[field] == b[field] for field in deterministic) and near(a["cost"], b["cost"]), f"State {state_id}, {method}: timing repeats have different decisions or work counts")
            collapsed[(state_id, method)] = dict(a, planning_ms=float(np.median([a["planning_ms"], b["planning_ms"]])), timing_repeats=2)
        reference = collapsed[(state_id, "full203")]
        for method in MATCHED_METHODS:
            row = collapsed[(state_id, method)]
            require(not row["verified"] or reference["verified"], f"State {state_id}, {method}: verified evidence contradicts full-library reference")
            if row["verified"]:
                require(row["cost"] >= reference["cost"] or near(row["cost"], reference["cost"]), f"State {state_id}, {method}: selected verified cost is below cost-ordered reference")
    expected_positions = {(x, y, heading) for x in (0., 25., 50.) for y in (-5., 0., 5.) for heading in (-.15, .15)}
    require(set(scene_positions) == set(SCENES), "Matched map set differs from episode map set")
    for scene, positions in scene_positions.items():
        require(len(positions) == 18 and all(any(all(near(a, b) for a, b in zip(position, expected)) for expected in expected_positions) for position in positions), f"{scene}: matched 3 × 3 × 2 state grid differs from frozen protocol")
    for row in data["paths"]:
        require((row["case_id"],) in cases, f"{row['_source']}: unknown path case")
        number(row, "step", integer=True, nonnegative=True)
        number(row, "time", nonnegative=True)
        prepare_state(row)
        require(near(row["time"], .2 * row["step"]), f"{row['_source']}: inconsistent path time")
    path_groups = group_by(data["paths"], ("case_id",))
    for (case_id,), episode in cases.items():
        paths = path_groups.get((case_id,), [])
        require(paths and paths[0]["step"] == 0 and paths[-1]["step"] == episode["steps"], f"{case_id}: missing initial/final path record")
        path_index = {}
        for row in paths:
            if row["step"] in path_index:
                require(all(near(row[f], path_index[row["step"]][f]) for f in STATE_FIELDS), f"{case_id}: conflicting duplicate path records")
            path_index[row["step"]] = row
        require(set(path_index) == set(range(episode["steps"] + 1)), f"{case_id}: path sequence has missing steps")
        last = paths[-1]
        require(near(math.hypot(250 - last["x"], last["y"]), episode["goal_distance"], 5e-5, 2e-7), f"{case_id}: final distance does not match trajectory")
        if episode["outcome"] == "goal":
            require(abs(last["z"] - 20) < 3, f"{case_id}: goal outcome violates altitude tolerance")
    for row in data["scenarios"]:
        require(row["scene"] in SCENES and row["family"] == row["scene"].rsplit("_", 1)[0], f"{row['_source']}: unknown geometry map/family")
        require(row["type"] in ("cylinder", "sphere"), f"{row['_source']}: unknown geometry type")
        for field in ("low", "high", "x", "y", "z", "r", "height"):
            number(row, field)
        require(row["r"] > 0 and row["high"] > row["low"], f"{row['_source']}: invalid geometry")
    require({r["scene"] for r in data["scenarios"]} == set(SCENES), "Scenario export is incomplete")
    for row in data["diagnostic"]:
        require(row["method"] in DIAGNOSTIC_METHODS, f"{row['_source']}: unknown diagnostic method")
        number(row, "step", integer=True, nonnegative=True)
        number(row, "time", nonnegative=True)
        prepare_state(row)
        prepare_decision(row)
    unique_index(data["diagnostic"], ("step", "method"))
    diag_groups = group_by(data["diagnostic"], ("step",))
    require(set(step for (step,) in diag_groups) == set(range(len(diag_groups))), "Diagnostic time sequence is incomplete")
    for (step,), rows in diag_groups.items():
        require({r["method"] for r in rows} == set(DIAGNOSTIC_METHODS), f"Diagnostic step {step}: missing comparator")
        require(all(all(near(row[field], rows[0][field], 1e-10, 1e-10) for field in STATE_FIELDS + ("time",)) for row in rows), f"Diagnostic step {step}: counterfactuals do not share complete state")
        require(near(rows[0]["time"], step * .2), f"Diagnostic step {step}: time differs from control interval")
        by_method = {r["method"]: r for r in rows}
        for limited, full in (("diverse120", "full120"), ("diverse_unlimited", "full_unlimited"), ("full120", "full_unlimited")):
            require(not by_method[limited]["verified"] or by_method[full]["verified"], f"Diagnostic step {step}: {limited} contradicts {full}")
    require(len(data["diagnostic_summary"]) == 1, "Expected one diagnostic terminal summary")
    terminal = data["diagnostic_summary"][0]
    require(terminal["outcome"] in OUTCOMES, "Unknown diagnostic terminal outcome")
    number(terminal, "steps", integer=True, nonnegative=True)
    for field in ("time", "goal_distance"):
        number(terminal, field, nonnegative=True)
    prepare_state(terminal)
    require(near(terminal["time"], terminal["steps"] * .2) and terminal["steps"] <= 250, "Invalid diagnostic terminal time")
    require(near(math.hypot(250 - terminal["x"], terminal["y"]), terminal["goal_distance"], 5e-5, 2e-7), "Diagnostic terminal distance differs from final state")
    require(len(diag_groups) == terminal["steps"] + (terminal["outcome"] == "no_feasible_primary"), "Diagnostic terminal summary does not match the logged sequence")
    if terminal["outcome"] == "goal":
        require(terminal["goal_distance"] < 15 and abs(terminal["z"] - 20) < 3, "Diagnostic goal outcome violates goal tolerance")
    if terminal["outcome"] == "timeout":
        require(terminal["steps"] == 250, "Premature diagnostic timeout")
    data["collapsed"] = collapsed
    data["inputs"] = inputs
    return data


def distribution(values):
    a = np.asarray(list(values), dtype=float)
    require(np.all(np.isfinite(a)), "Nonfinite numeric value reached a distribution")
    if not len(a):
        return {"n": 0, "mean": None, "median": None, "p95": None, "p99": None, "min": None, "max": None}
    return {"n": len(a), "mean": float(a.mean()), "median": float(np.median(a)), "p95": float(np.quantile(a, .95)), "p99": float(np.quantile(a, .99)), "min": float(a.min()), "max": float(a.max())}


def episode_summary(rows):
    counts = Counter(r["outcome"] for r in rows)
    return {"n": len(rows), "outcomes": {outcome: counts[outcome] for outcome in OUTCOMES}, "goal_fraction": counts["goal"] / len(rows), "sim_seconds": distribution(r["sim_seconds"] for r in rows), "final_goal_distance": distribution(r["goal_distance"] for r in rows)}


def grouped_episode_summary(rows, fields):
    return [dict(zip(fields, key), **episode_summary(group)) for key, group in sorted(group_by(rows, fields).items())]


def bootstrap_paired_map(numerators, denominators, seed=12345, repetitions=10000):
    """Percentile interval for a pooled ratio of paired differences.

    Each row is one map. Resample complete maps, retain every condition/state
    within each map, then recompute the ratio. Empty reference-positive samples
    are omitted and their number is reported (never assigned a zero recall).
    """
    numerators = np.asarray(numerators, dtype=float)
    denominators = np.asarray(denominators, dtype=float)
    require(len(numerators) == 24 and len(denominators) == 24, "Bootstrap requires all 24 map clusters")
    require(denominators.sum() > 0, "Cannot estimate recall without reference-positive states")
    draws = np.random.default_rng(seed).integers(0, 24, size=(repetitions, 24))
    den = denominators[draws].sum(axis=1)
    valid = den > 0
    ratios = numerators[draws].sum(axis=1)[valid] / den[valid]
    return {"estimate": float(numerators.sum() / denominators.sum()), "ci95": [float(x) for x in np.quantile(ratios, [.025, .975])], "map_wins": int((numerators > 0).sum()), "map_losses": int((numerators < 0).sum()), "map_ties": int((numerators == 0).sum()), "clusters": 24, "repetitions": repetitions, "seed": seed, "valid_repetitions": int(valid.sum()), "zero_denominator_repetitions": int((~valid).sum()), "unit": "paired map cluster", "interval": "percentile"}


def paired_summary(a, b, reference):
    wins = sum(y["verified"] and not x["verified"] for x, y in zip(a, b))
    losses = sum(x["verified"] and not y["verified"] for x, y in zip(a, b))
    both = [(x, y) for x, y in zip(a, b) if x["verified"] and y["verified"]]
    positives = sum(r["verified"] for r in reference)
    return {"states": len(a), "reference_positive": positives, "diverse_only": wins, "cost_only": losses, "both_verified": len(both), "neither_verified": len(a) - wins - losses - len(both), "recall_difference_diverse_minus_cost": (wins - losses) / positives if positives else None, "selected_cost_difference_diverse_minus_cost_both_verified": distribution(y["cost"] - x["cost"] for x, y in both), "paired_median_timing_difference_ms_diverse_minus_cost": distribution(y["planning_ms"] - x["planning_ms"] for x, y in zip(a, b)), "paired_median_timing_ratio_diverse_over_cost": distribution(y["planning_ms"] / x["planning_ms"] for x, y in zip(a, b) if x["planning_ms"] > 0), "backup_step_difference_diverse_minus_cost": distribution(y["backup_steps"] - x["backup_steps"] for x, y in zip(a, b))}


def analyze(data):
    episodes, collapsed = data["episodes"], data["collapsed"]
    state_ids = sorted({state_id for state_id, _ in collapsed})
    reference = [collapsed[(state_id, "full203")] for state_id in state_ids]
    positive = sum(r["verified"] for r in reference)
    require(positive > 0, "No full-library-positive states: recall cannot be estimated")
    methods = {}
    for method in MATCHED_METHODS:
        rows = [collapsed[(state_id, method)] for state_id in state_ids]
        good = [r for r in rows if r["verified"]]
        methods[method] = {"states": len(rows), "reference_positive": positive, "verified": len(good), "discovery_recall": len(good) / positive, "planning_ms_state_median": distribution(r["planning_ms"] for r in rows), "primary_steps": distribution(r["primary_steps"] for r in rows), "backup_steps": distribution(r["backup_steps"] for r in rows), "total_integration_steps": distribution(r["primary_steps"] + r["backup_steps"] + r["cached_steps"] for r in rows), "recovery_checked": distribution(r["recovery_checked"] for r in rows), "backup_evaluated": distribution(r["backup_evaluated"] for r in rows), "selected_cost_verified": distribution(r["cost"] for r in good), "selected_cost_excess_over_reference_verified": distribution(r["cost"] - collapsed[(r["state_id"], "full203")]["cost"] for r in good), "work_exhausted_states": sum(r["work_exhausted"] for r in rows), "truncated_rollouts": sum(r["truncated"] for r in rows), "status_counts": dict(Counter(r["status"] for r in rows))}
    pairs = {}
    for label, cost, diverse in (("K12", "cost12", "diverse12"), ("W3600", "cost_w3600", "diverse_w3600"), ("W10800", "cost_w10800", "diverse_w10800")):
        a = [collapsed[(state_id, cost)] for state_id in state_ids]
        b = [collapsed[(state_id, diverse)] for state_id in state_ids]
        pair = paired_summary(a, b, reference)
        numerators, denominators = [], []
        per_map = []
        for scene in SCENES:
            ids = [i for i, r in enumerate(reference) if r["scene"] == scene]
            num = sum(int(b[i]["verified"]) - int(a[i]["verified"]) for i in ids)
            den = sum(reference[i]["verified"] for i in ids)
            numerators.append(num)
            denominators.append(den)
            per_map.append({"scene": scene, "positive": den, "paired_verified_difference": num})
        pair["map_bootstrap"] = bootstrap_paired_map(numerators, denominators)
        pair["by_map"] = per_map
        pairs[label] = pair
    episode_index = {(r["scene"], r["condition"], r["method"]): r for r in episodes}
    wins = losses = 0
    deltas = []
    per_map = []
    for scene in SCENES:
        delta = 0
        for condition in CONDITIONS:
            cost = episode_index[(scene, condition, "cost12")]["outcome"] == "goal"
            diverse = episode_index[(scene, condition, "diverse12")]["outcome"] == "goal"
            wins += diverse and not cost
            losses += cost and not diverse
            delta += int(diverse) - int(cost)
        deltas.append(delta)
        per_map.append({"scene": scene, "paired_goal_difference_across_four_conditions": delta})
    success = {"paired_episodes": 96, "diverse_only_goal": wins, "cost_only_goal": losses, "map_bootstrap": bootstrap_paired_map(deltas, [4] * 24), "by_map": per_map}
    by_family = []
    for family in FAMILIES:
        family_ids = [r["state_id"] for r in reference if r["family"] == family]
        family_positive = sum(collapsed[(i, "full203")]["verified"] for i in family_ids)
        for method in MATCHED_METHODS:
            verified = sum(collapsed[(i, method)]["verified"] for i in family_ids)
            by_family.append({"family": family, "method": method, "states": len(family_ids), "reference_positive": family_positive, "verified": verified, "discovery_recall": verified / family_positive if family_positive else None})
    diagnostic = analyze_diagnostic(data)
    return {"schema_version": 3, "audit": {"passed": True, "episodes": len(episodes), "matched_rows": len(data["matched"]), "matched_states": len(state_ids), "maps": 24, "conditions": list(CONDITIONS), "families": list(FAMILIES), "timing_repeats": 2, "repeat_decision_disagreements": 0, "full_state_mismatches": 0, "work_limit_violations": 0, "reference_contradictions": 0, "cost_comparison_tolerance": {"absolute": 1e-7, "relative": 1e-8}}, "interpretation": {"reference": "Existence of a validated continuation in the same finite primary/backup library at the same input state.", "timing": "Median of two repeated timings at each state/method; percentiles summarize 432 state medians.", "bootstrap": "10,000 paired resamples of 24 maps, seed 12345; conditions and states remain within each resampled map. Percentile intervals describe this generated suite.", "outcomes": "All 480 episodes retained, including collisions, envelope failures, infeasible-primary stops and timeouts.", "cost": "Paired selected-cost differences use only states verified by both compared methods."}, "episode_overall": grouped_episode_summary(episodes, ("method",)), "episode_by_condition": grouped_episode_summary(episodes, ("condition", "method")), "episode_by_family": grouped_episode_summary(episodes, ("family", "method")), "episode_by_family_condition": grouped_episode_summary(episodes, ("family", "condition", "method")), "episode_success_pair": success, "matched": {"methods": methods, "paired_comparisons": pairs, "by_family": by_family, "reference_positive": positive, "reference_negative": len(reference) - positive}, "diagnostic": diagnostic}


def analyze_diagnostic(data):
    index = {(r["step"], r["method"]): r for r in data["diagnostic"]}
    steps = sorted({step for step, _ in index})
    series = [index[(step, "cache120")] for step in steps]
    summary = {"logged_states": len(steps), "first_observed_time": series[0]["time"], "last_observed_time": series[-1]["time"], "initial_goal_distance": math.hypot(250 - series[0]["x"], series[0]["y"]), "last_observed_goal_distance": math.hypot(250 - series[-1]["x"], series[-1]["y"]), "nearest_observed_goal_distance": min(math.hypot(250 - r["x"], r["y"]) for r in series), "methods": {}}
    for method in DIAGNOSTIC_METHODS:
        rows = [index[(step, method)] for step in steps]
        summary["methods"][method] = {"states": len(rows), "verified": sum(r["verified"] for r in rows), "primary_feasible": sum(r["primary_feasible"] for r in rows), "reused": sum(r["reused"] for r in rows), "validated_evidence_fresh_or_reused": sum(r["verified"] or r["reused"] for r in rows), "status_counts": dict(Counter(r["status"] for r in rows)), "rejections_and_truncations": {field: sum(r[field] for r in rows) for field in ("reject_collision", "reject_envelope", "reject_known", "reject_invalid", "truncated")}, "primary_steps": sum(r["primary_steps"] for r in rows), "backup_steps": sum(r["backup_steps"] for r in rows), "cached_steps": sum(r["cached_steps"] for r in rows), "planning_ms": distribution(r["planning_ms"] for r in rows)}
    counters = Counter()
    for step in steps:
        rows = {m: index[(step, m)] for m in DIAGNOSTIC_METHODS}
        known, unrestricted = rows["full120"]["verified"], rows["full_unlimited"]["verified"]
        counters["full120_negative"] += not known
        counters["full_unrestricted_negative"] += not unrestricted
        counters["known_space_exclusion_full_unrestricted_positive_full120_negative"] += unrestricted and not known
        counters["budget_miss_120m"] += known and not rows["diverse120"]["verified"]
        counters["budget_miss_unrestricted"] += unrestricted and not rows["diverse_unlimited"]["verified"]
        counters["cache_verified_while_fresh_k12_negative"] += rows["cache120"]["verified"] and not rows["diverse120"]["verified"]
        counters["cache_reused_while_fresh_k12_negative"] += rows["cache120"]["reused"] and not rows["diverse120"]["verified"]
    summary["counterfactual_counts"] = dict(counters)
    summary["verification_note"] = "verified counts fresh library witnesses; reused counts cached plans that passed revalidation. validated_evidence_fresh_or_reused is their union."
    summary["distance_at_observed_times"] = [{"time": r["time"], "distance": math.hypot(250 - r["x"], r["y"]), "altitude_error": r["z"] - 20, "x": r["x"], "y": r["y"]} for r in series]
    terminal = data.get("diagnostic_summary", [])
    summary["terminal_summary"] = {key: value for key, value in terminal[0].items() if not key.startswith("_")} if terminal else None
    summary["terminal_observation_note"] = "Terminal outcome is supplied in a separate summary." if terminal else "CSV rows precede control execution. Last observed time/distance are not the final state or an inferred terminal outcome."
    return summary


def style_figures():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10, "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 8, "axes.spines.top": False, "axes.spines.right": False, "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.dpi": 300, "figure.dpi": 130, "axes.axisbelow": True})
    return plt


def save_figure(fig, directory, name, artifacts):
    for extension in ("pdf", "png"):
        path = directory / f"{name}.{extension}"
        fig.savefig(path, bbox_inches="tight", bbox_extra_artists=fig.legends + ([fig._suptitle] if fig._suptitle is not None else []), facecolor="white")
        artifacts.append(path.name)


def plot_matched(data, summary, plt, directory, stem, artifacts):
    fig, axes = plt.subplots(2, 2, figsize=(9.1, 6.3), constrained_layout=True)
    methods = summary["matched"]["methods"]
    ax = axes[0, 0]
    groups = (("cost12", "diverse12"), ("cost_w3600", "diverse_w3600"), ("cost_w10800", "diverse_w10800"))
    for i, color, label in ((0, COLORS["cost12"], "Cost schedule"), (1, COLORS["diverse12"], "Diversity schedule")):
        values = [100 * methods[pair[i]]["discovery_recall"] for pair in groups]
        bars = ax.bar(np.arange(3) + (i - .5) * .34, values, width=.34, color=color, label=label)
        for bar, value in zip(bars, values):
            ax.annotate(f"{value:.1f}", (bar.get_x() + bar.get_width() / 2, value), xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7)
    ax.axhline(100, color=COLORS["full203"], ls="--", lw=1, label="Full-library reference")
    ax.set(xticks=range(3), xticklabels=["12 candidates", "3,600 steps", "10,800 steps"], ylabel="Discovery recall (%)", ylim=(0, 113), title=f"a  Discovery among {summary['matched']['reference_positive']} reference positives")
    ax.legend(loc="upper right", frameon=True, facecolor="white", framealpha=.95, fontsize=7)
    ax.grid(axis="y", alpha=.18)
    ax = axes[0, 1]
    for method, marker in (("cost12", "o"), ("diverse12", "s"), ("full203", "^")):
        d = methods[method]["planning_ms_state_median"]
        ax.plot(range(3), [d[key] for key in ("median", "p95", "p99")], marker=marker, lw=1.6, color=COLORS[method], label=LABELS[method])
    ax.set(xticks=range(3), xticklabels=["Median", "P95", "P99"], ylabel="Planning latency (ms)", title="b  Matched timing, two repeats per state")
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", alpha=.18)
    ax.legend(frameon=False)
    ax = axes[1, 0]
    order = ("cost12", "diverse12", "cost_w3600", "diverse_w3600", "cost_w10800", "diverse_w10800", "full203")
    y = np.arange(len(order))
    means = [methods[m]["backup_steps"]["mean"] for m in order]
    ax.barh(y, means, color=[COLORS["full203"] if m == "full203" else COLORS["diverse12"] if m.startswith("diverse") else COLORS["cost12"] for m in order], height=.65)
    ax.set(yticks=y, yticklabels=[LABELS[m] for m in order], xlabel="Mean backup integration steps", title="c  Actual verification work")
    ax.invert_yaxis()
    ax.grid(axis="x", alpha=.18)
    ax = axes[1, 1]
    collapsed = data["collapsed"]
    diffs = [b["cost"] - a["cost"] for (state_id, m), a in collapsed.items() if m == "cost12" for b in [collapsed[(state_id, "diverse12")]] if a["verified"] and b["verified"]]
    if diffs:
        values = np.sort(diffs)
        ax.step(values, np.arange(1, len(values) + 1) / len(values), where="post", color=COLORS["diverse12"], lw=1.8)
        ax.axvline(0, color="#777777", ls="--", lw=.8)
        if np.ptp(values) < 1e-9:
            ax.set_xlim(values[0] - 1, values[0] + 1)
    else:
        ax.text(.5, .5, "No jointly verified states", transform=ax.transAxes, ha="center")
    ax.set(xlabel="Selected cost: diversity − cost schedule", ylabel="Empirical cumulative fraction", ylim=(0, 1.03), title=f"d  Paired K=12 cost, n={len(diffs)}")
    ax.grid(alpha=.18)
    save_figure(fig, directory, f"{stem}_matched", artifacts)
    plt.close(fig)


def plot_outcomes(summary, plt, directory, stem, artifacts):
    fig, axes = plt.subplots(2, 2, figsize=(9.1, 6.2), sharex=True)
    colors = ("#009E73", "#E69F00", "#D55E00", "#CC79A7", "#222222", "#BDBDBD")
    labels = ("Goal", "No feasible primary", "Collision", "Envelope violation", "Invalid dynamics", "Timeout")
    rows = {(r["condition"], r["method"]): r for r in summary["episode_by_condition"]}
    for letter, condition, ax in zip("abcd", CONDITIONS, axes.flat):
        left = np.zeros(5)
        for outcome, color, label in zip(OUTCOMES, colors, labels):
            counts = [rows[(condition, method)]["outcomes"][outcome] for method in EPISODE_METHODS]
            ax.barh(range(5), counts, left=left, color=color, label=label, height=.65)
            for y, (count, start) in enumerate(zip(counts, left)):
                if count >= 2:
                    ax.text(start + count / 2, y, str(count), ha="center", va="center", fontsize=8, color="white" if outcome in ("goal", "collision", "invalid_dynamics") else "#222222")
            left += counts
        ax.set(yticks=range(5), yticklabels=[LABELS[m] for m in EPISODE_METHODS], xlim=(0, 24), xticks=[0, 6, 12, 18, 24], xlabel="Maps (24 per method)", title=f"{letter}  {LABELS[condition]}")
        ax.invert_yaxis()
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, legend_labels, loc="lower center", bbox_to_anchor=(.5, .005), ncol=3, frameon=False)
    fig.tight_layout(rect=(0, .10, 1, 1))
    save_figure(fig, directory, f"{stem}_outcomes", artifacts)
    plt.close(fig)


def plot_paths(data, plt, directory, stem, artifacts):
    from matplotlib.patches import Circle
    from matplotlib.lines import Line2D
    fig, axes = plt.subplots(2, 2, figsize=(9.1, 7.5))
    path_groups = group_by(data["paths"], ("case_id",))
    scenes = group_by(data["scenarios"], ("scene",))
    episodes = {(r["scene"], r["condition"], r["method"]): r for r in data["episodes"]}
    for letter, family, ax in zip("abcd", FAMILIES, axes.flat):
        scene = f"{family}_0"
        for obstacle in scenes[(scene,)]:
            ax.add_patch(Circle((obstacle["x"], obstacle["y"]), obstacle["r"], facecolor="#D5D9DE", edgecolor="#79818B", lw=.65, alpha=.8, linestyle=":" if obstacle["type"] == "sphere" else "-"))
        for method in EPISODE_METHODS:
            episode = episodes[(scene, "nominal", method)]
            rows = path_groups[(episode["case_id"],)]
            ax.plot([r["x"] for r in rows], [r["y"] for r in rows], color=COLORS[method], lw=1.45, alpha=.95, linestyle="--" if method.startswith("primary") else "-")
            final = rows[-1]
            ax.plot(final["x"], final["y"], marker="o" if episode["outcome"] == "goal" else "x", color=COLORS[method], ms=4, mew=1)
        ax.plot(0, 0, marker="o", color="#222222", ms=3)
        ax.plot(250, 0, marker="*", color="#222222", ms=9)
        ax.set(xlabel="x (m)", ylabel="y (m)", title=f"{letter}  {LABELS[family]}, seed 0")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.14)
    handles = [Line2D([0], [0], color=COLORS[m], lw=1.7, label=LABELS[m], linestyle="--" if m.startswith("primary") else "-") for m in EPISODE_METHODS]
    handles += [Line2D([0], [0], color="#222222", marker="o", lw=0, label="Goal endpoint"), Line2D([0], [0], color="#222222", marker="x", lw=0, label="Other endpoint")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .005), ncol=4, frameon=False)
    fig.suptitle("Nominal trajectories: first seed of every frozen map family\nPlan view; spatial-obstacle height is not represented", fontsize=10)
    fig.tight_layout(rect=(0, .10, 1, .94))
    save_figure(fig, directory, f"{stem}_paths", artifacts)
    plt.close(fig)


def plot_diagnostic(data, summary, plt, directory, stem, artifacts):
    fig, axes = plt.subplots(3, 1, figsize=(9.1, 6.6), sharex=True, constrained_layout=True, gridspec_kw={"height_ratios": [1.1, 1.2, 1]})
    rows = [r for r in data["diagnostic"] if r["method"] == "cache120"]
    t = [r["time"] for r in rows]
    distance = [math.hypot(250 - r["x"], r["y"]) for r in rows]
    terminal = summary["diagnostic"]["terminal_summary"]
    if terminal is not None:
        t.append(terminal["time"])
        distance.append(terminal["goal_distance"])
    axes[0].plot(t, distance, color="#333333", lw=1.6)
    if terminal is not None and terminal["outcome"] == "goal":
        axes[0].plot(terminal["time"], terminal["goal_distance"], marker="o", ms=4, color="#009E73", label=f"Goal at {terminal['time']:.1f} s")
    axes[0].axhline(15, color="#777777", lw=.8, ls="--", label="Horizontal goal threshold")
    axes[0].set(ylabel="Goal distance (m)", title="a  High-speed cache replay: observed distance over time")
    axes[0].legend(frameon=False)
    axes[0].grid(alpha=.15)
    colors = ("#6F4E7C", "#D55E00", "#009E73", "#E69F00", "#0072B2")
    for i, (method, color) in enumerate(zip(DIAGNOSTIC_METHODS, colors)):
        rr = [r for r in data["diagnostic"] if r["method"] == method]
        axes[1].scatter([r["time"] for r in rr], [i] * len(rr), s=5, color="#DEDEDE", marker="s")
        yes = [r for r in rr if r["verified"] or r["reused"]]
        axes[1].scatter([r["time"] for r in yes], [i] * len(yes), s=7, color=color, marker="s")
    reused = [r for r in rows if r["reused"]]
    axes[1].scatter([r["time"] for r in reused], [0] * len(reused), s=19, facecolors="none", edgecolors="#111111", marker="o", lw=.7, label="Cache reused")
    axes[1].set(yticks=range(5), yticklabels=[LABELS[m] for m in DIAGNOSTIC_METHODS], ylim=(-.6, 4.6), title="b  Continuation evidence at identical observed states (colored = validated)")
    axes[1].invert_yaxis()
    axes[1].legend(loc="upper right", frameon=False)
    full = [r for r in data["diagnostic"] if r["method"] == "full120"]
    for field, color, label in (("reject_collision", "#D55E00", "Collision"), ("reject_envelope", "#CC79A7", "Envelope"), ("reject_known", "#0072B2", "Observation radius")):
        axes[2].plot([r["time"] for r in full], [r[field] for r in full], color=color, lw=1, label=label)
    axes[2].set(xlabel="Observed replay time (s)", ylabel="Rejected backup rollouts", title="c  Cache-free full-library rejection counts with 120 m observation radius")
    axes[2].grid(alpha=.15)
    axes[2].legend(ncol=3, frameon=False)
    save_figure(fig, directory, f"{stem}_diagnostic", artifacts)
    plt.close(fig)


def tex_escape(value):
    return str(value).replace("\\", r"\textbackslash{}").replace("_", r"\_").replace("%", r"\%").replace("&", r"\&")


def write_tables(summary, directory, stem, artifacts):
    paths = []
    rows = [r"\begin{tabular}{llrrrrrr}", r"\toprule", r"Condition & Method & Goal & No primary & Collision & Envelope & Invalid & Timeout \\", r"\midrule"]
    index = {(r["condition"], r["method"]): r for r in summary["episode_by_condition"]}
    for i, condition in enumerate(CONDITIONS):
        if i:
            rows.append(r"\midrule")
        for j, method in enumerate(EPISODE_METHODS):
            item = index[(condition, method)]
            rows.append(" & ".join([LABELS[condition] if j == 0 else "", LABELS[method]] + [str(item["outcomes"][outcome]) for outcome in OUTCOMES]) + r" \\")
    rows += [r"\bottomrule", r"\end{tabular}"]
    paths.append(("outcomes_table", rows))
    rows = [r"\begin{tabular}{lrrrrrr}", r"\toprule", r"Method & Verified & Recall (\%) & Median (ms) & P95 (ms) & P99 (ms) & Mean backup steps \\", r"\midrule"]
    for method in MATCHED_METHODS:
        item = summary["matched"]["methods"][method]
        timing = item["planning_ms_state_median"]
        rows.append(" & ".join([LABELS[method], str(item["verified"]), f"{100 * item['discovery_recall']:.1f}", *[f"{timing[key]:.2f}" for key in ("median", "p95", "p99")], f"{item['backup_steps']['mean']:.0f}"]) + r" \\")
    rows += [r"\bottomrule", r"\end{tabular}"]
    paths.append(("matched_table", rows))
    rows = [r"\begin{tabular}{lrrr}", r"\toprule", r"Paired comparison & Diversity-only & Cost-only & Difference, pp [95\% map interval] \\", r"\midrule"]
    for name in ("K12", "W3600", "W10800"):
        pair = summary["matched"]["paired_comparisons"][name]
        boot = pair["map_bootstrap"]
        low, high = boot["ci95"]
        rows.append(f"Recall, {name} & {pair['diverse_only']} & {pair['cost_only']} & {100 * boot['estimate']:.2f} [{100 * low:.2f}, {100 * high:.2f}]" + r" \\")
    pair = summary["episode_success_pair"]
    boot = pair["map_bootstrap"]
    low, high = boot["ci95"]
    rows.append(f"Goal success, K12 & {pair['diverse_only_goal']} & {pair['cost_only_goal']} & {100 * boot['estimate']:.2f} [{100 * low:.2f}, {100 * high:.2f}]" + r" \\")
    rows += [r"\bottomrule", r"\end{tabular}"]
    paths.append(("paired_table", rows))
    rows = [r"\begin{tabular}{lrrrrrr}", r"\toprule", r"Diagnostic method & Fresh & Reused & Collision & Envelope & Radius & Invalid \\", r"\midrule"]
    for method in DIAGNOSTIC_METHODS:
        item = summary["diagnostic"]["methods"][method]
        rejects = item["rejections_and_truncations"]
        rows.append(" & ".join([LABELS[method], str(item["verified"]), str(item["reused"])] + [str(rejects[key]) for key in ("reject_collision", "reject_envelope", "reject_known", "reject_invalid")]) + r" \\")
    rows += [r"\bottomrule", r"\end{tabular}"]
    paths.append(("diagnostic_table", rows))
    for suffix, rows in paths:
        path = directory / f"{stem}_{suffix}.tex"
        path.write_text("% Generated by analyze_recovery_v3.py from audited observations.\n" + "\n".join(rows) + "\n")
        artifacts.append(path.name)
    macros = {"VThreeEpisodes": summary["audit"]["episodes"], "VThreeMatchedStates": summary["audit"]["matched_states"], "VThreeReferencePositive": summary["matched"]["reference_positive"], "VThreeReferenceNegative": summary["matched"]["reference_negative"]}
    for label, method in (("Cost", "cost12"), ("Diverse", "diverse12"), ("Full", "full203")):
        item = summary["matched"]["methods"][method]
        macros[f"VThree{label}Verified"] = item["verified"]
        macros[f"VThree{label}Recall"] = f"{100 * item['discovery_recall']:.1f}"
        macros[f"VThree{label}MedianMs"] = f"{item['planning_ms_state_median']['median']:.2f}"
    path = directory / f"{stem}_numbers.tex"
    path.write_text("% Generated values; recall macros are percentages without a percent sign.\n" + "\n".join("\\newcommand{\\" + name + "}{" + str(value) + "}" for name, value in macros.items()) + "\n")
    artifacts.append(path.name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("prefix", type=Path, help="CSV path prefix, without _episodes.csv")
    parser.add_argument("--output-dir", type=Path, default=None, help="Report directory; defaults to prefix parent")
    parser.add_argument("--audit-only", action="store_true", help="Validate complete inputs and exit without report artifacts")
    args = parser.parse_args()
    try:
        data = audit(args.prefix)
        if args.audit_only:
            print("PASS: 480 episodes, 432 matched states × 7 methods × 2 identical-decision repeats; complete state, budgets and references consistent.")
            return 0
        summary = analyze(data)
        output = args.output_dir or args.prefix.parent
        output.mkdir(parents=True, exist_ok=True)
        stem = args.prefix.name
        artifacts = []
        plt = style_figures()
        plot_matched(data, summary, plt, output, stem, artifacts)
        plot_outcomes(summary, plt, output, stem, artifacts)
        plot_paths(data, plt, output, stem, artifacts)
        plot_diagnostic(data, summary, plt, output, stem, artifacts)
        write_tables(summary, output, stem, artifacts)
        summary["provenance"] = {"inputs": [{"file": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()} for path in data["inputs"].values()], "analysis_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "numpy_version": np.__version__, "matplotlib_version": __import__("matplotlib").__version__}
        json_path = output / f"{stem}_analysis.json"
        summary["artifacts"] = artifacts + [json_path.name]
        json_path.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(f"PASS: audit complete. Wrote {len(summary['artifacts'])} artifacts to {output.resolve()}")
        print(f"Reference-positive states: {summary['matched']['reference_positive']}/432")
        for method in ("cost12", "diverse12", "full203"):
            item = summary["matched"]["methods"][method]
            print(f"{method}: {item['verified']} verified, recall {100 * item['discovery_recall']:.2f}%, median {item['planning_ms_state_median']['median']:.3f} ms")
        return 0
    except (AuditError, OSError, ValueError) as exc:
        print(f"V3 analysis refused: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
