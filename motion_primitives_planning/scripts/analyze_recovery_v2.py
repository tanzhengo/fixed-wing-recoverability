#!/usr/bin/env python3
"""Audit and summarize the v2 finite-continuation experiment CSVs.

Usage: python3 analyze_recovery_v2.py /path/to/prefix
Reads prefix_{episodes,plans,matched}.csv and writes prefix_analysis.{json,md}.
No third-party Python packages are required. Missing columns, duplicate records,
and invalid numeric data are errors. Input/decision inconsistencies are reported
in the outputs and cause exit status 1; CSV/schema errors cause exit status 2.

Matched-state cost203 is a reference search over a finite primitive library,
not ground truth for physical recoverability or a safety certificate. A repeated
timing measurement is not an independent task or an independent test state.
"""

import argparse
import collections
import csv
import itertools
import json
import math
from pathlib import Path
import statistics
import sys


EPISODE_COLUMNS = {
    "case_id", "split", "scenario", "variant", "primary_horizon",
    "backup_horizon", "max_candidates", "speed", "known_space_radius",
    "initial_y", "outcome", "steps", "sim_seconds", "goal_distance",
}
PLAN_COLUMNS = {
    "case_id", "step", "planning_ms", "status", "primary_evaluated",
    "primary_feasible_count", "recovery_checked", "backup_evaluated",
    "verified", "budget_exhausted", "primary_integration_steps",
    "backup_integration_steps",
}
MATCHED_COLUMNS = {
    "state_id", "scenario", "split", "variant", "backup_horizon", "speed",
    "known_space_radius", "repeat", "planning_ms", "verified", "status",
    "recovery_checked", "backup_evaluated", "cost", "selected_cost_rank",
    "primary_integration_steps", "backup_integration_steps",
}
MATCHED_VARIANTS = {"cost12", "cost203", "diverse12"}
COUNTERS = (
    "primary_evaluated", "primary_feasible_count", "recovery_checked",
    "backup_evaluated", "primary_integration_steps", "backup_integration_steps",
)


def number(row, key, minimum=None, finite=True):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        raise ValueError("{}: invalid {}={!r}".format(row.get("_source", "row"), key, row.get(key)))
    if (finite and not math.isfinite(value)) or (minimum is not None and value < minimum):
        raise ValueError("{}: {} must be {}{}".format(
            row["_source"], key, "finite " if finite else "", ">= " + str(minimum) if minimum is not None else ""))
    return value


def integer(row, key, minimum=0):
    value = number(row, key, minimum)
    if value != int(value):
        raise ValueError("{}: {} must be an integer".format(row["_source"], key))
    return int(value)


def boolean(row, key):
    raw = str(row.get(key, "")).lower()
    if raw in ("1", "true"):
        return True
    if raw in ("0", "false"):
        return False
    raise ValueError("{}: invalid boolean {}={!r}".format(row["_source"], key, row.get(key)))


def read_rows(path, required):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(required - set(reader.fieldnames or ()))
        if missing:
            raise ValueError("{}: missing columns {}".format(path, ", ".join(missing)))
        if len(reader.fieldnames) != len(set(reader.fieldnames)):
            raise ValueError("{}: duplicate column names".format(path))
        rows = []
        for line, raw in enumerate(reader, 2):
            if None in raw or any(raw.get(key) is None or raw.get(key) == "" for key in required):
                raise ValueError("{}:{}: missing or extra CSV fields".format(path, line))
            row = dict(raw, _source="{}:{}".format(path, line))
            rows.append(row)
    if not rows:
        raise ValueError("{}: no data rows".format(path))
    return rows


def quantile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    pos = (len(ordered) - 1) * fraction
    lo, hi = math.floor(pos), math.ceil(pos)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def distribution(values):
    values = list(values)
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None, "p99": None, "max": None}
    return {"count": len(values), "mean": statistics.mean(values),
            "median": statistics.median(values), "p95": quantile(values, .95),
            "p99": quantile(values, .99), "max": max(values)}


def matched_key(row, include_repeat=True):
    # Scenario/split are audited separately to catch accidental state-id reuse.
    key = (row["state_id"], row["backup_horizon"], row["speed"], row["known_space_radius"])
    return key + (row["repeat"],) if include_repeat else key


def prepare(episodes, plans, matched):
    cases = {}
    for row in episodes:
        if row["case_id"] in cases:
            raise ValueError("{}: duplicate case_id {}".format(row["_source"], row["case_id"]))
        for key in ("primary_horizon", "backup_horizon", "speed", "known_space_radius", "sim_seconds", "goal_distance"):
            row[key] = number(row, key, 0)
        row["initial_y"] = number(row, "initial_y")
        for key in ("max_candidates", "steps"):
            row[key] = integer(row, key)
        cases[row["case_id"]] = row
    seen_plans = set()
    for row in plans:
        if row["case_id"] not in cases:
            raise ValueError("{}: unknown case_id {}".format(row["_source"], row["case_id"]))
        row["step"] = integer(row, "step")
        key = (row["case_id"], row["step"])
        if key in seen_plans:
            raise ValueError("{}: duplicate case/step {}".format(row["_source"], key))
        seen_plans.add(key)
        row["planning_ms"] = number(row, "planning_ms", 0)
        for key in COUNTERS:
            row[key] = integer(row, key)
        for key in ("verified", "budget_exhausted"):
            row[key] = boolean(row, key)
    seen_matched = set()
    for row in matched:
        if row["variant"] not in MATCHED_VARIANTS:
            raise ValueError("{}: unexpected matched variant {}".format(row["_source"], row["variant"]))
        for key in ("backup_horizon", "speed", "known_space_radius", "planning_ms"):
            row[key] = number(row, key, 0)
        for key in ("repeat", "recovery_checked", "backup_evaluated", "primary_integration_steps", "backup_integration_steps"):
            row[key] = integer(row, key)
        # A failure may use -1 for an undefined selected rank and inf for cost.
        row["selected_cost_rank"] = integer(row, "selected_cost_rank", -1)
        row["verified"] = boolean(row, "verified")
        row["cost"] = number(row, "cost", finite=row["verified"])
        if not math.isfinite(row["cost"]):
            row["cost"] = None
        key = matched_key(row) + (row["variant"],)
        if key in seen_matched:
            raise ValueError("{}: duplicate matched record {}".format(row["_source"], key))
        seen_matched.add(key)
    return cases


def episode_summary(episodes, plans):
    outcomes = collections.Counter(row["outcome"] for row in episodes)
    times = [row["planning_ms"] for row in plans]
    per_case_times = collections.defaultdict(list)
    for row in plans:
        per_case_times[row["case_id"]].append(row["planning_ms"])
    return {
        "episodes": len(episodes), "outcomes": dict(sorted(outcomes.items())),
        "goal_fraction": outcomes.get("goal", 0) / len(episodes) if episodes else None,
        "planning_ms_pooled_cycles": distribution(times),
        "planning_ms_equal_case_mean": distribution(statistics.mean(v) for v in per_case_times.values()),
        "cycles_over_100ms_reference": sum(value > 100 for value in times),
        "status_counts": dict(sorted(collections.Counter(row["status"] for row in plans).items())),
        "verified_cycles": sum(row["verified"] for row in plans),
        "budget_exhausted_cycles": sum(row["budget_exhausted"] for row in plans),
        "per_cycle_counts": {key: distribution(row[key] for row in plans) for key in COUNTERS},
    }


def summarize_episodes(episodes, plans, fields):
    grouped = collections.defaultdict(list)
    for row in episodes:
        grouped[tuple(row[key] for key in fields)].append(row)
    plans_by_case = collections.defaultdict(list)
    for row in plans:
        plans_by_case[row["case_id"]].append(row)
    output = []
    for key, rows in sorted(grouped.items()):
        cycles = list(itertools.chain.from_iterable(plans_by_case[row["case_id"]] for row in rows))
        output.append(dict(zip(fields, key), **episode_summary(rows, cycles)))
    return output


def analyze_matched(rows, findings):
    groups = collections.defaultdict(dict)
    repeated = collections.defaultdict(list)
    for row in rows:
        groups[matched_key(row)][row["variant"]] = row
        repeated[matched_key(row, False) + (row["variant"],)].append(row)
    # Decisions must be repeatable when only wall-clock timing is being repeated.
    for key, repeated_rows in repeated.items():
        signatures = {(r["verified"], r["status"], r["selected_cost_rank"], r["cost"],
                       r["recovery_checked"], r["backup_evaluated"],
                       r["primary_integration_steps"], r["backup_integration_steps"])
                      for r in repeated_rows}
        if len(signatures) != 1:
            findings.append({"code": "nonrepeatable_decision", "key": list(key),
                             "message": "Repeated timing runs produced different decisions or work counts."})
    for key, variants in groups.items():
        missing = MATCHED_VARIANTS - set(variants)
        if missing:
            findings.append({"code": "incomplete_matched_group", "key": list(key), "missing": sorted(missing)})
        metadata = {(r["scenario"], r["split"]) for r in variants.values()}
        if len(metadata) > 1:
            findings.append({"code": "matched_metadata_mismatch", "key": list(key)})
        reference = variants.get("cost203")
        if reference is None:
            continue
        for name, row in variants.items():
            if name != "cost203" and row["verified"] and not reference["verified"]:
                findings.append({"code": "finite_bank_reference_mismatch", "key": list(key),
                                 "variant": name, "message": "Limited search found evidence but cost203 did not; check identical inputs, library, and budgets."})
            if name != "cost203" and row["verified"] and reference["verified"] and row["cost"] + 1e-5 < reference["cost"]:
                findings.append({"code": "reference_cost_order_mismatch", "key": list(key), "variant": name,
                                 "message": "Limited search has lower selected cost than exhaustive cost-ordered reference."})
    # Collapse repeated timings to a median per input before computing recall.
    collapsed = {}
    for key, repeated_rows in repeated.items():
        first = repeated_rows[0]
        collapsed[key] = dict(first, planning_ms=statistics.median(r["planning_ms"] for r in repeated_rows),
                              timing_repeats=len(repeated_rows))
    metrics = collections.defaultdict(list)
    for key, row in collapsed.items():
        group = (row["split"], row["scenario"], row["backup_horizon"], row["speed"], row["known_space_radius"], row["variant"])
        reference = collapsed.get(key[:-1] + ("cost203",))
        metrics[group].append((row, reference))
    output = []
    fields = ("split", "scenario", "backup_horizon", "speed", "known_space_radius", "variant")
    for key, pairs in sorted(metrics.items()):
        comparable = [(row, ref) for row, ref in pairs if ref is not None]
        positive = [(row, ref) for row, ref in comparable if ref["verified"]]
        both = [(row, ref) for row, ref in positive if row["verified"]]
        time_ratios = [row["planning_ms"] / ref["planning_ms"] for row, ref in comparable if ref["planning_ms"] > 0]
        step_ratios = []
        for row, ref in comparable:
            reference_steps = ref["primary_integration_steps"] + ref["backup_integration_steps"]
            if reference_steps:
                step_ratios.append((row["primary_integration_steps"] + row["backup_integration_steps"]) / reference_steps)
        output.append(dict(zip(fields, key), **{
            "unique_inputs": len(pairs), "timing_measurements": sum(row["timing_repeats"] for row, _ in pairs),
            "verified_inputs": sum(row["verified"] for row, _ in pairs),
            "reference_verified_inputs": len(positive),
            "finite_bank_reference_recall": len(both) / len(positive) if positive else None,
            "reference_positive_misses": len(positive) - len(both),
            "planning_ms_per_input_median": distribution(row["planning_ms"] for row, _ in pairs),
            "paired_time_ratio_to_cost203": distribution(time_ratios),
            "paired_integration_step_ratio_to_cost203": distribution(step_ratios),
            "selected_cost_excess_when_both_verified": distribution(row["cost"] - ref["cost"] for row, ref in both),
            "per_input_counts": {name: distribution(row[name] for row, _ in pairs)
                                 for name in ("recovery_checked", "backup_evaluated", "primary_integration_steps", "backup_integration_steps")},
        }))
    return output


def format_value(value):
    return "—" if value is None else "{:.4g}".format(value)


def write_markdown(path, result):
    lines = ["# Finite-continuation v2 experiment audit", "", result["scope"], "",
             "Timing repeats are collapsed per matched input. Development/validation are scenario labels; no independent generalization claim is implied.", "",
             "## Closed-loop outcomes", "",
             "| Split | Scenario | Variant | Episodes | Goals | Mean ms, pooled | P95 ms |",
             "|---|---|---|---:|---:|---:|---:|"]
    for row in result["episodes_by_split_scenario_variant"]:
        timing = row["planning_ms_pooled_cycles"]
        lines.append("| {} | {} | {} | {} | {} | {} | {} |".format(
            row["split"], row["scenario"], row["variant"], row["episodes"],
            row["outcomes"].get("goal", 0), format_value(timing["mean"]), format_value(timing["p95"])))
    lines += ["", "The JSON also separates speed, observation radius, and backup horizon and reports all failure outcomes. Pooled cycle times compare different visited states; use the matched table for ranking efficiency.", "",
              "## Matched-input finite-bank reference", "",
              "| Split / scene | Backup s | Speed | Radius | Variant | Inputs | Reference positives | Recall | Median ms | Median step ratio |",
              "|---|---:|---:|---:|---|---:|---:|---:|---:|---:|"]
    for row in result["matched"]:
        lines.append("| {} / {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
            row["split"], row["scenario"], row["backup_horizon"], row["speed"], row["known_space_radius"],
            row["variant"], row["unique_inputs"], row["reference_verified_inputs"],
            format_value(row["finite_bank_reference_recall"]),
            format_value(row["planning_ms_per_input_median"]["median"]),
            format_value(row["paired_integration_step_ratio_to_cost203"]["median"])))
    lines += ["", "Recall is conditional on cost203 finding a finite-library continuation; it is not physical safety recall. Zero reference positives produces an undefined recall, not zero or one.", "",
              "## Audit findings", ""]
    if result["findings"]:
        lines += ["- `" + item["code"] + "`: " + json.dumps(item, ensure_ascii=False, sort_keys=True)
                  for item in result["findings"]]
    else:
        lines.append("No schema, pairing, repeatability, or finite-bank consistency problems detected.")
    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--output-prefix", type=Path, help="Default: input prefix plus _analysis")
    args = parser.parse_args()
    try:
        episodes = read_rows(Path(str(args.prefix) + "_episodes.csv"), EPISODE_COLUMNS)
        plans = read_rows(Path(str(args.prefix) + "_plans.csv"), PLAN_COLUMNS)
        matched = read_rows(Path(str(args.prefix) + "_matched.csv"), MATCHED_COLUMNS)
        cases = prepare(episodes, plans, matched)
        findings = []
        planned_cases = {row["case_id"] for row in plans}
        for case_id in sorted(set(cases) - planned_cases):
            if cases[case_id]["steps"] > 0:
                findings.append({"code": "missing_case_plans", "case_id": case_id})
        result = {
            "scope": "Deterministic model-in-the-loop with shared planning/plant dynamics. Finite sampled continuations, no recursive-feasibility or physical-safety guarantee. Wall-clock planning cost is measured but not inserted as flight delay.",
            "input_files": {suffix: str(Path(str(args.prefix) + "_" + suffix + ".csv"))
                            for suffix in ("episodes", "plans", "matched")},
            "episodes_by_split": summarize_episodes(episodes, plans, ("split",)),
            "episodes_by_variant": summarize_episodes(episodes, plans, ("variant",)),
            "episodes_by_split_scenario_variant": summarize_episodes(episodes, plans, ("split", "scenario", "variant")),
            "episodes_by_condition": summarize_episodes(episodes, plans,
                ("split", "scenario", "variant", "primary_horizon", "backup_horizon", "speed", "known_space_radius")),
            "matched": analyze_matched(matched, findings), "findings": findings,
        }
        output = args.output_prefix or Path(str(args.prefix) + "_analysis")
        json_path, md_path = Path(str(output) + ".json"), Path(str(output) + ".md")
        json_path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
        write_markdown(md_path, result)
        print("Analyzed {} episodes, {} planning cycles, {} matched measurements; {} audit findings.".format(
            len(episodes), len(plans), len(matched), len(findings)))
        print(json_path)
        print(md_path)
        return 1 if findings else 0
    except (OSError, ValueError) as error:
        print("Analysis error: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
