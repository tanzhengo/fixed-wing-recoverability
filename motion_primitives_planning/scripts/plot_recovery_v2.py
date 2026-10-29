#!/usr/bin/env python3
"""Plot v2 CSV evidence without pooling robustness trials or timing repeats.

Reads PREFIX_{episodes,plans,paths,scenarios,matched}.csv. Writes path panels,
outcome/reference-recall panels (PNG and PDF), and an auditable JSON summary.
Requires matplotlib. --allow-incomplete is for explicitly labelled drafts only.
"""

import argparse
import collections
import csv
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch


VARIANTS = ["baseline", "long6", "cost12_h3", "cost203_h3", "diverse12_h3",
            "cost12_h6", "cost203_h6", "diverse12_h6", "diverse12_h6_cache"]
LABELS = {
    "baseline": "Baseline, 3 s", "long6": "Baseline, 6 s",
    "cost12_h3": "Cost 12, 3 + 3 s", "cost203_h3": "Cost 203, 3 + 3 s",
    "diverse12_h3": "Diverse 12, 3 + 3 s", "cost12_h6": "Cost 12, 3 + 6 s",
    "cost203_h6": "Cost 203, 3 + 6 s", "diverse12_h6": "Diverse 12, 3 + 6 s (cache off; default)",
    "diverse12_h6_cache": "Diverse 12 + saved remainder, 3 + 6 s (opt-in)",
}
PATH_VARIANTS = ["baseline", "cost12_h3", "diverse12_h3", "cost203_h6", "diverse12_h6", "diverse12_h6_cache"]
PATH_STYLE = {
    "baseline": ("#323b46", (0, (6, 2))),
    "cost12_h3": ("#d47c13", (0, (2, 2))),
    "diverse12_h3": ("#1769aa", "-"),
    "cost203_h6": ("#288a59", (0, (7, 2, 2, 2))),
    "diverse12_h6": ("#138f95", (0, (4, 2))),
    "diverse12_h6_cache": ("#9146b0", "-"),
}
OUTCOME_COLORS = {
    "goal": "#27865f", "no_feasible_primary": "#db9c46",
    "timeout": "#acb3bb", "invalid_dynamics": "#9b4c38", "collision": "#c3393f",
}


def read_csv(path, required):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(required) - set(reader.fieldnames or [])
        if missing:
            raise ValueError("{} missing columns: {}".format(path, ", ".join(sorted(missing))))
        rows = list(reader)
    if any(None in row or any(row.get(k) in (None, "") for k in required) for row in rows):
        raise ValueError("{} contains missing or extra CSV fields".format(path))
    return rows


def f(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError("Non-finite {}={!r}".format(key, row[key]))
    return value


def b(row, key):
    value = str(row[key]).lower()
    if value in ("1", "true"):
        return True
    if value in ("0", "false"):
        return False
    raise ValueError("Invalid boolean {}={!r}".format(key, row[key]))


def main_cases(episodes):
    return [row for row in episodes if row["split"] in ("development", "validation")
            and abs(f(row, "speed") - 15.5) < 1e-9 and f(row, "known_space_radius") == 0]


def cache_robustness(episodes):
    compared = {}
    conditions = []
    for name in ("diverse12_h6", "diverse12_h6_cache"):
        rows = [r for r in episodes if r["split"] == "robustness" and r["variant"] == name]
        if not rows:
            return {}, "Saved-remainder reuse is opt-in; these main-set paths do not establish uniform improvement."
        outcomes = collections.Counter(r["outcome"] for r in rows)
        compared[name] = {"episodes": len(rows), "outcomes": dict(outcomes)}
        conditions.append({(r["scenario"], f(r, "speed"), f(r, "known_space_radius"), f(r, "initial_y")) for r in rows})
    if conditions[0] != conditions[1]:
        raise ValueError("Cache/no-cache robustness conditions differ")
    off, on = compared["diverse12_h6"], compared["diverse12_h6_cache"]
    text = ("Speed/radius checks: cache off {}/{} goals; opt-in saved remainder {}/{} goals. "
            "Reuse is not uniformly better.").format(
                off["outcomes"].get("goal", 0), off["episodes"], on["outcomes"].get("goal", 0), on["episodes"])
    return compared, text


def plot_paths(episodes, paths, geometry, output, incomplete):
    selected = [row for row in main_cases(episodes)
                if f(row, "initial_y") == -2 and row["variant"] in PATH_VARIANTS]
    cases = {row["case_id"]: row for row in selected}
    scenes = list(dict.fromkeys(row["scenario"] for row in main_cases(episodes)))
    if not scenes:
        raise ValueError("No main-set scenarios for path panels")
    by_case = collections.defaultdict(list)
    for row in paths:
        if row["case_id"] in cases:
            by_case[row["case_id"]].append(row)
    scene_cases = collections.defaultdict(dict)
    for row in selected:
        if row["variant"] in scene_cases[row["scenario"]]:
            raise ValueError("Multiple selected cases for {} / {}".format(row["scenario"], row["variant"]))
        if not by_case[row["case_id"]]:
            raise ValueError("No path for {}".format(row["case_id"]))
        scene_cases[row["scenario"]][row["variant"]] = row
    obstacles = collections.defaultdict(list)
    for row in geometry:
        obstacles[row["scenario"]].append(row)
    # All geometry and bounds come from the CSVs; no hand-drawn scene obstacles.
    xy = [(f(row, "x"), f(row, "y")) for values in by_case.values() for row in values]
    for scene in scenes:
        for obs in obstacles[scene]:
            radius = f(obs, "inflated_r")
            xy.extend([(f(obs, "x") - radius, f(obs, "y") - radius),
                       (f(obs, "x") + radius, f(obs, "y") + radius)])
    xmin, xmax = min(x for x, _ in xy) - 10, max(x for x, _ in xy) + 10
    ymin, ymax = min(y for _, y in xy) - 12, max(y for _, y in xy) + 12
    if ymax - ymin < 60:
        middle = (ymin + ymax) / 2
        ymin, ymax = middle - 30, middle + 30
    nrows = math.ceil(len(scenes) / 2)
    fig, axes = plt.subplots(nrows, 2, figsize=(14, nrows * 3.45 + 1.8), squeeze=False)
    for ax, scene in zip(axes.flat, scenes):
        for obs in obstacles[scene]:
            center = (f(obs, "x"), f(obs, "y"))
            ax.add_patch(Circle(center, f(obs, "inflated_r"), facecolor="#dfe4e8",
                                edgecolor="#a9b2ba", linewidth=.7, linestyle=":"))
            ax.add_patch(Circle(center, f(obs, "r"), facecolor="#b6bec6",
                                edgecolor="#77828e", linewidth=.8))
        for variant in PATH_VARIANTS:
            case = scene_cases[scene].get(variant)
            if case is None:
                if not incomplete:
                    raise ValueError("Missing selected case {} / {}".format(scene, variant))
                continue
            trajectory = sorted(by_case[case["case_id"]], key=lambda row: (f(row, "step"), f(row, "time")))
            xs, ys = [f(r, "x") for r in trajectory], [f(r, "y") for r in trajectory]
            color, style = PATH_STYLE[variant]
            ax.plot(xs, ys, color=color, linestyle=style, linewidth=1.8, alpha=.93)
            ax.plot(xs[-1], ys[-1], marker="o" if case["outcome"] == "goal" else "x",
                    markersize=5.5, color=color, markeredgewidth=1.5)
        row = next(iter(scene_cases[scene].values()), None)
        split = row["split"] if row else "unknown split"
        ax.set_title("{}  ·  {}".format(scene.replace("_", " "), split), loc="left", fontsize=11, pad=7)
        ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.grid(True, alpha=.19, linewidth=.6)
        ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
    for ax in list(axes.flat)[len(scenes):]:
        ax.set_visible(False)
    handles = [Line2D([], [], color=PATH_STYLE[v][0], linestyle=PATH_STYLE[v][1], linewidth=2, label=LABELS[v])
               for v in PATH_VARIANTS if any(r["variant"] == v for r in selected)]
    handles += [Patch(facecolor="#b6bec6", edgecolor="#77828e", label="Obstacle"),
                Patch(facecolor="#dfe4e8", edgecolor="#a9b2ba", linestyle=":", label="Inflated geometry"),
                Line2D([], [], color="#444444", marker="o", linestyle="None", label="Goal reached"),
                Line2D([], [], color="#444444", marker="x", linestyle="None", label="Other termination")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .06), ncol=3, frameon=False, fontsize=9)
    draft = "DRAFT — incomplete variant set\n" if incomplete else ""
    fig.suptitle(draft + "Fixed-wing finite-continuation trajectories", fontsize=17, y=.985)
    fig.text(.5, .94 if not incomplete else .923,
             "One deterministic episode per method and scene · initial y = −2 m · V = 15.5 m/s · full known geometry", ha="center", fontsize=10)
    _, robustness_text = cache_robustness(episodes)
    fig.text(.5, .034, robustness_text, ha="center", fontsize=9, color="#56616c")
    fig.text(.5, .013, "Horizontal projection. A saved remainder is revalidated before reuse; finite continuation evidence is not a safety proof.", ha="center", fontsize=9, color="#56616c")
    fig.subplots_adjust(top=.89 if not incomplete else .87, bottom=.21, hspace=.38, wspace=.14)
    save_figure(fig, output, "paths")
    return {scene: {v: {"case_id": row["case_id"], "outcome": row["outcome"]}
                    for v, row in scene_cases[scene].items()} for scene in scenes}


def matched_recall(matched):
    grouped = collections.defaultdict(lambda: collections.defaultdict(list))
    for row in matched:
        if f(row, "speed") != 15.5 or f(row, "known_space_radius") != 0:
            continue
        key = (row["state_id"], f(row, "backup_horizon"), f(row, "speed"), f(row, "known_space_radius"))
        grouped[key][row["variant"]].append(row)
    result = collections.defaultdict(lambda: {"inputs": 0, "reference_positive": 0, "hits": collections.Counter()})
    for key, variants in grouped.items():
        if set(variants) != {"cost12", "cost203", "diverse12"}:
            raise ValueError("Incomplete or unexpected matched variants at {}".format(key))
        verified = {}
        for name, rows in variants.items():
            repeat_ids = [row["repeat"] for row in rows]
            if len(repeat_ids) != len(set(repeat_ids)):
                raise ValueError("Duplicate matched repeat at {} / {}".format(key, name))
            outcomes = {b(row, "verified") for row in rows}
            if len(outcomes) != 1:
                raise ValueError("Nonrepeatable matched verification at {} / {}".format(key, name))
            verified[name] = next(iter(outcomes))
        if not verified["cost203"] and any(verified[name] for name in ("cost12", "diverse12")):
            raise ValueError("Finite-bank reference mismatch at {}".format(key))
        item = result[key[1]]; item["inputs"] += 1
        if verified["cost203"]:
            item["reference_positive"] += 1
            for name, passed in verified.items():
                item["hits"][name] += passed
    if not result:
        raise ValueError("No matched inputs at V=15.5 and radius=0")
    return dict(result)


def plot_outcomes(episodes, matched, output, incomplete):
    main = main_cases(episodes)
    outcomes = collections.defaultdict(collections.Counter)
    for row in main:
        outcomes[row["variant"]][row["outcome"]] += 1
    unexpected = set(outcomes) - set(VARIANTS)
    if unexpected:
        raise ValueError("Unknown main variants: {}".format(sorted(unexpected)))
    missing = set(VARIANTS) - set(outcomes)
    if missing and not incomplete:
        raise ValueError("Missing main variants: {} (use --allow-incomplete for a draft)".format(sorted(missing)))
    names = [name for name in VARIANTS if name in outcomes]
    counts = [sum(outcomes[name].values()) for name in names]
    if len(set(counts)) != 1:
        raise ValueError("Unequal main-set episode counts: {}".format(dict(zip(names, counts))))
    recall = matched_recall(matched)
    fig, axes = plt.subplots(1, 2, figsize=(16, 8.2), gridspec_kw={"width_ratios": [1.5, 1]})
    ax = axes[0]
    left = [0] * len(names)
    statuses = [name for name in OUTCOME_COLORS if any(outcomes[v][name] for v in names)]
    unknown_status = set(itertools_flatten(counter.keys() for counter in outcomes.values())) - set(OUTCOME_COLORS)
    if unknown_status:
        raise ValueError("Unknown outcomes: {}".format(sorted(unknown_status)))
    for status in statuses:
        values = [outcomes[name][status] for name in names]
        ax.barh(range(len(names)), values, left=left, color=OUTCOME_COLORS[status], height=.72,
                label=status.replace("_", " "))
        for i, (offset, count) in enumerate(zip(left, values)):
            if count:
                ax.text(offset + count/2, i, str(count), ha="center", va="center", fontsize=9,
                        color="white" if status in ("goal", "invalid_dynamics", "collision") else "#202b34")
        left = [offset + count for offset, count in zip(left, values)]
    ax.set_yticks(range(len(names))); ax.set_yticklabels([LABELS[name] for name in names], fontsize=10)
    ax.invert_yaxis(); ax.set_xlim(0, max(counts))
    ax.set_xlabel("Episodes (counts include all failures)")
    ax.set_title("Main-set outcomes\n{} deterministic episodes per method".format(counts[0]), loc="left", fontsize=13, pad=12)
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.11), ncol=2, frameon=False, fontsize=9)
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
    ax.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax.grid(axis="x", alpha=.17); ax.set_axisbelow(True)
    ax = axes[1]
    methods = [("cost12", "Cost 12", "#d47c13"), ("diverse12", "Diverse 12", "#1769aa"), ("cost203", "Cost 203 reference", "#288a59")]
    horizons = sorted(recall)
    for i, (name, label, color) in enumerate(methods):
        positions, values = [], []
        for j, h in enumerate(horizons):
            positions.append(j + (i-1) * .24)
            item = recall[h]; denominator = item["reference_positive"]
            values.append(item["hits"][name] / denominator if denominator else 0)
        ax.bar(positions, values, width=.22, color=color, label=label)
        for x, value, h in zip(positions, values, horizons):
            item = recall[h]; denom = item["reference_positive"]
            text = "{}/{}".format(item["hits"][name], denom) if denom else "undefined"
            ax.text(x, value + .025, text, ha="center", fontsize=9)
    ax.set_xticks(range(len(horizons)))
    ax.set_xticklabels(["{} s continuation\n{} unique inputs".format(int(h), recall[h]["inputs"]) for h in horizons], fontsize=10)
    ax.set_ylim(0, 1.15); ax.set_yticks([0, .25, .5, .75, 1])
    ax.set_ylabel("Evidence found / reference-positive inputs")
    ax.set_title("Matched-input evidence recall\nRelative to the finite cost-203 library", loc="left", fontsize=13, pad=12)
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.11), frameon=False, fontsize=9)
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
    ax.grid(axis="y", alpha=.17); ax.set_axisbelow(True)
    draft = "DRAFT — incomplete variant set\n" if incomplete else ""
    fig.suptitle(draft + "Task completion and finite-library evidence", fontsize=17, y=.97)
    robustness, robustness_text = cache_robustness(episodes)
    fig.text(.5, .12, robustness_text, ha="center", fontsize=10, color="#7e3b34")
    fig.text(.5, .088, "Robustness episodes are excluded from the bars. Timing repeats count once per input; no statistical confidence claim is made.", ha="center", fontsize=9, color="#56616c")
    fig.text(.5, .055, "Saved-remainder policy is absent from matched tests, which have no saved remainder. Cost 203 is a finite-library reference, not a safety truth.", ha="center", fontsize=9, color="#56616c")
    fig.text(.5, .022, "Baseline and long6 retain original validity limits; recovery variants also enforce the configured flight envelope.", ha="center", fontsize=9, color="#56616c")
    fig.subplots_adjust(left=.29, right=.98, top=.83 if incomplete else .86, bottom=.29, wspace=.5)
    save_figure(fig, output, "outcomes_recall")
    return {"main_outcomes": {name: dict(outcomes[name]) for name in names}, "cache_robustness": robustness,
            "matched": {str(h): {"unique_inputs": item["inputs"], "reference_positive_inputs": item["reference_positive"],
                                  "evidence_hits": dict(item["hits"])} for h, item in recall.items()}}


def itertools_flatten(values):
    return [item for group in values for item in group]


def save_figure(fig, output, suffix):
    for extension in ("png", "pdf"):
        fig.savefig(str(output) + "_" + suffix + "." + extension, dpi=180, facecolor="white")
    plt.close(fig)


def cache_summary(episodes, plans):
    case_map = {row["case_id"]: row for row in episodes}
    result = collections.defaultdict(lambda: {"planning_cycles": 0, "reused_verified_plan_cycles": 0, "cached_integration_steps": 0})
    available = bool(plans) and "reused_verified_plan" in plans[0] and "cached_integration_steps" in plans[0]
    if not available:
        return {"available": False}
    for row in plans:
        case = case_map[row["case_id"]]
        key = "{} / {}".format(case["split"], case["variant"])
        item = result[key]; item["planning_cycles"] += 1
        item["reused_verified_plan_cycles"] += b(row, "reused_verified_plan")
        steps = f(row, "cached_integration_steps")
        if steps < 0 or int(steps) != steps:
            raise ValueError("Invalid cached_integration_steps")
        item["cached_integration_steps"] += int(steps)
    return {"available": True, "by_split_variant": dict(result)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--output-prefix", type=Path, help="Default: PREFIX_plot")
    parser.add_argument("--allow-incomplete", action="store_true", help="Label figures as drafts and allow missing variants")
    args = parser.parse_args()
    try:
        required = {
            "episodes": {"case_id", "split", "scenario", "variant", "initial_y", "speed", "known_space_radius", "outcome"},
            "plans": {"case_id", "step"}, "paths": {"case_id", "step", "time", "x", "y", "z"},
            "scenarios": {"scenario", "x", "y", "r", "inflated_r", "z", "height"},
            "matched": {"state_id", "backup_horizon", "speed", "known_space_radius", "variant", "repeat", "verified"},
        }
        data = {name: read_csv(Path(str(args.prefix) + "_" + name + ".csv"), cols) for name, cols in required.items()}
        ids = [row["case_id"] for row in data["episodes"]]
        if len(set(ids)) != len(ids):
            raise ValueError("Duplicate episode case_id")
        for name in ("plans", "paths"):
            if any(row["case_id"] not in set(ids) for row in data[name]):
                raise ValueError("{} references unknown episode case_id".format(name))
        output = args.output_prefix or Path(str(args.prefix) + "_plot")
        output.parent.mkdir(parents=True, exist_ok=True)
        plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "pdf.fonttype": 42})
        summary = {"draft": args.allow_incomplete,
                   "scope": "Deterministic same-model experiments. Main-set plots exclude robustness; timing repeats are not independent inputs.",
                   "paths": plot_paths(data["episodes"], data["paths"], data["scenarios"], output, args.allow_incomplete),
                   "comparison": plot_outcomes(data["episodes"], data["matched"], output, args.allow_incomplete),
                   "cache": cache_summary(data["episodes"], data["plans"])}
        Path(str(output) + "_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print("Wrote {}_paths.{{png,pdf}}, {}_outcomes_recall.{{png,pdf}}, and plot summary.".format(output, output))
        return 0
    except (OSError, ValueError) as error:
        print("Plot error: {}".format(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
