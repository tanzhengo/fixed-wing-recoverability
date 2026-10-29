#!/usr/bin/env python3
"""Create compact manuscript figures from the audited recovery experiment.

Run from any directory: python3 paper/create_result_figures.py
Outputs five fixed-size 7-inch-wide PDF/PNG pairs in paper/figures. Archived
figures in docs/recovery are not modified. All figures use the existing audit
summaries and hash-checked V3 raw observations; no trajectories are selected
based on their outcomes.
"""

import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch
from matplotlib.ticker import FuncFormatter
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "docs" / "recovery"
OUTPUT = ROOT / "paper" / "figures"
METHODS = ("primary3", "primary6", "cost12", "diverse12", "full203")
METHOD_LABELS = {"primary3": "Primary 3 s", "primary6": "Primary 6 s", "cost12": "Cost K12", "diverse12": "Diverse K12", "full203": "Full K203"}
COLORS = {"primary3": "#888888", "primary6": "#79518B", "cost12": "#0072B2", "diverse12": "#D55E00", "full203": "#009E73"}
CONDITIONS = ("nominal", "model_mismatch", "crosswind", "delay100ms")
CONDITION_LABELS = ("Nominal", "Model mismatch", "Crosswind", "100 ms delay")
OUTCOMES = ("goal", "no_feasible_primary", "collision", "envelope_violation", "invalid_dynamics", "timeout")
OUTCOME_LABELS = ("Goal", "No primary", "Margin incursion", "Envelope", "Invalid", "Timeout")
OUTCOME_COLORS = ("#009E73", "#E69F00", "#D55E00", "#CC79A7", "#222222", "#BDBDBD")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_inputs():
    summaries = {version: json.loads((SOURCE / version / f"recovery_{version}_analysis.json").read_text()) for version in ("v3", "v4")}
    for version, summary in summaries.items():
        require(summary["audit"]["passed"], f"{version}: source audit did not pass")
    require(summaries["v4"]["implementation_equivalence"]["passed"], "V4 implementation equivalence must pass before manuscript figures are produced")
    require(summaries["v3"]["audit"]["episodes"] == 480 and summaries["v3"]["matched"]["reference_positive"] == 335, "Unexpected frozen experiment dimensions")
    raw = {}
    hashes = {Path(item["file"]).name: item["sha256"] for item in summaries["v3"]["provenance"]["inputs"]}
    for name in ("episodes", "matched", "paths", "scenarios", "diagnostic", "diagnostic_summary"):
        path = SOURCE / "v3" / f"recovery_v3_{name}.csv"
        require(hashlib.sha256(path.read_bytes()).hexdigest() == hashes.get(path.name), f"{path.name}: data hash does not match the passed audit")
        with path.open(newline="") as stream:
            raw[name] = list(csv.DictReader(stream))
    return summaries["v3"], summaries["v4"], raw


def save(fig, name):
    # Deliberately do not use bbox_inches='tight': the PDF physical dimensions
    # must remain exactly the manuscript's requested 7-inch width.
    for suffix in ("pdf", "png"):
        fig.savefig(OUTPUT / f"{name}.{suffix}", dpi=300, facecolor="white")
    plt.close(fig)


def matched(summary, raw):
    fig, axes = plt.subplots(1, 2, figsize=(7, 2.65))
    fig.subplots_adjust(left=.075, right=.985, bottom=.25, top=.86, wspace=.38)
    ax = axes[0]
    groups = (("cost12", "diverse12"), ("cost_w3600", "diverse_w3600"), ("cost_w10800", "diverse_w10800"))
    stats = summary["matched"]["methods"]
    for i, method in enumerate(("cost12", "diverse12")):
        values = [100 * stats[pair[i]]["discovery_recall"] for pair in groups]
        bars = ax.bar(np.arange(3) + (i - .5) * .34, values, .32, color=COLORS[method])
        for bar, value in zip(bars, values):
            ax.annotate(f"{value:.1f}", (bar.get_x() + bar.get_width() / 2, value), xytext=(0, 8 if value > 90 else 2), textcoords="offset points", ha="center", fontsize=8)
    ax.axhline(100, color=COLORS["full203"], ls="--", lw=1)
    ax.set(xticks=range(3), xticklabels=["K12", "W3,600", "W10,800"], ylabel="Discovery recall (%)", ylim=(0, 113), title="(a) Complete-rollout verification")
    ax.grid(axis="y", alpha=.16)
    ax = axes[1]
    rows = {(row["state_id"], row["method"]): row for row in raw["matched"] if row["repeat"] == "0"}
    differences = []
    for (state, method), row in rows.items():
        if method != "cost12":
            continue
        other = rows[(state, "diverse12")]
        if row["verified"] == other["verified"] == "1":
            differences.append(float(other["cost"]) - float(row["cost"]))
    require(len(differences) == 239, "Jointly verified K12 cost sample differs from audit")
    values = np.sort(differences)
    ax.step(values, np.arange(1, len(values) + 1) / len(values), where="post", color=COLORS["diverse12"], lw=1.6)
    ax.axvline(0, color="#666666", ls="--", lw=.8)
    ax.set(xlabel="Selected cost: diversity − cost", ylabel="Cumulative fraction", ylim=(0, 1.03), title="(b) Paired K12 cost, n = 239")
    ax.grid(alpha=.16)
    handles = [Patch(color=COLORS["cost12"], label="Cost schedule"), Patch(color=COLORS["diverse12"], label="Diversity schedule"), Line2D([0], [0], color=COLORS["full203"], ls="--", label="Full-library reference")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .005), ncol=3, frameon=False)
    save(fig, "matched")


def incremental(summary):
    fig, axes = plt.subplots(1, 2, figsize=(7, 2.7))
    fig.subplots_adjust(left=.075, right=.985, bottom=.31, top=.86, wspace=.40)
    ax = axes[0]
    for vi, version in enumerate(("v3", "v4")):
        for mi, method in enumerate(("cost", "diverse")):
            color = COLORS["cost12" if method == "cost" else "diverse12"]
            values = [100 * summary["versions"][version]["methods"][f"{method}_w{budget}"]["discovery_recall"] for budget in (3600, 10800)]
            bars = ax.bar(np.arange(2) + (vi * 2 + mi - 1.5) * .19, values, .18, color=color, edgecolor=color, alpha=.42 if vi == 0 else 1, hatch="///" if vi == 0 else None, lw=.5)
            for bar, value in zip(bars, values):
                ax.annotate(f"{value:.1f}", (bar.get_x() + bar.get_width() / 2, value), xytext=(0, 2 if mi == 0 else 10), textcoords="offset points", ha="center", fontsize=8)
    ax.set(xticks=[0, 1], xticklabels=["W3,600", "W10,800"], ylabel="Discovery recall (%)", ylim=(0, 119), title="(a) Equal integration limits")
    ax.grid(axis="y", alpha=.16)
    ax = axes[1]
    order = ("cost12", "diverse12", "full203")
    for vi, version in enumerate(("v3", "v4")):
        values = [summary["versions"][version]["methods"][method]["backup_steps"]["mean"] for method in order]
        ax.bar(np.arange(3) + (vi - .5) * .33, values, .31, color=[COLORS[method] for method in order], edgecolor=[COLORS[method] for method in order], alpha=.42 if vi == 0 else 1, hatch="///" if vi == 0 else None, lw=.5)
    ax.set(xticks=range(3), xticklabels=["Cost K12", "Diverse K12", "Full K203"], ylabel="Mean backup steps", title="(b) Identical decisions, integration work")
    ax.yaxis.set_major_formatter(FuncFormatter(lambda x, _: f"{x / 1000:g}k"))
    ax.grid(axis="y", alpha=.16)
    handles = [Patch(color=COLORS[method], label=label) for method, label in (("cost12", "Cost"), ("diverse12", "Diversity"), ("full203", "Full library"))]
    handles += [Patch(facecolor="#888888", edgecolor="#555555", hatch="///", alpha=.42, label="Complete"), Patch(facecolor="#888888", label="Incremental")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .005), ncol=3, frameon=False, columnspacing=1.5)
    save(fig, "incremental")


def outcomes(summary):
    fig, axes = plt.subplots(2, 2, figsize=(7, 3.8), sharex=True)
    fig.subplots_adjust(left=.14, right=.985, bottom=.21, top=.91, wspace=.49, hspace=.65)
    rows = {(row["condition"], row["method"]): row for row in summary["episode_by_condition"]}
    for letter, condition, title, ax in zip("abcd", CONDITIONS, CONDITION_LABELS, axes.flat):
        left = np.zeros(5)
        for outcome, color, label in zip(OUTCOMES, OUTCOME_COLORS, OUTCOME_LABELS):
            counts = [rows[(condition, method)]["outcomes"][outcome] for method in METHODS]
            ax.barh(range(5), counts, left=left, color=color, label=label, height=.66)
            for y, (count, start) in enumerate(zip(counts, left)):
                if count >= 2:
                    ax.text(start + count / 2, y, str(count), ha="center", va="center", fontsize=8, color="white" if outcome in ("goal", "collision", "invalid_dynamics") else "#222222")
            left += counts
        require(all(left == 24), "Outcome bars must retain all 24 maps")
        ax.set(yticks=range(5), yticklabels=[METHOD_LABELS[method] for method in METHODS], xlim=(0, 24), xticks=[0, 6, 12, 18, 24], title=f"({letter}) {title}")
        ax.invert_yaxis()
        if letter in "cd":
            ax.set_xlabel("Maps")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(.5, .005), ncol=3, frameon=False)
    save(fig, "outcomes")


def paths(raw):
    fig, axes = plt.subplots(2, 2, figsize=(7, 4))
    fig.subplots_adjust(left=.085, right=.985, bottom=.21, top=.91, wspace=.31, hspace=.62)
    episode_index = {(row["scene"], row["condition"], row["method"]): row for row in raw["episodes"]}
    path_index = {}
    for row in raw["paths"]:
        path_index.setdefault(row["case_id"], []).append(row)
    for letter, family, title, ax in zip("abcd", ("staggered", "wall", "culdesac", "spatial"), ("Pillars", "Wall", "Cul-de-sac", "Spatial"), axes.flat):
        scene = f"{family}_0"
        for obstacle in raw["scenarios"]:
            if obstacle["scene"] == scene:
                ax.add_patch(Circle((float(obstacle["x"]), float(obstacle["y"])), float(obstacle["r"]), facecolor="#D6DBDF", edgecolor="#8B949E", lw=.65, linestyle=":" if obstacle["type"] == "sphere" else "-", alpha=.85))
        for method in METHODS:
            episode = episode_index[(scene, "nominal", method)]
            rows = path_index[episode["case_id"]]
            ax.plot([float(row["x"]) for row in rows], [float(row["y"]) for row in rows], color=COLORS[method], lw=1.35, ls="--" if method.startswith("primary") else "-")
            ax.plot(float(rows[-1]["x"]), float(rows[-1]["y"]), marker="o" if episode["outcome"] == "goal" else "x", color=COLORS[method], ms=3.5, mew=.9)
        ax.plot(0, 0, "o", color="#222222", ms=2.5)
        ax.plot(250, 0, "*", color="#222222", ms=7)
        ax.set(xlabel="x (m)", ylabel="y (m)", title=f"({letter}) {title}", xticks=[0, 100, 200], yticks=[-50, 0, 50])
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=.14)
    handles = [Line2D([0], [0], color=COLORS[method], lw=1.4, ls="--" if method.startswith("primary") else "-", label=METHOD_LABELS[method]) for method in METHODS]
    handles += [Line2D([0], [0], color="#222222", marker="o", lw=0, label="Goal endpoint"), Line2D([0], [0], color="#222222", marker="x", lw=0, label="Other endpoint")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .005), ncol=4, frameon=False, columnspacing=1.0, handlelength=1.8)
    save(fig, "paths")


def diagnostic(raw):
    fig, axes = plt.subplots(2, 1, figsize=(7, 3), sharex=True, gridspec_kw={"height_ratios": [.85, 1.15]})
    fig.subplots_adjust(left=.205, right=.985, bottom=.17, top=.90, hspace=.62)
    rows = [row for row in raw["diagnostic"] if row["method"] == "cache120"]
    terminal = raw["diagnostic_summary"][0]
    require(terminal["outcome"] == "goal" and float(terminal["time"]) == 31, "Diagnostic terminal outcome differs from audit")
    t = [float(row["time"]) for row in rows] + [float(terminal["time"])]
    d = [float(np.hypot(250 - float(row["x"]), float(row["y"]))) for row in rows] + [float(terminal["goal_distance"])]
    ax = axes[0]
    ax.plot(t, d, color="#333333", lw=1.4)
    ax.axhline(15, color="#777777", ls="--", lw=.7)
    ax.plot(t[-1], d[-1], "o", color=COLORS["full203"], ms=3.8)
    ax.annotate("Goal at 31 s", (t[-1], d[-1]), xytext=(-9, 14), textcoords="offset points", ha="right", fontsize=8)
    ax.set(ylabel="Goal distance (m)", title="(a) High-speed cache replay", yticks=[0, 100, 200])
    ax.grid(alpha=.15)
    methods = ("cache120", "diverse120", "full120", "diverse_unlimited", "full_unlimited")
    labels = ("Cache, 120 m", "Diverse, 120 m", "Full, 120 m", "Diverse, unlimited", "Full, unlimited")
    colors = ("#79518B", "#D55E00", "#009E73", "#E69F00", "#0072B2")
    ax = axes[1]
    for i, (method, color) in enumerate(zip(methods, colors)):
        rr = [row for row in raw["diagnostic"] if row["method"] == method]
        ax.scatter([float(row["time"]) for row in rr], [i] * len(rr), marker="s", s=6, color="#DCDCDC", linewidth=0)
        positive = [row for row in rr if row["verified"] == "1" or row["reused"] == "1"]
        ax.scatter([float(row["time"]) for row in positive], [i] * len(positive), marker="s", s=7, color=color, linewidth=0)
    reused = [row for row in rows if row["reused"] == "1"]
    require(len(reused) == 63, "Cache reuse count differs from audit")
    ax.scatter([float(row["time"]) for row in reused], [0] * len(reused), marker="o", s=12, facecolors="none", edgecolors="#111111", linewidth=.6)
    ax.set(yticks=range(5), yticklabels=labels, ylim=(-.55, 4.55), xlim=(-.5, 31.7), xlabel="Replay time (s)", title="(b) Validated evidence; circles mark cache reuse")
    ax.invert_yaxis()
    save(fig, "diagnostic")


def main():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 9, "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 8, "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True, "pdf.fonttype": 42, "ps.fonttype": 42, "figure.dpi": 150, "savefig.dpi": 300, "axes.titlepad": 5, "axes.labelpad": 3})
    summary3, summary4, raw = load_inputs()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    matched(summary3, raw)
    incremental(summary4)
    outcomes(summary3)
    paths(raw)
    diagnostic(raw)
    print("Wrote five compact 7-inch manuscript figure pairs from audited data.")


if __name__ == "__main__":
    main()
