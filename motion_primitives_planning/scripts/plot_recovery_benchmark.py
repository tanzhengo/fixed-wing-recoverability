#!/usr/bin/env python3
"""Plot the deterministic recovery benchmark without requiring a ROS session.

Usage: plot_recovery_benchmark.py --prefix /path/recovery --output-dir /path/figures

Reads PREFIX_{episodes,plans,paths,scenarios}.csv. Scene metadata contains
scenario,x,y,r,inflated_r,z,height exported by the same benchmark run.
The script requires complete results for all five variants and three offsets.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch
import numpy as np


VARIANTS = ["baseline_3s", "dense_3s", "long_6s", "recovery_3plus3s", "recovery_full"]
LABELS = {
    "baseline_3s": "Baseline 3 s", "dense_3s": "Dense 3 s",
    "long_6s": "Long horizon 6 s", "recovery_3plus3s": "Continuation: budget 12",
    "recovery_full": "Continuation: budget 203",
}
COLORS = dict(zip(VARIANTS, ["#0072B2", "#E69F00", "#009E73", "#CC79A7", "#D55E00"]))
STYLES = dict(zip(VARIANTS, ["-", "--", "-.", ":", (0, (7, 2))]))
SCENE_LABELS = {
    "open": "Open space", "offset_pillars": "Offset pillars",
    "late_wall": "Late wall", "cul_de_sac": "Cul-de-sac",
}
OUTCOMES = {
    "goal": ("Reached goal", "#009E73"),
    "no_feasible_primary": ("No feasible primary", "#D55E00"),
    "timeout": ("Time limit", "#E69F00"),
    "collision": ("Collision", "#CC79A7"),
    "invalid_dynamics": ("Invalid dynamics", "#6B7280"),
}


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def load_geometry(args, scene_names):
    metadata = args.geometry_csv or Path(str(args.prefix) + "_scenarios.csv")
    scenes = {name: [] for name in scene_names}
    observed = set()
    for row in read_csv(metadata):
        scene = row["scenario"]
        if scene not in scenes:
            raise ValueError("Geometry has unexpected scene: " + scene)
        observed.add(scene)
        if not row.get("x"):
            continue  # Optional empty row for obstacle-free scenes.
        obstacle = {key: float(row[key]) for key in ("x", "y", "r", "inflated_r")}
        if not all(math.isfinite(value) for value in obstacle.values()):
            raise ValueError("Nonfinite obstacle geometry")
        if obstacle["r"] < 0 or obstacle["inflated_r"] < obstacle["r"]:
            raise ValueError("Invalid obstacle radii")
        scenes[scene].append(obstacle)
    # An open scene may have no metadata row because it contains no obstacles.
    if any(scene != "open" and scene not in observed for scene in scenes):
        raise ValueError("Missing nonempty scenario geometry")
    return scenes


def episode_key(row):
    return row["scenario"], row["variant"], float(row["initial_y"])


def validate(episodes, plans, paths, scenes):
    expected = {(s, v, y) for s in scenes for v in VARIANTS for y in (-2., 0., 2.)}
    actual = [episode_key(row) for row in episodes]
    if set(actual) != expected or len(actual) != len(expected):
        raise ValueError("Incomplete or duplicate benchmark episodes: expected %d, got %d. "
                         "Wait for all runs to finish." % (len(expected), len(actual)))
    for row in episodes:
        if row["outcome"] not in OUTCOMES:
            raise ValueError("Unknown outcome: " + row["outcome"])
    for label, rows in (("plans", plans), ("paths", paths)):
        if not rows or any(episode_key(row) not in expected for row in rows):
            raise ValueError("Unexpected or empty " + label + " CSV")
    if set(episode_key(row) for row in paths) != expected:
        raise ValueError("Some episode trajectories are missing")
    for row in plans:
        value = float(row["planning_ms"])
        if not math.isfinite(value) or value < 0:
            raise ValueError("Invalid planning duration")
    for row in paths:
        if not all(math.isfinite(float(row[k])) for k in ("x", "y", "z", "time")):
            raise ValueError("Nonfinite trajectory coordinate")


def plot_paths(episodes, paths, geometry, output):
    fig, axes = plt.subplots(2, 2, figsize=(13.4, 8.5))
    selected = {episode_key(row): row for row in episodes if float(row["initial_y"]) == 0.0}
    for ax, (scene, obstacles) in zip(axes.flat, geometry.items()):
        bounds_x, bounds_y = [0., 250.], [0., 0.]
        for obstacle in obstacles:
            x, y, r, margin = [obstacle[k] for k in ("x", "y", "r", "inflated_r")]
            ax.add_patch(Circle((x, y), r, facecolor="#D1D5DB", edgecolor="#6B7280",
                                linewidth=0.8, zorder=1))
            ax.add_patch(Circle((x, y), margin, fill=False, edgecolor="#8A6D3B",
                                linestyle="--", linewidth=0.8, zorder=2))
            bounds_x += [x - margin, x + margin]
            bounds_y += [y - margin, y + margin]
        for variant in VARIANTS:
            trajectory = sorted((row for row in paths if episode_key(row) ==
                                 (scene, variant, 0.0)), key=lambda row: int(row["step"]))
            x = [float(row["x"]) for row in trajectory]
            y = [float(row["y"]) for row in trajectory]
            bounds_x += x
            bounds_y += y
            ax.plot(x, y, color=COLORS[variant], linestyle=STYLES[variant],
                    linewidth=2.0, zorder=4)
            if selected[(scene, variant, 0.0)]["outcome"] != "goal":
                ax.plot(x[-1], y[-1], marker="X", color=COLORS[variant],
                        markersize=8, markeredgecolor="white", markeredgewidth=0.5, zorder=5)
        ax.plot(0, 0, "o", color="#111827", markersize=5, zorder=6)
        ax.plot(250, 0, "*", color="#111827", markersize=11, zorder=6)
        ax.add_patch(Circle((250, 0), 15, fill=False, edgecolor="#9CA3AF",
                            linestyle=":", linewidth=0.8))
        # Keep equal physical scale; an unobstructed scene still gets visible y range.
        x_min, x_max = min(bounds_x) - 12, max(bounds_x) + 18
        y_min, y_max = min(bounds_y) - 12, max(bounds_y) + 12
        mid = (y_min + y_max) / 2
        span = max(y_max - y_min, 0.47 * (x_max - x_min))
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(mid - span / 2, mid + span / 2)
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(SCENE_LABELS.get(scene, scene), loc="left", fontweight="bold", pad=7)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(color="#E5E7EB", linewidth=0.5, zorder=0)
    handles = [Line2D([0], [0], color=COLORS[v], linestyle=STYLES[v], lw=2,
                      label=LABELS[v]) for v in VARIANTS]
    handles += [Patch(facecolor="#D1D5DB", edgecolor="#6B7280", label="Obstacle footprint"),
                Line2D([0], [0], color="#8A6D3B", linestyle="--", label="Planning boundary"),
                Line2D([0], [0], marker="X", color="#4B5563", linestyle="None",
                       label="Failed run: last recorded point"),
                Line2D([0], [0], marker="*", color="#111827", linestyle="None", label="Goal")]
    fig.suptitle("Deterministic same-model diagnostic | flight paths, initial y = 0 m", x=0.05,
                 ha="left", fontsize=17, fontweight="bold", y=0.98)
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.055),
               ncol=3, frameon=False, fontsize=9)
    fig.text(0.05, 0.018,
             "Planner and rollout plant share dynamics; global known map, altitude 16–24 m. "
             "Finite continuation checks do not establish indefinite safety.", fontsize=9, color="#4B5563")
    fig.subplots_adjust(left=0.06, right=0.98, top=0.91, bottom=0.22, hspace=0.30, wspace=0.20)
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def plot_summary(episodes, plans, output):
    fig, axes = plt.subplots(1, 2, figsize=(13.4, 5.5))
    x = np.arange(len(VARIANTS))
    bottom = np.zeros(len(VARIANTS))
    present = [key for key in OUTCOMES if any(row["outcome"] == key for row in episodes)]
    for outcome in present:
        values = np.array([sum(row["variant"] == v and row["outcome"] == outcome
                               for row in episodes) for v in VARIANTS])
        label, color = OUTCOMES[outcome]
        axes[0].bar(x, values, bottom=bottom, width=0.62, color=color, label=label)
        for index, value in enumerate(values):
            if value:
                axes[0].text(index, bottom[index] + value / 2, str(value),
                             ha="center", va="center", color="white", fontweight="bold")
        bottom += values
    axes[0].set_ylim(0, max(bottom) + 1)
    axes[0].set_yticks(range(0, int(max(bottom)) + 1, 2))
    axes[0].set_ylabel("Episodes (all 4 scenes × 3 offsets)")
    axes[0].set_title("Observed outcomes", loc="left", fontweight="bold", pad=12)
    axes[0].legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.22),
                   ncol=2, fontsize=9)
    timing = [[float(row["planning_ms"]) for row in plans if row["variant"] == v]
              for v in VARIANTS]
    p95 = [float(np.percentile(values, 95)) for values in timing]
    axes[1].bar(x, p95, color=[COLORS[v] for v in VARIANTS], width=0.62)
    for index, value in enumerate(p95):
        axes[1].text(index, value + max(p95) * .025, "%.2f" % value, ha="center", fontsize=10)
    axes[1].set_ylim(0, max(p95) * 1.2)
    axes[1].set_ylabel("Planning-call wall time (ms)")
    axes[1].set_title("Aggregate P95 planning time | includes failed runs",
                      loc="left", fontweight="bold", pad=12, fontsize=11)
    for ax in axes:
        ax.set_xticks(x)
        ax.set_axisbelow(True)
        ax.grid(axis="y", color="#E5E7EB", linewidth=0.6)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
    short_labels = ["Baseline\n3 s", "Dense\n3 s", "Long horizon\n6 s", "Continuation\nbudget 12", "Continuation\nbudget 203"]
    axes[0].set_xticklabels(short_labels)
    axes[1].set_xticklabels([label + "\nn = " + str(len(values))
                            for label, values in zip(short_labels, timing)])
    fig.suptitle("Deterministic same-model diagnostic | " + str(len(episodes)) + " episodes", x=0.05, ha="left",
                 fontsize=17, fontweight="bold", y=0.98)
    fig.text(.05, .025,
             "Offsets −2, 0, +2 m are deterministic cases, not independent statistical trials. "
             "Aggregate timings mix different states and episode lengths.\n"
             "P95 is not a matched-workload speed comparison; timings are machine-specific and exclude sensing/control latency.",
             fontsize=9, color="#4B5563")
    fig.subplots_adjust(left=.06, right=.98, top=.84, bottom=.29, wspace=.28)
    fig.savefig(output, dpi=180, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True,
                        help="CSV prefix, e.g. /tmp/recovery")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--geometry-csv", type=Path,
                        help="Override PREFIX_scenarios.csv metadata path")
    args = parser.parse_args()
    episodes = read_csv(str(args.prefix) + "_episodes.csv")
    plans = read_csv(str(args.prefix) + "_plans.csv")
    paths = read_csv(str(args.prefix) + "_paths.csv")
    scene_names = list(dict.fromkeys(row["scenario"] for row in episodes))
    geometry = load_geometry(args, scene_names)
    if len(geometry) != 4:
        raise ValueError("This figure layout requires the four benchmark scenes")
    validate(episodes, plans, paths, geometry)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "text.color": "#111827", "axes.labelcolor": "#374151"})
    path_figure = args.output_dir / "recovery_paths.png"
    summary_figure = args.output_dir / "recovery_summary.png"
    plot_paths(episodes, paths, geometry, path_figure)
    plot_summary(episodes, plans, summary_figure)
    print(path_figure)
    print(summary_figure)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, KeyError) as exc:
        raise SystemExit("Plotting failed: " + str(exc))
