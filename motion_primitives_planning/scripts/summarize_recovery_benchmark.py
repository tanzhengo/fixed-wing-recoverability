#!/usr/bin/env python3
"""Summarize every episode and planning cycle, including failed runs."""
import argparse
import collections
import csv
import json
from pathlib import Path


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lo = int(index)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("prefix", type=Path)
    args = parser.parse_args()
    groups = collections.defaultdict(lambda: {"episodes": [], "plans": []})
    for suffix in ("episodes", "plans"):
        with open(str(args.prefix) + "_" + suffix + ".csv", newline="") as source:
            for row in csv.DictReader(source):
                groups[row["variant"]][suffix].append(row)
    result = {
        "scope": "Deterministic model-in-the-loop, same plant and prediction model; full known static geometry. Not independent Gazebo validation or a safety proof.",
        "variants": {},
    }
    for variant, data in groups.items():
        times = [float(row["planning_ms"]) for row in data["plans"]]
        outcomes = collections.Counter(row["outcome"] for row in data["episodes"])
        result["variants"][variant] = {
            "episodes": len(data["episodes"]), "outcomes": dict(outcomes),
            "planning_cycles": len(times),
            "mean_ms": sum(times) / len(times),
            "p95_ms": percentile(times, 0.95), "p99_ms": percentile(times, 0.99),
            "max_ms": max(times), "cycles_over_100ms_reference": sum(t > 100 for t in times),
            "status_counts": dict(collections.Counter(row["status"] for row in data["plans"])),
            "verified_cycles": sum(int(row["verified"]) for row in data["plans"]),
            "budget_exhausted_cycles": sum(int(row["budget_exhausted"]) for row in data["plans"]),
        }
    output = Path(str(args.prefix) + "_summary.json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
