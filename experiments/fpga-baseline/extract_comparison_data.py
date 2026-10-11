#!/usr/bin/env python3
"""
Extract performance metrics and convergence times for the heterogeneous FPGA-CPU implementation
across benchmark CVRP instances to compare against FPGA (Heer) and CPU/GPU (Abdelatti et al.) baselines.

Target configuration:
  - Population Multiplier (p): 20
  - Crossover Probability (c): 100%
  - Mutation Probability  (m): 10%
  - Precheck              (r): 0
  - Generations           (g): 50000
  - Elitism               (e): 5.0%
"""

import glob
import json
import os
import re
from pathlib import Path
import numpy as np

# Best Known Solutions (CVRPLIB Set-P and Set-X)
BKS = {
    "P-n70-k10": 827,
    "X-n101-k25": 27591,
    "X-n125-k30": 55539,
    "X-n139-k10": 13590,
    "X-n148-k46": 43448,
}

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent
LARGE_INSTANCES_DIR = EXPERIMENTS_DIR / "large_instances" / "results"
DOE_DIR = EXPERIMENTS_DIR / "doe" / "results"


def parse_run_data(filepath: Path, bks_val: float) -> dict:
    """Parse JSON run file and extract key metrics."""
    with open(filepath, "r") as fp:
        data = json.load(fp)

    sol = data["solution"]
    timings = data["timings"]
    hist = data.get("best_fitness_history", [])

    exact_fitness = sol["exact_best_fitness"]
    gap_pct = (exact_fitness - bks_val) / bks_val * 100.0
    gen_best = sol["gen_best_found"]

    time_to_best_s = (
        hist[gen_best]["timestamp_ms"] / 1000.0
        if 0 <= gen_best < len(hist)
        else np.nan
    )
    total_wall_s = timings["total_wall_time_ms"] / 1000.0

    return {
        "file": filepath.name,
        "exact_fitness": exact_fitness,
        "gap_pct": gap_pct,
        "gen_best": gen_best,
        "time_to_best_s": time_to_best_s,
        "total_wall_s": total_wall_s,
    }


def extract_instance_stats(inst_name: str) -> dict:
    """Find and compute average metrics for all matching runs of an instance."""
    bks_val = BKS[inst_name]

    if inst_name.startswith("P-"):
        # P-n70-k10 in doe/results (k=30 matches adaptive rule k = round(5 * L_bar) = 35)
        pattern = DOE_DIR / f"{inst_name}_p20_g50000_e5.0_k30_r0_c100_m10_*.json"
    else:
        # X-series instances in large_instances/results
        pattern = LARGE_INSTANCES_DIR / f"{inst_name}_p20_g50000_e5.0_*_r0_c100_m10_*.json"

    matching_files = sorted(glob.glob(str(pattern)))
    if not matching_files:
        raise FileNotFoundError(f"No matching files found for pattern: {pattern}")

    runs = [parse_run_data(Path(f), bks_val) for f in matching_files]

    gaps = [r["gap_pct"] for r in runs]
    times_to_best = [r["time_to_best_s"] for r in runs]
    total_walls = [r["total_wall_s"] for r in runs]

    return {
        "instance": inst_name,
        "runs_count": len(runs),
        "runs": runs,
        "avg_gap": float(np.mean(gaps)),
        "std_gap": float(np.std(gaps)),
        "min_gap": float(np.min(gaps)),
        "avg_time_to_best": float(np.mean(times_to_best)),
        "std_time_to_best": float(np.std(times_to_best)),
        "min_time_to_best": float(np.min(times_to_best)),
        "avg_total_wall": float(np.mean(total_walls)),
    }


def main():
    print("=" * 105)
    print(" HETEROGENEOUS FPGA-CPU GA-CVRP: BENCHMARK CONVERGENCE & PERFORMANCE EXTRACTION")
    print(" Target Config: p=20, c=100%, m=10%, r=0, g=50000, e=5.0%")
    print("=" * 105)

    instances = ["P-n70-k10", "X-n101-k25", "X-n125-k30", "X-n139-k10", "X-n148-k46"]
    results = [extract_instance_stats(inst) for inst in instances]

    # Detailed console table
    header = (
        f"{'Instance':12s} | {'Runs':4s} | {'Avg Gap (%)':11s} | {'Std Gap':8s} | "
        f"{'Avg T_best':11s} | {'Std T_best':10s} | {'Avg Wall (s)':12s}"
    )
    print(header)
    print("-" * 105)
    for res in results:
        line = (
            f"{res['instance']:12s} | {res['runs_count']:4d} | "
            f"{res['avg_gap']:10.2f}% | {res['std_gap']:7.2f}% | "
            f"{res['avg_time_to_best']:10.1f}s | {res['std_time_to_best']:9.1f}s | "
            f"{res['avg_total_wall']:11.1f}s"
        )
        print(line)
    print("=" * 105 + "\n")

if __name__ == "__main__":
    main()

