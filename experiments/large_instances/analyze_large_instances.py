#!/usr/bin/env python3
"""
Analysis script for large CVRP instances (X-series).
1. Finds the best run per instance and plots Gap to BKS (%) vs. elapsed wall-clock time.
2. Plots CPU and FPGA utilization (%) against problem size and 2-opt iterations.
"""

import json
import os
import re
import sys
from pathlib import Path

# Configure writable cache directory for matplotlib before importing
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import pandas as pd
import seaborn as sns

# ==============================================================================
# BEST KNOWN SOLUTIONS (BKS) - CVRPLIB Set-X (Uchoa et al., 2014/2017)
# ==============================================================================
BKS = {
    "X-n101-k25": 27591,
    "X-n125-k30": 55539,
    "X-n139-k10": 13590,
    "X-n148-k46": 43448,
    "X-n256-k16": 18839,
    "X-n513-k21": 24201,
    "X-n936-k151": 132715,
    "X-n1001-k43": 72346,
}

SCRIPT_DIR = Path(__file__).resolve().parent
RESULTS_DIR = SCRIPT_DIR / "results"


def parse_filename_meta(filename: str) -> dict:
    """Extract metadata from result filename."""
    m = re.search(
        r"(?P<inst>[^_]+)_p(?P<p>\d+)_g(?P<g>\d+)_e(?P<e>[\d\.]+)_k(?P<k>\d+)_r(?P<r>\d+)_c(?P<c>\d+)_m(?P<m>\d+)_s(?P<s>\d+)\.json",
        filename,
    )
    if not m:
        return {}
    d = m.groupdict()
    return {
        "inst": d["inst"],
        "p": int(d["p"]),
        "g": int(d["g"]),
        "e": float(d["e"]),
        "k": int(d["k"]),
        "r": int(d["r"]),
        "c": int(d["c"]),
        "m": int(d["m"]),
        "seed": int(d["s"]),
    }


def load_all_runs(results_dir: Path) -> tuple[dict, list]:
    """Load and parse all JSON runs, grouping by instance."""
    json_files = sorted(results_dir.glob("*.json"))
    if not json_files:
        print(f"Error: No JSON files found in {results_dir}", file=sys.stderr)
        sys.exit(1)

    by_instance = {}
    all_runs = []

    for fpath in json_files:
        try:
            with open(fpath) as fp:
                data = json.load(fp)
        except Exception as err:
            print(f"[WARN] Failed to read {fpath.name}: {err}", file=sys.stderr)
            continue

        meta = parse_filename_meta(fpath.name)
        inst_name = data.get("problem", {}).get("name") or meta.get("inst")
        if not inst_name:
            continue

        bks_val = BKS.get(inst_name)
        exact_fit = data["solution"]["exact_best_fitness"]
        hw_fit = data["solution"].get("hw_best_fitness")
        gap_pct = ((exact_fit - bks_val) / bks_val * 100.0) if bks_val else None

        cpu_util = data["timings"]["cpu"]["utilization_pct"]
        workers = data["timings"].get("workers", [])
        fpga_util = sum(w["utilization_pct"] for w in workers) / len(workers) if workers else None
        wall_time_s = data["timings"]["total_wall_time_ms"] / 1000.0

        run_info = {
            "file": fpath.name,
            "path": fpath,
            "inst": inst_name,
            "dimension": data["problem"]["dimension"],
            "capacity": data["problem"]["capacity"],
            "k": meta.get("k", 0),
            "seed": meta.get("seed", 0),
            "exact_fitness": exact_fit,
            "hw_fitness": hw_fit,
            "bks": bks_val,
            "gap_pct": gap_pct,
            "wall_time_s": wall_time_s,
            "cpu_util": cpu_util,
            "fpga_util": fpga_util,
            "raw_data": data,
        }
        all_runs.append(run_info)
        by_instance.setdefault(inst_name, []).append(run_info)

    return by_instance, all_runs


def find_best_runs(by_instance: dict) -> dict:
    """Find the best run for each problem instance using the minimum exact solution value (exact_best_fitness)."""
    best_runs = {}
    for inst, runs in by_instance.items():
        best = min(runs, key=lambda r: r["exact_fitness"])
        best_runs[inst] = best
    return best_runs


def plot_convergence(best_runs: dict, output_dir: Path):
    """Plot Gap to BKS (%) relative to elapsed wall-clock time on a log-log scale for the best run of each instance."""
    fig, ax = plt.subplots(figsize=(10, 6.2))
    palette = sns.color_palette("tab10", len(best_runs))

    for idx, (inst, run) in enumerate(sorted(best_runs.items(), key=lambda x: x[1]["dimension"])):
        bks = run["bks"]
        history = run["raw_data"].get("best_fitness_history", [])
        if not history or not bks:
            continue

        # Extract step improvements to accurately render the step curve without bloat
        pts = [history[0]]
        for rec in history[1:]:
            if rec["best_fitness"] < pts[-1]["best_fitness"]:
                pts.append(rec)
        pts.append(history[-1])

        # Ensure values > 0 for log-log scale (trajectory curve reflects intermediate hardware fitness evaluations)
        times = [max(p["timestamp_ms"] / 1000.0, 1e-3) for p in pts]
        gaps = [max((p["best_fitness"] - bks) / bks * 100.0, 1e-2) for p in pts]

        # Use the best exact solution value to calculate the best BKS gap for the legend (only legend)
        best_exact = run["exact_fitness"]
        best_bks_gap = ((best_exact - bks) / bks * 100.0) if bks else 0.0
        label = f"{inst} (best: {best_bks_gap:.2f}%)"
        color = palette[idx % len(palette)]

        ax.step(times, gaps, where="post", label=label, color=color, linewidth=2.0, alpha=0.9)
        # Highlight final point
        ax.scatter(times[-1], gaps[-1], color=color, edgecolor="black", s=50, zorder=5)

    ax.set_xscale("log")
    ax.set_yscale("log")

    ax.set_title("Gap to BKS vs. Wall-Clock Time", fontweight="bold", fontsize=13)
    ax.set_xlabel("Elapsed Wall-Clock Time (s)", fontsize=11)
    ax.set_ylabel("Gap to BKS (%)", fontsize=11)

    # Format y-axis percentage ticks
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda y, _: f"{y:g}%"))
    ax.yaxis.set_minor_formatter(ticker.FuncFormatter(lambda y, _: f"{y:g}%" if y in [0.5, 1, 2, 5, 20, 50, 200, 500] else ""))

    ax.grid(True, which="major", ls="-", alpha=0.6)
    ax.grid(True, which="minor", ls=":", alpha=0.35)
    ax.legend(loc="upper right", fontsize=9.5, framealpha=0.92)

    plt.tight_layout()
    out_file = output_dir / "convergence_gap_vs_time.png"
    fig.savefig(out_file, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_file.name}")


def plot_utilization(best_runs: dict, output_dir: Path):
    """Plot CPU and FPGA utilization (%) vs. Problem Size and vs. 2-Opt Iterations."""
    # Create DataFrame from best run of each instance (or any run)
    rows = []
    for inst, r in sorted(best_runs.items()):
        # Calculate vehicle count K from instance name
        m_k = re.search(r"[-_]k(\d+)", inst)
        k_vehicles = int(m_k.group(1)) if m_k else 1
        l_bar = (r["dimension"] - 1) / k_vehicles
        rows.append({
            "inst": inst,
            "N": r["dimension"],
            "k": r["k"],
            "K_vehicles": k_vehicles,
            "L_bar": l_bar,
            "cpu_util": r["cpu_util"],
            "fpga_util": r["fpga_util"],
        })
    df_util = pd.DataFrame(rows)

    offsets_n = {
        "X-n101-k25": (-28, 14),
        "X-n125-k30": (-10, 26),
        "X-n139-k10": (0, -22),
        "X-n148-k46": (28, 14),
        "X-n256-k16": (0, 12),
        "X-n513-k21": (0, -18),
        "X-n936-k151": (0, 12),
        "X-n1001-k43": (0, -18),
    }
    offsets_k = {
        "X-n148-k46": (-24, 14),
        "X-n101-k25": (-8, -22),
        "X-n125-k30": (24, 16),
        "X-n936-k151": (0, -20),
        "X-n139-k10": (0, 12),
        "X-n256-k16": (0, -20),
        "X-n1001-k43": (-8, -20),
        "X-n513-k21": (6, 14),
    }

    def annotate_points(ax, df, x_col, offsets):
        for _, row in df.iterrows():
            inst = row["inst"]
            x_val = row[x_col]
            y_val = row["cpu_util"]
            ox, oy = offsets.get(inst, (0, 12))
            needs_arrow = abs(ox) > 10 or abs(oy) > 18
            arrow = dict(arrowstyle="-", color="#777777", lw=0.6, shrinkA=2, shrinkB=2) if needs_arrow else None
            ax.annotate(
                inst,
                (x_val, y_val),
                textcoords="offset points",
                xytext=(ox, oy),
                ha="center",
                fontsize=8,
                color="#222222",
                arrowprops=arrow,
            )

    # 1. Combined 2-panel plot (Problem Size and 2-Opt Iterations)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5.2))

    # Panel 1: Utilization vs. Problem Size
    df_n = df_util.sort_values("N")
    ax1.plot(df_n["N"], df_n["fpga_util"], marker="s", markersize=7, linewidth=2.0,
             color="tab:green", label="FPGA Utilization (%)")
    ax1.plot(df_n["N"], df_n["cpu_util"], marker="o", markersize=7, linewidth=2.0,
             color="tab:blue", label="CPU Utilization (%)")
    annotate_points(ax1, df_n, "N", offsets_n)
    ax1.set_title("Hardware Utilization vs. Problem Size", fontweight="bold")
    ax1.set_xlabel("Problem Size")
    ax1.set_ylabel("Utilization (%)")
    ax1.set_ylim(-5, 110)
    ax1.legend(loc="center right", fontsize=9.0)

    # Panel 2: Utilization vs. 2-Opt Iterations
    df_k = df_util.sort_values("k")
    ax2.plot(df_k["k"], df_k["fpga_util"], marker="s", markersize=7, linewidth=2.0,
             color="tab:green", label="FPGA Utilization (%)")
    ax2.plot(df_k["k"], df_k["cpu_util"], marker="o", markersize=7, linewidth=2.0,
             color="tab:blue", label="CPU Utilization (%)")
    annotate_points(ax2, df_k, "k", offsets_k)
    ax2.set_title("Hardware Utilization vs. 2-Opt Iterations", fontweight="bold")
    ax2.set_xlabel("2-Opt Iterations per Subtour")
    ax2.set_ylabel("Utilization (%)")
    ax2.set_ylim(-5, 110)
    ax2.legend(loc="center right", fontsize=9.0)

    plt.tight_layout()
    out_file = output_dir / "utilization_analysis.png"
    fig.savefig(out_file, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {out_file.name}")

    # 2. Individual Standalone Plots
    # Standalone 1: Utilization vs Problem Size
    fig1, ax_s1 = plt.subplots(figsize=(7.5, 4.8))
    ax_s1.plot(df_n["N"], df_n["fpga_util"], marker="s", markersize=7, linewidth=2.0,
               color="tab:green", label="FPGA Utilization (%)")
    ax_s1.plot(df_n["N"], df_n["cpu_util"], marker="o", markersize=7, linewidth=2.0,
               color="tab:blue", label="CPU Utilization (%)")
    annotate_points(ax_s1, df_n, "N", offsets_n)
    ax_s1.set_title("Hardware Utilization vs. Problem Size", fontweight="bold")
    ax_s1.set_xlabel("Problem Size")
    ax_s1.set_ylabel("Utilization (%)")
    ax_s1.set_ylim(-5, 110)
    ax_s1.legend(loc="center right", fontsize=9)
    plt.tight_layout()
    fig1.savefig(output_dir / "utilization_vs_problem_size.png", dpi=200, bbox_inches="tight")
    plt.close(fig1)
    print("Saved: utilization_vs_problem_size.png")

    # Standalone 2: Utilization vs k
    fig2, ax_s2 = plt.subplots(figsize=(7.5, 4.8))
    ax_s2.plot(df_k["k"], df_k["fpga_util"], marker="s", markersize=7, linewidth=2.0,
               color="tab:green", label="FPGA Utilization (%)")
    ax_s2.plot(df_k["k"], df_k["cpu_util"], marker="o", markersize=7, linewidth=2.0,
               color="tab:blue", label="CPU Utilization (%)")
    annotate_points(ax_s2, df_k, "k", offsets_k)
    ax_s2.set_title("Hardware Utilization vs. 2-Opt Iterations", fontweight="bold")
    ax_s2.set_xlabel("2-Opt Iterations per Subtour")
    ax_s2.set_ylabel("Utilization (%)")
    ax_s2.set_ylim(-5, 110)
    ax_s2.legend(loc="center right", fontsize=9)
    plt.tight_layout()
    fig2.savefig(output_dir / "utilization_vs_k.png", dpi=200, bbox_inches="tight")
    plt.close(fig2)
    print("Saved: utilization_vs_k.png")


def print_summary_table(best_runs: dict):
    """Print ASCII summary table of the best solutions and profiling metrics."""
    print("=" * 125)
    print("                        LARGE CVRP INSTANCES: BEST SOLUTIONS & HARDWARE PROFILING")
    print("=" * 125)
    header = (
        f"{'Instance':12s} | {'N':4s} | {'L_bar':6s} | {'k':3s} | {'Best Fitness':12s} | {'BKS':7s} | "
        f"{'Gap (%)':7s} | {'Wall Time':10s} | {'CPU Util':9s} | {'FPGA Util':9s} | {'Seed':10s}"
    )
    print(header)
    print("-" * 125)

    for inst, r in sorted(best_runs.items(), key=lambda x: x[1]["dimension"]):
        m_k = re.search(r"[-_]k(\d+)", inst)
        k_veh = int(m_k.group(1)) if m_k else 1
        l_bar = (r["dimension"] - 1) / k_veh
        line = (
            f"{inst:12s} | {r['dimension']:4d} | {l_bar:6.2f} | {r['k']:3d} | {r['exact_fitness']:12.2f} | "
            f"{r['bks']:7d} | {r['gap_pct']:6.2f}% | {r['wall_time_s']:8.1f} s | "
            f"{r['cpu_util']:8.2f}% | {r['fpga_util']:8.2f}% | {r['seed']:10d}"
        )
        print(line)

    print("=" * 125 + "\n")


def main():
    sns.set_theme(style="whitegrid", font_scale=1.05)

    results_dir = RESULTS_DIR
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        results_dir = Path(sys.argv[1]).resolve()

    print(f"Loading experiment results from: {results_dir} ...")
    by_instance, all_runs = load_all_runs(results_dir)
    print(f"Loaded {len(all_runs)} runs across {len(by_instance)} instances.")

    best_runs = find_best_runs(by_instance)
    print_summary_table(best_runs)

    output_dir = SCRIPT_DIR
    plot_convergence(best_runs, output_dir)
    plot_utilization(best_runs, output_dir)


if __name__ == "__main__":
    main()
