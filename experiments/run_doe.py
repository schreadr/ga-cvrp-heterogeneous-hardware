#!/usr/bin/env python3
"""
Design of Experiment (DoE) Execution Script for Heterogeneous GA-CVRP.

Iterates through all hyperparameter combinations and random seeds.
For each combination, checks if the corresponding output JSON file already exists:
- If yes: skips execution.
- If no: runs the GA binary with the specified parameters and writes the output JSON.

After execution, aggregates all experiment results into 'results/experiments_summary.csv'.
"""

import csv
import itertools
import json
import os
import subprocess
import sys
from pathlib import Path

# ==============================================================================
# PARAMETER SEARCH SPACE DEFINITION
# ==============================================================================

PROBLEM_INSTANCES_DIR = (Path(__file__).resolve().parent.parent / "problem_instances").resolve()

# Path(s) to CVRP problem instance file(s)
VRP_FILES = [
    str(PROBLEM_INSTANCES_DIR / "P-n40-k5.vrp"),
    # str(PROBLEM_INSTANCES_DIR / "P-n16-k8.vrp"),
    # str(PROBLEM_INSTANCES_DIR / "P-n50-k7.vrp"),
]

# Population size multiplier (pop_size = dimension * multiplier)
POPULATION_MULTIPLIERS = [10, 20]

# Number of generations to run
GENERATIONS = [1000]

# Elitism percentage (% of best individuals to preserve)
ELITISM_PERCENTAGES = [2.0, 5.0]

# Number of 2-opt iterations per subtour (k)
TWO_OPT_KS = [0, 1]

# Enable precheck for 2-opt moves (1 = enable, 0 = disable)
TWO_OPT_PRECHECKS = [1]

# Crossover probability (0 to 100)
CROSSOVER_PROBABILITIES = [100]

# Mutation probability (0 to 100)
MUTATION_PROBABILITIES = [100]

# List of random seeds (the number of seeds defines how many times each experiment runs)
SEEDS = [0xC0FFEE, 0x123456, 0xDEADBEEF]

# Hardcoded path to the compiled GA C++ binary
BINARY_PATH = (Path(__file__).resolve().parent.parent / "sw" / "build" / "test").resolve()


# ==============================================================================
# HELPER FUNCTIONS
# ==============================================================================


def aggregate_results_to_csv(results_dir: Path, csv_path: Path):
    """Aggregate all completed JSON experiment results into a master CSV table."""
    records = []
    for json_file in sorted(results_dir.glob("*.json")):
        if json_file.stat().st_size == 0:
            continue
        try:
            with open(json_file, "r") as f:
                data = json.load(f)
        except Exception:
            continue

        problem = data.get("problem", {})
        params = data.get("ga_parameters", {})
        sol = data.get("solution", {})
        timings = data.get("timings", {})
        cpu = timings.get("cpu", {})
        workers = timings.get("workers", [])

        row = {
            "output_file": json_file.name,
            "problem_name": problem.get("name", ""),
            "dimension": problem.get("dimension", 0),
            "capacity": problem.get("capacity", 0),
            "scaling": problem.get("scaling", 1),
            "n_generations": params.get("n_generations", 0),
            "pop_size": params.get("pop_size", 0),
            "initial_best_fitness": sol.get("initial_best_fitness", 0.0),
            "hw_best_fitness": sol.get("hw_best_fitness", 0.0),
            "exact_best_fitness": sol.get("exact_best_fitness", 0.0),
            "gen_best_found": sol.get("gen_best_found", 0),
            "improvement_pct": sol.get("improvement_pct", 0.0),
            "total_wall_time_ms": timings.get("total_wall_time_ms", 0.0),
            "throughput_evals_sec": timings.get("throughput_evals_sec", 0.0),
            "cpu_active_time_ms": cpu.get("active_time_ms", 0.0),
            "cpu_idle_time_ms": cpu.get("idle_time_ms", 0.0),
            "cpu_utilization_pct": cpu.get("utilization_pct", 0.0),
        }

        # Per-vFPGA worker metrics
        for w in workers:
            vfid = w.get("vfid", 0)
            row[f"vfpga_{vfid}_active_ms"] = w.get("active_time_ms", 0.0)
            row[f"vfpga_{vfid}_idle_ms"] = w.get("idle_time_ms", 0.0)
            row[f"vfpga_{vfid}_utilization_pct"] = w.get("utilization_pct", 0.0)
            row[f"vfpga_{vfid}_batches"] = w.get("batches_processed", 0)

        records.append(row)

    if not records:
        return

    # Extract all distinct fieldnames
    fieldnames = list(records[0].keys())
    for r in records[1:]:
        for k in r.keys():
            if k not in fieldnames:
                fieldnames.append(k)

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    print(f"[SUMMARY] Aggregated {len(records)} runs into: {csv_path.name}")


# ==============================================================================
# MAIN EXECUTION LOOP
# ==============================================================================

def main():
    script_dir = Path(__file__).resolve().parent
    results_dir = script_dir / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    summary_csv = results_dir / "experiments_summary.csv"

    binary_path = BINARY_PATH

    print("================================================================================")
    print("                     DESIGN OF EXPERIMENT (DoE) RUNNER                          ")
    print("================================================================================")
    print(f"Results Directory: {results_dir}")
    print(f"Binary:            {binary_path} (exists: {binary_path.exists()})")
    print(f"Seeds ({len(SEEDS)}):       {', '.join(hex(s) if isinstance(s, int) and s > 1000 else str(s) for s in SEEDS)}")
    print("--------------------------------------------------------------------------------")

    # Generate all parameter combinations
    param_combinations = list(itertools.product(
        VRP_FILES,
        POPULATION_MULTIPLIERS,
        GENERATIONS,
        ELITISM_PERCENTAGES,
        TWO_OPT_KS,
        TWO_OPT_PRECHECKS,
        CROSSOVER_PROBABILITIES,
        MUTATION_PROBABILITIES,
    ))

    total_runs = len(param_combinations) * len(SEEDS)
    print(f"Total Configurations: {len(param_combinations)}")
    print(f"Total Planned Runs:   {total_runs} (configurations * {len(SEEDS)} seeds)")
    print("================================================================================\n")

    run_idx = 0
    skipped_count = 0
    executed_count = 0

    for combo in param_combinations:
        vrp_file, p_mult, gen, elitism, k, precheck, cx_prob, mut_prob = combo
        vrp_path = Path(vrp_file).resolve()
        if not vrp_path.exists():
            print(f"Error: Problem instance file '{vrp_path}' not found.", file=sys.stderr)
            sys.exit(1)
        instance_stem = vrp_path.stem

        for seed in SEEDS:
            run_idx += 1
            # Deterministic, unique output filename for these parameters & seed
            filename = (
                f"{instance_stem}_p{p_mult}_g{gen}_e{elitism:.1f}_k{k}_r{precheck}"
                f"_c{cx_prob}_m{mut_prob}_s{seed}.json"
            )
            output_file = results_dir / filename

            # Check if output file already exists and is valid
            if output_file.exists() and output_file.stat().st_size > 0:
                print(f"[{run_idx}/{total_runs}] [SKIP] {filename} already exists.")
                skipped_count += 1
                continue

            # Check binary existence before running
            if not binary_path.exists():
                print(f"Error: Binary '{binary_path}' not found. Please compile the C++ software first.")
                sys.exit(1)

            print(f"[{run_idx}/{total_runs}] [RUN]  {filename}...")

            cmd = [
                str(binary_path),
                "-f", str(vrp_path),
                "-p", str(p_mult),
                "-g", str(gen),
                "-e", str(elitism),
                "-s", str(seed),
                "-k", str(k),
                "-r", str(precheck),
                "-c", str(cx_prob),
                "-m", str(mut_prob),
                "-o", str(output_file),
            ]

            try:
                subprocess.run(cmd, check=True)
                executed_count += 1
            except subprocess.CalledProcessError as e:
                print(f"Error executing run {filename}: {e}", file=sys.stderr)
                # Remove incomplete file if left behind
                if output_file.exists() and output_file.stat().st_size == 0:
                    output_file.unlink()

    print("\n================================================================================")
    print(f"Done! Executed: {executed_count}, Skipped (already completed): {skipped_count}, Total: {total_runs}")
    print("================================================================================")

    # Aggregate all completed JSONs into summary CSV
    aggregate_results_to_csv(results_dir, summary_csv)


if __name__ == "__main__":
    main()
