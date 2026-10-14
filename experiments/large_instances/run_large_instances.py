#!/usr/bin/env python3
"""
Minimal execution script for large CVRP instances (X-series).
Configured for high solution quality with subtour-adaptive 2-opt (k).
"""

import json
import os
import re
import subprocess
import sys
from pathlib import Path

# ==============================================================================
# 1. HYPERPARAMETERS
# ==============================================================================
POP_MULTIPLIER = 20    
CROSSOVER_PROB = 100   
MUTATION_PROB  = 10
PRECHECK       = 0     
GENERATIONS    = 50000 
ELITISM_PCT    = 5.0   

# Seeds for evaluation (run multiple seeds to ensure statistical confidence)
SEEDS = [1842948753, 3109482751, 754928163, 4019284752, 2384951076]

# ==============================================================================
# 2. INSTANCES TO EVALUATE
# Subtour length L_bar and 2-opt budget k are parsed dynamically from filename:
#   L_bar = (N - 1) / K,   k = round(5 * L_bar)
# ==============================================================================
LARGE_INSTANCES = [
    "X-n101-k25.vrp",
    "X-n125-k30.vrp",
    "X-n139-k10.vrp",
    "X-n148-k46.vrp",
    "X-n256-k16.vrp",
    "X-n513-k21.vrp",
    "X-n936-k151.vrp",
    "X-n1001-k43.vrp",
]

# Large instances restricted to the first two seeds due to long runtimes
TWO_SEED_INSTANCES = {
    "X-n513-k21.vrp",
    "X-n936-k151.vrp",
    "X-n1001-k43.vrp",
}


def get_seeds_for_instance(filename: str) -> list[int]:
    """Return the first two seeds for large instances, and all seeds for smaller instances."""
    if filename in TWO_SEED_INSTANCES:
        return SEEDS[:2]
    return SEEDS



def parse_subtour_k(filename: str) -> tuple[float, int]:
    """Parse N and K from filename, then compute L_bar = (N - 1) / K and k = round(5 * L_bar)."""
    match = re.search(r"[-_]n(?P<n>\d+)[-_]k(?P<k>\d+)", filename, re.IGNORECASE)
    if not match:
        raise ValueError(f"Could not parse n and k from filename: {filename}")
    n = int(match.group("n"))
    k_trucks = int(match.group("k"))
    l_bar = (n - 1) / k_trucks
    k_2opt = round(5 * l_bar)
    return l_bar, k_2opt


# ==============================================================================
# 3. DIRECTORY & BINARY RESOLUTION
# ==============================================================================
SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = next((p for p in [SCRIPT_DIR, SCRIPT_DIR.parent, SCRIPT_DIR.parent.parent] if (p / "problem_instances").exists()), SCRIPT_DIR)
PROBLEM_DIR = REPO_ROOT / "problem_instances"
RESULTS_DIR = SCRIPT_DIR / "results"
RESULTS_DIR.mkdir(parents=True, exist_ok=True)

# Locate compiled Coyote GA binary
CANDIDATES = [
    REPO_ROOT / "build" / "test",
    REPO_ROOT / "sw" / "build" / "test",
    REPO_ROOT / "sw" / "build_sw" / "test",
]
BINARY_PATH = next((p for p in CANDIDATES if p.is_file() and os.access(p, os.X_OK)), None)


def main():
    if not BINARY_PATH:
        print(f"Error: Could not locate compiled GA binary in {CANDIDATES}", file=sys.stderr)
        sys.exit(1)

    total_runs = sum(len(get_seeds_for_instance(f)) for f in LARGE_INSTANCES)
    print("=" * 80)
    print("        RUN LARGE INSTANCES (HIGH QUALITY SETUP)")
    print(f"Binary:     {BINARY_PATH}")
    print(f"Fixed:      p={POP_MULTIPLIER}, c={CROSSOVER_PROB}%, m={MUTATION_PROB}%, r={PRECHECK}, g={GENERATIONS}")
    print(f"Instances:  {len(LARGE_INSTANCES)} files ({total_runs} runs total: 5 seeds for smaller, 2 seeds for large)")
    print(f"Results:    {RESULTS_DIR}")
    print("=" * 80)

    for vrp_filename in LARGE_INSTANCES:
        vrp_path = PROBLEM_DIR / vrp_filename
        if not vrp_path.is_file():
            print(f"[WARN] Instance file not found: {vrp_path}, skipping.", file=sys.stderr)
            continue

        try:
            l_bar, k = parse_subtour_k(vrp_filename)
        except ValueError as err:
            print(f"[WARN] {err}, skipping.", file=sys.stderr)
            continue

        stem = vrp_path.stem
        seeds = get_seeds_for_instance(vrp_filename)

        for seed in seeds:
            out_file = RESULTS_DIR / f"{stem}_p{POP_MULTIPLIER}_g{GENERATIONS}_e{ELITISM_PCT:.1f}_k{k}_r{PRECHECK}_c{CROSSOVER_PROB}_m{MUTATION_PROB}_s{seed}.json"

            # Skip already completed valid runs
            if out_file.is_file() and out_file.stat().st_size > 0:
                try:
                    with open(out_file) as f:
                        json.load(f)
                    print(f"[SKIP] Already exists: {out_file.name}")
                    continue
                except Exception:
                    out_file.unlink(missing_ok=True)

            cmd = [
                str(BINARY_PATH),
                "-f", str(vrp_path),
                "-p", str(POP_MULTIPLIER),
                "-g", str(GENERATIONS),
                "-e", str(ELITISM_PCT),
                "-s", str(seed),
                "-k", str(k),
                "-r", str(PRECHECK),
                "-c", str(CROSSOVER_PROB),
                "-m", str(MUTATION_PROB),
                "-o", str(out_file),
            ]

            print(f"\n[RUN] {stem} | L̄={l_bar:.1f} -> k={k} | Seed={seed} ...")
            try:
                subprocess.run(cmd, check=True)
            except KeyboardInterrupt:
                print("\n[STOP] Sweep interrupted by user.", file=sys.stderr)
                if out_file.exists() and out_file.stat().st_size == 0:
                    out_file.unlink()
                sys.exit(0)
            except subprocess.CalledProcessError as err:
                print(f"[ERROR] Execution failed for {stem}: {err}", file=sys.stderr)
                if out_file.exists() and out_file.stat().st_size == 0:
                    out_file.unlink()


if __name__ == "__main__":
    import os
    main()
