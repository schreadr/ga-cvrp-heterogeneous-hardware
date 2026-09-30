# Design of Experiment (DoE) Harness for Heterogeneous GA-CVRP

This folder contains the automated execution harness for conducting systematic parameter sweeps (Design of Experiments) on the heterogeneous CPU-FPGA CVRP Genetic Algorithm.

## Structure

- `run_doe.py`: Main experiment orchestration script.
- `results/`: Directory where each completed experiment's JSON results file and the master `experiments_summary.csv` are stored.

## Parameters Explored

The script explores combinations of the following CLI parameters supported by `main.cpp`:

| Parameter | CLI Flag | Description | Default Grid |
| :--- | :--- | :--- | :--- |
| `vrp_file` | `-f` | Path to CVRP instance | `../../problem_instances/P-n40-k5.vrp` |
| `pop_mult` | `-p` | Population size multiplier ($P = \text{dimension} \times p$) | `10, 20` |
| `generations` | `-g` | Number of complete generations | `1000` |
| `elitism` | `-e` | Elitism percentage | `2.0, 5.0` |
| `two_opt_k` | `-k` | Number of 2-opt iterations per subtour | `0, 1` |
| `two_opt_precheck` | `-r` | Enable precheck for 2-opt moves | `1` |
| `crossover_prob` | `-c` | Crossover probability (0 to 100) | `100` |
| `mutation_prob` | `-m` | Mutation probability (0 to 100) | `100` |
| `seeds` | `-s` | Random seed (defines number of repeats) | `0xC0FFEE, 0x123456, 0xDEADBEEF` |
| `output_file` | `-o` | Output JSON file path | Automatically set per run |

## How to Run

1. Build the software binary:
   ```bash
   cd ../sw
   mkdir -p build && cd build
   cmake ..
   make -j
   ```

2. Run the experiment sweep:
   ```bash
   cd ../../experiments
   python3 run_doe.py
   ```
   *(Optional: pass a custom binary path: `python3 run_doe.py /path/to/binary`)*

## Duplicate Skip & Fault Tolerance

The script assigns a unique, deterministic filename to each parameter combination and seed:
`results/{instance}_p{pop_mult}_g{gen}_e{elitism}_k{k}_r{precheck}_c{cx}_m{mut}_s{seed}.json`

Before launching any run:
- Checks if the file already exists and is non-empty.
- If it exists, skips the execution with `[SKIP]`.
- If interrupted, simply re-running `python3 run_doe.py` resumes seamlessly without re-running finished experiments.

## Output Format

### Per-Run JSON
Each JSON file contains:
- `problem`: Instance name, dimension, capacity, coordinate scaling factor.
- `ga_parameters`: Population size, generation count.
- `solution`: Initial best fitness, HW scaled fitness, exact double-precision sqrt fitness, generation of best discovery, improvement percentage.
- `timings`:
  - `total_wall_time_ms`
  - `throughput_evals_sec`
  - `cpu`: `active_time_ms`, `idle_time_ms`, `utilization_pct`, breakdown (init, selection, update).
  - `workers`: Per-vFPGA `active_time_ms`, `idle_time_ms`, `utilization_pct`, `batches_processed`.
- `best_fitness_history`: Complete generation-by-generation best fitness array (`[g0, g1, ..., gN]`).

### Master Summary CSV (`results/experiments_summary.csv`)
Automatically compiled after the sweep finishes, with one row per completed experiment for rapid tabular analysis in Excel, Pandas, or R.
