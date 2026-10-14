# Genetic Algorithm for the Capacitated Vehicle Routing Problem on Heterogeneous Hardware

This example demonstrates a high-performance, heterogeneous CPU-FPGA solver for the **Capacitated Vehicle Routing Problem (CVRP)** integrated into the [Coyote](https://github.com/fpgasystems/Coyote) operating system framework for FPGAs.

---

## Table of Contents
- [Project Overview](#project-overview)
  - [The CVRP Problem](#the-cvrp-problem)
  - [Heterogeneous Architecture & Partitioning](#heterogeneous-architecture--partitioning)
  - [Hardware Processing Node Pipeline](#hardware-processing-node-pipeline)
  - [Directory Layout](#directory-layout)
- [Building and Deploying](#building-and-deploying)
  - [5. Running the Application](#5-running-the-application)
- [Testing and Verification](#testing-and-verification)
  - [HLS Automation Runner (`run_hls.sh`)](#hls-automation-runner-run_hlssh)
  - [Software Unit & Integration Tests](#software-unit--integration-tests)
  - [Hardware Smoke Test](#hardware-smoke-test)

---

## Project Overview

### The CVRP Problem
The CVRP is an NP-hard combinatorial optimization problem where a fleet of homogeneous vehicles with uniform capacity $C$ must service a set of geographically distributed customer nodes from a central depot. The objective is to determine a set of routes that minimizes the total travel distance such that each customer is visited exactly once and the total demand on any route does not exceed vehicle capacity.

### Heterogeneous Architecture & Partitioning
Standard genetic algorithms for CVRP spend significant execution time on randomized genetic operators, route validation, feasibility repair, and local search heuristics. This project partitions work across heterogeneous hardware to maximize throughput:

- **Host CPU (Software Orchestration)**:
  - Manages global population state, tournament selection, and elitism across generations.
  - Performs genome hashing and deduplication to maintain population diversity.
  - Orchestrates asynchronous task distribution using a thread-safe batch queue (`SafeQueue`) across multiple virtual FPGAs (vFPGAs).
  - Handles problem parsing (`.vrp` TSPLIB format) and result logging.

- **FPGA Accelerators (vFPGAs via Coyote)**:
  - Configured with Coyote multi-tenancy (`N_REGIONS = 2`) to instantiate multiple concurrent vFPGA processing nodes.
  - Leverages dual 512-bit AXI streams per vFPGA (`N_STRM_AXI = 2`) for streaming parents directly from host memory over PCIe.
  - Executes HLS-synthesized genetic operators, constraint repair, fitness calculations, and 2-opt local search completely on-chip with pipelined dataflow.

### Hardware Processing Node Pipeline
Each hardware processing node (`processing_node`) operates in a dataflow pipeline:
1. **Node Cache Initialization**: On-chip BRAM cache stores node coordinates and demands, loaded once per problem via Stream 0.
2. **Streaming Ingestion & Beat Unpacking**: Streams individual genomes from dual 512-bit AXI streams.
3. **Random Number Generation**: On-chip PRNG based on the `Xoroshiro128++` algorithm generates random numbers with minimal latency.
4. **OX Crossover**: Combines pairs of parent genomes across the two streams.
5. **Mutation**: Applies swap mutations according to configurable probabilities.
6. **CVRP Repair & Fitness Evaluation**: Evaluates route feasibility against capacity constraints, inserts depot delimiters when capacity is exceeded, and computes route cost using coordinate lookups from BRAM.
7. **2-opt Local Search**: Iterative intra-route 2-opt optimization with configurable iteration count $k$ and delta-distance precheck heuristics to accelerate convergence.
8. **Egress Streaming & Beat Packing**: Packs offspring back into 512-bit AXI beats and streams results back to host memory.

Control configuration (problem dimension, vehicle capacity, probabilities, 2-opt parameters, random seeds) is programmed dynamically through Coyote 64-bit Control & Status Registers (CSRs) bridged to the HLS 32-bit AXI4-Lite control interface.

### Directory Layout
```text
ga-cvrp-heterogeneous-hardware/
├── hw/
│   ├── CMakeLists.txt            # Coyote HW build definition (N_REGIONS=2, N_STRM_AXI=2)
│   ├── scripts/
│   │   ├── run_hls.sh            # Automated HLS testbench & synthesis runner
│   │   └── hls_config.cfg        # Vitis HLS configuration and source mapping
│   └── src/
│       ├── vfpga_top.svh         # Top-level vFPGA wrapper connecting AXI streams & CSRs
│       ├── init_ip.tcl           # Vivado IP initialization
│       ├── hdl/                  # SystemVerilog/Verilog AXI-Lite adapters
│       └── hls/processing_node/  # HLS kernel, operators (crossover, mutate, repair, 2-opt), TB
├── sw/
│   ├── CMakeLists.txt            # Software build system and test target definitions
│   ├── src/                      # Host GA engine, Coyote interface, streaming pipeline, parser
│   └── tests/                    # Unit tests (encoding, operators, async multi-vFPGA pipeline)
├── problem_instances/            # Standard CVRP benchmark instances (.vrp)
└── experiments/                  # Evaluation scripts (DoE, baseline comparisons, large instances)
```

---

## Building and Deploying

For comprehensive instructions on building Coyote hardware shells, compiling the driver, and system deployment across supported platforms (AMD Alveo U55C, U280, U250, and V80), refer to the top-level [Coyote Examples Guide](https://github.com/fpgasystems/Coyote/blob/master/examples/README.md).


### Running the Application
Run the binary from `sw/build_sw`:

```bash
cd sw/build_sw
./test -f ../../problem_instances/P-n40-k5.vrp -p 20 -g 100 -k 5 -r 1
```

#### Command-Line Options
| Option | Argument | Default | Description |
| :--- | :--- | :--- | :--- |
| `-h` | None | — | Display help message and options. |
| `-f` | String | `../../problem_instances/P-n40-k5.vrp` | Path to the CVRP problem instance (`.vrp`). |
| `-p` | Integer | `20` | Population size multiplier (`population = dimension * multiplier`). |
| `-g` | Integer | `100` | Number of generations to evolve. |
| `-e` | Double | `5.0` | Elitism percentage (% of top solutions preserved across generations). |
| `-s` | Hex/Int | `0xC0FFEE` | Random seed for PRNG initialization. |
| `-k` | Integer | `0` | Number of 2-opt local search iterations per subtour. |
| `-r` | Boolean | `1` | Enable 2-opt distance precheck heuristic (`1` = enabled, `0` = disabled). |
| `-c` | Integer | `100` | Crossover probability in percent ($0$ to $100$). |
| `-m` | Integer | `100` | Mutation probability in percent ($0$ to $100$). |
| `-o` | String | `""` | Optional path to export JSON execution and convergence results. |
| `-t` | None | `false` | Run single-generation hardware smoke test and exit. |

---

## Testing and Verification

### HLS Automation Runner (`run_hls.sh`)
The HLS processing node can be validated independently of full bitstream generation using the test runner script located at [`hw/scripts/run_hls.sh`](hw/scripts/run_hls.sh).

The script automatically stages all necessary HLS kernel sources, testbenches, software data types, and problem instances into `hw/build_hls/` and drives Vitis HLS according to `hw/scripts/hls_config.cfg`.

#### Usage
```bash
cd hw
./scripts/run_hls.sh [OPTION]
```

#### Available Runner Options
- `./scripts/run_hls.sh --csim`: Stages files and runs **C-Simulation** (`vitis-run --mode hls --csim`). Verifies algorithmic correctness of crossover, mutation, repair/fitness, and 2-opt against host C++ implementations.
- `./scripts/run_hls.sh --syn`: Stages files and executes **C-Synthesis** (`v++ -c --mode hls`). Validates pipeline initiation intervals (II), resource utilization (BRAM, DSP, FF, LUT), and clock constraints (3.33 ns / 300 MHz target).
- `./scripts/run_hls.sh --cosim`: Stages files and runs **C/RTL Co-Simulation** (`vitis-run --mode hls --cosim`). Validates cycle-accurate RTL behavior with testbench vectors.
- `./scripts/run_hls.sh --all`: Sequentially executes CSIM, SYN, and COSIM.
- `./scripts/run_hls.sh --stage-only`: Stages required dependencies into `hw/build_hls/` without launching Vitis HLS.
- `./scripts/run_hls.sh --clean`: Removes the staged build directory (`hw/build_hls/`).

### Software Unit & Integration Tests
The software directory includes standalone unit tests that verify software operations without requiring an FPGA or Coyote driver:

```bash
cd sw/build_sw

# Test genome encoding/decoding and beat alignment
./test_encode_decode

# Test selection, parent-pairing, and population update operators
./ga_test

# Test thread-safe queues and asynchronous multi-vFPGA pipelining
./test_async_pipeline
```

### Hardware Smoke Test
Once the FPGA is programmed and the driver is inserted, verify end-to-end host-to-FPGA streaming, CSR configuration, and kernel execution using the smoke test flag:

```bash
cd sw/build_sw
./test -f ../../problem_instances/P-n16-k8.vrp -t
```
This runs a single-generation execution across all configured vFPGAs, validates returned fitness values, and outputs pass/fail status.
