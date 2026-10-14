#!/usr/bin/env bash
# ==============================================================================
# HLS Test & Build Automation Runner
# Stages required sources and runs Vitis HLS csim, synthesis, and cosim.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
HW_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
REPO_ROOT=$(cd "${HW_DIR}/.." && pwd)

HLS_SRC_DIR="${HW_DIR}/src/hls/processing_node"
SW_SRC_DIR="${REPO_ROOT}/sw/src"
PROBLEM_DIR="${REPO_ROOT}/problem_instances"
BUILD_DIR="${HW_DIR}/build_hls"
CONFIG_FILE="${SCRIPT_DIR}/hls_config.cfg"

# Colors for terminal output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_usage() {
    echo -e "${BLUE}Usage:${NC} $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  --csim        Stage files and run C-Simulation (vitis-run --mode hls --csim)"
    echo "  --syn         Stage files and run C-Synthesis (v++ -c --mode hls)"
    echo "  --cosim       Stage files and run Co-Simulation (vitis-run --mode hls --cosim)"
    echo "  --all         Stage files and run CSIM, SYN, and COSIM sequentially"
    echo "  --stage-only  Only stage required files into build directory"
    echo "  --clean       Remove the staged build directory (${BUILD_DIR})"
    echo "  --help, -h    Show this help message"
}

stage_files() {
    echo -e "${BLUE}==>${NC} Staging files into ${GREEN}${BUILD_DIR}${NC}..."
    mkdir -p "${BUILD_DIR}/tb"
    mkdir -p "${BUILD_DIR}/sw"
    mkdir -p "${BUILD_DIR}/problem_instances"

    # 1. Stage HLS Kernel sources
    cp "${HLS_SRC_DIR}"/*.hpp "${BUILD_DIR}/"
    cp "${HLS_SRC_DIR}"/processing_node.cpp "${BUILD_DIR}/"

    # 2. Stage HLS Testbench sources
    cp "${HLS_SRC_DIR}/tb"/* "${BUILD_DIR}/tb/"

    # 3. Stage SW Dependencies (for testbench compilation and linking)
    cp "${SW_SRC_DIR}/cvrp_types.hpp" "${SW_SRC_DIR}/cvrp_types.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/genome_hash.hpp" "${SW_SRC_DIR}/genome_hash.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/ga_operators.hpp" "${SW_SRC_DIR}/ga_operators.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/encoding_validator.hpp" "${SW_SRC_DIR}/encoding_validator.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/ga_engine.hpp" "${SW_SRC_DIR}/ga_engine.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/ga_streaming.hpp" "${SW_SRC_DIR}/ga_streaming.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/problem_parser.hpp" "${SW_SRC_DIR}/problem_parser.cpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/batch_job.hpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/safe_queue.hpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/static_block.hpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/ga_encoding_constants.hpp" "${BUILD_DIR}/sw/"
    cp "${SW_SRC_DIR}/ga.hpp" "${BUILD_DIR}/sw/"

    # 4. Stage Problem Instances
    cp "${PROBLEM_DIR}"/*.vrp "${BUILD_DIR}/problem_instances/"

    # 5. Stage HLS Configuration
    cp "${CONFIG_FILE}" "${BUILD_DIR}/hls_config.cfg"

    echo -e "${GREEN}==>${NC} Staging complete."
}

run_csim() {
    stage_files
    echo -e "${BLUE}==>${NC} Running C-Simulation (csim)..."
    (cd "${BUILD_DIR}" && vitis-run --mode hls --csim --config hls_config.cfg --work_dir build)
    echo -e "${GREEN}==>${NC} C-Simulation completed successfully."
}

run_syn() {
    stage_files
    echo -e "${BLUE}==>${NC} Running C-Synthesis (v++ -c --mode hls)..."
    (cd "${BUILD_DIR}" && v++ -c --mode hls --config hls_config.cfg --work_dir ./build)
    echo -e "${GREEN}==>${NC} C-Synthesis completed successfully."
}

run_cosim() {
    stage_files
    echo -e "${BLUE}==>${NC} Running Co-Simulation (cosim)..."
    (cd "${BUILD_DIR}" && vitis-run --mode hls --cosim --config hls_config.cfg --work_dir build)
    echo -e "${GREEN}==>${NC} Co-Simulation completed successfully."
}

clean_build() {
    echo -e "${YELLOW}==>${NC} Cleaning ${BUILD_DIR}..."
    rm -rf "${BUILD_DIR}"
    echo -e "${GREEN}==>${NC} Clean complete."
}

if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

case "$1" in
    --csim)
        run_csim
        ;;
    --syn)
        run_syn
        ;;
    --cosim)
        run_cosim
        ;;
    --all)
        run_csim
        run_syn
        run_cosim
        ;;
    --stage-only)
        stage_files
        ;;
    --clean)
        clean_build
        ;;
    --help|-h)
        print_usage
        exit 0
        ;;
    *)
        echo -e "${RED}Error:${NC} Unknown option '$1'"
        print_usage
        exit 1
        ;;
esac
