/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2021-2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <cstdlib>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <sys/mman.h>

#include <boost/program_options.hpp>
#include <coyote/cBench.hpp>
#include <coyote/cThread.hpp>

#include "ga_streaming.hpp"
#include "ga.hpp"
#include "problem_parser.hpp"
#include "ga_streaming.hpp"

// Constants
#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 1

// Default vFPGA to assign cThreads to; for designs with one region (vFPGA) this is the only possible value
#define DEFAULT_VFPGA_ID 0

constexpr std::size_t kPopulationMultiplier = 20; // must be even to work atm
constexpr std::uint32_t kPopulationSeed = 0xC0FFEEu;

// Control register indices, matching the AXI4-Lite control register map generated
// by Vitis HLS for the scalar configuration arguments of the processing_node kernel
// (see hw/src/hls/processing_node/processing_node.cpp). The byte offsets of the
// kernel registers are 0x10 (num_individuals0), 0x18 (num_individuals1),
// 0x20 (num_cities) and 0x28 (vehicle_capacity); offsets 0x00-0x0F are reserved by the AXI4-Lite
// control block. Coyote CSRs are 64-bit wide, so the setCSR index equals
// byte offset / 8. The kernel is free-running and starts processing once
// beats_per_individual reads non-zero, so it must be written last.
constexpr uint32_t CSR_CONTROL     = 0x00;
constexpr uint32_t CSR_CONTROL_AUTORESTART     = 0x81;
constexpr uint32_t CSR_NUM_INDIVIDUALS     = 0x10 / 8;
constexpr uint32_t CSR_NUM_CITIES  = 0x18 / 8;
constexpr uint32_t CSR_VEHICLE_CAPACITY = 0x20 / 8;
constexpr uint32_t CSR_DEPOT_COORDS_CAPACITY = 0x28 / 8;
constexpr const char kDefaultProblemFile[] = "../../P-n40-k5_edited.vrp";

std::size_t round_up(std::size_t value, std::size_t alignment) {
    return ((value + alignment - 1) / alignment) * alignment;
}

std::uint8_t *allocate_buffer(coyote::cThread &coyote_thread, std::size_t bytes, bool mapped, bool hugepages) {
    const std::size_t allocation_bytes = round_up(bytes, coyote::PAGE_SIZE);

    if (mapped) {
        if (hugepages) {
            return reinterpret_cast<std::uint8_t *>(coyote_thread.getMem({coyote::CoyoteAllocType::HPF, allocation_bytes}));
        }
        return reinterpret_cast<std::uint8_t *>(coyote_thread.getMem({coyote::CoyoteAllocType::REG, allocation_bytes}));
    }

    if (hugepages) {
        return reinterpret_cast<std::uint8_t *>(mmap(nullptr, allocation_bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0));
    }

    return reinterpret_cast<std::uint8_t *>(aligned_alloc(coyote::PAGE_SIZE, allocation_bytes));
}

// Note, how the Coyote thread is passed by reference; to avoid creating a copy of 
// the thread object which can lead to undefined behaviour and bugs. 
int main(int argc, char *argv[])  {
    bool hugepages, mapped, stream;
    unsigned int n_runs;
    std::string vrp_file;

    boost::program_options::options_description runtime_options("Coyote CVRP Dual Stream");
    runtime_options.add_options()
        ("hugepages,h", boost::program_options::value<bool>(&hugepages)->default_value(true), "Use hugepages")
        ("mapped,m", boost::program_options::value<bool>(&mapped)->default_value(true), "Use mapped memory")
        ("stream,s", boost::program_options::value<bool>(&stream)->default_value(true), "Source / destination data stream: HOST(1) or FPGA(0)")
        ("runs,r", boost::program_options::value<unsigned int>(&n_runs)->default_value(1), "Number of times to repeat the test")
        ("vrp_file,f", boost::program_options::value<std::string>(&vrp_file)->default_value(kDefaultProblemFile), "Path to the CVRP instance file");
    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);

    HEADER("CLI PARAMETERS:");
    std::cout << "Enable hugepages: " << hugepages << std::endl;
    std::cout << "Enable mapped pages: " << mapped << std::endl;
    std::cout << "Data stream: " << (stream ? "HOST" : "CARD") << std::endl;
    std::cout << "Number of test runs: " << n_runs << std::endl;
    std::cout << "VRP file: " << vrp_file << std::endl << std::endl;

    const ProblemInstance problem = parse_vrp(vrp_file);
    const std::vector<Individual> population = build_initial_population(problem, problem.dimension * kPopulationMultiplier, kPopulationSeed);

    // Split the population vector into two equal halves
    const std::size_t half_pop_size = population.size() / 2;
    const std::vector<Individual> pop_0(population.begin(), population.begin() + half_pop_size);
    const std::vector<Individual> pop_1(population.begin() + half_pop_size, population.end());

    // Encode each split vector individually
    const std::vector<std::uint8_t> encoded_pop_0 = encode_population_fixed(problem, pop_0);
    const std::vector<std::uint8_t> encoded_pop_1 = encode_population_fixed(problem, pop_1);

    const std::size_t bytes_0 = encoded_pop_0.size();
    const std::size_t bytes_1 = encoded_pop_1.size();

    std::cout << "Loaded problem: " << problem.name << std::endl;
    std::cout << "Dimension: " << problem.dimension << std::endl;
    std::cout << "Vehicle capacity: " << problem.capacity << std::endl;
    std::cout << "Total population size: " << population.size() << " (Stream 0: " << pop_0.size() << ", Stream 1: " << pop_1.size() << ")" << std::endl;
    std::cout << "Stream 0 payload: " << bytes_0 << " B | Stream 1 payload: " << bytes_1 << " B" << std::endl << std::endl;

    // Obtain Coyote thread
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());

    // Allocate memory for both streams (2 inputs, 2 outputs)
    std::uint8_t *src_mem_0 = allocate_buffer(coyote_thread, bytes_0, mapped, hugepages);
    std::uint8_t *src_mem_1 = allocate_buffer(coyote_thread, bytes_1, mapped, hugepages);
    std::uint8_t *dst_mem_0 = allocate_buffer(coyote_thread, bytes_0, mapped, hugepages);
    std::uint8_t *dst_mem_1 = allocate_buffer(coyote_thread, bytes_1, mapped, hugepages);

    if (!src_mem_0 || !src_mem_1 || !dst_mem_0 || !dst_mem_1) {
        throw std::runtime_error("Could not allocate split buffers; exiting...");
    }

    // Copy encoded population data to respective allocated source buffers
    std::memcpy(src_mem_0, encoded_pop_0.data(), bytes_0);
    std::memcpy(src_mem_1, encoded_pop_1.data(), bytes_1);
    std::memset(dst_mem_0, 0, bytes_0);
    std::memset(dst_mem_1, 0, bytes_1);

    // Setup Scatter-Gather descriptors for Stream 0 and Stream 1
    coyote::localSg src_sg_0 = { .addr = src_mem_0, .len = bytes_0, .stream = stream, .dest = 0 };
    coyote::localSg src_sg_1 = { .addr = src_mem_1, .len = bytes_1, .stream = stream, .dest = 1 };

    coyote::localSg dst_sg_0 = { .addr = dst_mem_0, .len = bytes_0, .stream = stream, .dest = 0 };
    coyote::localSg dst_sg_1 = { .addr = dst_mem_1, .len = bytes_1, .stream = stream, .dest = 1 };

    const uint32_t reg_num_ind0 = static_cast<uint32_t>(pop_0.size());
    const uint32_t reg_num_ind1 = static_cast<uint32_t>(pop_1.size());
    // depot is excluded from the number of cities
    const uint32_t reg_num_cities = problem.dimension - 1;

    // Set the kernel configuration registers; the free-running HLS kernel starts
    // processing as soon as beats_per_individual reads non-zero, so it must be
    // written last (see hw/src/hls/processing_node/processing_node.cpp)
    coyote_thread.setCSR(reg_num_ind0, CSR_NUM_INDIVIDUALS);
    coyote_thread.setCSR(reg_num_cities, CSR_NUM_CITIES);
    coyote_thread.setCSR(problem.capacity, CSR_VEHICLE_CAPACITY);
    coyote_thread.setCSR(problem.depot.x << COORD_BITS | problem.depot.y, CSR_DEPOT_COORDS_CAPACITY);
    coyote_thread.setCSR(CSR_CONTROL_AUTORESTART, CSR_CONTROL);

    std::cout << coyote_thread.getCSR(CSR_NUM_INDIVIDUALS) << " individuals on Stream 0" << std::endl;
    std::cout << coyote_thread.getCSR(CSR_NUM_CITIES) << " num cities" << std::endl << std::endl;
    std::cout << coyote_thread.getCSR(CSR_VEHICLE_CAPACITY) << " vehicle capacity" << std::endl << std::endl;
    std::cout << "Depot coordinates: (" << (coyote_thread.getCSR(CSR_DEPOT_COORDS_CAPACITY) >> COORD_BITS) << ", " << (coyote_thread.getCSR(CSR_DEPOT_COORDS_CAPACITY) & ((1 << COORD_BITS) - 1)) << ")" << std::endl << std::endl;

    HEADER("PERF DUAL STREAM LOCAL");
    std::cout << "Streaming encoded individuals across 2 streams..." << std::endl;

    coyote::cBench bench(n_runs);

    auto prep_fn = [&]() {
        coyote_thread.clearCompleted();
        std::memset(dst_mem_0, 0, bytes_0);
        std::memset(dst_mem_1, 0, bytes_1);
    };

    auto bench_fn = [&]() {
        for (uint i = 0; i < N_THROUGHPUT_REPS; ++i) {
            // Queue operations on Stream 0 and Stream 1
            coyote_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_0, dst_sg_0);
            coyote_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_1, dst_sg_1);
        }
        // Wait for both stream completions (2 transfers per iteration)
        while (coyote_thread.checkCompleted(coyote::CoyoteOper::LOCAL_TRANSFER) != (2 * N_THROUGHPUT_REPS)) {}
    };

    bench.execute(bench_fn, prep_fn);

    if (!stream) {
        coyote::syncSg sg_sync;
        sg_sync = {.addr = src_mem_0, .len = bytes_0 }; coyote_thread.invoke(coyote::CoyoteOper::LOCAL_SYNC, sg_sync);
        sg_sync = {.addr = src_mem_1, .len = bytes_1 }; coyote_thread.invoke(coyote::CoyoteOper::LOCAL_SYNC, sg_sync);
        sg_sync = {.addr = dst_mem_0, .len = bytes_0 }; coyote_thread.invoke(coyote::CoyoteOper::LOCAL_SYNC, sg_sync);
        sg_sync = {.addr = dst_mem_1, .len = bytes_1 }; coyote_thread.invoke(coyote::CoyoteOper::LOCAL_SYNC, sg_sync);
    }

    const double throughput_time = bench.getAvg();
    const double total_transferred_bytes = (bytes_0 + bytes_1) * N_THROUGHPUT_REPS;
    const double throughput = total_transferred_bytes / (1024.0 * 1024.0 * throughput_time * 1e-9);
    std::cout << "Average throughput: " << std::setw(8) << throughput << " MB/s" << std::endl;

    // Decoding output streams after execution
    std::vector<Individual> result_pop_0 = decode_population_fixed(problem, dst_mem_0, pop_0.size());
    std::vector<Individual> result_pop_1 = decode_population_fixed(problem, dst_mem_1, pop_1.size());

    for(int i=0; i < static_cast<int>(pop_0.size()); i++) {
        if (result_pop_0[i] != pop_0[i]) {
            std::cerr << "Mismatch in Stream 0 at index " << i << ":\n -" << result_pop_0[i] << "\n -" << pop_0[i] << std::endl;
            return EXIT_FAILURE;
        }
        if (result_pop_1[i] != pop_1[i]) {
            std::cerr << "Mismatch in Stream 1 at index " << i << ":\n -" << result_pop_1[i] << "\n -" << pop_1[i] << std::endl;
            return EXIT_FAILURE;
        }
    }

    if (!mapped) {
        const std::size_t alloc_0 = round_up(bytes_0, coyote::PAGE_SIZE);
        const std::size_t alloc_1 = round_up(bytes_1, coyote::PAGE_SIZE);
        if (!hugepages) { 
            free(src_mem_0); free(src_mem_1); free(dst_mem_0); free(dst_mem_1); 
        } else { 
            munmap(src_mem_0, alloc_0); munmap(src_mem_1, alloc_1); 
            munmap(dst_mem_0, alloc_0); munmap(dst_mem_1, alloc_1); 
        }
    }

    return EXIT_SUCCESS;
}
