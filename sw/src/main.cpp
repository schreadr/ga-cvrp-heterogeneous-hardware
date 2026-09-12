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
#include <iostream>
#include <iomanip>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/program_options.hpp>
#include <coyote/cBench.hpp>
#include <coyote/cThread.hpp>

#include "ga_encoding_constants.hpp"
#include "ga_streaming.hpp"
#include "ga.hpp"
#include "problem_parser.hpp"

// Default vFPGA to assign cThreads to; for designs with one region (vFPGA) this is the only possible value
#define DEFAULT_VFPGA_ID 0

// Control register indices, matching the AXI4-Lite control register map generated
// by Vitis HLS for the scalar configuration arguments of the processing_node kernel
// (see hw/src/hls/processing_node/processing_node.cpp). The byte offsets of the
// kernel registers are 0x10 (num_individuals), 0x18 (num_cities), 0x20 (vehicle_capacity),
// and 0x28 (depot_cords); offsets 0x00-0x0F are reserved by the AXI4-Lite control block.
// Coyote CSRs are 64-bit wide, so the setCSR index equals byte offset / 8.
constexpr uint32_t CSR_CONTROL                 = 0x00;
constexpr uint32_t CSR_CONTROL_AUTORESTART     = 0x81;
constexpr uint32_t CSR_NUM_INDIVIDUALS         = 0x10 / 8;
constexpr uint32_t CSR_NUM_CITIES              = 0x18 / 8;
constexpr uint32_t CSR_VEHICLE_CAPACITY        = 0x20 / 8;
constexpr uint32_t CSR_DEPOT_COORDS_CAPACITY   = 0x28 / 8;

constexpr const char kDefaultProblemFile[] = "../../P-n40-k5_edited.vrp";

std::size_t round_up(std::size_t value, std::size_t alignment) {
    return ((value + alignment - 1) / alignment) * alignment;
}

std::uint8_t *allocate_buffer(coyote::cThread &coyote_thread, std::size_t bytes) {
    const std::size_t allocation_bytes = round_up(bytes, coyote::PAGE_SIZE);
    return reinterpret_cast<std::uint8_t *>(coyote_thread.getMem({coyote::CoyoteAllocType::HPF, allocation_bytes}));
}

int main(int argc, char *argv[]) {
    std::size_t population_size_multiplier = 0;
    std::size_t n_generations = 0;
    double n_best_keep = 0.0;
    std::uint32_t seed = 0;
    bool smoke_test = false;
    std::string vrp_file;

    boost::program_options::options_description runtime_options("Coyote CVRP GA Options");
    runtime_options.add_options()
        ("help,h", "Produce help message")
        ("population_size_multiplier,p", boost::program_options::value<std::size_t>(&population_size_multiplier)->default_value(20), "Population size multiplier (population_size = dimension * multiplier)")
        ("n_generations,g", boost::program_options::value<std::size_t>(&n_generations)->default_value(100), "Number of generations")
        ("n_best_keep,k", boost::program_options::value<double>(&n_best_keep)->default_value(5.0), "Percentage of best individuals to keep across generations (e.g. 5.0 for 5%)")
        ("seed,s", boost::program_options::value<std::uint32_t>(&seed)->default_value(0xC0FFEEu), "Random seed")
        ("smoke_test,t", boost::program_options::bool_switch(&smoke_test), "Run a single generation smoke test on FPGA and exit")
        ("vrp_file,f", boost::program_options::value<std::string>(&vrp_file)->default_value(kDefaultProblemFile), "Path to the CVRP instance file");

    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);

    if (command_line_arguments.count("help")) {
        std::cout << runtime_options << std::endl;
        return EXIT_SUCCESS;
    }

    if (population_size_multiplier == 0) {
        throw std::invalid_argument("population_size_multiplier must be greater than 0");
    }

    HEADER("CLI PARAMETERS:");
    std::cout << "Population size multiplier: " << population_size_multiplier << std::endl;
    std::cout << "Number of generations: " << n_generations << std::endl;
    std::cout << "Percentage of best individuals to keep: " << n_best_keep << "%" << std::endl;
    std::cout << "Seed: " << seed << std::endl;
    std::cout << "Smoke test mode: " << (smoke_test ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Genome hasher: " << get_hash_backend_name() << std::endl;
    std::cout << "VRP file: " << vrp_file << std::endl << std::endl;

    const ProblemInstance problem = parse_vrp(vrp_file);
    validate_encoding_constants(problem);

    std::size_t pop_size = problem.dimension * population_size_multiplier;
    if (pop_size % 2 != 0) {
        pop_size += 1;
    }
    const std::size_t num_ind_per_stream = pop_size / 2;
    const int beats_per_individual = get_total_beats(problem.dimension - 1);
    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;
    const std::size_t pop_bytes = pop_size * bytes_per_individual;
    const std::size_t half_pop_bytes = num_ind_per_stream * bytes_per_individual;
    const std::size_t random_bytes = get_random_bytes(num_ind_per_stream);

    std::cout << "Loaded problem: " << problem.name << std::endl;
    std::cout << "Dimension: " << problem.dimension << std::endl;
    std::cout << "Vehicle capacity: " << problem.capacity << std::endl;
    std::cout << "Total population size: " << pop_size << std::endl;

    // Obtain Coyote thread
    coyote::cThread coyote_thread(DEFAULT_VFPGA_ID, getpid());

    // Allocate memory in Coyote style using hugepages (HPF)
    std::uint8_t *src_mem = allocate_buffer(coyote_thread, pop_bytes);
    std::uint8_t *src_mem_rnd = allocate_buffer(coyote_thread, random_bytes);
    std::uint8_t *dst_mem = allocate_buffer(coyote_thread, pop_bytes);
    std::uint8_t *pop_mem = allocate_buffer(coyote_thread, pop_bytes);

    if (!src_mem || !src_mem_rnd || !dst_mem || !pop_mem) {
        throw std::runtime_error("Could not allocate Coyote hugepage buffers; exiting...");
    }

    // Setup Scatter-Gather descriptors for Stream 0, Stream 1, and Stream 2 (random numbers)
    coyote::localSg src_sg_0 = { .addr = src_mem, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 0 };
    coyote::localSg src_sg_1 = { .addr = src_mem + half_pop_bytes, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 1 };
    coyote::localSg src_sg_rnd = { .addr = src_mem_rnd, .len = static_cast<uint32_t>(random_bytes), .stream = true, .dest = 2 };

    coyote::localSg dst_sg_0 = { .addr = dst_mem, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 0 };
    coyote::localSg dst_sg_1 = { .addr = dst_mem + half_pop_bytes, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 1 };

    // Program kernel configuration registers
    const uint32_t reg_num_ind = static_cast<uint32_t>(num_ind_per_stream);
    const uint32_t reg_num_cities = problem.dimension - 1;

    coyote_thread.setCSR(reg_num_ind, CSR_NUM_INDIVIDUALS);
    coyote_thread.setCSR(reg_num_cities, CSR_NUM_CITIES);
    coyote_thread.setCSR(problem.capacity, CSR_VEHICLE_CAPACITY);
    coyote_thread.setCSR((problem.depot.x << COORD_BITS) | problem.depot.y, CSR_DEPOT_COORDS_CAPACITY);
    coyote_thread.setCSR(CSR_CONTROL_AUTORESTART, CSR_CONTROL);


    if(coyote_thread.getCSR(CSR_NUM_INDIVIDUALS) == reg_num_ind &&
       coyote_thread.getCSR(CSR_NUM_CITIES) == reg_num_cities &&
       coyote_thread.getCSR(CSR_VEHICLE_CAPACITY) == problem.capacity &&
       coyote_thread.getCSR(CSR_DEPOT_COORDS_CAPACITY) == ((problem.depot.x << COORD_BITS) | problem.depot.y)) {
        std::cout << "Successfully programmed vFPGA kernel configuration registers." << std::endl;
    } else {
        std::cerr << "Error: Failed to program vFPGA kernel configuration registers." << std::endl;
        return EXIT_FAILURE;
    }

    auto create_new_generation = [&]() {
        coyote_thread.clearCompleted();
        // Queue operations on Stream 2 (random numbers), Stream 0 (parents 1), Stream 1 (parents 2)
        coyote_thread.invoke(coyote::CoyoteOper::LOCAL_READ, src_sg_rnd);
        coyote_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_0, dst_sg_0);
        coyote_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_1, dst_sg_1);

        // Wait for transfers to complete
        while (coyote_thread.checkCompleted(coyote::CoyoteOper::LOCAL_TRANSFER) != 2) {}
    };

    if (smoke_test) {
        HEADER("SMOKE TEST (1 GENERATION ON FPGA)");
        std::cout << "Generating initial population for smoke test..." << std::endl;
        const std::vector<Individual> init_pop_0 = build_initial_population(problem, num_ind_per_stream, seed);
        const std::vector<Individual> init_pop_1 = build_initial_population(problem, num_ind_per_stream, seed ^ 0xBEEF);

        const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, init_pop_0);
        const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, init_pop_1);

        std::memcpy(src_mem, enc_0.data(), half_pop_bytes);
        std::memcpy(src_mem + half_pop_bytes, enc_1.data(), half_pop_bytes);
        std::memset(dst_mem, 0, pop_bytes);

        generate_random_numbers(num_ind_per_stream, src_mem_rnd, reg_num_cities, seed ^ 0xDEADBEEF);

        std::cout << "Invoking FPGA for 1 generation..." << std::endl;
        coyote::cBench bench(1, 0);
        bench.execute(create_new_generation, [](){});
        double throughput = ((double) pop_bytes) / (1024.0 * 1024.0 * 1e-9 * bench.getAvg());
        std::cout << "Average throughput: " << std::setw(8) << throughput << " MB/s; " << std::endl;

        std::cout << "Decoding output individuals and verifying fitness..." << std::endl;
        const std::vector<Individual> children = decode_population_fixed(problem, dst_mem, pop_size);

        int errors = 0;
        for (std::size_t i = 0; i < pop_size; ++i) {
            uint32_t exp_fit = compute_fitness(problem, children[i]);
            if (children[i].fitness != exp_fit) {
                std::cerr << "Mismatch in individual " << i << ": got "
                          << children[i].fitness << ", expected " << exp_fit << std::endl;
                errors++;
            }
        }

        if (errors == 0) {
            std::cout << "SMOKE TEST PASSED: Processed " << pop_size << " individuals successfully on FPGA." << std::endl;
            return EXIT_SUCCESS;
        } else {
            std::cerr << "SMOKE TEST FAILED with " << errors << " errors." << std::endl;
            return EXIT_FAILURE;
        }
    } else {
        HEADER("RUNNING GA ON FPGA");
        GAParams params;
        params.pop_size = pop_size;
        params.n_generations = n_generations;
        params.n_best_keep = n_best_keep;
        params.seed = seed;
        params.buffer_src = src_mem;
        params.buffer_src_rnd = src_mem_rnd;
        params.buffer_dst = dst_mem;
        params.buffer_pop = pop_mem;

        GASummary summary = run_ga(problem, params, create_new_generation);
        summary.print();
    }

    return EXIT_SUCCESS;
}
