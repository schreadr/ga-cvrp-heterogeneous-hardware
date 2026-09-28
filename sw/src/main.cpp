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
#include <memory>

#include <boost/program_options.hpp>
#include <coyote/cBench.hpp>
#include <coyote/cThread.hpp>

#include "ga_encoding_constants.hpp"
#include "ga_streaming.hpp"
#include "ga.hpp"
#include "batch_job.hpp"
#include "safe_queue.hpp"
#include "problem_parser.hpp"

// Default vFPGA to assign cThreads to; for designs with one region (vFPGA) this is the only possible value
#define DEFAULT_VFPGA_ID 0

// Control register indices, matching the AXI4-Lite control register map generated
// by Vitis HLS for the scalar configuration arguments of the processing_node kernel
// (see hw/src/hls/processing_node/processing_node.cpp). The byte offsets of the
// kernel registers are 0x10 (num_individuals), 0x18 (num_cities), 0x20 (vehicle_capacity),
// 0x28 (init_city_cache), 0x30 (two_opt_k), 0x38 (two_opt_enable_precheck), and 0x40 (seed); offsets 0x00-0x0F are reserved by the AXI4-Lite control block.
// Coyote CSRs are 64-bit wide, so the setCSR index equals byte offset / 8.
constexpr uint32_t CSR_CONTROL                 = 0x00;
constexpr uint32_t CSR_CONTROL_START           = 0x01;
constexpr uint32_t CSR_NUM_INDIVIDUALS         = 0x10 / 8;
constexpr uint32_t CSR_NUM_CITIES              = 0x18 / 8;
constexpr uint32_t CSR_VEHICLE_CAPACITY        = 0x20 / 8;
constexpr uint32_t CSR_INIT_CITY_CACHE         = 0x28 / 8;
constexpr uint32_t CSR_TWO_OPT_K               = 0x30 / 8;
constexpr uint32_t CSR_TWO_OPT_ENABLE_PRECHECK = 0x38 / 8;
constexpr uint32_t CSR_SEED                    = 0x40 / 8;
constexpr uint32_t CSR_CROSSOVER_PROB          = 0x48 / 8;
constexpr uint32_t CSR_MUTATION_PROB           = 0x50 / 8;

constexpr const char kDefaultProblemFile[] = "../../problem_instances/P-n40-k5.vrp";

std::size_t round_up(std::size_t value, std::size_t alignment) {
    return ((value + alignment - 1) / alignment) * alignment;
}

std::uint8_t *allocate_buffer(coyote::cThread &coyote_thread, std::size_t bytes) {
    const std::size_t allocation_bytes = round_up(bytes, coyote::HUGE_PAGE_SIZE);
    return reinterpret_cast<std::uint8_t *>(coyote_thread.getMem({coyote::CoyoteAllocType::HPF, allocation_bytes}));
}

bool run_smoke_test(int vfpga_id,
                    const ProblemInstance &problem,
                    std::size_t pop_size,
                    int two_opt_k,
                    bool two_opt_enable_precheck,
                    std::uint32_t seed) {
    HEADER("SMOKE TEST (1 GENERATION ON vFPGA " + std::to_string(vfpga_id) + ")");
    coyote::cThread coyote_thread(vfpga_id, getpid());

    const std::size_t num_ind_per_stream = pop_size / 2;
    const int beats_per_individual = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;
    const std::size_t pop_bytes = pop_size * bytes_per_individual;
    const std::size_t half_pop_bytes = num_ind_per_stream * bytes_per_individual;

    std::uint8_t *src_mem_0 = allocate_buffer(coyote_thread, half_pop_bytes);
    std::uint8_t *src_mem_1 = allocate_buffer(coyote_thread, half_pop_bytes);
    std::uint8_t *dst_mem_0 = allocate_buffer(coyote_thread, half_pop_bytes);
    std::uint8_t *dst_mem_1 = allocate_buffer(coyote_thread, half_pop_bytes);

    if (!src_mem_0 || !src_mem_1 || !dst_mem_0 || !dst_mem_1) {
        throw std::runtime_error("Could not allocate Coyote hugepage buffers for vFPGA " + std::to_string(vfpga_id) + "; exiting...");
    }

    coyote::localSg src_sg_0 = { .addr = src_mem_0, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 0 };
    coyote::localSg src_sg_1 = { .addr = src_mem_1, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 1 };
    coyote::localSg dst_sg_0 = { .addr = dst_mem_0, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 0 };
    coyote::localSg dst_sg_1 = { .addr = dst_mem_1, .len = static_cast<uint32_t>(half_pop_bytes), .stream = true, .dest = 1 };

    const uint32_t reg_num_ind = static_cast<uint32_t>(num_ind_per_stream);
    const uint32_t reg_num_cities = problem.dimension - 1;

    std::cout << "Initializing node cache on vFPGA " << vfpga_id << " beforehand via stream 0..." << std::endl;
    const std::vector<uint8_t> cache_bytes = encode_node_cache(problem);
    std::uint8_t *cache_mem = allocate_buffer(coyote_thread, cache_bytes.size());
    if (!cache_mem) {
        throw std::runtime_error("Could not allocate Coyote hugepage buffer for cache on vFPGA " + std::to_string(vfpga_id) + "; exiting...");
    }
    std::memcpy(cache_mem, cache_bytes.data(), cache_bytes.size());

    coyote::localSg cache_sg = {
        .addr = cache_mem,
        .len = static_cast<uint32_t>(cache_bytes.size()),
        .stream = true,
        .dest = 0
    };

    coyote_thread.setCSR(reg_num_ind, CSR_NUM_INDIVIDUALS);
    coyote_thread.setCSR(reg_num_cities, CSR_NUM_CITIES);
    coyote_thread.setCSR(problem.capacity, CSR_VEHICLE_CAPACITY);
    coyote_thread.setCSR(two_opt_k, CSR_TWO_OPT_K);
    coyote_thread.setCSR(two_opt_enable_precheck ? 1 : 0, CSR_TWO_OPT_ENABLE_PRECHECK);
    coyote_thread.setCSR(seed, CSR_SEED);
    coyote_thread.setCSR(100, CSR_CROSSOVER_PROB);
    coyote_thread.setCSR(100, CSR_MUTATION_PROB);

    coyote_thread.setCSR(1, CSR_INIT_CITY_CACHE);
    coyote_thread.setCSR(CSR_CONTROL_START, CSR_CONTROL);

    coyote_thread.invoke(coyote::CoyoteOper::LOCAL_READ, cache_sg);
    while (coyote_thread.checkCompleted(coyote::CoyoteOper::LOCAL_READ) != 1) {}
    std::cout << "Node cache successfully initialized on vFPGA " << vfpga_id << "." << std::endl;
    coyote_thread.setCSR(0, CSR_INIT_CITY_CACHE);

    auto create_new_generation = [&]() {
        coyote_thread.clearCompleted();
        coyote_thread.setCSR(CSR_CONTROL_START, CSR_CONTROL);
        coyote_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_0, dst_sg_0);
        coyote_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_1, dst_sg_1);
        while (coyote_thread.checkCompleted(coyote::CoyoteOper::LOCAL_TRANSFER) != 2) {}
    };

    std::cout << "Generating initial population for smoke test on vFPGA " << vfpga_id << "..." << std::endl;
    const std::vector<Individual> init_pop_0 = build_initial_population(problem, num_ind_per_stream, seed);
    const std::vector<Individual> init_pop_1 = build_initial_population(problem, num_ind_per_stream, seed ^ 0xBEEF);

    const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, init_pop_0);
    const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, init_pop_1);

    std::memcpy(src_mem_0, enc_0.data(), half_pop_bytes);
    std::memcpy(src_mem_1, enc_1.data(), half_pop_bytes);
    std::memset(dst_mem_0, 0, half_pop_bytes);
    std::memset(dst_mem_1, 0, half_pop_bytes);

    std::cout << "Invoking vFPGA " << vfpga_id << " for 1 generation..." << std::endl;
    coyote::cBench bench(1, 0);
    bench.execute(create_new_generation, [](){});
    double throughput = ((double) pop_bytes) / (1024.0 * 1024.0 * 1e-9 * bench.getAvg());
    std::cout << "Average throughput: " << std::setw(8) << throughput << " MB/s; " << std::endl;

    std::cout << "Decoding output individuals and verifying fitness..." << std::endl;
    const std::vector<Individual> children_0 = decode_population_fixed(problem, dst_mem_0, num_ind_per_stream);
    const std::vector<Individual> children_1 = decode_population_fixed(problem, dst_mem_1, num_ind_per_stream);

    std::vector<Individual> children;
    children.reserve(pop_size);
    children.insert(children.end(), children_0.begin(), children_0.end());
    children.insert(children.end(), children_1.begin(), children_1.end());

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
        std::cout << "SMOKE TEST PASSED on vFPGA " << vfpga_id << ": Processed " << pop_size << " individuals successfully on FPGA." << std::endl;
        return true;
    } else {
        std::cerr << "SMOKE TEST FAILED on vFPGA " << vfpga_id << " with " << errors << " errors." << std::endl;
        return false;
    }
}

int main(int argc, char *argv[]) {
    std::size_t population_size_multiplier = 0;
    std::size_t n_generations = 0;
    double n_best_keep = 0.0;
    std::uint32_t seed = 0;
    bool smoke_test = false;
    std::string vrp_file;
    int two_opt_k = 0;
    bool two_opt_enable_precheck = true;
    int crossover_prob = 100;
    int mutation_prob = 100;
    unsigned int num_vfpgas = 2;
    std::size_t micro_batch_size = 0;

    boost::program_options::options_description runtime_options("Coyote CVRP GA Options");
    runtime_options.add_options()
        ("help,h", "Produce help message")
        ("num_vfpgas,n", boost::program_options::value<unsigned int>(&num_vfpgas)->default_value(2), "Number of Coyote vFPGAs to use simultaneously")
        ("batch_size,b", boost::program_options::value<std::size_t>(&micro_batch_size)->default_value(0), "Micro-batch size per stream (0 = auto-calculate based on pop_size / (2 * num_vfpgas))")
        ("population_size_multiplier,p", boost::program_options::value<std::size_t>(&population_size_multiplier)->default_value(20), "Population size multiplier (population_size = dimension * multiplier)")
        ("n_generations,g", boost::program_options::value<std::size_t>(&n_generations)->default_value(100), "Number of generations")
        ("n_best_keep,k", boost::program_options::value<double>(&n_best_keep)->default_value(5.0), "Percentage of best individuals to keep across generations (e.g. 5.0 for 5%)")
        ("seed,s", boost::program_options::value<std::uint32_t>(&seed)->default_value(0xC0FFEEu), "Random seed")
        ("smoke_test,t", boost::program_options::bool_switch(&smoke_test), "Run a single generation smoke test on FPGA and exit")
        ("vrp_file,f", boost::program_options::value<std::string>(&vrp_file)->default_value(kDefaultProblemFile), "Path to the CVRP instance file")
        ("two_opt_k,o", boost::program_options::value<int>(&two_opt_k)->default_value(0), "Number of 2-opt iterations per subtour")
        ("two_opt_enable_precheck", boost::program_options::value<bool>(&two_opt_enable_precheck)->default_value(true), "Enable precheck for 2-opt moves")
        ("crossover_prob", boost::program_options::value<int>(&crossover_prob)->default_value(100), "Crossover probability (0 to 100)")
        ("mutation_prob", boost::program_options::value<int>(&mutation_prob)->default_value(100), "Mutation probability (0 to 100)");

    boost::program_options::variables_map command_line_arguments;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, runtime_options), command_line_arguments);
    boost::program_options::notify(command_line_arguments);

    if (command_line_arguments.count("help")) {
        std::cout << runtime_options << std::endl;
        return EXIT_SUCCESS;
    }

    if (num_vfpgas == 0) {
        throw std::invalid_argument("num_vfpgas must be greater than 0");
    }

    if (population_size_multiplier == 0) {
        throw std::invalid_argument("population_size_multiplier must be greater than 0");
    }

    if (crossover_prob < 0 || crossover_prob > 100) {
        throw std::invalid_argument("crossover_prob must be between 0 and 100");
    }

    if (mutation_prob < 0 || mutation_prob > 100) {
        throw std::invalid_argument("mutation_prob must be between 0 and 100");
    }

    HEADER("CLI PARAMETERS:");
    std::cout << "Number of vFPGAs: " << num_vfpgas << std::endl;
    std::cout << "Micro-batch size: " << (micro_batch_size == 0 ? "AUTO" : std::to_string(micro_batch_size)) << std::endl;
    std::cout << "Population size multiplier: " << population_size_multiplier << std::endl;
    std::cout << "Number of generations: " << n_generations << std::endl;
    std::cout << "Percentage of best individuals to keep: " << n_best_keep << "%" << std::endl;
    std::cout << "Seed: " << seed << std::endl;
    std::cout << "Two-opt k: " << two_opt_k << std::endl;
    std::cout << "Two-opt enable precheck: " << (two_opt_enable_precheck ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Crossover probability: " << crossover_prob << "%" << std::endl;
    std::cout << "Mutation probability: " << mutation_prob << "%" << std::endl;
    std::cout << "Smoke test mode: " << (smoke_test ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Genome hasher: " << get_hash_backend_name() << std::endl;
    std::cout << "VRP file: " << vrp_file << std::endl << std::endl;

    const ProblemInstance problem = parse_vrp(vrp_file);
    validate_encoding_constants(problem);

    std::size_t pop_size = problem.dimension * population_size_multiplier;
    if (pop_size % 2 != 0) {
        pop_size += 1;
    }
    const int beats_per_individual = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;
    const std::size_t pop_bytes = pop_size * bytes_per_individual;
    const int max_subtours = calc_max_n_subtours(problem);

    std::cout << "Loaded problem: " << problem.name << std::endl;
    std::cout << "Dimension: " << problem.dimension << std::endl;
    std::cout << "Vehicle capacity: " << problem.capacity << std::endl;
    std::cout << "Precision: " << problem.scaling << std::endl;
    std::cout << "Total population size: " << pop_size << std::endl;
    std::cout << "Max subtours: " << max_subtours << std::endl;

    if (smoke_test) {
        HEADER("SMOKE TEST (1 GENERATION ON EACH vFPGA)");
        bool all_passed = true;
        for (unsigned int vfid = 0; vfid < num_vfpgas; ++vfid) {
            if (!run_smoke_test(vfid, problem, pop_size, two_opt_k, two_opt_enable_precheck, seed)) {
                all_passed = false;
            }
        }
        if (all_passed) {
            std::cout << "ALL SMOKE TESTS PASSED (" << num_vfpgas << " vFPGA(s))." << std::endl;
            return EXIT_SUCCESS;
        } else {
            std::cerr << "SMOKE TEST FAILED on one or more vFPGAs." << std::endl;
            return EXIT_FAILURE;
        }
    } else {
        // Pipelined Multi-vFPGA GA execution across num_vfpgas
        std::vector<std::unique_ptr<coyote::cThread>> coyote_threads;
        coyote_threads.reserve(num_vfpgas);
        for (unsigned int vfid = 0; vfid < num_vfpgas; ++vfid) {
            coyote_threads.emplace_back(std::make_unique<coyote::cThread>(vfid, getpid()));
        }

        // Determine micro-batch size
        std::size_t batch_ind_per_stream = micro_batch_size;
        if (batch_ind_per_stream == 0) {
            batch_ind_per_stream = (pop_size / 2) / num_vfpgas;
        }
        if (batch_ind_per_stream == 0) {
            batch_ind_per_stream = 1;
        }
        const std::size_t batch_bytes = batch_ind_per_stream * bytes_per_individual;

        std::cout << "Micro-batch size: " << batch_ind_per_stream << " individuals/stream ("
                  << (2 * batch_ind_per_stream) << " offspring per batch, "
                  << batch_bytes << " bytes per stream)" << std::endl;

        // Program kernel configuration registers on all vFPGAs
        const uint32_t reg_num_ind = static_cast<uint32_t>(batch_ind_per_stream);
        const uint32_t reg_num_cities = problem.dimension - 1;

        for (unsigned int vfid = 0; vfid < num_vfpgas; ++vfid) {
            const uint32_t vfpga_seed = seed ^ (0x9E3779B9u * (vfid + 1));
            coyote_threads[vfid]->setCSR(reg_num_ind, CSR_NUM_INDIVIDUALS);
            coyote_threads[vfid]->setCSR(reg_num_cities, CSR_NUM_CITIES);
            coyote_threads[vfid]->setCSR(problem.capacity, CSR_VEHICLE_CAPACITY);
            coyote_threads[vfid]->setCSR(two_opt_k, CSR_TWO_OPT_K);
            coyote_threads[vfid]->setCSR(two_opt_enable_precheck ? 1 : 0, CSR_TWO_OPT_ENABLE_PRECHECK);
            coyote_threads[vfid]->setCSR(vfpga_seed, CSR_SEED);
            coyote_threads[vfid]->setCSR(crossover_prob, CSR_CROSSOVER_PROB);
            coyote_threads[vfid]->setCSR(mutation_prob, CSR_MUTATION_PROB);

            if (coyote_threads[vfid]->getCSR(CSR_NUM_INDIVIDUALS) != reg_num_ind ||
                coyote_threads[vfid]->getCSR(CSR_NUM_CITIES) != reg_num_cities ||
                coyote_threads[vfid]->getCSR(CSR_VEHICLE_CAPACITY) != static_cast<uint64_t>(problem.capacity)) {
                std::cerr << "Error: Failed to program vFPGA " << vfid << " kernel configuration registers." << std::endl;
                return EXIT_FAILURE;
            }
        }
        std::cout << "Successfully programmed vFPGA kernel configuration registers on all " << num_vfpgas << " vFPGAs." << std::endl;

        // Initialize node cache on all vFPGAs beforehand via Stream 0
        std::cout << "Initializing node cache on " << num_vfpgas << " vFPGA(s) beforehand via stream 0..." << std::endl;
        const std::vector<uint8_t> cache_bytes = encode_node_cache(problem);
        for (unsigned int vfid = 0; vfid < num_vfpgas; ++vfid) {
            std::uint8_t *cache_mem = allocate_buffer(*coyote_threads[vfid], cache_bytes.size());
            if (!cache_mem) {
                throw std::runtime_error("Could not allocate Coyote hugepage buffer for cache on vFPGA " + std::to_string(vfid));
            }
            std::memcpy(cache_mem, cache_bytes.data(), cache_bytes.size());

            coyote::localSg cache_sg = {
                .addr = cache_mem,
                .len = static_cast<uint32_t>(cache_bytes.size()),
                .stream = true,
                .dest = 0
            };

            coyote_threads[vfid]->clearCompleted();
            coyote_threads[vfid]->setCSR(1, CSR_INIT_CITY_CACHE);
            coyote_threads[vfid]->setCSR(CSR_CONTROL_START, CSR_CONTROL);

            coyote_threads[vfid]->invoke(coyote::CoyoteOper::LOCAL_READ, cache_sg);
            while (coyote_threads[vfid]->checkCompleted(coyote::CoyoteOper::LOCAL_READ) != 1) {}
            coyote_threads[vfid]->setCSR(0, CSR_INIT_CITY_CACHE);
            coyote_threads[vfid]->clearCompleted();
        }
        std::cout << "Node cache successfully initialized on all " << num_vfpgas << " vFPGA(s)." << std::endl;

        // Allocate double-buffered slots per vFPGA
        const std::size_t slots_per_vfpga = 2;
        std::vector<BatchBufferSlot> buffer_slots;
        for (unsigned int vfid = 0; vfid < num_vfpgas; ++vfid) {
            for (std::size_t s = 0; s < slots_per_vfpga; ++s) {
                std::uint8_t *src_0 = allocate_buffer(*coyote_threads[vfid], batch_bytes);
                std::uint8_t *src_1 = allocate_buffer(*coyote_threads[vfid], batch_bytes);
                std::uint8_t *dst_0 = allocate_buffer(*coyote_threads[vfid], batch_bytes);
                std::uint8_t *dst_1 = allocate_buffer(*coyote_threads[vfid], batch_bytes);

                if (!src_0 || !src_1 || !dst_0 || !dst_1) {
                    throw std::runtime_error("Could not allocate Coyote hugepage buffers for vFPGA " + std::to_string(vfid));
                }

                BatchBufferSlot slot;
                slot.slot_id = static_cast<int>(vfid * slots_per_vfpga + s);
                slot.vfid = static_cast<int>(vfid);
                slot.num_ind_per_stream = batch_ind_per_stream;
                slot.src_buf_0 = src_0;
                slot.src_buf_1 = src_1;
                slot.dst_buf_0 = dst_0;
                slot.dst_buf_1 = dst_1;
                buffer_slots.push_back(slot);
            }
        }

        // Host population scratchpad
        std::unique_ptr<std::uint8_t[], void(*)(void*)> pop_mem(nullptr, std::free);
        void *ptr = nullptr;
        if (posix_memalign(&ptr, AXI_BYTES, pop_bytes) != 0 || !ptr) {
            throw std::bad_alloc();
        }
        pop_mem.reset(reinterpret_cast<std::uint8_t *>(ptr));

        // Hardware batch execution callback
        auto execute_vfpga_batch = [&](int vfid, const BatchJob &job) {
            auto &c_thread = *coyote_threads[vfid];
            c_thread.clearCompleted();
            c_thread.setCSR(job.batch_seed, CSR_SEED);
            c_thread.setCSR(CSR_CONTROL_START, CSR_CONTROL);

            coyote::localSg src_sg_0 = { .addr = job.src_buf_0, .len = static_cast<uint32_t>(batch_bytes), .stream = true, .dest = 0 };
            coyote::localSg src_sg_1 = { .addr = job.src_buf_1, .len = static_cast<uint32_t>(batch_bytes), .stream = true, .dest = 1 };
            coyote::localSg dst_sg_0 = { .addr = job.dst_buf_0, .len = static_cast<uint32_t>(batch_bytes), .stream = true, .dest = 0 };
            coyote::localSg dst_sg_1 = { .addr = job.dst_buf_1, .len = static_cast<uint32_t>(batch_bytes), .stream = true, .dest = 1 };

            c_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_0, dst_sg_0);
            c_thread.invoke(coyote::CoyoteOper::LOCAL_TRANSFER, src_sg_1, dst_sg_1);

            while (c_thread.checkCompleted(coyote::CoyoteOper::LOCAL_TRANSFER) != 2) {
                std::this_thread::yield();
            }
        };

        HEADER("RUNNING ASYNC PIPELINED GA ON " + std::to_string(num_vfpgas) + " vFPGAs");
        AsyncGAParams params;
        params.pop_size = pop_size;
        params.n_generations = n_generations;
        params.n_best_keep = n_best_keep;
        params.seed = seed;
        params.two_opt_k = two_opt_k;
        params.num_vfpgas = num_vfpgas;
        params.num_ind_per_stream = batch_ind_per_stream;
        params.buffer_slots = buffer_slots;
        params.buffer_pop = pop_mem.get();

        GASummary summary = run_ga_pipelined(problem, params, execute_vfpga_batch);
        summary.print();
    }

    return EXIT_SUCCESS;
}
