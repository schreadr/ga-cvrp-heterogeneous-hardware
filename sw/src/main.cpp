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

// External library for easier parsing of CLI arguments by the executable
#include <boost/program_options.hpp>

// Coyote-specific includes
#include <coyote/cBench.hpp>
#include <coyote/cThread.hpp>

// Constants
#define N_LATENCY_REPS 1
#define N_THROUGHPUT_REPS 1

// Default vFPGA to assign cThreads to; for designs with one region (vFPGA) this is the only possible value
#define DEFAULT_VFPGA_ID 0

namespace {

constexpr std::size_t kAxiBeatBytes = 64;
constexpr std::size_t kPackedWordBytes = 9;
constexpr std::size_t kPackedFieldsPerWord = 3;
constexpr std::size_t kPackedFieldBits = 21;
constexpr std::size_t kPopulationMultiplier = 20;
constexpr std::uint32_t kPopulationSeed = 0xC0FFEEu;
constexpr const char kDefaultProblemFile[] = "../../P-n16-k8.vrp";

struct ProblemNode {
    int id = 0;
    int x = 0;
    int y = 0;
    int demand = 0;
};

struct ProblemInstance {
    std::string name;
    int capacity = 0;
    int dimension = 0;
    int depotId = 1;
    std::vector<ProblemNode> nodes;
};

struct TourGene {
    int node_id = 0;
    bool depot_after = false;
};

struct Individual {
    std::vector<TourGene> tour;
    std::uint32_t fitness = 0;
};

std::string trim(const std::string &text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::string value_after_colon(const std::string &line) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
        return trim(line);
    }
    return trim(line.substr(colon + 1));
}

ProblemInstance parse_vrp(const std::string &path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error("Could not open VRP file: " + path);
    }

    ProblemInstance problem;
    std::vector<ProblemNode> nodes;
    int expected_dimension = -1;

    enum class Section {
        Header,
        NodeCoords,
        Demands,
        Depot
    } section = Section::Header;

    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty()) {
            continue;
        }

        if (line == "NODE_COORD_SECTION") {
            section = Section::NodeCoords;
            continue;
        }
        if (line == "DEMAND_SECTION") {
            section = Section::Demands;
            continue;
        }
        if (line == "DEPOT_SECTION") {
            section = Section::Depot;
            continue;
        }
        if (line == "EOF") {
            break;
        }

        if (section == Section::Header) {
            if (line.rfind("NAME", 0) == 0) {
                problem.name = value_after_colon(line);
            } else if (line.rfind("DIMENSION", 0) == 0) {
                expected_dimension = std::stoi(value_after_colon(line));
                nodes.resize(static_cast<std::size_t>(expected_dimension) + 1);
            } else if (line.rfind("CAPACITY", 0) == 0) {
                problem.capacity = std::stoi(value_after_colon(line));
            }
            continue;
        }

        if (section == Section::NodeCoords) {
            std::istringstream iss(line);
            int id = 0;
            int x = 0;
            int y = 0;
            if (!(iss >> id >> x >> y)) {
                throw std::runtime_error("Failed to parse NODE_COORD_SECTION line: " + line);
            }
            if (nodes.empty()) {
                throw std::runtime_error("DIMENSION must appear before NODE_COORD_SECTION");
            }
            nodes.at(static_cast<std::size_t>(id)) = ProblemNode{id, x, y, 0};
            continue;
        }

        if (section == Section::Demands) {
            std::istringstream iss(line);
            int id = 0;
            int demand = 0;
            if (!(iss >> id >> demand)) {
                throw std::runtime_error("Failed to parse DEMAND_SECTION line: " + line);
            }
            if (nodes.empty()) {
                throw std::runtime_error("DIMENSION must appear before DEMAND_SECTION");
            }
            nodes.at(static_cast<std::size_t>(id)).demand = demand;
            continue;
        }

        if (section == Section::Depot) {
            int depot_id = 0;
            std::istringstream iss(line);
            if (!(iss >> depot_id)) {
                throw std::runtime_error("Failed to parse DEPOT_SECTION line: " + line);
            }
            if (depot_id == -1) {
                section = Section::Header;
                continue;
            }
            problem.depotId = depot_id;
            continue;
        }
    }

    if (expected_dimension <= 0 || nodes.size() != static_cast<std::size_t>(expected_dimension) + 1) {
        throw std::runtime_error("VRP file did not provide a valid DIMENSION");
    }
    if (problem.capacity <= 0) {
        throw std::runtime_error("VRP file did not provide a valid CAPACITY");
    }

    problem.dimension = expected_dimension;
    problem.nodes = std::move(nodes);

    int min_x = std::numeric_limits<int>::max();
    int min_y = std::numeric_limits<int>::max();
    for (int id = 1; id <= problem.dimension; ++id) {
        min_x = std::min(min_x, problem.nodes.at(static_cast<std::size_t>(id)).x);
        min_y = std::min(min_y, problem.nodes.at(static_cast<std::size_t>(id)).y);
    }
    for (int id = 1; id <= problem.dimension; ++id) {
        problem.nodes.at(static_cast<std::size_t>(id)).x -= min_x;
        problem.nodes.at(static_cast<std::size_t>(id)).y -= min_y;
    }

    return problem;
}

std::uint32_t ceil_euclidean_distance(const ProblemNode &lhs, const ProblemNode &rhs) {
    const long long dx = static_cast<long long>(lhs.x) - static_cast<long long>(rhs.x);
    const long long dy = static_cast<long long>(lhs.y) - static_cast<long long>(rhs.y);
    const long long squared = dx * dx + dy * dy;
    return static_cast<std::uint32_t>(std::ceil(std::sqrt(static_cast<long double>(squared))));
}

std::uint32_t encode_city_entry(const ProblemNode &node, bool depot_after) {
    const std::uint32_t x = static_cast<std::uint32_t>(node.x) & 0x7fu;
    const std::uint32_t y = static_cast<std::uint32_t>(node.y) & 0x7fu;
    const std::uint32_t demand = static_cast<std::uint32_t>(node.demand) & 0x3fu;
    const std::uint32_t depot = depot_after ? 1u : 0u;
    return (x << 13) | (y << 6) | (demand << 1) | depot;
}

std::uint32_t compute_fitness(const ProblemInstance &problem, const Individual &individual) {
    const ProblemNode &depot = problem.nodes.at(static_cast<std::size_t>(problem.depotId));
    const ProblemNode *previous = &depot;
    std::uint32_t total_distance = 0;

    for (const auto &gene : individual.tour) {
        const ProblemNode &current = problem.nodes.at(static_cast<std::size_t>(gene.node_id));
        total_distance += ceil_euclidean_distance(*previous, current);
        if (gene.depot_after) {
            total_distance += ceil_euclidean_distance(current, depot);
            previous = &depot;
        } else {
            previous = &current;
        }
    }

    if (previous != &depot) {
        total_distance += ceil_euclidean_distance(*previous, depot);
    }

    return total_distance;
}

Individual build_individual(const ProblemInstance &problem, const std::vector<int> &customer_order) {
    Individual individual;
    individual.tour.reserve(customer_order.size());

    int route_load = 0;
    for (std::size_t idx = 0; idx < customer_order.size(); ++idx) {
        const int node_id = customer_order[idx];
        const ProblemNode &customer = problem.nodes.at(static_cast<std::size_t>(node_id));
        if (customer.demand > problem.capacity) {
            throw std::runtime_error("Customer demand exceeds vehicle capacity");
        }

        if (!individual.tour.empty() && route_load + customer.demand > problem.capacity) {
            individual.tour.back().depot_after = true;
            route_load = 0;
        }

        individual.tour.push_back(TourGene{node_id, false});
        route_load += customer.demand;
    }

    if (!individual.tour.empty()) {
        individual.tour.back().depot_after = true;
    }

    individual.fitness = compute_fitness(problem, individual);
    return individual;
}

std::vector<Individual> build_initial_population(const ProblemInstance &problem) {
    const std::size_t population_size = static_cast<std::size_t>(problem.dimension) * kPopulationMultiplier;
    std::vector<Individual> population;
    population.reserve(population_size);

    std::vector<int> customer_ids;
    customer_ids.reserve(static_cast<std::size_t>(problem.dimension - 1));
    for (int id = 1; id <= problem.dimension; ++id) {
        if (id != problem.depotId) {
            customer_ids.push_back(id);
        }
    }

    for (std::size_t individual_idx = 0; individual_idx < population_size; ++individual_idx) {
        std::vector<int> order = customer_ids;
        std::mt19937 rng(kPopulationSeed + static_cast<std::uint32_t>(individual_idx) * 2654435761u);
        std::shuffle(order.begin(), order.end(), rng);

        if (!order.empty()) {
            const std::size_t rotate_by = individual_idx % order.size();
            std::rotate(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(rotate_by), order.end());
            if ((individual_idx & 1u) != 0u) {
                std::reverse(order.begin(), order.end());
            }
        }

        population.push_back(build_individual(problem, order));
    }

    return population;
}

void write_bits_le(std::uint8_t *word_bytes, std::size_t start_bit, unsigned width, std::uint32_t value) {
    for (unsigned bit = 0; bit < width; ++bit) {
        if ((value >> bit) & 1u) {
            const std::size_t absolute_bit = start_bit + bit;
            word_bytes[absolute_bit / 8] |= static_cast<std::uint8_t>(1u << (absolute_bit % 8));
        }
    }
}

std::array<std::uint8_t, kAxiBeatBytes> encode_individual(const ProblemInstance &problem, const Individual &individual) {
    constexpr std::size_t kWordsPerIndividual = 6;

    std::array<std::uint8_t, kAxiBeatBytes> beat{};
    std::vector<std::uint32_t> fields(static_cast<std::size_t>(problem.dimension) + 1u, 0u);

    fields[0] = individual.fitness;
    for (std::size_t idx = 0; idx < individual.tour.size() && idx < static_cast<std::size_t>(problem.dimension); ++idx) {
        const auto &gene = individual.tour[idx];
        const ProblemNode &node = problem.nodes.at(static_cast<std::size_t>(gene.node_id));
        fields[idx + 1] = encode_city_entry(node, gene.depot_after);
    }

    for (std::size_t word_idx = 0; word_idx < kWordsPerIndividual; ++word_idx) {
        std::uint8_t *word_bytes = beat.data() + word_idx * kPackedWordBytes;
        const std::size_t field_base = word_idx * kPackedFieldsPerWord;

        for (std::size_t slot = 0; slot < kPackedFieldsPerWord; ++slot) {
            const std::size_t field_idx = field_base + slot;
            if (field_idx >= fields.size()) {
                continue;
            }
            const std::size_t bit_offset = 51 - slot * kPackedFieldBits;
            write_bits_le(word_bytes, bit_offset, kPackedFieldBits, fields[field_idx]);
        }
    }

    return beat;
}

std::vector<std::uint8_t> encode_population(const ProblemInstance &problem, const std::vector<Individual> &population) {
    std::vector<std::uint8_t> encoded(population.size() * kAxiBeatBytes, 0u);
    for (std::size_t idx = 0; idx < population.size(); ++idx) {
        const auto beat = encode_individual(problem, population[idx]);
        std::memcpy(encoded.data() + idx * kAxiBeatBytes, beat.data(), beat.size());
    }
    return encoded;
}

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

} // namespace

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
    const std::vector<Individual> population = build_initial_population(problem);

    // Split the population vector into two equal halves
    const std::size_t half_pop_size = population.size() / 2;
    const std::vector<Individual> pop_0(population.begin(), population.begin() + half_pop_size);
    const std::vector<Individual> pop_1(population.begin() + half_pop_size, population.end());

    // Encode each split vector individually
    const std::vector<std::uint8_t> encoded_pop_0 = encode_population(problem, pop_0);
    const std::vector<std::uint8_t> encoded_pop_1 = encode_population(problem, pop_1);

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
