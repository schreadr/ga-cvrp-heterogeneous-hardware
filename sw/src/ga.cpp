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

#include "ga.hpp"
#include "ga_streaming.hpp"
#include "safe_queue.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <thread>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#define HAS_X86_CRC32 1
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC push_options
#pragma GCC target("sse4.2")
#include <nmmintrin.h>
#pragma GCC pop_options
#elif defined(_MSC_VER)
#include <intrin.h>
#endif
#endif

const char *get_hash_backend_name() {
#if defined(HAS_X86_CRC32)
    return "Hardware CRC32 (SSE 4.2)";
#else
    return "Software FNV-1a (Portable)";
#endif
}

#if defined(HAS_X86_CRC32) && (defined(__GNUC__) || defined(__clang__))
__attribute__((target("sse4.2")))
#endif
std::uint64_t hash_genome(const std::uint8_t *bytes, std::size_t length) {
#if defined(HAS_X86_CRC32)
    std::uint64_t crc = 0xFFFFFFFFull;
    std::size_t i = 0;
    for (; i + 8 <= length; i += 8) {
        std::uint64_t chunk;
        std::memcpy(&chunk, bytes + i, sizeof(chunk));
        crc = _mm_crc32_u64(crc, chunk);
    }
    for (; i < length; ++i) {
        crc = _mm_crc32_u8(static_cast<std::uint32_t>(crc), bytes[i]);
    }
    return crc == 0 ? 1ull : crc;
#else
    // Fallback: 64-bit FNV-1a
    std::uint64_t h = 14695981039346656037ull;
    for (std::size_t i = 0; i < length; ++i) {
        h ^= static_cast<std::uint64_t>(bytes[i]);
        h *= 1099511628211ull;
    }
    return h == 0 ? 1ull : h;
#endif
}

template <typename T>
static inline int bit_width(T n) {
    if (n <= 0) return 0;
    if constexpr (sizeof(T) <= sizeof(unsigned int)) {
        return 32 - __builtin_clz(static_cast<unsigned int>(n));
    } else {
        return 64 - __builtin_clzll(static_cast<unsigned long long>(n));
    }
}

bool validate_encoding_constants(const ProblemInstance &problem, bool throw_on_error) {
    int max_trans_x = problem.depot.x;
    int max_trans_y = problem.depot.y;
    int max_demand = 0;
    for (const auto &node : problem.nodes) {
        max_trans_x = std::max(max_trans_x, node.x);
        max_trans_y = std::max(max_trans_y, node.y);
        max_demand = std::max(max_demand, node.demand);
    }

    bool failed = false;

    int expected_coord_bits = bit_width(std::max(max_trans_x, max_trans_y));
    if (COORD_BITS < expected_coord_bits) {
        std::cout << "COORD_BITS is too small for the given problem instance. Required: " + std::to_string(expected_coord_bits) << std::endl;
        failed = true;
    }

    int expected_capacity_bits = bit_width(problem.capacity);
    if (CAPACITY_BITS < expected_capacity_bits) {
        std::cout << "CAPACITY_BITS is too small for the given problem instance (max capacity must fit in). Required: " + std::to_string(expected_capacity_bits) << std::endl;
        failed = true;
    }

    int expected_demand_bits = bit_width(max_demand);
    if (CAPACITY_BITS < expected_demand_bits) {
        std::cout << "CAPACITY_BITS is too small for the given problem instance (max demand must fit in). Required: " + std::to_string(expected_demand_bits) << std::endl;
        failed = true;
    }

    int num_cities = problem.dimension - 1;
    if (MAX_CITIES < num_cities) {
        std::cout << "MAX_CITIES is too small for the given problem instance. Required: " + std::to_string(num_cities) << std::endl;
        failed = true;
    }

    int total_cache_entries = num_cities + 1;
    if (MAX_CACHE_ENTRIES < total_cache_entries) {
        std::cout << "MAX_CACHE_ENTRIES is too small for the given problem instance. Required: " + std::to_string(total_cache_entries) << std::endl;
        failed = true;
    }

    int max_fitness = 2 * std::ceil(std::sqrt(max_trans_x * max_trans_x + max_trans_y * max_trans_y)) * (problem.dimension - 1);
    int expected_fitness_bits = bit_width(max_fitness);
    if (FITNESS_BITS < expected_fitness_bits) {
        std::cout << "FITNESS_BITS is too small for the given problem instance (max fitness must fit in). Required: " + std::to_string(expected_fitness_bits) << std::endl;
        failed = true;
    }

    if (MAX_CITIES < num_cities) {
        std::cout << "MAX_CITIES is too small for the given problem instance. Required: " + std::to_string(num_cities) << std::endl;
        failed = true;
    }

    int req_node_id_bits = std::max<int>(1, bit_width(std::max<int>(MAX_CITIES, num_cities)));
    int req_capacity_bits = std::max({(int)CAPACITY_BITS, expected_capacity_bits, expected_demand_bits});
    int req_coord_bits = std::max<int>(COORD_BITS, expected_coord_bits);
    int req_packed_bits = DEPOT_BITS + req_node_id_bits;
    if (CITY_BITS < req_packed_bits) {
        std::cout << "CITY_BITS is too small for packed city field. Required: " + std::to_string(req_packed_bits) << std::endl;
        failed = true;
    }

    int req_cache_entry_bits = 2 * req_coord_bits + req_capacity_bits;
    if (NODE_CACHE_ENTRY_BITS < req_cache_entry_bits) {
        std::cout << "NODE_CACHE_ENTRY_BITS is too small for cache entry. Required: " + std::to_string(req_cache_entry_bits) << std::endl;
        failed = true;
    }

    if (failed) {
        int new_coord_bits = req_coord_bits;
        int new_capacity_bits = req_capacity_bits;
        int new_node_id_bits = std::max<int>(NODE_ID_BITS, req_node_id_bits);
        int new_max_cities = std::max<int>(MAX_CITIES, (1 << new_node_id_bits) - 1);

        int new_city_bits = CITY_BITS;
        while (new_city_bits < DEPOT_BITS + new_node_id_bits) {
            new_city_bits *= 2;
        }
        int new_fitness_bits = std::max<int>(FITNESS_BITS, expected_fitness_bits);

        int new_city_bits_log2 = static_cast<int>(std::log2(new_city_bits));
        int new_axi_beat_width = AXI_BEAT_WIDTH;
        int new_axi_bytes = new_axi_beat_width / 8;
        int new_axi_cities_per_beat = new_axi_beat_width / new_city_bits;
        int new_axi_cities_per_beat_log2 = static_cast<int>(std::log2(new_axi_cities_per_beat));
        int new_axi_max_city_beats = (new_max_cities + new_axi_cities_per_beat - 1) / new_axi_cities_per_beat;
        int new_axi_max_beats = new_axi_max_city_beats + AXI_FITNESS_BEATS;
        int new_axi_max_beats_log2 = static_cast<int>(std::ceil(std::log2(new_axi_max_beats)));

        int new_hw_cities_per_beat = HW_CITIES_PER_BEAT;
        while (new_hw_cities_per_beat * new_city_bits < new_fitness_bits) {
            new_hw_cities_per_beat *= 2;
        }
        int new_hw_cities_per_beat_log2 = static_cast<int>(std::log2(new_hw_cities_per_beat));
        int new_hw_beat_width = new_city_bits * new_hw_cities_per_beat;
        int new_hw_beats_per_axi_beat = new_axi_beat_width / new_hw_beat_width;
        int new_hw_beats_per_axi_beat_log2 = new_axi_cities_per_beat_log2 - new_hw_cities_per_beat_log2;
        int new_hw_max_city_beats = (new_max_cities + new_hw_cities_per_beat - 1) / new_hw_cities_per_beat;
        int new_hw_max_beats = new_hw_max_city_beats + HW_FITNESS_BEATS;
        int new_hw_max_beats_log2 = static_cast<int>(std::ceil(std::log2(new_hw_max_beats)));

        int new_cache_entry_bits = NODE_CACHE_ENTRY_BITS;
        while (new_cache_entry_bits < 2 * new_coord_bits + new_capacity_bits) {
            new_cache_entry_bits *= 2;
        }
        int new_cache_entry_bits_log2 = static_cast<int>(std::log2(new_cache_entry_bits));
        int new_cache_entries_per_beat = new_axi_beat_width / new_cache_entry_bits;
        int new_cache_entries_per_beat_log2 = static_cast<int>(std::log2(new_cache_entries_per_beat));
        int new_max_cache_entries = new_max_cities + 1;
        int new_max_cache_beats = (new_max_cache_entries + new_cache_entries_per_beat - 1) / new_cache_entries_per_beat;

        std::cout << "\n======================================================================\n";
        std::cout << "Suggested updated constants for ga_encoding_constants.hpp:\n";
        std::cout << "======================================================================\n";
        std::cout << "// 1. Problem & Coordinate Constants\n";
        std::cout << "#define COORD_BITS                  " << new_coord_bits << "\n";
        std::cout << "#define CAPACITY_BITS               " << new_capacity_bits << "\n";
        std::cout << "#define MAX_CITIES                  " << new_max_cities << "\n";
        std::cout << "#define MAX_CITIES_LOG2             " << new_node_id_bits << "\n\n";
        std::cout << "// 2. Fitness Constants\n";
        std::cout << "#define FITNESS_BITS                " << new_fitness_bits << "\n\n";
        std::cout << "// 3. City Gene Encoding Constants\n";
        std::cout << "#define NODE_ID_BITS                " << new_node_id_bits << " // ceil(log2(MAX_CITIES))\n";
        std::cout << "#define DEPOT_BITS                   1\n";
        std::cout << "#define DEPOT_FLAG_OFFSET           (NODE_ID_BITS)\n";
        std::cout << "#define CITY_BITS                   " << new_city_bits << " // next power of two of (DEPOT_BITS + NODE_ID_BITS)\n";
        std::cout << "#define CITY_BITS_LOG2               " << new_city_bits_log2 << " // log2(CITY_BITS)\n\n";
        std::cout << "// 4. Host / AXI Interface Beat Constants\n";
        std::cout << "#define AXI_BEAT_WIDTH              " << new_axi_beat_width << "\n";
        std::cout << "#define AXI_WIDTH                   AXI_BEAT_WIDTH\n";
        std::cout << "#define AXI_BYTES                   (AXI_BEAT_WIDTH / 8) // " << new_axi_bytes << "\n";
        std::cout << "#define AXI_CITIES_PER_BEAT         " << new_axi_cities_per_beat << " // AXI_BEAT_WIDTH / CITY_BITS\n";
        std::cout << "#define AXI_CITIES_PER_BEAT_LOG2     " << new_axi_cities_per_beat_log2 << " // log2(AXI_CITIES_PER_BEAT)\n";
        std::cout << "#define AXI_MAX_CITY_BEATS          " << new_axi_max_city_beats << "\n";
        std::cout << "#define AXI_FITNESS_BEATS            1\n";
        std::cout << "#define AXI_MAX_BEATS               " << new_axi_max_beats << "\n";
        std::cout << "#define AXI_MAX_BEATS_LOG2           " << new_axi_max_beats_log2 << "\n\n";
        std::cout << "// 5. Hardware Pipeline Beat Constants\n";
        std::cout << "#define HW_CITIES_PER_BEAT          " << new_hw_cities_per_beat << "\n";
        std::cout << "#define HW_CITIES_PER_BEAT_LOG2     " << new_hw_cities_per_beat_log2 << "\n";
        std::cout << "#define HW_BEAT_WIDTH               (CITY_BITS * HW_CITIES_PER_BEAT) // " << new_hw_beat_width << "\n";
        std::cout << "#define HW_BEATS_PER_AXI_BEAT       (AXI_BEAT_WIDTH / HW_BEAT_WIDTH) // " << new_hw_beats_per_axi_beat << "\n";
        std::cout << "#define HW_BEATS_PER_AXI_BEAT_LOG2  (AXI_CITIES_PER_BEAT_LOG2 - HW_CITIES_PER_BEAT_LOG2) // " << new_hw_beats_per_axi_beat_log2 << "\n";
        std::cout << "#define HW_MAX_CITY_BEATS           " << new_hw_max_city_beats << "\n";
        std::cout << "#define HW_FITNESS_BEATS            1\n";
        std::cout << "#define HW_MAX_BEATS                " << new_hw_max_beats << "\n";
        std::cout << "#define HW_MAX_BEATS_LOG2           " << new_hw_max_beats_log2 << "\n\n";
        std::cout << "// 6. Node Cache Encoding Constants\n";
        std::cout << "#define NODE_CACHE_ENTRY_BITS        " << new_cache_entry_bits << "\n";
        std::cout << "#define NODE_CACHE_ENTRY_BITS_LOG2    " << new_cache_entry_bits_log2 << "\n";
        std::cout << "#define NODE_CACHE_COORD_X_OFFSET     0\n";
        std::cout << "#define NODE_CACHE_COORD_Y_OFFSET    (COORD_BITS)\n";
        std::cout << "#define NODE_CACHE_DEMAND_OFFSET     (2 * COORD_BITS)\n";
        std::cout << "#define NODE_CACHE_ENTRIES_PER_BEAT      " << new_cache_entries_per_beat << "\n";
        std::cout << "#define NODE_CACHE_ENTRIES_PER_BEAT_LOG2 " << new_cache_entries_per_beat_log2 << "\n";
        std::cout << "#define CITIES_PER_CACHE_BEAT            NODE_CACHE_ENTRIES_PER_BEAT\n";
        std::cout << "#define MAX_CACHE_ENTRIES            " << new_max_cache_entries << " // MAX_CITIES + 1\n";
        std::cout << "#define MAX_CACHE_BEATS              " << new_max_cache_beats << "\n";
        std::cout << "======================================================================\n\n";

        if (throw_on_error) {
            throw std::runtime_error("Problem instance is not valid.");
        }
        return false;
    }

    return true;
}

void GASummary::print() const {
    print(std::cout);
}

void GASummary::print(std::ostream &os) const {
    const double gen_total = generational_time_ms();
    const double pct_parents = gen_total > 0.0 ? (timings.parent_selection_ms / gen_total * 100.0) : 0.0;
    const double pct_offspring = gen_total > 0.0 ? (timings.offspring_generation_ms / gen_total * 100.0) : 0.0;
    const double pct_update = gen_total > 0.0 ? (timings.population_update_ms / gen_total * 100.0) : 0.0;

    auto old_flags = os.flags();
    auto old_prec = os.precision();

    os << "\n============================== GA EXECUTION SUMMARY ==============================\n";
    os << "--- Solution Overview ------------------------------------------------------------\n";
    os << "  Generations:          " << n_generations << "\n";
    os << "  Population Size:      " << pop_size << "\n";
    os << "  Best Fitness:         " << best_fitness << "\n";
    os << "----------------------------------------------------------------------------------\n";
    os << "\n--- Execution Timing -------------------------------------------------------------\n";
    os << "  " << std::left << std::setw(28) << "Phase"
       << std::right << std::setw(16) << "Total (ms)"
       << std::right << std::setw(16) << "Avg/Gen (ms)"
       << std::right << std::setw(14) << "Share (%)" << "\n";
    os << "  --------------------------------------------------------------------------------\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Parents Selection"
       << std::right << std::setw(16) << timings.parent_selection_ms
       << std::right << std::setw(16) << avg_parent_selection_ms()
       << std::right << std::setw(13) << std::setprecision(1) << pct_parents << " %\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Offspring Generation"
       << std::right << std::setw(16) << timings.offspring_generation_ms
       << std::right << std::setw(16) << avg_offspring_generation_ms()
       << std::right << std::setw(13) << std::setprecision(1) << pct_offspring << " %\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Population Update"
       << std::right << std::setw(16) << timings.population_update_ms
       << std::right << std::setw(16) << avg_population_update_ms()
       << std::right << std::setw(13) << std::setprecision(1) << pct_update << " %\n";
    os << "  --------------------------------------------------------------------------------\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Generational Total"
       << std::right << std::setw(16) << gen_total
       << std::right << std::setw(16) << avg_generational_time_ms()
       << std::right << std::setw(13) << std::setprecision(1) << 100.0 << " %\n";
    if (timings.init_time_ms > 0.0) {
        os << std::fixed << std::setprecision(3);
        os << "  " << std::left << std::setw(28) << "Initialization"
           << std::right << std::setw(16) << timings.init_time_ms
           << std::right << std::setw(16) << "-"
           << std::right << std::setw(14) << "-\n";
    }
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Total Wall Time"
       << std::right << std::setw(16) << timings.total_time_ms
       << std::right << std::setw(16) << "-"
       << std::right << std::setw(14) << "-\n";
    os << "==================================================================================\n";

    os.flags(old_flags);
    os.precision(old_prec);
}

std::ostream& operator<<(std::ostream &os, const GASummary &summary) {
    summary.print(os);
    return os;
}

int calc_max_n_subtours(const ProblemInstance &problem) {
    if (problem.nodes.empty()) return 0;
    std::vector<int> demands;
    demands.reserve(problem.nodes.size());
    for (const auto &node : problem.nodes) {
        demands.push_back(node.demand);
    }
    std::sort(demands.rbegin(), demands.rend());

    int k_min = 0;
    int load = 0;
    for (int d : demands) {
        if (load + d <= problem.capacity) {
            load += d;
            k_min++;
        } else {
            break;
        }
    }
    if (k_min == 0) k_min = 1;

    int num_customers = static_cast<int>(problem.nodes.size());
    return (num_customers + k_min - 1) / k_min;
}

std::uint32_t floor_euclidean_distance(const ProblemNode &lhs, const ProblemNode &rhs) {
    const long long dx = static_cast<long long>(lhs.x) - static_cast<long long>(rhs.x);
    const long long dy = static_cast<long long>(lhs.y) - static_cast<long long>(rhs.y);
    const long long squared = dx * dx + dy * dy;
    return static_cast<std::uint32_t>(std::floor(std::sqrt(static_cast<long double>(squared))));
}

std::uint32_t compute_fitness(const ProblemInstance &problem, const Individual &individual) {
    const ProblemNode &depot = problem.depot;
    const ProblemNode *previous = &depot;
    std::uint32_t total_distance = 0;

    for (const auto &gene : individual.tour) {
        const ProblemNode &current = problem.nodes.at(static_cast<std::size_t>(gene.node_id));
        total_distance += floor_euclidean_distance(*previous, current);
        if (gene.depot_after) {
            total_distance += floor_euclidean_distance(current, depot);
            previous = &depot;
        } else {
            previous = &current;
        }
    }

    if (previous != &depot) {
        total_distance += floor_euclidean_distance(*previous, depot);
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

std::vector<Individual> build_initial_population(const ProblemInstance &problem, std::size_t population_size, std::uint32_t seed) {
    std::vector<Individual> population;
    population.reserve(population_size);

    std::mt19937 rng(seed);

    std::vector<int> customer_ids;
    customer_ids.reserve(problem.nodes.size());
    for (int id = 0; id < static_cast<int>(problem.nodes.size()); ++id) {
        customer_ids.push_back(id);
    }

    for (std::size_t individual_idx = 0; individual_idx < population_size; ++individual_idx) {
        std::vector<int> order = customer_ids;
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

/**
 * Update the population with the new generation, keeping the best n_keep_best individuals over both generations. The new generation is sorted by fitness in ascending order (lower fitness is better). Duplicates are handled by replacing them with individuals from the new generation.
 */
void update_population(std::vector<EncodedIndividual> &population, std::vector<EncodedIndividual> &new_generation, std::vector<EncodedIndividual> &new_population, int n_keep_best){
    std::sort(new_generation.begin(), new_generation.end());

    int i_pop = 0;
    int i_new = 0;
    EncodedIndividual last_ind = {nullptr, std::numeric_limits<std::uint32_t>::max(), std::numeric_limits<std::uint64_t>::max()};
    int i_new_pop = 0;

    while (i_new_pop < n_keep_best && (i_new < new_generation.size() || i_pop < population.size())) {
        if (i_new < new_generation.size() && i_pop < population.size()) {
            // If both individuals are equal (same fitness and hash), we prefer the new generation individual
            if (new_generation[i_new] == population[i_pop]) {
                new_population[i_new_pop] = new_generation[i_new];
                last_ind = new_generation[i_new];
                while (++i_new < new_generation.size() && new_generation[i_new] == last_ind);
                i_pop++;
                i_new_pop++;
            // If the new generation individual has better (fitness, hash), we take it
            } else if (new_generation[i_new] < population[i_pop]) {
                new_population[i_new_pop] = new_generation[i_new];
                last_ind = new_generation[i_new];
                while (++i_new < new_generation.size() && new_generation[i_new] == last_ind);
                i_new_pop++;
            } else {
                new_population[i_new_pop] = population[i_pop];
                last_ind = population[i_pop];
                while (++i_pop < population.size() && population[i_pop] == last_ind);
                i_new_pop++;
            }
        } else if (i_new < new_generation.size()) {
            new_population[i_new_pop] = new_generation[i_new];
            last_ind = new_generation[i_new];
            while (++i_new < new_generation.size() && new_generation[i_new] == last_ind);
            i_new_pop++;
        } else {
            new_population[i_new_pop] = population[i_pop];
            last_ind = population[i_pop];
            while (++i_pop < population.size() && population[i_pop] == last_ind);
            i_new_pop++;
        }
    }

    // Fill the rest of the new population with individuals from the new generation eliminating duplicates
    int n_keep_actual = i_new_pop;
    for (int i = i_new; i < new_generation.size() && i_new_pop < population.size(); i++){
        if (new_generation[i] == last_ind) {
            continue;
        }

        new_population[i_new_pop] = new_generation[i];
        i_new_pop++;
        last_ind = new_generation[i];
    }
    
    // Fill the rest of the new population with individuals from the old population eliminating duplicates
    int new_pop_ge_index = n_keep_actual;
    for (int i = i_pop; i < population.size() && i_new_pop < population.size(); i++){
        while (new_pop_ge_index < i_new_pop && new_population[new_pop_ge_index] < population[i]){
            new_pop_ge_index++;
        }

        // if not equal then new_pop must be greater thus pop[i] does not yet exist
        if (new_pop_ge_index >= i_new_pop || new_population[new_pop_ge_index] != population[i]){
            new_population[i_new_pop] = population[i];
            i_new_pop++;
        }
    }

    if (i_new_pop < population.size()) {
        std::cerr << "Warning: Only "
                  << i_new_pop << " individuals were available for population size "
                  << population.size() << ". Filling remaining "
                  << (population.size() - i_new_pop) << " slots." << std::endl;
        while (i_new_pop < population.size()) {
            new_population[i_new_pop] = new_population[0];
            i_new_pop++;
        }
    }

    std::sort(new_population.begin(), new_population.end());
}
void create_parent_pairs(const std::vector<EncodedIndividual> &population, std::vector<EncodedIndividual> &parents1, std::vector<EncodedIndividual> &parents2, std::uint32_t seed) {
    std::mt19937 gen(seed);
    std::uniform_int_distribution<std::size_t> dist(0, population.size() - 1);

    const std::size_t n_pairs = std::min(parents1.size(), parents2.size());
    for (std::size_t i = 0; i < n_pairs; ++i) {
        std::size_t idx1 = dist(gen);
        std::size_t idx2 = dist(gen);
        std::size_t idx3 = dist(gen);
        std::size_t idx4 = dist(gen);

        parents1[i] = population[idx1].fitness < population[idx2].fitness ? population[idx1] : population[idx2];
        parents2[i] = population[idx3].fitness < population[idx4].fitness ? population[idx3] : population[idx4];
    }
}

GASummary run_ga(const ProblemInstance& problem, GAParams& params, std::function<void()> create_new_generation){
    auto wall_start = std::chrono::steady_clock::now();
    GASummary summary;
    summary.n_generations = params.n_generations;
    summary.pop_size = params.pop_size;

    auto init_start = std::chrono::steady_clock::now();
    std::mt19937 new_seed(params.seed);

    const std::size_t pop_size = params.pop_size;
    const std::size_t half_pop = pop_size / 2;
    const int beats_per_ind = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_ind = beats_per_ind * AXI_BYTES;
    const std::size_t half_bytes = half_pop * bytes_per_ind;

    std::vector<Individual> initial_population = build_initial_population(problem, pop_size, new_seed());
    const std::vector<std::uint8_t> encoded_population = encode_population_fixed(problem, initial_population);

    std::vector<EncodedIndividual> vec1 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> population = vec1;

    std::vector<EncodedIndividual> vec2 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> next_generation = vec2;

    std::vector<EncodedIndividual> vec3 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> next_population = vec3;

    std::vector<EncodedIndividual> parents1(half_pop);
    std::vector<EncodedIndividual> parents2(half_pop);

    std::uint8_t *buffer_src_0 = params.buffer_src_0;
    std::uint8_t *buffer_src_1 = params.buffer_src_1;
    std::uint8_t *buffer_dst_0 = params.buffer_dst_0;
    std::uint8_t *buffer_dst_1 = params.buffer_dst_1;
    std::uint8_t *pop_buffer = params.buffer_pop;

    const std::size_t print_interval = std::max<std::size_t>(1, params.n_generations / 10);

    // Individuals of current population must be sorted on the buffer
    std::memcpy(buffer_src_0, encoded_population.data(), half_bytes);
    std::memcpy(buffer_src_1, encoded_population.data() + half_bytes, half_bytes);
    population_from_buffer(buffer_src_0, half_pop, population, problem.dimension, 0);
    population_from_buffer(buffer_src_1, half_pop, population, problem.dimension, half_pop);
    std::sort(population.get().begin(), population.get().end());
    population_to_buffer(population, pop_buffer, problem.dimension);

    const std::size_t n_keep = std::min(pop_size, static_cast<std::size_t>(std::round(pop_size * (params.n_best_keep / 100.0))));
    auto init_end = std::chrono::steady_clock::now();
    summary.timings.init_time_ms = std::chrono::duration<double, std::milli>(init_end - init_start).count();

    for (int i=0; i< params.n_generations; i++){
        auto t0 = std::chrono::steady_clock::now();
        create_parent_pairs(population, parents1, parents2, new_seed());
        population_to_buffer(parents1, buffer_src_0, problem.dimension);
        population_to_buffer(parents2, buffer_src_1, problem.dimension);
        auto t1 = std::chrono::steady_clock::now();
        
        create_new_generation();
        auto t2 = std::chrono::steady_clock::now();

        population_from_buffer(buffer_dst_0, half_pop, next_generation, problem.dimension, 0);
        population_from_buffer(buffer_dst_1, half_pop, next_generation, problem.dimension, half_pop);
        update_population(population, next_generation, next_population, n_keep);
        std::swap(population, next_population);
        population_to_buffer(population, pop_buffer, problem.dimension, true);
        auto t3 = std::chrono::steady_clock::now();

        summary.timings.parent_selection_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
        summary.timings.offspring_generation_ms += std::chrono::duration<double, std::milli>(t2 - t1).count();
        summary.timings.population_update_ms += std::chrono::duration<double, std::milli>(t3 - t2).count();

        if (i % print_interval == 0) {
            const double current_best = static_cast<double>(population.get()[0].fitness) / static_cast<double>(problem.scaling);
            std::cout << "Iteration " << i << " completed. Best fitness: " << current_best << std::endl;
        }
    }

    if (!population.get().empty() && population.get()[0].bytes != nullptr) {
        summary.best_individual = decode_individual_fixed(problem, population.get()[0].bytes);
        summary.best_fitness = static_cast<double>(summary.best_individual.fitness) / static_cast<double>(problem.scaling);
    }

    auto wall_end = std::chrono::steady_clock::now();
    summary.timings.total_time_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();
    return summary;
}

GASummary run_ga_pipelined(
    const ProblemInstance &problem,
    AsyncGAParams &params,
    VfpgaExecuteFn execute_vfpga_batch
) {
    auto wall_start = std::chrono::steady_clock::now();
    GASummary summary;
    summary.n_generations = params.n_generations;
    summary.pop_size = params.pop_size;

    auto init_start = std::chrono::steady_clock::now();
    std::mt19937 new_seed(params.seed);

    const std::size_t pop_size = params.pop_size;
    const std::size_t num_vfpgas = std::max<std::size_t>(1, params.num_vfpgas);
    const std::size_t batch_ind_per_stream = params.num_ind_per_stream > 0 ? params.num_ind_per_stream : (pop_size / (2 * num_vfpgas));
    const std::size_t offspring_per_batch = 2 * batch_ind_per_stream;

    if (params.buffer_slots.empty()) {
        throw std::runtime_error("run_ga_pipelined: params.buffer_slots is empty. Pre-allocated buffer slots required.");
    }
    if (!params.buffer_pop) {
        throw std::runtime_error("run_ga_pipelined: params.buffer_pop is null. Population scratchpad buffer required.");
    }

    // Build initial population
    std::vector<Individual> initial_population = build_initial_population(problem, pop_size, new_seed());
    const std::vector<std::uint8_t> encoded_population = encode_population_fixed(problem, initial_population);

    const int beats_per_ind = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_ind = beats_per_ind * AXI_BYTES;
    const std::size_t pop_bytes = pop_size * bytes_per_ind;

    std::memcpy(params.buffer_pop, encoded_population.data(), pop_bytes);

    std::vector<EncodedIndividual> population_vec(pop_size);
    std::vector<EncodedIndividual> next_population_vec(pop_size);
    population_from_buffer(params.buffer_pop, pop_size, population_vec, problem.dimension, 0);
    std::sort(population_vec.begin(), population_vec.end());
    population_to_buffer(population_vec, params.buffer_pop, problem.dimension);

    const std::size_t n_keep = std::min(pop_size, static_cast<std::size_t>(std::round(pop_size * (params.n_best_keep / 100.0))));

    // Calculate total evaluation batches
    const std::size_t total_evaluations_target = pop_size * params.n_generations;
    const std::size_t total_batches_to_process = std::max<std::size_t>(
        num_vfpgas,
        (total_evaluations_target + offspring_per_batch - 1) / offspring_per_batch
    );

    auto init_end = std::chrono::steady_clock::now();
    summary.timings.init_time_ms = std::chrono::duration<double, std::milli>(init_end - init_start).count();

    // Queues: 1 parent queue per vFPGA, 1 shared offspring completion queue
    std::vector<SafeQueue<BatchJob>> parent_queues(num_vfpgas);
    SafeQueue<BatchJob> offspring_queue;

    // Launch worker threads (one per vFPGA)
    std::vector<std::thread> worker_threads;
    worker_threads.reserve(num_vfpgas);
    for (std::size_t v = 0; v < num_vfpgas; ++v) {
        worker_threads.emplace_back([v, &parent_queues, &offspring_queue, &execute_vfpga_batch]() {
            while (true) {
                auto job_opt = parent_queues[v].pop();
                if (!job_opt) {
                    break; // Queue shut down, terminate worker
                }
                BatchJob job = std::move(*job_opt);
                execute_vfpga_batch(static_cast<int>(v), job);
                offspring_queue.push(std::move(job));
            }
        });
    }

    // Population Manager staging structures
    std::vector<EncodedIndividual> parents1(batch_ind_per_stream);
    std::vector<EncodedIndividual> parents2(batch_ind_per_stream);
    std::vector<EncodedIndividual> batch_offspring(offspring_per_batch);

    uint32_t batches_dispatched = 0;
    uint32_t batches_completed = 0;

    // Priming phase: Fill all pre-allocated buffer slots and enqueue initial jobs
    for (const auto &slot : params.buffer_slots) {
        if (batches_dispatched >= total_batches_to_process) break;

        auto t0 = std::chrono::steady_clock::now();
        create_parent_pairs(population_vec, parents1, parents2, new_seed());
        population_to_buffer(parents1, slot.src_buf_0, problem.dimension);
        population_to_buffer(parents2, slot.src_buf_1, problem.dimension);
        auto t1 = std::chrono::steady_clock::now();
        summary.timings.parent_selection_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        BatchJob job;
        job.batch_id = batches_dispatched++;
        job.batch_seed = new_seed();
        job.vfid = slot.vfid;
        job.slot_id = slot.slot_id;
        job.num_ind_per_stream = batch_ind_per_stream;
        job.src_buf_0 = slot.src_buf_0;
        job.src_buf_1 = slot.src_buf_1;
        job.dst_buf_0 = slot.dst_buf_0;
        job.dst_buf_1 = slot.dst_buf_1;

        parent_queues[job.vfid].push(std::move(job));
    }

    const std::size_t log_interval_batches = std::max<std::size_t>(1, total_batches_to_process / 10);

    // Continuous streaming loop: collect finished offspring in deterministic in-order sequence
    while (batches_completed < total_batches_to_process) {
        auto t_wait_start = std::chrono::steady_clock::now();
        auto finished_job_opt = offspring_queue.pop_expected(batches_completed);
        auto t_wait_end = std::chrono::steady_clock::now();
        summary.timings.offspring_generation_ms += std::chrono::duration<double, std::milli>(t_wait_end - t_wait_start).count();

        if (!finished_job_opt) {
            break; // Unexpected early shutdown
        }

        BatchJob finished_job = std::move(*finished_job_opt);
        batches_completed++;

        // 1. Population Update: deserialize offspring and merge into population pool
        auto t_up_start = std::chrono::steady_clock::now();
        population_from_buffer(finished_job.dst_buf_0, finished_job.num_ind_per_stream, batch_offspring, problem.dimension, 0);
        population_from_buffer(finished_job.dst_buf_1, finished_job.num_ind_per_stream, batch_offspring, problem.dimension, finished_job.num_ind_per_stream);

        update_population(population_vec, batch_offspring, next_population_vec, n_keep);
        std::swap(population_vec, next_population_vec);
        population_to_buffer(population_vec, params.buffer_pop, problem.dimension, true);
        auto t_up_end = std::chrono::steady_clock::now();
        summary.timings.population_update_ms += std::chrono::duration<double, std::milli>(t_up_end - t_up_start).count();

        // 2. Refill the returned buffer slot and dispatch next batch if budget remains
        if (batches_dispatched < total_batches_to_process) {
            auto t_sel_start = std::chrono::steady_clock::now();
            create_parent_pairs(population_vec, parents1, parents2, new_seed());
            population_to_buffer(parents1, finished_job.src_buf_0, problem.dimension);
            population_to_buffer(parents2, finished_job.src_buf_1, problem.dimension);
            auto t_sel_end = std::chrono::steady_clock::now();
            summary.timings.parent_selection_ms += std::chrono::duration<double, std::milli>(t_sel_end - t_sel_start).count();

            finished_job.batch_id = batches_dispatched++;
            finished_job.batch_seed = new_seed();
            parent_queues[finished_job.vfid].push(std::move(finished_job));
        }

        if (batches_completed % log_interval_batches == 0 || batches_completed == total_batches_to_process) {
            const double current_best = static_cast<double>(population_vec[0].fitness) / static_cast<double>(problem.scaling);
            std::cout << "Pipelined Batch " << batches_completed << "/" << total_batches_to_process
                      << " completed. Best fitness: " << current_best << std::endl;
        }
    }

    // Shutdown queues to terminate workers gracefully
    for (std::size_t v = 0; v < num_vfpgas; ++v) {
        parent_queues[v].shutdown();
    }
    for (auto &worker : worker_threads) {
        if (worker.joinable()) {
            worker.join();
        }
    }

    // Decode final best solution
    if (!population_vec.empty() && population_vec[0].bytes != nullptr) {
        summary.best_individual = decode_individual_fixed(problem, population_vec[0].bytes);
        summary.best_fitness = static_cast<double>(summary.best_individual.fitness) / static_cast<double>(problem.scaling);
    }

    auto wall_end = std::chrono::steady_clock::now();
    summary.timings.total_time_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();
    return summary;
}


