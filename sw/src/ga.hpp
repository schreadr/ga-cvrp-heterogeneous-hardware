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

/**
 * GA data structures and population generation for the CVRP.
 * Pure host code, no Coyote dependency.
 */

#ifndef CVRP_INDIVIDUAL_HPP
#define CVRP_INDIVIDUAL_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <iosfwd>
#include <string>
#include <vector>
#include <iostream>
#include <functional>

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
    ProblemNode depot;
    std::vector<ProblemNode> nodes;
};

// Number of customers of a problem instance (all nodes except the depot)
constexpr std::size_t numCustomers(const ProblemInstance &problem) {
    return static_cast<std::size_t>(problem.dimension - 1);
}

struct TourGene {
    int node_id = 0;
    bool depot_after = false;

    bool operator==(const TourGene &rhs) const {
        return node_id == rhs.node_id && depot_after == rhs.depot_after;
    }

    bool operator!=(const TourGene &rhs) const {
        return !(*this == rhs);
    }
};

struct Individual {
    std::vector<TourGene> tour;
    std::uint32_t fitness = 0;

    bool operator==(const Individual &rhs) const {
        return fitness == rhs.fitness && tour == rhs.tour;
    }

    bool operator!=(const Individual &rhs) const {
        return !(*this == rhs);
    }

    friend std::ostream &operator<<(std::ostream &os, const Individual &ind) {
        os << "Fitness: " << ind.fitness << ", Tour: ";
        for (std::size_t i = 0; i < ind.tour.size(); ++i) {
            const auto &gene = ind.tour[i];
            os << gene.node_id;

            if (gene.depot_after) {
                if (i + 1 < ind.tour.size()) {
                    os << " | ";
                } else {
                    os << " |";
                }
            } else if (i + 1 < ind.tour.size()) {
                os << ", ";
            }
        }
        return os;
    }
};

/**
 * Fast genome hasher: computes a 64-bit hash of an individual's tour bytes (excluding fitness).
 * Uses hardware CRC32 (SSE 4.2) when supported by the CPU, with fallback to 64-bit FNV-1a.
 */
std::uint64_t hash_genome(const std::uint8_t *bytes, std::size_t length);

/**
 * Returns a human-readable description of the hashing backend in use.
 */
const char *get_hash_backend_name();

struct EncodedIndividual {
    const std::uint8_t* bytes = nullptr;
    std::uint32_t fitness = 0;
    std::uint64_t hash = 0;

    bool operator<(const EncodedIndividual& other) const {
        if (fitness != other.fitness) {
            return fitness < other.fitness;
        }
        return hash < other.hash;
    }

    bool operator==(const EncodedIndividual& other) const {
        return fitness == other.fitness && hash == other.hash;
    }

    bool operator!=(const EncodedIndividual& other) const {
        return !(*this == other);
    }
};

struct GAParams {
    std::size_t pop_size = 0;
    std::size_t n_generations = 0;
    double n_best_keep = 0.0;
    std::uint32_t seed = 0;

    std::uint8_t* buffer_src_0 = nullptr;
    std::uint8_t* buffer_src_1 = nullptr;
    std::uint8_t* buffer_src_rnd = nullptr;
    std::uint8_t* buffer_dst_0 = nullptr;
    std::uint8_t* buffer_dst_1 = nullptr;

    std::uint8_t* buffer_pop = nullptr;
};

// Ceiled Euclidean distance between two problem nodes
std::uint32_t ceil_euclidean_distance(const ProblemNode &lhs, const ProblemNode &rhs);

// Total tour distance of an individual (depot -> ... -> depot per route)
std::uint32_t compute_fitness(const ProblemInstance &problem, const Individual &individual);

/*
 * Build one individual from a customer visit order: greedily splits the order
 * into routes that respect the vehicle capacity and computes the fitness.
 */
Individual build_individual(const ProblemInstance &problem, const std::vector<int> &customer_order);

/*
 * Build a deterministic initial population of population_size individuals by
 * shuffling / rotating / reversing the customer order with a fixed seed.
 */
std::vector<Individual> build_initial_population(const ProblemInstance &problem, std::size_t population_size, std::uint32_t population_seed);

/*
 * Update the population with the new generation, keeping the best n_keep_best individuals over both generations. The new generation is sorted by fitness in ascending order (lower fitness is better). No duplicates are allowed in the new population. If a duplicate is found, it is replaced by the next best individual from the current population.
 */
void update_population(std::vector<EncodedIndividual> &population, std::vector<EncodedIndividual> &new_generation, std::vector<EncodedIndividual> &new_population, int n_keep_best);

struct GATimings {
    double init_time_ms = 0.0;
    double parent_selection_ms = 0.0;
    double offspring_generation_ms = 0.0;
    double population_update_ms = 0.0;
    double total_time_ms = 0.0;

    double generational_time_ms() const {
        return parent_selection_ms + offspring_generation_ms + population_update_ms;
    }

    double avg_parent_selection_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (parent_selection_ms / n_generations) : 0.0;
    }

    double avg_offspring_generation_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (offspring_generation_ms / n_generations) : 0.0;
    }

    double avg_population_update_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (population_update_ms / n_generations) : 0.0;
    }

    double avg_generational_time_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (generational_time_ms() / n_generations) : 0.0;
    }
};

struct GASummary {
    // --- Part 1: Solution & Run Information ---
    std::size_t n_generations = 0;
    std::size_t pop_size = 0;
    Individual best_individual;

    // --- Part 2: Execution Timing ---
    GATimings timings;

    // Convenience forwarders
    double generational_time_ms() const {
        return timings.generational_time_ms();
    }
    double avg_parent_selection_ms() const {
        return timings.avg_parent_selection_ms(n_generations);
    }
    double avg_offspring_generation_ms() const {
        return timings.avg_offspring_generation_ms(n_generations);
    }
    double avg_population_update_ms() const {
        return timings.avg_population_update_ms(n_generations);
    }
    double avg_generational_time_ms() const {
        return timings.avg_generational_time_ms(n_generations);
    }

    void print() const;
    void print(std::ostream &os) const;
    friend std::ostream& operator<<(std::ostream &os, const GASummary &summary);
};

void create_parent_pairs(const std::vector<EncodedIndividual> &population, std::vector<EncodedIndividual> &parents1, std::vector<EncodedIndividual> &parents2, std::uint32_t seed);

GASummary run_ga(const ProblemInstance& problem, GAParams& params, std::function<void()> create_new_generation);

bool validate_encoding_constants(const ProblemInstance &problem, bool throw_on_error = true);

#endif
