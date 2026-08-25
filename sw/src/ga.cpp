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

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

std::uint32_t ceil_euclidean_distance(const ProblemNode &lhs, const ProblemNode &rhs) {
    const long long dx = static_cast<long long>(lhs.x) - static_cast<long long>(rhs.x);
    const long long dy = static_cast<long long>(lhs.y) - static_cast<long long>(rhs.y);
    const long long squared = dx * dx + dy * dy;
    return static_cast<std::uint32_t>(std::ceil(std::sqrt(static_cast<long double>(squared))));
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

std::vector<Individual> build_initial_population(const ProblemInstance &problem, std::size_t population_size, std::uint32_t population_seed) {
    std::vector<Individual> population;
    population.reserve(population_size);

    std::vector<int> customer_ids;
    customer_ids.reserve(numCustomers(problem));
    for (int id = 1; id <= problem.dimension; ++id) {
        if (id != problem.depotId) {
            customer_ids.push_back(id);
        }
    }

    for (std::size_t individual_idx = 0; individual_idx < population_size; ++individual_idx) {
        std::vector<int> order = customer_ids;
        std::mt19937 rng(population_seed + static_cast<std::uint32_t>(individual_idx) * 2654435761u);
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
