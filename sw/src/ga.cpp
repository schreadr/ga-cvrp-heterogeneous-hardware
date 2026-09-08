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

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <cstring>

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

    int i_pop=0;
    int i_new=0;
    std::uint32_t last_fitness = std::numeric_limits<std::uint32_t>::max();
    for (int i = 0; i < n_keep_best && i_new < new_generation.size(); ++i) {
        // If the fitness values are equal, we prefer the new generation individual
        if(new_generation[i_new].fitness == population[i_pop].fitness){
            new_population[i]=new_generation[i_new];
            last_fitness = new_generation[i_new].fitness;
            while (new_generation[++i_new].fitness == last_fitness && i_new < new_generation.size());
            
            i_pop++;
        // If the new generation individual has a better fitness, we take it
        }else if(new_generation[i_new].fitness < population[i_pop].fitness){
            new_population[i]=new_generation[i_new];
            last_fitness = new_generation[i_new].fitness;
            while (new_generation[++i_new].fitness == last_fitness && i_new < new_generation.size());
        } else {
            new_population[i]=population[i_pop];
            i_pop++;
        }
    }

    // Fill the rest of the new population with individuals from the new generation eliminating duplicates
    int i_new_pop=n_keep_best;
    for (int i = i_new; i < new_generation.size() && i_new_pop < population.size(); i++){
        if(new_generation[i].fitness == last_fitness){
            continue;
        }

        new_population[i_new_pop] = new_generation[i];
        i_new_pop++;
        last_fitness = new_generation[i].fitness;
    }
    
    // Fill the rest of the new population with individuals from the old population eliminating duplicates
    int new_pop_ge_index = n_keep_best;
    for (int i = i_pop; i < population.size() && i_new_pop < population.size(); i++){
        while(new_pop_ge_index < new_population.size() && new_population[new_pop_ge_index].fitness < population[i].fitness){
            new_pop_ge_index++;
        }

        // if not equal then must new_pop must be greater thus pop[i] does not yet exist
        if(new_population[new_pop_ge_index].fitness != population[i].fitness){
            new_population[i] = population[i];
        }
    }

    std::sort(new_population.begin(), new_population.end());
}

void create_parent_pairs(const std::vector<EncodedIndividual> &population, std::vector<EncodedIndividual> &parents1, std::vector<EncodedIndividual> &parents2, std::uint32_t seed) {
    std::mt19937 gen(seed);
    std::uniform_int_distribution<std::size_t> dist(0, population.size() - 1);

    for (std::size_t i = 0; i < population.size(); ++i) {
        std::size_t idx1 = dist(gen);
        std::size_t idx2 = dist(gen);
        std::size_t idx3 = dist(gen);
        std::size_t idx4 = dist(gen);

        parents1[i] = population[idx1].fitness < population[idx2].fitness ? population[idx1] : population[idx2];
        parents2[i] = population[idx3].fitness < population[idx4].fitness ? population[idx3] : population[idx4];
    }
}


void run_ga(const ProblemInstance& problem, GAParams& params, std::function<void()> create_new_generation){
    std::mt19937 new_seed(params.seed);

    const std::vector<Individual> initial_population = build_initial_population(problem, problem.dimension * params.population_size_multiplier, new_seed());
    const std::vector<std::uint8_t> encoded_population = encode_population_fixed(problem, initial_population);
    const std::size_t n_bytes = encoded_population.size();
    const std::size_t pop_size = initial_population.size();

    std::vector<EncodedIndividual> vec1 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> population = vec1;

    std::vector<EncodedIndividual> vec2 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> parents1 = vec2;
    std::reference_wrapper<std::vector<EncodedIndividual>> next_generation = vec2;

    std::vector<EncodedIndividual> vec3 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> parents2 = vec3;
    std::reference_wrapper<std::vector<EncodedIndividual>> next_population = vec3;

    std::uint8_t *buffer_src_0 = params.buffer_src_0;
    std::uint8_t *buffer_src_1 = params.buffer_src_1;
    std::uint8_t *buffer_dst_0 = params.buffer_dst_0;
    std::uint8_t *pop_buffer = params.buffer_pop;

    std::memcpy(pop_buffer, encoded_population.data(), n_bytes);
    population_from_buffer(pop_buffer, pop_size, population, problem.dimension);
    
    std::sort(population.get().begin(), population.get().end());

    for (int i=0; i< params.n_generations; i++){
        create_parent_pairs(population, parents1, parents2, new_seed());
        population_to_buffer(parents1, buffer_src_0, problem.dimension);
        population_to_buffer(parents2, buffer_src_1, problem.dimension);

        create_new_generation();

        population_from_buffer(buffer_dst_0, pop_size, next_generation, problem.dimension);
        update_population(population, next_generation, next_population, 100);
        std::swap(population, next_population);
        population_to_buffer(population, pop_buffer,problem.dimension, true);

        std::cout << "Best fitness in iteration " << i << ": " << population.get()[0].fitness << std::endl;
    }

    std::cout << "Best solution found:\n" << decode_individual_fixed(problem, population.get()[0].bytes) << std::endl;
}
