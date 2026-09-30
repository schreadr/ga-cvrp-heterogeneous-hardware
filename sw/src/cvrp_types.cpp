#include "cvrp_types.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <stdexcept>
#include <string>

std::uint32_t floor_euclidean_distance(const ProblemNode &lhs, const ProblemNode &rhs) {
    const long long dx = static_cast<long long>(lhs.x) - static_cast<long long>(rhs.x);
    const long long dy = static_cast<long long>(lhs.y) - static_cast<long long>(rhs.y);
    const long long squared = dx * dx + dy * dy;
    return static_cast<std::uint32_t>(std::floor(std::sqrt(static_cast<long double>(squared))));
}

double exact_euclidean_distance(const ProblemNode &lhs, const ProblemNode &rhs) {
    const double x1 = (lhs.raw_x != 0.0 || lhs.raw_y != 0.0) ? lhs.raw_x : static_cast<double>(lhs.x);
    const double y1 = (lhs.raw_x != 0.0 || lhs.raw_y != 0.0) ? lhs.raw_y : static_cast<double>(lhs.y);
    const double x2 = (rhs.raw_x != 0.0 || rhs.raw_y != 0.0) ? rhs.raw_x : static_cast<double>(rhs.x);
    const double y2 = (rhs.raw_x != 0.0 || rhs.raw_y != 0.0) ? rhs.raw_y : static_cast<double>(rhs.y);
    const double dx = x1 - x2;
    const double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
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

double compute_exact_fitness(const ProblemInstance &problem, const Individual &individual) {
    const ProblemNode &depot = problem.depot;
    const ProblemNode *previous = &depot;
    double total_distance = 0.0;

    for (const auto &gene : individual.tour) {
        const ProblemNode &current = problem.nodes.at(static_cast<std::size_t>(gene.node_id));
        total_distance += exact_euclidean_distance(*previous, current);
        if (gene.depot_after) {
            total_distance += exact_euclidean_distance(current, depot);
            previous = &depot;
        } else {
            previous = &current;
        }
    }

    if (previous != &depot) {
        total_distance += exact_euclidean_distance(*previous, depot);
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

    if (population_size == 0) {
        return population;
    }

    std::mt19937 rng(seed);

    std::vector<int> customer_ids;
    customer_ids.reserve(problem.nodes.size());
    for (int id = 0; id < static_cast<int>(problem.nodes.size()); ++id) {
        customer_ids.push_back(id);
    }

    std::set<std::vector<int>> unique_permutations;

    std::vector<int> order = customer_ids;
    std::size_t attempts = 0;
    const std::size_t max_attempts = 2 * population_size;

    while (unique_permutations.size() < population_size) {
        if (attempts >= max_attempts) {
            throw std::runtime_error("Failed to generate " + std::to_string(population_size) +
                                     " distinct individuals in initial population after " +
                                     std::to_string(max_attempts) + " attempts.");
        }
        attempts++;
        std::shuffle(order.begin(), order.end(), rng);
        unique_permutations.insert(order);
    }

    for (const auto &permutation : unique_permutations) {
        population.push_back(build_individual(problem, permutation));
    }

    return population;
}
