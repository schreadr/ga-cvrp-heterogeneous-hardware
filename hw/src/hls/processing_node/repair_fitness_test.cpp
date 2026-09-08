#include "repair_fitness.hpp"
#include "repair_fitness_test.hpp"
#include "test_helper.hpp"

#include <iostream>
#include <vector>
#include <random>

#include "ga.hpp"
#include "ga_encoding_constants.hpp"

static constexpr int kNumCities = 40;
static constexpr int kNumIndividuals = 2;
static constexpr int kVehicleCapacity = 19;

static constexpr int seed = 17;


static std::vector<std::vector<int>> tours = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

static ProblemInstance make_problem() {
    ProblemInstance problem;
    problem.name = "repair_fitness_test";
    problem.dimension = kNumCities + 1;
    problem.capacity = kVehicleCapacity;
    problem.depot = ProblemNode{0, 42, 42, 0};
    problem.nodes.resize(kNumCities);

    std::mt19937 gen(seed);
    std::uniform_int_distribution<std::size_t> dist(0, (1 << COORD_BITS) - 1);
    
    for (int city = 0; city < kNumCities; ++city) {
        problem.nodes[city] = ProblemNode{city, dist(gen), dist(gen), dist(gen) % (kVehicleCapacity + 1)};
    }

    return problem;
}

int run_repair_fitness_test() {
    std::cout << "Repair/Fitness Unit Test" << std::endl;

    hls::stream_of_blocks<IndividualAsBeats> input;
    hls::stream_of_blocks<IndividualAsBeats> output;
    const ProblemInstance problem = make_problem();
    const std::vector<Individual> input_population = build_population(problem, tours, 0);

    tb_push_individuals(input, problem, input_population);
    repair_fitness(input, output, kNumIndividuals, kNumCities, Capacity(kVehicleCapacity),
                   (problem.depot.x << COORD_BITS) | problem.depot.y);

    const std::vector<Individual> result = tb_pull_individuals(output, problem, kNumIndividuals);
    const std::vector<Individual> expected_population = build_population(problem, tours);

    std::cout << "Verifying Outputs..." << std::endl;
    int errors = 0;
    for (std::size_t i = 0; i < kNumIndividuals; ++i) {
        if (result[i] != expected_population[i]) {
            std::cout << "Indiv " << i << " mismatch:\n -g:"
                      << result[i] << "\n -e:" << expected_population[i] << std::endl;
            errors++;
        }
    }

    if (errors == 0) {
        std::cout << "TEST PASSED!" << std::endl;
    } else {
        std::cout << "TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}

