#include "mutate.hpp"
#include "test_helper.hpp"

#include <cstring>
#include <iostream>
#include <vector>

#include "ga_encoding_constants.hpp"
#include "ga.hpp"
#include "ga_streaming.hpp"

static int kNumCities = 40;
static std::size_t kNumIndividuals = 2;
static std::uint32_t kFitness0Base = 1000;

static int kDepotId = 1;

static std::vector<ProblemNode> make_nodes() {
    std::vector<ProblemNode> nodes(kNumCities);
    for (int i = 1; i <= kNumCities; ++i) {
        nodes[i-1] = ProblemNode{
           i,
           i & ((1 << COORD_BITS) - 1),
           (i * 2) & ((1 << COORD_BITS) - 1),
           i & ((1 << CAPACITY_BITS) - 1)
        };
    }
    return nodes;
}

static ProblemInstance make_problem() {
    ProblemInstance p;
    p.name = "tb_hardcoded";
    p.dimension = kNumCities + 1;
    p.capacity = (1 << CAPACITY_BITS) - 1;
    p.depot = ProblemNode{kDepotId, 0, 0, 0};
    p.nodes = make_nodes();
    return p;
}

static std::vector<std::vector<int>> kPopTours = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

static std::vector<std::vector<int>> kExpChildTours = {
    {0, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 34, 35, 36, 37, 38, 39},
    {39, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 5, 4, 3, 2, 1, 0}

};

int run_mutate_test() {
    std::cout << "Mutate Unit Test" << std::endl;
    hls::stream_of_blocks<IndividualAsBeats> in0;
    hls::stream_of_blocks<IndividualAsBeats> out0;

    const ProblemInstance problem = make_problem();
    const std::vector<Individual> pop_0 = build_population(problem, kPopTours, kFitness0Base);

    std::cout << "Streaming parents into block streams..." << std::endl;
    tb_push_individuals(in0, problem, pop_0);

    const FieldIndex kMutateStart = 1;
    const FieldIndex kMutateEnd = 33;
    hls::stream<CityPair> mutate_pairs("mutate_pairs");

    for (std::size_t i = 0; i < kNumIndividuals; ++i) {
        mutate_pairs.write(CityPair{kMutateStart, kMutateEnd});
    }

    std::cout << "Executing Mutate..." << std::endl;
    mutate(in0, mutate_pairs, out0, static_cast<int>(kNumIndividuals), kNumCities);

    std::cout << "Reading children from block streams..." << std::endl;
    const std::vector<Individual> child_0 = tb_pull_individuals(out0, problem, kNumIndividuals);
    const std::vector<Individual> exp_child_0 = build_population(problem, kExpChildTours, kFitness0Base);
    
    std::cout << "Verifying Outputs..." << std::endl;
    int errors = 0;
    for (std::size_t i = 0; i < kNumIndividuals; ++i) {
        if (child_0[i] != exp_child_0[i]) {
            std::cout << "Stream 0 indiv " << i << " mismatch:\n -g:"
                      << child_0[i] << "\n -e:" << exp_child_0[i] << std::endl;
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

static int kNumCities_border_cases = 15;
static std::size_t kNumIndividuals_border_cases = 2;

static std::vector<ProblemNode> make_nodes_border_cases() {
    std::vector<ProblemNode> nodes(kNumCities_border_cases);
    for (int i = 1; i <= kNumCities_border_cases; ++i) {
        nodes[i-1] = ProblemNode{
           i,
           i & ((1 << COORD_BITS) - 1),
           (i * 2) & ((1 << COORD_BITS) - 1),
           i & ((1 << CAPACITY_BITS) - 1)
        };
    }
    return nodes;
}

static ProblemInstance make_problem_border_cases() {
    ProblemInstance p;
    p.name = "tb_border_cases";
    p.dimension = kNumCities_border_cases + 1;
    p.capacity = (1 << CAPACITY_BITS) - 1;
    p.depot = ProblemNode{kDepotId, 0, 0, 0};
    p.nodes = make_nodes_border_cases();
    return p;
}

static std::vector<std::vector<int>> kPopTours_border_cases = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

static std::vector<std::vector<int>> kExpChildTours_border_cases = {
    {0, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 14},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}
};

int run_mutate_test_border_cases() {
    std::cout << "Mutate Unit Test (Border Cases, 15 cities)" << std::endl;
    hls::stream_of_blocks<IndividualAsBeats> in0;
    hls::stream_of_blocks<IndividualAsBeats> out0;

    const ProblemInstance problem = make_problem_border_cases();
    const std::vector<Individual> pop_0 = build_population(problem, kPopTours_border_cases, kFitness0Base);

    std::cout << "Streaming parents into block streams..." << std::endl;
    tb_push_individuals(in0, problem, pop_0);

    hls::stream<CityPair> mutate_pairs("mutate_pairs_border_cases");
    const std::vector<CityPair> cut_points = {
        CityPair{1, 13},
        CityPair{0, 14}
    };

    for (std::size_t i = 0; i < kNumIndividuals_border_cases; ++i) {
        mutate_pairs.write(cut_points[i]);
    }

    std::cout << "Executing Mutate (border cases)..." << std::endl;
    mutate(in0, mutate_pairs, out0, static_cast<int>(kNumIndividuals_border_cases), kNumCities_border_cases);

    std::cout << "Reading children from block streams..." << std::endl;
    int errors = 0;
    std::vector<Individual> child_0;
    try {
        child_0 = tb_pull_individuals(out0, problem, kNumIndividuals_border_cases);
    } catch (const std::exception &e) {
        std::cout << "Exception decoding children (corrupted individual in border cases mutate test): " << e.what() << std::endl;
        return 1;
    }

    const std::vector<Individual> exp_child_0 = build_population(problem, kExpChildTours_border_cases, kFitness0Base);

    std::cout << "Verifying Outputs..." << std::endl;
    for (std::size_t i = 0; i < kNumIndividuals_border_cases; ++i) {
        if (child_0[i] != exp_child_0[i]) {
            std::cout << "Indiv " << i << " (cut [" << cut_points[i].start << ", " << cut_points[i].end << "]) mismatch:\n -g:"
                      << child_0[i] << "\n -e:" << exp_child_0[i] << std::endl;
            errors++;
        }
    }

    if (errors == 0) {
        std::cout << "BORDER CASES MUTATE TEST PASSED!" << std::endl;
    } else {
        std::cout << "BORDER CASES MUTATE TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}

