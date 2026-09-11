#include "crossover.hpp"
#include "test_helper.hpp"

#include <cstring>
#include <iostream>
#include <vector>

#include "ga_encoding_constants.hpp"
#include "ga.hpp"
#include "ga_streaming.hpp"

static int kNumCities = 40;
static std::size_t kNumIndividuals = 3;
static std::uint32_t kFitness0Base = 1000;
static std::uint32_t kFitness1Base = 2000;

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

static std::vector<std::vector<int>> kPop0Tours = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39}
};

static std::vector<std::vector<int>> kPop1Tours = {
    {39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {10, 24, 6, 3, 22, 13, 29, 23, 33, 14, 38, 7, 21, 5, 32, 28, 25, 15, 36, 20, 37, 16, 39, 4, 9, 8, 30, 11, 1, 2, 26, 27, 17, 18, 19, 31, 34, 12, 35, 0}
};

static std::vector<std::vector<int>> kExpChild0Tours = {
    {34, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 0, 39, 38, 37, 36, 35},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {39, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 0, 38, 36, 37}
};

static std::vector<std::vector<int>> kExpChild1Tours = {
    {5, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 39, 0, 1, 2, 3, 4},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {31, 24, 6, 3, 22, 13, 29, 23, 33, 14, 38, 7, 21, 5, 32, 28, 25, 15, 36, 20, 37, 16, 39, 4, 9, 8, 30, 11, 1, 2, 26, 27, 17, 18, 34, 35, 0, 10, 12, 19}
};

int run_crossover_test() {
    std::cout << "Crossover Unit Test" << std::endl;
    hls::stream_of_blocks<IndividualAsBeats> in0;
    hls::stream_of_blocks<IndividualAsBeats> out0;
    hls::stream_of_blocks<IndividualAsBeats> in1;
    hls::stream_of_blocks<IndividualAsBeats> out1;

    const ProblemInstance problem = make_problem();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours, kFitness0Base);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours, kFitness1Base);

    std::cout << "Streaming parents into block streams..." << std::endl;
    tb_push_individuals(in0, problem, pop_0);
    tb_push_individuals(in1, problem, pop_1);

    const FieldIndex kOxStart = 2;
    const FieldIndex kOxEnd = 2 + FIELDS_PER_BEAT * 2;
    hls::stream<CityPair> crossover_pairs("crossover_pairs");

    for (std::size_t i = 0; i < kNumIndividuals; ++i) {
        crossover_pairs.write(CityPair{kOxStart, kOxEnd});
    }

    std::cout << "Executing Crossover..." << std::endl;
    crossover(in0, in1, crossover_pairs, out0, out1,
              static_cast<int>(kNumIndividuals),
              kNumCities);

    std::cout << "Reading children from block streams..." << std::endl;
    const std::vector<Individual> child_0 = tb_pull_individuals(out0, problem, kNumIndividuals);
    const std::vector<Individual> child_1 = tb_pull_individuals(out1, problem, kNumIndividuals);
    const std::vector<Individual> exp_child_0 = build_population(problem, kExpChild0Tours, kFitness0Base);
    const std::vector<Individual> exp_child_1 = build_population(problem, kExpChild1Tours, kFitness1Base);

    std::cout << "Verifying Outputs..." << std::endl;
    int errors = 0;
    for (std::size_t i = 0; i < kNumIndividuals; ++i) {
        if (child_0[i] != exp_child_0[i]) {
            std::cout << "Stream 0 indiv " << i << " mismatch:\n -g:"
                      << child_0[i] << "\n -e:" << exp_child_0[i] << std::endl;
            errors++;
        }
        if (child_1[i] != exp_child_1[i]) {
            std::cout << "Stream 1 indiv " << i << " mismatch:\n -g:"
                      << child_1[i] << "\n -e:" << exp_child_1[i] << std::endl;
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

static int kNumCities_1b = 15;
static std::size_t kNumIndividuals_1b = 4;

static std::vector<ProblemNode> make_nodes_1beat() {
    std::vector<ProblemNode> nodes(kNumCities_1b);
    for (int i = 1; i <= kNumCities_1b; ++i) {
        nodes[i-1] = ProblemNode{
           i,
           i & ((1 << COORD_BITS) - 1),
           (i * 2) & ((1 << COORD_BITS) - 1),
           i & ((1 << CAPACITY_BITS) - 1)
        };
    }
    return nodes;
}

static ProblemInstance make_problem_1beat() {
    ProblemInstance p;
    p.name = "tb_1beat";
    p.dimension = kNumCities_1b + 1;
    p.capacity = (1 << CAPACITY_BITS) - 1;
    p.depot = ProblemNode{kDepotId, 0, 0, 0};
    p.nodes = make_nodes_1beat();
    return p;
}

static std::vector<std::vector<int>> kPop0Tours_1b = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}
};

static std::vector<std::vector<int>> kPop1Tours_1b = {
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

static std::vector<std::vector<int>> kExpChild0Tours_1b = {
    {14, 13, 12, 11, 10, 5, 6, 7, 8, 9, 4, 3, 2, 1, 0},
    {3, 2, 1, 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {0, 1, 2, 3, 4, 9, 8, 7, 6, 5, 14, 13, 12, 11, 10},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}
};

static std::vector<std::vector<int>> kExpChild1Tours_1b = {
    {0, 1, 2, 3, 4, 9, 8, 7, 6, 5, 10, 11, 12, 13, 14},
    {11, 12, 13, 14, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
    {14, 13, 12, 11, 10, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

int run_crossover_test_1beat() {
    std::cout << "Crossover Unit Test (1-Beat Instance, 15 cities)" << std::endl;
    hls::stream_of_blocks<IndividualAsBeats> in0;
    hls::stream_of_blocks<IndividualAsBeats> out0;
    hls::stream_of_blocks<IndividualAsBeats> in1;
    hls::stream_of_blocks<IndividualAsBeats> out1;

    const ProblemInstance problem = make_problem_1beat();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours_1b, kFitness0Base);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours_1b, kFitness1Base);

    std::cout << "Streaming parents into block streams..." << std::endl;
    tb_push_individuals(in0, problem, pop_0);
    tb_push_individuals(in1, problem, pop_1);

    hls::stream<CityPair> crossover_pairs("crossover_pairs_1b");
    const std::vector<CityPair> cut_points = {
        CityPair{6, 10},
        CityPair{5, 15},
        CityPair{1, 5},
        CityPair{1, 15}
    };

    for (std::size_t i = 0; i < kNumIndividuals_1b; ++i) {
        crossover_pairs.write(cut_points[i]);
    }

    std::cout << "Executing Crossover (1-beat)..." << std::endl;
    crossover(in0, in1, crossover_pairs, out0, out1,
              static_cast<int>(kNumIndividuals_1b),
              kNumCities_1b);

    std::cout << "Reading children from block streams..." << std::endl;
    int errors = 0;
    std::vector<Individual> child_0;
    std::vector<Individual> child_1;
    try {
        child_0 = tb_pull_individuals(out0, problem, kNumIndividuals_1b);
        child_1 = tb_pull_individuals(out1, problem, kNumIndividuals_1b);
    } catch (const std::exception &e) {
        std::cout << "Exception decoding children (corrupted individual in 1-beat test): " << e.what() << std::endl;
        return 1;
    }

    const std::vector<Individual> exp_child_0 = build_population(problem, kExpChild0Tours_1b, kFitness0Base);
    const std::vector<Individual> exp_child_1 = build_population(problem, kExpChild1Tours_1b, kFitness1Base);

    std::cout << "Verifying Outputs..." << std::endl;
    for (std::size_t i = 0; i < kNumIndividuals_1b; ++i) {
        if (child_0[i] != exp_child_0[i]) {
            std::cout << "Stream 0 indiv " << i << " (cut [" << cut_points[i].start << ", " << cut_points[i].end << "]) mismatch:\n -g:"
                      << child_0[i] << "\n -e:" << exp_child_0[i] << std::endl;
            errors++;
        }
        if (child_1[i] != exp_child_1[i]) {
            std::cout << "Stream 1 indiv " << i << " (cut [" << cut_points[i].start << ", " << cut_points[i].end << "]) mismatch:\n -g:"
                      << child_1[i] << "\n -e:" << exp_child_1[i] << std::endl;
            errors++;
        }
    }

    if (errors == 0) {
        std::cout << "1-BEAT CROSSOVER TEST PASSED!" << std::endl;
    } else {
        std::cout << "1-BEAT CROSSOVER TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}
