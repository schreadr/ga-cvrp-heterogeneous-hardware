#include "crossover.hpp"
#include "test_helper.hpp"

#include <cstring>
#include <iostream>
#include <vector>

#include "../../../../sw/src/ga_encoding_constants.hpp"
#include "../../../../sw/src/ga.hpp"
#include "../../../../sw/src/ga_streaming.hpp"

static int kNumCities = 6;
static std::size_t kNumIndividuals = 4;
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
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0},
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0}
};

static std::vector<std::vector<int>> kPop1Tours = {
    {5, 3, 1, 0, 2, 4},
    {2, 0, 4, 1, 5, 3},
    {5, 3, 1, 0, 2, 4},
    {2, 0, 4, 1, 5, 3}
};

static std::vector<std::vector<int>> kExpChild0Tours = {
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0},
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0}
};

static std::vector<std::vector<int>> kExpChild1Tours = {
    {4, 3, 1, 0, 2, 5},
    {2, 0, 4, 1, 5, 3},
    {4, 3, 1, 0, 2, 5},
    {2, 0, 4, 1, 5, 3}
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

    std::cout << "Executing Crossover..." << std::endl;
    crossover(in0, in1, out0, out1,
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
