#include "processing_node.hpp"
#include "test_helper.hpp"
#include "crossover_test.hpp"
#include "util_test.hpp"
#include "mutate_test.hpp"
#include "repair_fitness_test.hpp"

#include <cstring>
#include <iostream>
#include <vector>

#include "ga_encoding_constants.hpp"
#include "ga.hpp"
#include "ga_streaming.hpp"

static int kNumCities = 40;
static std::size_t kNumIndividuals = 3;

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
    {34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 39, 38, 37, 36, 35},
    {0, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 34, 35, 36, 37, 38, 39},
    {39, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 34, 35, 0, 38, 36, 37}
};

static std::vector<std::vector<int>> kExpChild1Tours = {
    {5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 0, 1, 2, 3, 4},
    {0, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 34, 35, 36, 37, 38, 39},
    {31, 18, 17, 27, 26, 2, 1, 11, 30, 8, 9, 4, 39, 16, 37, 20, 36, 15, 25, 28, 32, 5, 21, 7, 38, 14, 33, 23, 29, 13, 22, 3, 6, 24, 34, 35, 0, 10, 12, 19}
};

int run_integration_test(){
    hls::stream<axi_word_t> in0("in0"), out0("out0");
    hls::stream<axi_word_t> in1("in1"), out1("out1");

    const ProblemInstance problem = make_problem();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours);

    const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, pop_0);
    const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, pop_1);

    const std::size_t total_bytes_0 = enc_0.size();
    const std::size_t total_bytes_1 = enc_1.size();

    int dbg_indiv0, dbg_cities;

    std::cout << "Streaming encoded populations..." << std::endl;
    tb_push_bytes(in0, enc_0.data(), total_bytes_0, true);
    tb_push_bytes(in1, enc_1.data(), total_bytes_1, true);

    std::cout << "Executing Processing Node..." << std::endl;
    processing_node(in0, out0, in1, out1,
                    kNumIndividuals,
                    kNumCities,
                    problem.capacity,
                    (problem.depot.x << COORD_BITS) | problem.depot.y,
                    &dbg_indiv0, &dbg_cities);

    std::vector<std::uint8_t> got_0(total_bytes_0);
    std::vector<std::uint8_t> got_1(total_bytes_1);
    tb_pull_bytes(out0, got_0.data(), total_bytes_0);
    tb_pull_bytes(out1, got_1.data(), total_bytes_1);

    // Build the hard-coded expected children via the host codec.
    std::vector<Individual> exp_child_0 = build_population(problem, kExpChild0Tours);
    std::vector<Individual> exp_child_1 = build_population(problem, kExpChild1Tours);
    const std::vector<Individual> got_child_0 = decode_population_fixed(problem, got_0.data(), kNumIndividuals);
    const std::vector<Individual> got_child_1 = decode_population_fixed(problem, got_1.data(), kNumIndividuals);

    std::cout << "Verifying Outputs..." << std::endl;
    int errors = 0;
    for (std::size_t i = 0; i < kNumIndividuals; ++i) {
        if (got_child_0[i] != exp_child_0[i]) {
            std::cout << "Stream 0 indiv " << i << " mismatch:\n -g:"
                      << got_child_0[i] << "\n -e:" << exp_child_0[i] << std::endl;
            errors++;
        }
        if (got_child_1[i] != exp_child_1[i]) {
            std::cout << "Stream 1 indiv " << i << " mismatch:\n -g:"
                      << got_child_1[i] << "\n -e:" << exp_child_1[i] << std::endl;
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

int main() {
    int errors=0;
    std::cout << "\nRunning Util Unit Test..." << std::endl;
    errors = test_stream_beats_to_individual();
    if (errors != 0) {
        std::cout << "Util Unit Test failed with " << errors << " errors." << std::endl;
        return errors;
    }
    std::cout << "\nRunning Mutate Unit Test..." << std::endl;
    errors = run_mutate_test();
    if (errors != 0) {
        std::cout << "Mutate Unit Test failed with " << errors << " errors." << std::endl;
        return errors;
    }

    std::cout << "\nRunning Crossover Unit Test..." << std::endl;
    errors = run_crossover_test();
    if (errors != 0) {
        std::cout << "Crossover Unit Test failed with " << errors << " errors." << std::endl;
        return errors;
    }

    std::cout << "\nRunning Repair/Fitness Unit Test..." << std::endl;
    errors = run_repair_fitness_test();
    if (errors != 0) {
        std::cout << "Repair/Fitness Unit Test failed with " << errors << " errors." << std::endl;
        return errors;
    }

    std::cout << "\nRunning Integration Test..." << std::endl;
    errors = run_integration_test();
    if (errors != 0) {
        std::cout << "Integration Test failed with " << errors << " errors." << std::endl;
        return errors;
    }

    std::cout << "\nAll tests passed!" << std::endl;
    return errors;
}
