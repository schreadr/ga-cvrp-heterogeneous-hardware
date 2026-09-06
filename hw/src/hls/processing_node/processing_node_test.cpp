#include "processing_node.hpp"
#include "test_helper.hpp"
#include "crossover_test.hpp"
#include "util_test.hpp"
#include "mutate_test.hpp"

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

int run_integration_test(){
    hls::stream<axi_word_t> in0("in0"), out0("out0");
    hls::stream<axi_word_t> in1("in1"), out1("out1");

    const ProblemInstance problem = make_problem();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours, kFitness0Base);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours, kFitness1Base);

    const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, pop_0);
    const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, pop_1);

    const std::size_t total_bytes_0 = enc_0.size();
    const std::size_t total_bytes_1 = enc_1.size();

    int dbg_indiv0, dbg_indiv1, dbg_cities;

    std::cout << "Streaming encoded populations..." << std::endl;
    tb_push_bytes(in0, enc_0.data(), total_bytes_0, true);
    tb_push_bytes(in1, enc_1.data(), total_bytes_1, true);

    std::cout << "Executing Processing Node..." << std::endl;
    processing_node(in0, out0, in1, out1,
                    static_cast<int>(kNumIndividuals),
                    static_cast<int>(kNumIndividuals),
                    kNumCities,
                    &dbg_indiv0, &dbg_indiv1, &dbg_cities);

    std::vector<std::uint8_t> got_0(total_bytes_0);
    std::vector<std::uint8_t> got_1(total_bytes_1);
    tb_pull_bytes(out0, got_0.data(), total_bytes_0);
    tb_pull_bytes(out1, got_1.data(), total_bytes_1);

    // Build the hard-coded expected children via the host codec.
    const std::vector<Individual> exp_child_0 = build_population(problem, kExpChild0Tours, kFitness0Base);
    const std::vector<Individual> exp_child_1 = build_population(problem, kExpChild1Tours, kFitness1Base);
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
    #ifndef __RTL_SIMULATION__
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
    #endif

    std::cout << "\nRunning Integration Test..." << std::endl;
    errors = run_integration_test();
    if (errors != 0) {
        std::cout << "Integration Test failed with " << errors << " errors." << std::endl;
        return errors;
    }

    std::cout << "\nAll tests passed!" << std::endl;
    return errors;
}
