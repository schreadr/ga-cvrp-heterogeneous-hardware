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
#include "problem_parser.hpp"

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

int test_random_number_converter() {
    std::cout << "Random Number Converter Unit Test" << std::endl;
    hls::stream<axi_word_t> in_random("in_random");
    hls::stream<CityPair> cx_pairs("cx_pairs");
    hls::stream<CityPair> mut_pairs_0("mut_pairs_0");
    hls::stream<CityPair> mut_pairs_1("mut_pairs_1");

    const int test_individuals = 5;
    std::vector<uint32_t> raw_randoms = {
        15, 3, 8, 20, 10, 5,
        2, 2, 30, 1, 16, 22,
        25, 40, 14, 12, 1, 35,
        19, 11, 5, 9, 28, 4,
        39, 4, 7, 18, 13, 13
    };

    std::vector<std::pair<FieldIndex, FieldIndex>> exp_cx = {
        {3, 15}, {2, 2}, {25, 40}, {11, 19}, {4, 39}
    };
    std::vector<std::pair<FieldIndex, FieldIndex>> exp_mut_0 = {
        {8, 20}, {1, 30}, {12, 14}, {5, 9}, {7, 18}
    };
    std::vector<std::pair<FieldIndex, FieldIndex>> exp_mut_1 = {
        {5, 10}, {16, 22}, {1, 35}, {4, 28}, {13, 13}
    };

    const std::size_t payload_bytes = raw_randoms.size() * sizeof(uint32_t);
    const std::size_t total_bytes = get_random_bytes(test_individuals);
    std::vector<uint8_t> beat_bytes(total_bytes, 0);
    std::memcpy(beat_bytes.data(), raw_randoms.data(), payload_bytes);

    tb_push_bytes(in_random, beat_bytes.data(), total_bytes, true);

    random_number_converter(in_random, cx_pairs, mut_pairs_0, mut_pairs_1, test_individuals);

    int errors = 0;
    for (int i = 0; i < test_individuals; ++i) {
        CityPair got_cx = cx_pairs.read();
        CityPair got_mut_0 = mut_pairs_0.read();
        CityPair got_mut_1 = mut_pairs_1.read();

        if (got_cx.start != exp_cx[i].first || got_cx.end != exp_cx[i].second) {
            std::cout << "Mismatch in cx_pair[" << i << "]: got ("
                      << got_cx.start << ", " << got_cx.end << ") expected ("
                      << exp_cx[i].first << ", " << exp_cx[i].second << ")\n";
            errors++;
        }
        if (got_mut_0.start != exp_mut_0[i].first || got_mut_0.end != exp_mut_0[i].second) {
            std::cout << "Mismatch in mut_pair_0[" << i << "]: got ("
                      << got_mut_0.start << ", " << got_mut_0.end << ") expected ("
                      << exp_mut_0[i].first << ", " << exp_mut_0[i].second << ")\n";
            errors++;
        }
        if (got_mut_1.start != exp_mut_1[i].first || got_mut_1.end != exp_mut_1[i].second) {
            std::cout << "Mismatch in mut_pair_1[" << i << "]: got ("
                      << got_mut_1.start << ", " << got_mut_1.end << ") expected ("
                      << exp_mut_1[i].first << ", " << exp_mut_1[i].second << ")\n";
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

int run_integration_test(){
    hls::stream<axi_word_t> in0("in0"), out0("out0");
    hls::stream<axi_word_t> in1("in1"), out1("out1");
    hls::stream<axi_word_t> in_random("in_random");

    const ProblemInstance problem = make_problem();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours);

    const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, pop_0);
    const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, pop_1);

    const std::size_t total_bytes_0 = enc_0.size();
    const std::size_t total_bytes_1 = enc_1.size();

    // Stream the cut points corresponding to the hardcoded test values
    // kOxStart = 2, kOxEnd = 34; kMutateStart = 2, kMutateEnd = 34
    // Pass them in mixed order to verify the converter sorts smaller first.
    std::vector<uint32_t> rnd_nums = {
        34, 2, 2, 34, 2, 34,
        2, 34, 34, 2, 2, 34,
        34, 2, 34, 2, 34, 2
    };
    const std::size_t payload_bytes_rnd = rnd_nums.size() * sizeof(uint32_t);
    const std::size_t total_bytes_rnd = get_random_bytes(kNumIndividuals);
    std::vector<uint8_t> enc_rnd(total_bytes_rnd, 0);
    std::memcpy(enc_rnd.data(), rnd_nums.data(), payload_bytes_rnd);

    int dbg_indiv0, dbg_cities;

    std::cout << "Streaming encoded populations and random numbers..." << std::endl;
    tb_push_bytes(in0, enc_0.data(), total_bytes_0, true);
    tb_push_bytes(in1, enc_1.data(), total_bytes_1, true);
    tb_push_bytes(in_random, enc_rnd.data(), total_bytes_rnd, true);

    std::cout << "Executing Processing Node..." << std::endl;
    processing_node(in0, out0, in1, out1, in_random,
                    kNumIndividuals,
                    kNumCities,
                    problem.capacity,
                    (problem.depot.x << COORD_BITS) | problem.depot.y,
                    &dbg_indiv0, &dbg_cities);

    std::cout << "Processing Node finished. Reading outputs from stream..." << std::endl;
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

static std::vector<std::vector<int>> kPop0Tours_border_cases = {
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}
};

static std::vector<std::vector<int>> kPop1Tours_border_cases = {
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
    {14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

static std::vector<std::vector<int>> kExpChild0Tours_border_cases = {
    {14, 13, 12, 11, 10, 5, 6, 7, 8, 9, 4, 3, 2, 1, 0},
    {3, 2, 1, 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}
};

static std::vector<std::vector<int>> kExpChild1Tours_border_cases = {
    {0, 1, 2, 3, 4, 9, 8, 7, 6, 5, 10, 11, 12, 13, 14},
    {11, 12, 13, 14, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}
};

int run_integration_test_border_cases() {
    std::cout << "Running Integration Test (Border Cases, 15 cities)..." << std::endl;
    hls::stream<axi_word_t> in0("in0_border_cases"), out0("out0_border_cases");
    hls::stream<axi_word_t> in1("in1_border_cases"), out1("out1_border_cases");
    hls::stream<axi_word_t> in_random("in_random_border_cases");

    const ProblemInstance problem = make_problem_border_cases();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours_border_cases);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours_border_cases);

    const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, pop_0);
    const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, pop_1);

    const std::size_t total_bytes_0 = enc_0.size();
    const std::size_t total_bytes_1 = enc_1.size();

    std::vector<uint32_t> rnd_nums = {
        6, 10, 1, 1, 1, 1,
        5, 15, 1, 1, 1, 1
    };
    const std::size_t payload_bytes_rnd = rnd_nums.size() * sizeof(uint32_t);
    const std::size_t total_bytes_rnd = get_random_bytes(kNumIndividuals_border_cases);
    std::vector<uint8_t> enc_rnd(total_bytes_rnd, 0);
    std::memcpy(enc_rnd.data(), rnd_nums.data(), payload_bytes_rnd);

    int dbg_indiv0, dbg_cities;

    std::cout << "Streaming encoded populations and random numbers (border cases)..." << std::endl;
    tb_push_bytes(in0, enc_0.data(), total_bytes_0, true);
    tb_push_bytes(in1, enc_1.data(), total_bytes_1, true);
    tb_push_bytes(in_random, enc_rnd.data(), total_bytes_rnd, true);

    std::cout << "Executing Processing Node (border cases)..." << std::endl;
    processing_node(in0, out0, in1, out1, in_random,
                    static_cast<int>(kNumIndividuals_border_cases),
                    kNumCities_border_cases,
                    problem.capacity,
                    (problem.depot.x << COORD_BITS) | problem.depot.y,
                    &dbg_indiv0, &dbg_cities);

    std::cout << "Processing Node (border cases) finished. Reading outputs from stream..." << std::endl;
    std::vector<std::uint8_t> got_0(total_bytes_0);
    std::vector<std::uint8_t> got_1(total_bytes_1);
    tb_pull_bytes(out0, got_0.data(), total_bytes_0);
    tb_pull_bytes(out1, got_1.data(), total_bytes_1);

    std::vector<Individual> exp_child_0 = build_population(problem, kExpChild0Tours_border_cases);
    std::vector<Individual> exp_child_1 = build_population(problem, kExpChild1Tours_border_cases);

    std::cout << "Decoding and verifying output individuals (border cases)..." << std::endl;
    std::vector<Individual> got_child_0;
    std::vector<Individual> got_child_1;
    try {
        got_child_0 = decode_population_fixed(problem, got_0.data(), kNumIndividuals_border_cases);
        got_child_1 = decode_population_fixed(problem, got_1.data(), kNumIndividuals_border_cases);
    } catch (const std::exception &e) {
        std::cout << "Exception decoding output population in border cases integration test: "
                  << e.what() << std::endl;
        std::cout << "BORDER CASES INTEGRATION TEST FAILED with exception." << std::endl;
        return 1;
    }

    int errors = 0;
    for (std::size_t i = 0; i < kNumIndividuals_border_cases; ++i) {
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
        std::cout << "BORDER CASES INTEGRATION TEST PASSED!" << std::endl;
    } else {
        std::cout << "BORDER CASES INTEGRATION TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}

int run_integration_test_big_instance() {
    std::cout << "Running Integration Test (Big Instance: X-n1001-k43)..." << std::endl;
    hls::stream<axi_word_t> in0("in0_big"), out0("out0_big");
    hls::stream<axi_word_t> in1("in1_big"), out1("out1_big");
    hls::stream<axi_word_t> in_random("in_random_big");

    const std::string vrp_path = "X-n1001-k43.vrp";
    std::cout << "Loading problem from: " << vrp_path << std::endl;
    const ProblemInstance problem = parse_vrp(vrp_path);
    validate_encoding_constants(problem);

    const std::size_t num_ind = 2;
    const uint32_t seed = 12648430;

    std::cout << "Generating initial population (" << num_ind << " individuals per stream)..." << std::endl;
    const std::vector<Individual> pop_0 = build_initial_population(problem, num_ind, seed);
    const std::vector<Individual> pop_1 = build_initial_population(problem, num_ind, seed ^ 0xBEEF);

    const std::vector<std::uint8_t> enc_0 = encode_population_fixed(problem, pop_0);
    const std::vector<std::uint8_t> enc_1 = encode_population_fixed(problem, pop_1);

    const std::size_t total_bytes_0 = enc_0.size();
    const std::size_t total_bytes_1 = enc_1.size();

    const std::size_t total_bytes_rnd = get_random_bytes(num_ind);
    std::vector<uint8_t> enc_rnd(total_bytes_rnd, 0);
    generate_random_numbers(num_ind, enc_rnd.data(), problem.dimension - 1, seed ^ 0xDEADBEEF);

    int dbg_indiv0 = 0, dbg_cities = 0;

    std::cout << "Streaming encoded populations and random numbers (big instance)..." << std::endl;
    tb_push_bytes(in0, enc_0.data(), total_bytes_0, true);
    tb_push_bytes(in1, enc_1.data(), total_bytes_1, true);
    tb_push_bytes(in_random, enc_rnd.data(), total_bytes_rnd, true);

    std::cout << "Executing Processing Node (big instance)..." << std::endl;
    processing_node(in0, out0, in1, out1, in_random,
                    static_cast<int>(num_ind),
                    problem.dimension - 1,
                    problem.capacity,
                    (problem.depot.x << COORD_BITS) | problem.depot.y,
                    &dbg_indiv0, &dbg_cities);

    std::cout << "Processing Node (big instance) finished. Reading outputs from stream..." << std::endl;
    std::vector<std::uint8_t> got_0(total_bytes_0);
    std::vector<std::uint8_t> got_1(total_bytes_1);
    tb_pull_bytes(out0, got_0.data(), total_bytes_0);
    tb_pull_bytes(out1, got_1.data(), total_bytes_1);

    std::cout << "Decoding and verifying output individuals (big instance)..." << std::endl;
    std::vector<Individual> got_child_0;
    std::vector<Individual> got_child_1;
    try {
        got_child_0 = decode_population_fixed(problem, got_0.data(), num_ind);
        got_child_1 = decode_population_fixed(problem, got_1.data(), num_ind);
    } catch (const std::exception &e) {
        std::cout << "Exception decoding output population in big instance test: "
                  << e.what() << std::endl;
        std::cout << "BIG INSTANCE INTEGRATION TEST FAILED with exception." << std::endl;
        return 1;
    }

    int errors = 0;
    for (std::size_t i = 0; i < num_ind; ++i) {
        uint32_t exp_fit_0 = compute_fitness(problem, got_child_0[i]);
        if (got_child_0[i].fitness != exp_fit_0) {
            std::cout << "Stream 0 indiv " << i << " fitness mismatch: got "
                      << got_child_0[i].fitness << ", expected " << exp_fit_0 << std::endl;
            errors++;
        }
        uint32_t exp_fit_1 = compute_fitness(problem, got_child_1[i]);
        if (got_child_1[i].fitness != exp_fit_1) {
            std::cout << "Stream 1 indiv " << i << " fitness mismatch: got "
                      << got_child_1[i].fitness << ", expected " << exp_fit_1 << std::endl;
            errors++;
        }
    }

    if (errors == 0) {
        std::cout << "BIG INSTANCE INTEGRATION TEST PASSED!" << std::endl;
    } else {
        std::cout << "BIG INSTANCE INTEGRATION TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}

int main() {
    int errors = 0;
    int cur_err = 0;

    std::cout << "\nRunning Converter Unit Test..." << std::endl;
    cur_err = test_random_number_converter();
    if (cur_err != 0) std::cout << "Converter Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Util Unit Test..." << std::endl;
    cur_err = test_util();
    if (cur_err != 0) std::cout << "Util Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Mutate Unit Test..." << std::endl;
    cur_err = run_mutate_test();
    if (cur_err != 0) std::cout << "Mutate Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Mutate Border Cases Unit Test..." << std::endl;
    cur_err = run_mutate_test_border_cases();
    if (cur_err != 0) std::cout << "Mutate Border Cases Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Crossover Unit Test..." << std::endl;
    cur_err = run_crossover_test();
    if (cur_err != 0) std::cout << "Crossover Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Crossover Border Cases Unit Test..." << std::endl;
    cur_err = run_crossover_test_border_cases();
    if (cur_err != 0) std::cout << "Crossover Border Cases Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Repair/Fitness Unit Test..." << std::endl;
    cur_err = run_repair_fitness_test();
    if (cur_err != 0) std::cout << "Repair/Fitness Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Repair/Fitness Border Cases Unit Test..." << std::endl;
    cur_err = run_repair_fitness_test_border_cases();
    if (cur_err != 0) std::cout << "Repair/Fitness Border Cases Unit Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Integration Test..." << std::endl;
    cur_err = run_integration_test();
    if (cur_err != 0) std::cout << "Integration Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Integration Border Cases Test..." << std::endl;
    cur_err = run_integration_test_border_cases();
    if (cur_err != 0) std::cout << "Integration Border Cases Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    std::cout << "\nRunning Integration Big Instance Test..." << std::endl;
    cur_err = run_integration_test_big_instance();
    if (cur_err != 0) std::cout << "Integration Big Instance Test failed with " << cur_err << " errors." << std::endl;
    errors += cur_err;

    if (errors == 0) {
        std::cout << "\nAll tests passed!" << std::endl;
    } else {
        std::cout << "\nTests finished with " << errors << " total errors." << std::endl;
    }
    return errors;
}

