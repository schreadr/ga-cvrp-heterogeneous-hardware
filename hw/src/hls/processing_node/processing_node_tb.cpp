#include "processing_node.hpp"

#include <cstring>
#include <iostream>
#include <vector>

#include "../../../../sw/src/ga_encoding_constants.hpp"
#include "../../../../sw/src/ga.hpp"
#include "../../../../sw/src/ga_streaming.hpp"

constexpr int kNumCities = 6;
constexpr std::size_t kNumIndividuals = 4;
constexpr std::uint32_t kFitness0Base = 1000;
constexpr std::uint32_t kFitness1Base = 2000;

constexpr int kDepotId = 1;

std::vector<ProblemNode> make_nodes() {
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

ProblemInstance make_problem() {
    ProblemInstance p;
    p.name = "tb_hardcoded";
    p.dimension = kNumCities + 1;
    p.capacity = (1 << CAPACITY_BITS) - 1;
    p.depot = ProblemNode{kDepotId, 0, 0, 0};
    p.nodes = make_nodes();
    return p;
}

std::vector<std::vector<int>> kPop0Tours = {
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0},
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0}
};

std::vector<std::vector<int>> kPop1Tours = {
    {5, 3, 1, 0, 2, 4},
    {2, 0, 4, 1, 5, 3},
    {5, 3, 1, 0, 2, 4},
    {2, 0, 4, 1, 5, 3}
};

std::vector<Individual> build_population(const ProblemInstance &problem,
                                         const std::vector<std::vector<int>> &tours,
                                         std::uint32_t fitness_base) {
    std::vector<Individual> out(tours.size());
    for (std::size_t i = 0; i < tours.size(); ++i) {
        out[i].fitness = (fitness_base + static_cast<std::uint32_t>(i)) & ((1u << FIELD_BITS) - 1u);
        out[i].tour.reserve(tours[i].size());
        for (int node_id : tours[i]) {
            (void)problem;
            out[i].tour.push_back(TourGene{node_id, false});
        }
    }
    return out;
}

std::vector<std::vector<int>> kExpChild0Tours = {
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0},
    {0, 1, 2, 3, 4, 5},
    {5, 4, 3, 2, 1, 0}
};

std::vector<std::vector<int>> kExpChild1Tours = {
    {4, 3, 1, 0, 2, 5},
    {2, 0, 4, 1, 5, 3},
    {4, 3, 1, 0, 2, 5},
    {2, 0, 4, 1, 5, 3}
};

static void tb_push_bytes(hls::stream<axi_word_t> &stream,
                          const std::uint8_t *bytes,
                          std::size_t n_bytes,
                          bool is_last) {
    constexpr std::size_t beat_bytes_count = AXI_BYTES;          // bytes per beat (e.g., 64)
    constexpr std::size_t beat_words = AXI_BYTES / sizeof(uint64_t); // number of 64-bit words (8)
    const std::size_t total_beats = (n_bytes + beat_bytes_count - 1) / beat_bytes_count;

    for (std::size_t b = 0; b < total_beats; b++) {
        axi_word_t word;
        word.data = 0;
        word.keep = -1;
        word.strb = -1;
        word.last = (is_last && b + 1 == total_beats) ? 1 : 0;

        for (std::size_t w = 0; w < beat_words; ++w) {
            std::uint64_t chunk = 0;
            std::size_t offset = b * beat_bytes_count + w * sizeof(uint64_t);
            if (offset < n_bytes) {
                std::size_t remaining = n_bytes - offset;
                std::memcpy(&chunk, bytes + offset,
                            std::min(sizeof(chunk), remaining));
            }
            word.data.range((w + 1) * 64 - 1, w * 64) = chunk;
        }
        stream.write(word);
    }
}

static void tb_pull_bytes(hls::stream<axi_word_t> &stream,
                          std::uint8_t *bytes,
                          std::size_t n_bytes) {
    constexpr std::size_t beat_bytes_count = AXI_BYTES;
    constexpr std::size_t beat_words = AXI_BYTES / sizeof(uint64_t);
    const std::size_t total_beats = (n_bytes + beat_bytes_count - 1) / beat_bytes_count;

    for (std::size_t b = 0; b < total_beats; b++) {
        axi_word_t word = stream.read();
        for (std::size_t w = 0; w < beat_words; ++w) {
            std::uint64_t chunk = word.data.range((w + 1) * 64 - 1, w * 64);
            std::size_t offset = b * beat_bytes_count + w * sizeof(uint64_t);
            if (offset < n_bytes) {
                std::size_t remaining = n_bytes - offset;
                std::memcpy(bytes + offset, &chunk,
                            std::min(sizeof(chunk), remaining));
            }
        }
    }
}

int main() {
    hls::stream<axi_word_t> in0("in0"), out0("out0");
    hls::stream<axi_word_t> in1("in1"), out1("out1");

    const ProblemInstance problem = make_problem();
    const std::vector<Individual> pop_0 = build_population(problem, kPop0Tours, kFitness0Base);
    const std::vector<Individual> pop_1 = build_population(problem, kPop1Tours, kFitness1Base);

    std::cout << "Problem: ";
    for( int i = 0; i < problem.nodes.size(); ++i) {
        const auto &node = problem.nodes[i];
        std::cout << "Node " << i
                  << ": (" << node.x << ", " << node.y
                  << "), demand=" << node.demand << "\n";
    }

    // Encode both halves with the host codec.
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

    std::cout << "total_bytes_0 = " << total_bytes_0 << std::endl;
    std::cout << "Expected beats from design: " << (total_bytes_0 / AXI_BYTES) << std::endl;
    std::cout << "out0.size() before read = " << out0.size() << std::endl;
    std::cout << "out1.size() before read = " << out1.size() << std::endl;
    std::cout << "Pulling data..." << std::endl;
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