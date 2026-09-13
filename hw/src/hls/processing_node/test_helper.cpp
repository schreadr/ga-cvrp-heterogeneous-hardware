#include "test_helper.hpp"
#include "ga.hpp"

#include <cstring>
#include <algorithm>

void tb_push_bytes(hls::stream<axi_word_t> &stream,
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

void tb_pull_bytes(hls::stream<axi_word_t> &stream,
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

void make_individual_beats(const ProblemInstance &problem, const Individual &individual, IndividualAsBeats &out) {
    set_fitness(out, individual.fitness);

    for (std::size_t i = 0; i < individual.tour.size(); ++i) {
        const auto &gene = individual.tour[i];
        const ProblemNode &node = problem.nodes.at(static_cast<std::size_t>(gene.node_id));

        FitnessOrCity field = 0;
        set_coord_x(field, node.x);
        set_coord_y(field, node.y);
        set_capacity(field, node.demand);
        set_is_depot(field, gene.depot_after);
        set_node_id(field, gene.node_id);

        set_field(out, static_cast<int>(i + 1), field);
    }
}

Individual decode_individual(const ProblemInstance &problem, const IndividualAsBeats &ind, int num_cities) {
    Individual individual;
    individual.fitness = get_fitness(ind);

    for (int i = 1; i <= num_cities; ++i) {
        FitnessOrCity field = get_field(ind, i);
        int node_id = static_cast<int>(get_node_id(field));
        const ProblemNode &node = problem.nodes.at(static_cast<std::size_t>(node_id));
        bool depot_after = get_is_depot(field);
        individual.tour.push_back(TourGene{node_id, depot_after});
    }

    return individual;
}

void tb_push_individuals(hls::stream_of_blocks<IndividualAsBeats> &stream, const ProblemInstance &problem, const std::vector<Individual> &individuals){
    for (const auto &individual : individuals) {
        hls::write_lock<IndividualAsBeats> ind(stream);
        make_individual_beats(problem, individual, ind);
    }
}

std::vector<Individual> tb_pull_individuals(hls::stream_of_blocks<IndividualAsBeats> &stream, const ProblemInstance &problem, int population_size){
    std::vector<Individual> individuals;
    for (int i = 0; i < population_size; ++i) {
        hls::read_lock<IndividualAsBeats> ind(stream);
        individuals.push_back(decode_individual(problem, ind, problem.nodes.size()));
    }
    return individuals;
}

std::vector<Individual> build_population(const ProblemInstance &problem,
                                         const std::vector<std::vector<int>> &tours) {
    std::vector<Individual> out;
    for (const std::vector<int> &tour : tours) {
        out.push_back(build_individual(problem, tour));
    }
    return out;
}

std::vector<Individual> build_population(const ProblemInstance &problem,
                                         const std::vector<std::vector<int>> &tours,
                                         std::uint32_t fitness_base) {
    std::vector<Individual> out(tours.size());
    for (std::size_t i = 0; i < tours.size(); ++i) {
        out[i].fitness = fitness_base + static_cast<std::uint32_t>(i);
        out[i].tour.reserve(tours[i].size());
        for (int node_id : tours[i]) {
            (void)problem;
            out[i].tour.push_back(TourGene{node_id, false});
        }
    }
    return out;
}
