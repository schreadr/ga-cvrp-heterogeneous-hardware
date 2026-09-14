#include <array>
#include <cstring>
#include <random>

#include "ga_streaming.hpp"
#include "ga.hpp"

void write_bits_le(std::uint8_t *word_bytes, std::size_t start_bit, unsigned width, std::uint32_t value) {
    for (unsigned bit = 0; bit < width; ++bit) {
        if ((value >> bit) & 1u) {
            const std::size_t absolute_bit = start_bit + bit;
            word_bytes[absolute_bit / 8] |= static_cast<std::uint8_t>(1u << (absolute_bit % 8));
        }
    }
}

std::vector<uint8_t> encode_node_cache(const ProblemInstance &problem) {
    const int num_cities = static_cast<int>(problem.nodes.size());
    const int total_beats = get_total_cache_beats(num_cities);
    std::vector<uint8_t> buffer(total_beats * AXI_BYTES, 0);

    const uint64_t coord_mask = (1ULL << COORD_BITS) - 1ULL;
    const uint64_t demand_mask = (1ULL << CAPACITY_BITS) - 1ULL;

    auto encode_entry = [&](int idx, uint64_t x_val, uint64_t y_val, uint64_t demand_val) {
        int beat_idx = idx >> NODE_CACHE_ENTRIES_PER_BEAT_LOG2;
        int word_idx = idx & (NODE_CACHE_ENTRIES_PER_BEAT - 1);

        uint64_t *words = reinterpret_cast<uint64_t *>(buffer.data() + beat_idx * AXI_BYTES);
        uint64_t x = x_val & coord_mask;
        uint64_t y = y_val & coord_mask;
        uint64_t demand = demand_val & demand_mask;

        words[word_idx] = (x << NODE_CACHE_COORD_X_OFFSET) |
                          (y << NODE_CACHE_COORD_Y_OFFSET) |
                          (demand << NODE_CACHE_DEMAND_OFFSET);
    };

    for (int i = 0; i < num_cities; ++i) {
        const ProblemNode &node = problem.nodes[i];
        encode_entry(i, node.x, node.y, node.demand);
    }

    // Depot coordinates stored at index num_cities
    encode_entry(num_cities, problem.depot.x, problem.depot.y, 0);

    return buffer;
}

std::vector<std::array<uint64_t, 8>> encode_individual_fixed(
    const ProblemInstance &problem,
    const Individual &individual
) {
    std::vector<uint64_t> fields;
    fields.reserve(problem.dimension + FITNESS_FIELDS);

    // 1. City fields: each integer contains only node_id (and depot_after if set)
    const uint64_t id_mask = (1ULL << NODE_ID_BITS) - 1ULL;

    for (const auto &gene : individual.tour) {
        uint64_t id_val    = static_cast<uint64_t>(gene.node_id) & id_mask;
        uint64_t depot_val = gene.depot_after ? 1ULL : 0ULL;

        uint64_t city = id_val | (depot_val << DEPOT_FLAG_FIELD_OFFSET);
        fields.push_back(city);
    }

    // 2. Fitness fields: stored across FITNESS_FIELDS fields at indices num_cities .. num_cities + FITNESS_FIELDS - 1
    const uint64_t field_mask = (FIELD_BITS >= 64) ? ~0ULL : ((1ULL << FIELD_BITS) - 1ULL);
    uint64_t fitness_val = individual.fitness;
    for (int k = 0; k < FITNESS_FIELDS; ++k) {
        fields.push_back((fitness_val >> (k * FIELD_BITS)) & field_mask);
    }

    const int beats_per_individual = get_total_beats(static_cast<int>(problem.nodes.size()));
    while (fields.size() < static_cast<std::size_t>(beats_per_individual * FIELDS_PER_BEAT)) {
        fields.push_back(0);
    }

    // Write fields LSB-first directly into 512-bit beat buffer
    std::vector<std::array<uint64_t, 8>> beats(beats_per_individual);
    for (size_t beat_idx = 0; beat_idx < static_cast<size_t>(beats_per_individual); ++beat_idx) {
        std::array<uint64_t, 8> &beat = beats[beat_idx];
        beat.fill(0);

        for (size_t slot = 0; slot < FIELDS_PER_BEAT; ++slot) {
            size_t field_idx = beat_idx * FIELDS_PER_BEAT + slot;
            uint64_t value = fields[field_idx];
            size_t bit_offset = slot * FIELD_BITS;

            beat[bit_offset / 64] |= (value << (bit_offset % 64));
        }
    }
    return beats;
}

std::vector<uint8_t> encode_population_fixed(
    const ProblemInstance &problem,
    const std::vector<Individual> &population
) {
    const int beats_per_individual = get_total_beats(static_cast<int>(problem.nodes.size()));

    std::vector<uint8_t> buffer;
    buffer.reserve(population.size() * beats_per_individual * AXI_BYTES);
    for (const auto &ind : population) {
        auto beats = encode_individual_fixed(problem, ind);
        for (const auto &beat : beats) {
            const uint8_t *bytes = reinterpret_cast<const uint8_t *>(beat.data());
            buffer.insert(buffer.end(), bytes, bytes + AXI_BYTES);
        }
    }
    return buffer;
}

Individual decode_individual_fixed(
    const ProblemInstance &problem,
    const std::uint8_t *beat_bytes
) {
    const int beats_per_individual = get_total_beats(static_cast<int>(problem.nodes.size()));

    Individual individual;
    std::vector<std::uint64_t> fields;
    fields.reserve(beats_per_individual * FIELDS_PER_BEAT);

    const uint64_t field_mask = (FIELD_BITS >= 64) ? ~0ULL : ((1ULL << FIELD_BITS) - 1ULL);

    for (std::size_t beat_idx = 0; beat_idx < static_cast<std::size_t>(beats_per_individual); ++beat_idx) {
        const std::uint64_t *words = reinterpret_cast<const std::uint64_t *>(
            beat_bytes + (beat_idx * AXI_BYTES)
        );

        for (std::size_t slot = 0; slot < FIELDS_PER_BEAT; ++slot) {
            std::size_t bit_offset = slot * FIELD_BITS;
            std::size_t word_idx = bit_offset / 64;
            std::size_t bit_in_word = bit_offset % 64;
            std::uint64_t value = (words[word_idx] >> bit_in_word) & field_mask;
            fields.push_back(value);
        }
    }

    const std::uint64_t id_mask = (1ULL << NODE_ID_BITS) - 1ULL;
    const std::size_t num_cities = problem.nodes.size();

    individual.tour.reserve(num_cities);
    for (std::size_t i = 0; i < num_cities; ++i) {
        std::uint64_t city = fields[i];
        int node_id = static_cast<int>(city & id_mask);
        bool depot_after = ((city >> DEPOT_FLAG_FIELD_OFFSET) & 1ULL) != 0ULL;

        if (node_id >= 0 && node_id < static_cast<int>(num_cities)) {
            individual.tour.push_back(TourGene{node_id, depot_after});
        } else {
            throw std::runtime_error("Decoded node_id is out of range.");
        }
    }

    uint64_t fit = 0;
    for (int k = 0; k < FITNESS_FIELDS; ++k) {
        fit |= (fields[num_cities + k] & field_mask) << (k * FIELD_BITS);
    }
    individual.fitness = static_cast<uint32_t>(fit);

    return individual;
}

std::vector<Individual> decode_population_fixed(
    const ProblemInstance &problem,
    const std::uint8_t *buffer,
    std::size_t num_individuals
) {
    const int beats_per_individual = get_total_beats(static_cast<int>(problem.nodes.size()));

    std::vector<Individual> population;
    population.reserve(num_individuals);

    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;

    for (std::size_t i = 0; i < num_individuals; ++i) {
        const std::uint8_t *ind_bytes = buffer + (i * bytes_per_individual);
        population.push_back(decode_individual_fixed(problem, ind_bytes));
    }

    return population;
}

void population_from_buffer(
    const std::uint8_t *buffer,
    std::size_t num_individuals,
    std::vector<EncodedIndividual> &population,
    int problem_dimension,
    std::size_t pop_offset
) {
    const int num_cities = problem_dimension - 1;
    const int beats_per_individual = get_total_beats(num_cities);
    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;
    const std::size_t city_bytes = (num_cities * FIELD_BITS) / 8;
    const std::size_t fitness_bytes = (FITNESS_FIELDS * FIELD_BITS) / 8;

    for (std::size_t i = 0; i < num_individuals; ++i) {
        const std::uint8_t *ind_bytes = buffer + (i * bytes_per_individual);
        std::uint64_t raw_fitness = 0;
        std::memcpy(&raw_fitness, ind_bytes + city_bytes, std::min(sizeof(raw_fitness), fitness_bytes));
        population[pop_offset + i].fitness = static_cast<std::uint32_t>(raw_fitness);
        population[pop_offset + i].bytes = ind_bytes;
        population[pop_offset + i].hash = hash_genome(ind_bytes, city_bytes);
    }
}

void population_to_buffer(std::vector<EncodedIndividual> &population, std::uint8_t* bytes, int problem_dimension, bool reversedOrder) {
    const int beats_per_individual = get_total_beats(problem_dimension - 1);
    if (reversedOrder) {
        for (int i=population.size()-1; i>=0; --i) {
            // Serialize each individual and write to bytes
            std::uint8_t *dest = bytes + (i * beats_per_individual * AXI_BYTES);
            std::memcpy(dest, population[i].bytes, beats_per_individual * AXI_BYTES);
            population[i] = EncodedIndividual{dest, population[i].fitness, population[i].hash};
        }
    } else {
        for (int i=0; i<population.size(); ++i) {
            // Serialize each individual and write to bytes
            std::memcpy(bytes, population[i].bytes, beats_per_individual * AXI_BYTES);
            population[i] = EncodedIndividual{bytes, population[i].fitness, population[i].hash};
            bytes += beats_per_individual * AXI_BYTES;
        }
    }
}

void generate_random_numbers(
    std::size_t num_individuals,
    std::uint8_t *buffer,
    int num_cities,
    std::uint64_t seed
) {
    const std::size_t total_bytes = get_random_bytes(num_individuals);
    std::memset(buffer, 0, total_bytes);

    const std::size_t num_rnd = num_individuals * RANDOMS_PER_INDIV;
    std::mt19937_64 rng(seed);
    std::uniform_int_distribution<uint32_t> dist(0, num_cities - 1);

    uint32_t *words = reinterpret_cast<uint32_t *>(buffer);
    for (std::size_t i = 0; i < num_rnd; ++i) {
        words[i] = dist(rng);
    }
}