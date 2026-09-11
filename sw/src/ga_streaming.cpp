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

std::vector<std::array<uint64_t, 8>> encode_individual_fixed(
    const ProblemInstance &problem,
    const Individual &individual
) {
    std::vector<uint64_t> fields;
    fields.reserve(problem.dimension);

    // 1. Fitness field: LSB placed at bit 0 of field 0
    uint64_t fitness_mask = (FIELD_BITS >= 64) ? ~0ULL : ((1ULL << FIELD_BITS) - 1ULL);
    fields.push_back(static_cast<uint64_t>(individual.fitness) & fitness_mask);

    // 2. City fields: each integer LSB-aligned to its starting offset L
    const uint64_t id_mask    = (1ULL << NODE_ID_BITS) - 1ULL;
    const uint64_t coord_mask = (1ULL << COORD_BITS) - 1ULL;
    const uint64_t cap_mask   = (1ULL << CAPACITY_BITS) - 1ULL;

    for (const auto &gene : individual.tour) {
        const ProblemNode &node = problem.nodes.at(static_cast<std::size_t>(gene.node_id));

        uint64_t id_val    = static_cast<uint64_t>(gene.node_id) & id_mask;
        uint64_t x_val     = static_cast<uint64_t>(node.x) & coord_mask;
        uint64_t y_val     = static_cast<uint64_t>(node.y) & coord_mask;
        uint64_t cap_val   = static_cast<uint64_t>(node.demand) & cap_mask;
        uint64_t depot_val = gene.depot_after ? 1ULL : 0ULL;

        uint64_t city = 0;
        city |= (x_val << 0);                                              // Interval [COORD_BITS-1 : 0]
        city |= (y_val << COORD_BITS);                                     // Interval [2*COORD_BITS-1 : COORD_BITS]
        city |= (cap_val << (2 * COORD_BITS));                             // Interval [2*COORD_BITS+CAPACITY_BITS-1 : 2*COORD_BITS]
        city |= (depot_val << (2 * COORD_BITS + CAPACITY_BITS));           // Interval [2*COORD_BITS+CAPACITY_BITS]
        city |= (id_val << (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS)); // Interval [2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS + NODE_ID_BITS - 1 : 2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS]

        fields.push_back(city);
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

    individual.fitness = static_cast<uint32_t>(fields[0] & field_mask);

    const std::uint64_t coord_mask    = (1ULL << COORD_BITS) - 1ULL;
    const std::uint64_t capacity_mask = (1ULL << CAPACITY_BITS) - 1ULL;
    const std::uint64_t id_mask       = (1ULL << NODE_ID_BITS) - 1ULL;
    const std::size_t num_cities       = problem.nodes.size();

    individual.tour.reserve(num_cities);
    for (std::size_t i = 1; i <= num_cities; ++i) {
        std::uint64_t city = fields[i];

        // Extract using exact lower bounds
        std::uint32_t x      = static_cast<uint32_t>((city >> 0) & coord_mask);
        std::uint32_t y      = static_cast<uint32_t>((city >> COORD_BITS) & coord_mask);
        std::uint32_t demand = static_cast<uint32_t>((city >> (2 * COORD_BITS)) & capacity_mask);
        bool depot_after     = ((city >> (2 * COORD_BITS + CAPACITY_BITS)) & 1ULL) != 0ULL;
        std::uint32_t node_id_enc = static_cast<uint32_t>((city >> (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS)) & id_mask);

        // Validate the encoded node_id matches the problem node
        int node_id = static_cast<int>(node_id_enc);
        if (node_id >= 0 && node_id < static_cast<int>(problem.nodes.size())) {
            const auto &node = problem.nodes.at(static_cast<std::size_t>(node_id));
            if (node.x != static_cast<int>(x) || node.y != static_cast<int>(y)) {
                std::cout << "Decoded node_id: " << node_id << ", x: " << x << ", y: " << y
                          << ", demand: " << demand << ", depot_after: " << depot_after
                          << " but ProblemInstance has x: " << node.x << ", y: " << node.y
                          << std::endl;
                throw std::runtime_error("Decoded node_id does not match coordinates in ProblemInstance. ");
            }
        } else {
            throw std::runtime_error("Decoded node_id is out of range.");
        }

        individual.tour.push_back(TourGene{node_id, depot_after});
    }

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
    int problem_dimension
) {
    const int beats_per_individual = get_total_beats(problem_dimension - 1);
    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;
    const std::size_t fitness_bytes = FIELD_BITS / 8;
    const std::size_t genome_bytes = bytes_per_individual > fitness_bytes
                                   ? bytes_per_individual - fitness_bytes : 0;
    const uint64_t fitness_mask = (FIELD_BITS >= 64) ? ~0ULL : ((1ULL << FIELD_BITS) - 1ULL);

    for (std::size_t i = 0; i < num_individuals; ++i) {
        const std::uint8_t *ind_bytes = buffer + (i * bytes_per_individual);
        std::uint64_t raw_fitness = 0;
        std::memcpy(&raw_fitness, ind_bytes, std::min(sizeof(raw_fitness), fitness_bytes));
        raw_fitness &= fitness_mask;
        population[i].fitness = static_cast<std::uint32_t>(raw_fitness);
        population[i].bytes = ind_bytes;
        population[i].hash = hash_genome(ind_bytes + fitness_bytes, genome_bytes);
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
    std::uniform_int_distribution<uint32_t> dist(1, num_cities);

    uint32_t *words = reinterpret_cast<uint32_t *>(buffer);
    for (std::size_t i = 0; i < num_rnd; ++i) {
        words[i] = dist(rng);
    }
}