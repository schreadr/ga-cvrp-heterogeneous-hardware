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
    std::vector<uint32_t> fields;
    fields.reserve(problem.dimension);

    // 1. Fitness field: LSB placed at bit 0 of field 0
    fields.push_back(individual.fitness);

    // 2. City fields: each integer LSB-aligned to its starting offset L
    for (const auto &gene : individual.tour) {
        const ProblemNode &node = problem.nodes.at(static_cast<std::size_t>(gene.node_id));

        // TODO add proper validation and may use long for fitness
        uint32_t id_val    = static_cast<uint32_t>(gene.node_id) & ((1u << NODE_ID_BITS) - 1u);
        uint32_t x_val     = static_cast<uint32_t>(node.x) & ((1u << COORD_BITS) - 1u);
        uint32_t y_val     = static_cast<uint32_t>(node.y) & ((1u << COORD_BITS) - 1u);
        uint32_t cap_val   = static_cast<uint32_t>(node.demand) & ((1u << CAPACITY_BITS) - 1u);
        uint32_t depot_val = gene.depot_after ? 1u : 0u;

        uint32_t city = 0;
        city |= (x_val << 0);                                              // Interval [COORD_BITS-1 : 0]
        city |= (y_val << COORD_BITS);                                     // Interval [2*COORD_BITS-1 : COORD_BITS]
        city |= (cap_val << (2 * COORD_BITS));                             // Interval [2*COORD_BITS+CAPACITY_BITS-1 : 2*COORD_BITS]
        city |= (depot_val << (2 * COORD_BITS + CAPACITY_BITS));           // Interval [2*COORD_BITS+CAPACITY_BITS]
        city |= (id_val << (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS)); // Interval [2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS + NODE_ID_BITS - 1 : 2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS]

        fields.push_back(city);
    }

    const int beats_per_individual = get_total_beats(static_cast<int>(problem.nodes.size()));
    while (fields.size() < beats_per_individual * FIELDS_PER_BEAT) {
        fields.push_back(0);
    }

    // Write fields LSB-first directly into 512-bit beat buffer
    std::vector<std::array<uint64_t, 8>> beats(beats_per_individual);
    for (size_t beat_idx = 0; beat_idx < beats_per_individual; ++beat_idx) {
        std::array<uint64_t, 8> &beat = beats[beat_idx];
        beat.fill(0);

        for (size_t slot = 0; slot < FIELDS_PER_BEAT; ++slot) {
            size_t field_idx = beat_idx * FIELDS_PER_BEAT + slot;
            uint32_t value = fields[field_idx];
            size_t bit_offset = slot * FIELD_BITS;

            for (int bit = 0; bit < FIELD_BITS; ++bit) {
                if ((value >> bit) & 1u) {
                    size_t abs_bit = bit_offset + bit;
                    beat[abs_bit / 64] |= (1ULL << (abs_bit % 64));
                }
            }
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
    std::vector<std::uint32_t> fields;
    fields.reserve(beats_per_individual * FIELDS_PER_BEAT);

    for (std::size_t beat_idx = 0; beat_idx < beats_per_individual; ++beat_idx) {
        const std::uint64_t *words = reinterpret_cast<const std::uint64_t *>(
            beat_bytes + (beat_idx * AXI_BYTES)
        );

        for (std::size_t slot = 0; slot < FIELDS_PER_BEAT; ++slot) {
            std::size_t bit_offset = slot * FIELD_BITS;
            std::uint32_t value = 0;

            for (int bit = 0; bit < FIELD_BITS; ++bit) {
                std::size_t abs_bit = bit_offset + bit;
                if ((words[abs_bit / 64] >> (abs_bit % 64)) & 1ULL) {
                    value |= (1u << bit);
                }
            }
            fields.push_back(value);
        }
    }

    individual.fitness = fields[0] & ((1ul << FIELD_BITS) - 1ul);

    const std::uint32_t coord_mask    = (1u << COORD_BITS) - 1u;
    const std::uint32_t capacity_mask = (1u << CAPACITY_BITS) - 1u;
    const std::uint32_t id_mask       = (1u << NODE_ID_BITS) - 1u;
    const std::size_t num_cities       = problem.nodes.size();

    individual.tour.reserve(num_cities);
    for (std::size_t i = 1; i <= num_cities; ++i) {
        std::uint32_t city = fields[i];

        // Extract using exact lower bounds
        std::uint32_t x      = (city >> 0) & coord_mask;
        std::uint32_t y      = (city >> COORD_BITS) & coord_mask;
        std::uint32_t demand = (city >> (2 * COORD_BITS)) & capacity_mask;
        bool depot_after     = ((city >> (2 * COORD_BITS + CAPACITY_BITS)) & 1u) != 0u;
        std::uint32_t node_id_enc = (city >> (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS)) & id_mask;

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
    // return a the population of ecoded individuals, each with a pointer to its bytes, read the fitness from the offset, see constants. make it as efficent as possible
    const std::size_t bytes_per_individual = beats_per_individual * AXI_BYTES;
    for (std::size_t i = 0; i < num_individuals; ++i) {
        const std::uint8_t *ind_bytes = buffer + (i * bytes_per_individual);
        std::uint32_t fitness;
        std::memcpy(&fitness, ind_bytes, sizeof(fitness));
        fitness = fitness & ((1ul << FIELD_BITS) - 1u);
        population[i].fitness=fitness;
        population[i].bytes=ind_bytes;
    }
}

void population_to_buffer(std::vector<EncodedIndividual> &population, std::uint8_t* bytes, int problem_dimension, bool reversedOrder) {
    const int beats_per_individual = get_total_beats(problem_dimension - 1);
    if (reversedOrder) {
        for (int i=population.size()-1; i>=0; --i) {
            // Serialize each individual and write to bytes
            std::uint8_t *dest = bytes + (i * beats_per_individual * AXI_BYTES);
            std::memcpy(dest, population[i].bytes, beats_per_individual * AXI_BYTES);
            population[i] = EncodedIndividual{dest, population[i].fitness};
        }
    } else {
        for (int i=0; i<population.size(); ++i) {
            // Serialize each individual and write to bytes
            std::memcpy(bytes, population[i].bytes, beats_per_individual * AXI_BYTES);
            population[i] = EncodedIndividual{bytes, population[i].fitness};
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