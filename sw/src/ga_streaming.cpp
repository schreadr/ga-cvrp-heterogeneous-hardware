#include <array>
#include <cstring>

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
    fields.reserve(TOTAL_FIELDS);

    // 1. Fitness field: LSB placed at bit 0 of field 0
    fields.push_back(individual.fitness & ((1u << FIELD_BITS) - 1u));

    // 2. City fields: each integer LSB-aligned to its starting offset L
    for (const auto &gene : individual.tour) {
        const ProblemNode &node = problem.nodes.at(gene.node_id);

        uint32_t x_val     = static_cast<uint32_t>(node.x) & ((1u << COORD_BITS) - 1u);
        uint32_t y_val     = static_cast<uint32_t>(node.y) & ((1u << COORD_BITS) - 1u);
        uint32_t cap_val   = static_cast<uint32_t>(node.demand) & ((1u << CAPACITY_BITS) - 1u);
        uint32_t depot_val = gene.depot_after ? 1u : 0u;

        uint32_t city = 0;
        city |= (x_val << 0);                                       // Interval [COORD_BITS-1 : 0]
        city |= (y_val << COORD_BITS);                              // Interval [2*COORD_BITS-1 : COORD_BITS]
        city |= (cap_val << (2 * COORD_BITS));                      // Interval [2*COORD_BITS+CAPACITY_BITS-1 : 2*COORD_BITS]
        city |= (depot_val << (2 * COORD_BITS + CAPACITY_BITS));    // Interval [2*COORD_BITS+CAPACITY_BITS]

        fields.push_back(city);
    }

    while (fields.size() < BEATS_PER_INDIVIDUAL * FIELDS_PER_BEAT) {
        fields.push_back(0);
    }

    // Write fields LSB-first directly into 512-bit beat buffer
    std::vector<std::array<uint64_t, 8>> beats(BEATS_PER_INDIVIDUAL);
    for (size_t beat_idx = 0; beat_idx < BEATS_PER_INDIVIDUAL; ++beat_idx) {
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
    std::vector<uint8_t> buffer;
    buffer.reserve(population.size() * BEATS_PER_INDIVIDUAL * AXI_BYTES);
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
    Individual individual;
    std::vector<std::uint32_t> fields;
    fields.reserve(BEATS_PER_INDIVIDUAL * FIELDS_PER_BEAT);

    for (std::size_t beat_idx = 0; beat_idx < BEATS_PER_INDIVIDUAL; ++beat_idx) {
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

    individual.fitness = fields[0] & ((1u << FIELD_BITS) - 1u);

    const std::uint32_t coord_mask    = (1u << COORD_BITS) - 1u;
    const std::uint32_t capacity_mask = (1u << CAPACITY_BITS) - 1u;
    const std::size_t num_cities       = static_cast<std::size_t>(problem.dimension - 1);

    individual.tour.reserve(num_cities);
    for (std::size_t i = 1; i <= num_cities; ++i) {
        std::uint32_t city = fields[i];

        // Extract using exact lower bounds
        std::uint32_t x      = (city >> 0) & coord_mask;
        std::uint32_t y      = (city >> COORD_BITS) & coord_mask;
        std::uint32_t demand = (city >> (2 * COORD_BITS)) & capacity_mask;
        bool depot_after     = ((city >> (2 * COORD_BITS + CAPACITY_BITS)) & 1u) != 0u;

        int node_id = -1;
        for (int id = 1; id <= problem.dimension; ++id) {
            const auto &node = problem.nodes.at(static_cast<std::size_t>(id));
            // unique points assumed
            if (node.x == static_cast<int>(x) && 
                node.y == static_cast<int>(y)) {
                node_id = id;
                break;
            }
        }

        if (node_id == -1) {
            throw std::runtime_error("Failed to decode gene: no matching node found in ProblemInstance.");
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
    std::vector<Individual> population;
    population.reserve(num_individuals);

    const std::size_t bytes_per_individual = BEATS_PER_INDIVIDUAL * AXI_BYTES;

    for (std::size_t i = 0; i < num_individuals; ++i) {
        const std::uint8_t *ind_bytes = buffer + (i * bytes_per_individual);
        population.push_back(decode_individual_fixed(problem, ind_bytes));
    }

    return population;
}

void population_from_buffer(
    const std::uint8_t *buffer,
    std::size_t num_individuals,
    std::vector<EncodedIndividual> &population
) {
    // return a the population of ecoded individuals, each with a pointer to its bytes, read the fitness from the offset, see constants. make it as efficent as possible
    const std::size_t bytes_per_individual = BEATS_PER_INDIVIDUAL * AXI_BYTES;
    for (std::size_t i = 0; i < num_individuals; ++i) {
        const std::uint8_t *ind_bytes = buffer + (i * bytes_per_individual);
        std::uint32_t fitness;
        std::memcpy(&fitness, ind_bytes, sizeof(fitness));
        fitness = fitness & ((1u << FIELD_BITS) - 1u);
        population[i].fitness=fitness;
        population[i].bytes=ind_bytes;
    }
}

void population_to_buffer(std::vector<EncodedIndividual> &population, std::uint8_t* bytes, bool reversedOrder) {
    if (reversedOrder) {
        for (int i=population.size()-1; i>=0; --i) {
            // Serialize each individual and write to bytes
            std::memcpy(bytes, population[i].bytes, BEATS_PER_INDIVIDUAL * AXI_BYTES);
            population[i] = EncodedIndividual{bytes, population[i].fitness};
            bytes += BEATS_PER_INDIVIDUAL * AXI_BYTES;
        }
    } else {
        for (int i=0; i<population.size(); ++i) {
            // Serialize each individual and write to bytes
            std::memcpy(bytes, population[i].bytes, BEATS_PER_INDIVIDUAL * AXI_BYTES);
            population[i] = EncodedIndividual{bytes, population[i].fitness};
            bytes += BEATS_PER_INDIVIDUAL * AXI_BYTES;
        }
    }
}