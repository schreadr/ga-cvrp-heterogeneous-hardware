#include "ga_streaming.hpp"

#include <array>

#include <ga.hpp>


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
    // Build vector of field values (each fits in FIELD_BITS)
    std::vector<uint32_t> fields;
    fields.reserve(TOTAL_FIELDS);
    fields.push_back(individual.fitness & ((1 << FIELD_BITS) - 1)); // fitness

    for (const auto &gene : individual.tour) {
        const ProblemNode &node = problem.nodes.at(gene.node_id);
        uint32_t city = 0;
        city |= (uint32_t)(node.x & ((1 << COORD_BITS) - 1)) << 0;
        city |= (uint32_t)(node.y & ((1 << COORD_BITS) - 1)) << COORD_BITS;
        city |= (uint32_t)(node.demand & ((1 << CAPACITY_BITS) - 1)) << (2 * COORD_BITS);
        city |= (gene.depot_after ? 1u : 0u) << (2 * COORD_BITS + CAPACITY_BITS);
        fields.push_back(city);
    }

    // Pad with zeros to fill the last beat completely
    while (fields.size() < BEATS_PER_INDIVIDUAL * FIELDS_PER_BEAT) {
        fields.push_back(0);
    }

    // Pack fields into beats, each beat holding FIELDS_PER_BEAT fields
    std::vector<std::array<uint64_t, 8>> beats(BEATS_PER_INDIVIDUAL);
    for (size_t beat_idx = 0; beat_idx < BEATS_PER_INDIVIDUAL; ++beat_idx) {
        std::array<uint64_t, 8> &beat = beats[beat_idx];
        beat.fill(0);

        // For each field in this beat, place it at its bit offset
        for (size_t slot = 0; slot < FIELDS_PER_BEAT; ++slot) {
            size_t field_idx = beat_idx * FIELDS_PER_BEAT + slot;
            uint32_t value = fields[field_idx];

            // Bit offset within the beat: slot * FIELD_BITS
            size_t bit_offset = slot * FIELD_BITS;

            // Write the FIELD_BITS bits of `value` into the beat
            for (int bit = 0; bit < FIELD_BITS; ++bit) {
                if ((value >> bit) & 1) {
                    size_t abs_bit = bit_offset + bit;
                    size_t word_idx = abs_bit / 64;
                    size_t bit_in_word = abs_bit % 64;
                    beat[word_idx] |= (1ULL << bit_in_word);
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

    // 1. Unpack FIELD_BITS (21 bits) out of each 512-bit (64-byte) beat
    for (std::size_t beat_idx = 0; beat_idx < BEATS_PER_INDIVIDUAL; ++beat_idx) {
        const std::uint64_t *words = reinterpret_cast<const std::uint64_t *>(
            beat_bytes + (beat_idx * AXI_BYTES)
        );

        for (std::size_t slot = 0; slot < FIELDS_PER_BEAT; ++slot) {
            std::size_t bit_offset = slot * FIELD_BITS;
            std::uint32_t value = 0;

            for (int bit = 0; bit < FIELD_BITS; ++bit) {
                std::size_t abs_bit = bit_offset + bit;
                std::size_t word_idx = abs_bit / 64;
                std::size_t bit_in_word = abs_bit % 64;

                if ((words[word_idx] >> bit_in_word) & 1ULL) {
                    value |= (1u << bit);
                }
            }
            fields.push_back(value);
        }
    }

    // 2. Decode scalar fitness from field 0
    individual.fitness = fields[0] & ((1u << FIELD_BITS) - 1u);

    // 3. Unpack tour genes for each city (fields 1 to NUM_CITIES)
    individual.tour.reserve(NUM_CITIES);
    const std::uint32_t coord_mask = (1u << COORD_BITS) - 1u;
    const std::uint32_t capacity_mask = (1u << CAPACITY_BITS) - 1u;

    for (std::size_t i = 1; i <= NUM_CITIES; ++i) {
        std::uint32_t city = fields[i];

        // Extract bitfields
        std::uint32_t x = (city >> 0) & coord_mask;
        std::uint32_t y = (city >> COORD_BITS) & coord_mask;
        std::uint32_t demand = (city >> (2 * COORD_BITS)) & capacity_mask;
        bool depot_after = ((city >> (2 * COORD_BITS + CAPACITY_BITS)) & 1u) != 0u;

        // Match coordinates and demand back to ProblemNode ID
        int node_id = -1;
        for (int id = 1; id <= problem.dimension; ++id) {
            const auto &node = problem.nodes.at(static_cast<std::size_t>(id));
            if (node.x == static_cast<int>(x) && 
                node.y == static_cast<int>(y) && 
                node.demand == static_cast<int>(demand)) {
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