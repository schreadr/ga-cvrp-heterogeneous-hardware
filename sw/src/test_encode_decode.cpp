/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2021-2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Standalone encode/decode roundtrip test for the CVRP population codec.
 *
 * Reads the 16-node P-n16-k8 problem instance, generates a small population
 * of two individuals, encodes the population into the 512-bit beat layout,
 * prints the bytes with field-boundary markers, decodes the buffer again
 * and checks that the decoded individuals match the originals.
 *
 *
 * Build (standalone, no Coyote dependency):
 *   g++ -std=c++17 -I . test_encode_decode.cpp \
       ga.cpp ga_streaming.cpp problem_parser.cpp -o test_encode_decode & & ./test_encode_decode
 */

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include "ga.hpp"
#include "ga_streaming.hpp"
#include "problem_parser.hpp"

namespace {

constexpr const char kDefaultProblemFile[] = "../../P-n40-k5_edited.vrp";

void print_hex_dump(const std::string &label, const std::uint8_t *bytes, std::size_t size) {
    std::cout << label << " (" << size << " B):\n";
    for (std::size_t i = 0; i < size; i += 16) {
        std::printf("  %04zx: ", i);
        for (std::size_t j = 0; j < 16; ++j) {
            if (i + j < size) {
                std::printf("%02x ", bytes[i + j]);
            } else {
                std::printf("   ");
            }
        }
        std::printf("\n");
    }
}

void print_beat_markers(const std::string &label, const std::uint8_t *beat_bytes) {
    std::cout << label << " bit layout (Memory Stream Order LSB->MSB, '|' field, '-' sub-field):\n  ";

    for (int abs_bit = 0; abs_bit < AXI_WIDTH; ++abs_bit) {
        // Sample bits LSB-first within each byte
        int byte_idx = abs_bit / 8;
        int bit_in_byte = abs_bit % 8;

        bool set = (beat_bytes[byte_idx] >> bit_in_byte) & 1;
        std::printf("%c", set ? '1' : '0');

        int field_idx = abs_bit / FIELD_BITS;
        int bit_in_field = abs_bit % FIELD_BITS;

        // Print dividers (except after the very last bit of the AXI beat)
        if (abs_bit + 1 < AXI_WIDTH) {
            // End of field boundary (|)
            if (bit_in_field == FIELD_BITS - 1) {
                std::printf("|");
            } 
            // Sub-field boundaries inside city fields (-)
            else if (field_idx > 0) {
                if (bit_in_field == COORD_BITS - 1 || 
                    bit_in_field == (2 * COORD_BITS) - 1 || 
                    bit_in_field == (2 * COORD_BITS + CAPACITY_BITS) - 1 ||
                    bit_in_field == (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS) - 1 ||
                    bit_in_field == (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS + NODE_ID_BITS) - 1) {
                    std::printf("-");
                }
            }

            // New line formatting at field boundaries for readability
            if (bit_in_field == FIELD_BITS - 1 && (field_idx + 1) % (64/FIELD_BITS + 1) == 0) {
                std::printf("\n  ");
            }
        }
    }
    std::printf("\n");
}

void print_decoded_fields(const std::string &label, Individual& individual, const ProblemInstance &problem) {
    std::cout << label << " decoded fields:\n";

    std::printf("  field  0 [bits %3d..%3d]  fitness=%u\n",
                0, FIELD_BITS - 1, individual.fitness);

    for(std::size_t  i = 0; i < individual.tour.size(); ++i) {
        const auto &city = individual.tour[i];
        std::printf("  field %2zu [bits %3zu..%3zu] x=%3u y=%3u demand=%2u depot_after=%u node_id=%d\n",
                    i+1, (i+1) * FIELD_BITS, (i + 2) * FIELD_BITS - 1, problem.nodes[city.node_id].x, problem.nodes[city.node_id].y, problem.nodes[city.node_id].demand, city.depot_after, city.node_id);
    }
}

bool test_encode_decode(const ProblemInstance &problem) {
    const std::size_t population_size = 2;
    const std::vector<Individual> population = build_initial_population(problem, population_size, 0xC0FFEEu);

    for (std::size_t i = 0; i < population.size(); ++i) {
        std::cout << "Individual " << i << ": " << population[i] << "\n";
    }
    std::cout << "\n";

    const std::vector<std::uint8_t> encoded = encode_population_fixed(problem, population);
    std::vector<Individual> decoded = decode_population_fixed(problem, encoded.data(), population.size());
    const std::size_t bytes_per_individual = get_total_beats(problem.dimension - 1) * AXI_BYTES;

    for (std::size_t i = 0; i < population.size(); ++i) {
        const std::uint8_t *bytes = encoded.data() + i * bytes_per_individual;
        print_hex_dump("Individual " + std::to_string(i), bytes, bytes_per_individual);
        print_beat_markers("Individual " + std::to_string(i), bytes);
        print_decoded_fields("Individual " + std::to_string(i), decoded[i], problem);
        std::cout << "\n";
    }

    bool ok = decoded.size() == population.size();
    for (std::size_t i = 0; ok && i < population.size(); ++i) {
        if (decoded[i] != population[i]) {
            std::cerr << "Mismatch at individual " << i << ":\n"
                      << "  original: " << population[i] << "\n"
                      << "  decoded:  " << decoded[i] << "\n";
            ok = false;
        }
    }

    if (ok) {
        std::cout << "PASS: encode/decode round-trip matches for " << population.size() << " individuals.\n";
        return true;
    } else {
        std::cout << "FAIL: round-trip mismatch.\n";
        return false;
    }
}

bool test_generate_random_numbers(const ProblemInstance &problem) {
    const std::size_t test_rnd_indivs = 5;
    const std::size_t rnd_bytes = get_random_bytes(test_rnd_indivs);
    std::vector<std::uint8_t> rnd_buf1(rnd_bytes, 0xFF);
    std::vector<std::uint8_t> rnd_buf2(rnd_bytes, 0xAA);

    const int num_cities = static_cast<int>(problem.nodes.size());
    const std::uint64_t seed = 0x12345678ULL;
    generate_random_numbers(test_rnd_indivs, rnd_buf1.data(), num_cities, seed);
    generate_random_numbers(test_rnd_indivs, rnd_buf2.data(), num_cities, seed);

    // Verify repeatability
    if (rnd_buf1 != rnd_buf2) {
        std::cerr << "FAIL: random number generators with same seed produced different outputs.\n";
        return false;
    }

    // Verify values within bounds and padding is zero
    const uint32_t *words = reinterpret_cast<const uint32_t *>(rnd_buf1.data());
    const std::size_t total_random_words = test_rnd_indivs * RANDOMS_PER_INDIV;
    for (std::size_t i = 0; i < total_random_words; ++i) {
        if (words[i] < 1 || words[i] > static_cast<uint32_t>(num_cities)) {
            std::cerr << "FAIL: random number out of bounds at index " << i << ": " << words[i] << "\n";
            return false;
        }
    }
    for (std::size_t b = total_random_words * sizeof(uint32_t); b < rnd_bytes; ++b) {
        if (rnd_buf1[b] != 0) {
            std::cerr << "FAIL: non-zero padding byte at index " << b << "\n";
            return false;
        }
    }
    std::cout << "PASS: generate_random_numbers verified for " << test_rnd_indivs << " individuals.\n";
    return true;
}

} // namespace

int main(int argc, char *argv[]) {
    const std::string vrp_file = (argc > 1) ? argv[1] : kDefaultProblemFile;

    try {
        const ProblemInstance problem = parse_vrp(vrp_file);
        std::cout << "Loaded problem: " << problem.name
                  << "  dimension=" << problem.dimension
                  << "  capacity=" << problem.capacity
                  << "  customers=" << numCustomers(problem) << "\n\n";

        if (!test_encode_decode(problem)) {
            return EXIT_FAILURE;
        }

        if (!test_generate_random_numbers(problem)) {
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
