#ifndef GA_ENCODING_CONSTANTS_HPP
#define GA_ENCODING_CONSTANTS_HPP

#include <cassert>
#include <cmath>
#include <cstddef>

#include "static_block.hpp"

#define COORD_BITS       16
#define CAPACITY_BITS    16
#define MAX_CITIES       1023

// Encoding constants
#define NODE_ID_BITS     10 // ceil(log2(MAX_CITIES))
#define DEPOT_BITS        1
#define FIELD_BITS       64 // next power of two of (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS + NODE_ID_BITS)
#define FIELD_BITS_LOG2   6 // log2(FIELD_BITS)

#define AXI_WIDTH        512
#define AXI_BYTES         64 // AXI_WIDTH / 8
#define FIELDS_PER_BEAT   8 // AXI_WIDTH / FIELD_BITS
#define FIELDS_PER_BEAT_LOG2  3 // log2(FIELDS_PER_BEAT)
#define MAX_FIELDS       1024 // MAX_CITIES + 1
#define MAX_FIELDS_LOG2  10 // log2(MAX_FIELDS)
#define MAX_BEATS        128 // MAX_FIELDS + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT
#define MAX_BEATS_LOG2   7 // log2(MAX_BEATS)
#define BEAT_WIDTH       512 // FIELDS_PER_BEAT * FIELD_BITS


// Random number stream constants
#define RANDOM_BITS              32
#define RANDOM_BITS_LOG2          5 // log2(RANDOM_BITS)
#define RANDOMS_PER_INDIV         4 // 2 cut points for crossover, 2 cut points for mutation
#define RANDOMS_PER_BEAT         16 // AXI_WIDTH / RANDOM_BITS
#define RANDOMS_PER_BEAT_LOG2     4 // log2(RANDOMS_PER_BEAT)
#define INDIVS_PER_RANDOM_BEAT    4 // RANDOMS_PER_BEAT / RANDOMS_PER_INDIV
#define INDIVS_PER_RANDOM_BEAT_LOG2 2 // log2(INDIVS_PER_RANDOM_BEAT)

static void assert_power_of_two(int x) {
    assert(x && !(x & (x - 1)));
}

static_block {
    assert_power_of_two(FIELD_BITS);
    assert_power_of_two(FIELDS_PER_BEAT);
    assert_power_of_two(AXI_WIDTH);
    assert_power_of_two(AXI_BYTES);

    assert(NODE_ID_BITS == std::ceil(std::log2(MAX_CITIES)));
    assert(FIELD_BITS >= 2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS + NODE_ID_BITS);

    assert(AXI_BYTES * 8 == AXI_WIDTH);
    assert(FIELDS_PER_BEAT * FIELD_BITS == AXI_WIDTH);

    assert((1ULL << FIELD_BITS_LOG2) == FIELD_BITS);
    assert((1ULL << FIELDS_PER_BEAT_LOG2) == FIELDS_PER_BEAT);

    assert(MAX_FIELDS == MAX_CITIES + 1);
    assert(MAX_FIELDS_LOG2 == std::ceil(std::log2(MAX_FIELDS)));

    assert(MAX_BEATS == (MAX_FIELDS + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT);
    assert(BEAT_WIDTH == FIELDS_PER_BEAT * FIELD_BITS);

    assert_power_of_two(RANDOM_BITS);
    assert_power_of_two(RANDOMS_PER_INDIV);
    assert_power_of_two(RANDOMS_PER_BEAT);
    assert_power_of_two(INDIVS_PER_RANDOM_BEAT);

    assert(RANDOMS_PER_BEAT * RANDOM_BITS == AXI_WIDTH);
    assert((1ULL << RANDOM_BITS_LOG2) == RANDOM_BITS);
    assert((1ULL << RANDOMS_PER_BEAT_LOG2) == RANDOMS_PER_BEAT);
    assert(INDIVS_PER_RANDOM_BEAT * RANDOMS_PER_INDIV == RANDOMS_PER_BEAT);
    assert((1ULL << INDIVS_PER_RANDOM_BEAT_LOG2) == INDIVS_PER_RANDOM_BEAT);
}

// inlined for hw side
static inline int get_total_beats(int num_cities) {
    int total_fields = 1 + num_cities;
    return (total_fields + FIELDS_PER_BEAT - 1) >> FIELDS_PER_BEAT_LOG2; // ... / FIELDS_PER_BEAT
}

static inline int get_total_random_beats(int num_individuals) {
    return (num_individuals * RANDOMS_PER_INDIV + RANDOMS_PER_BEAT - 1) >> RANDOMS_PER_BEAT_LOG2;
}

static inline std::size_t get_random_bytes(int num_individuals) {
    return static_cast<std::size_t>(get_total_random_beats(num_individuals)) * AXI_BYTES;
}

#endif