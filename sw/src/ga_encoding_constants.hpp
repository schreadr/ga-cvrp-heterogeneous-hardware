#ifndef GA_ENCODING_CONSTANTS_HPP
#define GA_ENCODING_CONSTANTS_HPP

#include <cassert>
#include <cmath>
#include <cstddef>

#include "static_block.hpp"

#define COORD_BITS       16
#define CAPACITY_BITS    16
#define MAX_CITIES       1022

// Encoding constants
#define NODE_ID_BITS     10 // ceil(log2(MAX_CITIES))
#define DEPOT_BITS        1
#define DEPOT_FLAG_FIELD_OFFSET (NODE_ID_BITS)
#define FIELD_BITS       16 // next power of two of (DEPOT_BITS + NODE_ID_BITS)
#define FIELD_BITS_LOG2   4 // log2(FIELD_BITS)

#define FITNESS_FIELDS    2 // Number of FIELD_BITS fields to store fitness (2 * 16 = 32 bits)

#define AXI_WIDTH        512
#define AXI_BYTES         64 // AXI_WIDTH / 8
#define FIELDS_PER_BEAT   32 // AXI_WIDTH / FIELD_BITS
#define FIELDS_PER_BEAT_LOG2  5 // log2(FIELDS_PER_BEAT)
#define MAX_FIELDS       (MAX_CITIES + FITNESS_FIELDS) // 1022 + 2 = 1024
#define MAX_FIELDS_LOG2  10 // ceil(log2(MAX_FIELDS))
#define MAX_BEATS        ((MAX_FIELDS + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT) // (1024 + 31) / 32 = 32
#define MAX_BEATS_LOG2   5 // ceil(log2(MAX_BEATS))
#define BEAT_WIDTH       512 // FIELDS_PER_BEAT * FIELD_BITS

// Node cache encoding constants (for streaming static coordinates and demands)
#define NODE_CACHE_ENTRY_BITS        64
#define NODE_CACHE_ENTRY_BITS_LOG2    6
#define NODE_CACHE_COORD_X_OFFSET     0
#define NODE_CACHE_COORD_Y_OFFSET    (COORD_BITS)
#define NODE_CACHE_DEMAND_OFFSET     (2 * COORD_BITS)

#define NODE_CACHE_ENTRIES_PER_BEAT      (AXI_WIDTH / NODE_CACHE_ENTRY_BITS) // 8
#define NODE_CACHE_ENTRIES_PER_BEAT_LOG2 3
#define CITIES_PER_CACHE_BEAT            NODE_CACHE_ENTRIES_PER_BEAT

#define MAX_CACHE_ENTRIES            (MAX_CITIES + 1) // MAX_CITIES customers + 1 depot
#define MAX_CACHE_BEATS              ((MAX_CACHE_ENTRIES + NODE_CACHE_ENTRIES_PER_BEAT - 1) / NODE_CACHE_ENTRIES_PER_BEAT)


// Random number stream constants
#define RANDOM_BITS              32
#define RANDOM_BITS_LOG2          5 // log2(RANDOM_BITS)
#define RANDOMS_PER_INDIV         6 // 2 cut points for crossover, 2 for stream 0 mutate, 2 for stream 1 mutate
#define RANDOMS_PER_BEAT         16 // AXI_WIDTH / RANDOM_BITS
#define RANDOMS_PER_BEAT_LOG2     4 // log2(RANDOMS_PER_BEAT)

static void assert_power_of_two(int x) {
    assert(x && !(x & (x - 1)));
}

static_block {
    assert_power_of_two(FIELD_BITS);
    assert_power_of_two(FIELDS_PER_BEAT);
    assert_power_of_two(AXI_WIDTH);
    assert_power_of_two(AXI_BYTES);

    assert(NODE_ID_BITS == std::ceil(std::log2(MAX_CITIES)));
    assert(FIELD_BITS >= DEPOT_BITS + NODE_ID_BITS);

    assert(AXI_BYTES * 8 == AXI_WIDTH);
    assert(FIELDS_PER_BEAT * FIELD_BITS == AXI_WIDTH);

    assert((1ULL << FIELD_BITS_LOG2) == FIELD_BITS);
    assert((1ULL << FIELDS_PER_BEAT_LOG2) == FIELDS_PER_BEAT);

    assert(FITNESS_FIELDS * FIELD_BITS >= 32);
    assert(MAX_FIELDS == MAX_CITIES + FITNESS_FIELDS);
    assert(MAX_FIELDS_LOG2 == std::ceil(std::log2(MAX_FIELDS)));

    assert(MAX_BEATS == (MAX_FIELDS + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT);
    assert(BEAT_WIDTH == FIELDS_PER_BEAT * FIELD_BITS);

    assert_power_of_two(RANDOM_BITS);
    assert_power_of_two(RANDOMS_PER_BEAT);

    assert(RANDOMS_PER_BEAT * RANDOM_BITS == AXI_WIDTH);
    assert((1ULL << RANDOM_BITS_LOG2) == RANDOM_BITS);
    assert((1ULL << RANDOMS_PER_BEAT_LOG2) == RANDOMS_PER_BEAT);

    assert_power_of_two(NODE_CACHE_ENTRY_BITS);
    assert_power_of_two(NODE_CACHE_ENTRIES_PER_BEAT);
    assert(NODE_CACHE_ENTRY_BITS >= 2 * COORD_BITS + CAPACITY_BITS);
    assert(NODE_CACHE_ENTRIES_PER_BEAT * NODE_CACHE_ENTRY_BITS == AXI_WIDTH);
    assert((1ULL << NODE_CACHE_ENTRY_BITS_LOG2) == NODE_CACHE_ENTRY_BITS);
    assert((1ULL << NODE_CACHE_ENTRIES_PER_BEAT_LOG2) == NODE_CACHE_ENTRIES_PER_BEAT);
    assert(NODE_CACHE_COORD_X_OFFSET == 0);
    assert(NODE_CACHE_COORD_Y_OFFSET == COORD_BITS);
    assert(NODE_CACHE_DEMAND_OFFSET == 2 * COORD_BITS);
    assert(MAX_CACHE_ENTRIES == MAX_CITIES + 1);
    assert(MAX_CACHE_BEATS == (MAX_CACHE_ENTRIES + NODE_CACHE_ENTRIES_PER_BEAT - 1) / NODE_CACHE_ENTRIES_PER_BEAT);
}

// inlined for hw side
static inline int get_total_beats(int num_cities) {
    int total_fields = num_cities + FITNESS_FIELDS;
    return (total_fields + FIELDS_PER_BEAT - 1) >> FIELDS_PER_BEAT_LOG2; // ... / FIELDS_PER_BEAT
}

static inline int get_total_city_beats(int num_cities) {
    return (num_cities + FIELDS_PER_BEAT - 1) >> FIELDS_PER_BEAT_LOG2;
}

static inline int get_total_cache_beats(int num_cities) {
    int total_entries = num_cities + 1; // customers + depot
    return (total_entries + NODE_CACHE_ENTRIES_PER_BEAT - 1) >> NODE_CACHE_ENTRIES_PER_BEAT_LOG2;
}

static inline int get_total_random_beats(int num_individuals) {
    return (num_individuals * RANDOMS_PER_INDIV + RANDOMS_PER_BEAT - 1) >> RANDOMS_PER_BEAT_LOG2;
}

static inline std::size_t get_random_bytes(int num_individuals) {
    return static_cast<std::size_t>(get_total_random_beats(num_individuals)) * AXI_BYTES;
}

#endif