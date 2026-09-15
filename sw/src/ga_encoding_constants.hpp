#ifndef GA_ENCODING_CONSTANTS_HPP
#define GA_ENCODING_CONSTANTS_HPP

#include <cassert>
#include <cmath>
#include <cstddef>

#include "static_block.hpp"


// Problem & Coordinate Constants
#define COORD_BITS                  16
#define CAPACITY_BITS               16
#define MAX_CITIES                  1023
#define MAX_CITIES_LOG2             10


// Fitness Constants
#define FITNESS_BITS                32


// City Gene Encoding Constants
#define NODE_ID_BITS                10 // ceil(log2(MAX_CITIES))
#define DEPOT_BITS                   1
#define DEPOT_FLAG_OFFSET           (NODE_ID_BITS)
#define CITY_BITS                   16 // next power of two of (DEPOT_BITS + NODE_ID_BITS)
#define CITY_BITS_LOG2               4 // log2(CITY_BITS)


// Host / AXI Interface Beat Constants
#define AXI_BEAT_WIDTH              512
#define AXI_WIDTH                   AXI_BEAT_WIDTH // alias for Coyote compatibility
#define AXI_BYTES                   (AXI_BEAT_WIDTH / 8) // 64
#define AXI_CITIES_PER_BEAT         (AXI_BEAT_WIDTH / CITY_BITS) // 32
#define AXI_CITIES_PER_BEAT_LOG2     5 // log2(AXI_CITIES_PER_BEAT)

#define AXI_MAX_CITY_BEATS          ((MAX_CITIES + AXI_CITIES_PER_BEAT - 1) / AXI_CITIES_PER_BEAT) // 32
#define AXI_FITNESS_BEATS            1
#define AXI_MAX_BEATS               (AXI_MAX_CITY_BEATS + AXI_FITNESS_BEATS) // 33
#define AXI_MAX_BEATS_LOG2           6 // ceil(log2(AXI_MAX_BEATS))

// Hardware Pipeline Beat Constants
#define HW_CITIES_PER_BEAT          2
#define HW_CITIES_PER_BEAT_LOG2     1 // log2(HW_CITIES_PER_BEAT)
#define HW_BEAT_WIDTH               (CITY_BITS * HW_CITIES_PER_BEAT) // 32
#define HW_BEATS_PER_AXI_BEAT       (AXI_BEAT_WIDTH / HW_BEAT_WIDTH) // 16
#define HW_BEATS_PER_AXI_BEAT_LOG2  (AXI_CITIES_PER_BEAT_LOG2 - HW_CITIES_PER_BEAT_LOG2) // 4

#define HW_MAX_CITY_BEATS           ((MAX_CITIES + HW_CITIES_PER_BEAT - 1) / HW_CITIES_PER_BEAT) // 512
#define HW_FITNESS_BEATS            1
#define HW_MAX_BEATS                (HW_MAX_CITY_BEATS + HW_FITNESS_BEATS) // 513
#define HW_MAX_BEATS_LOG2           10 // ceil(log2(HW_MAX_BEATS))


// Node Cache Encoding Constants
#define NODE_CACHE_ENTRY_BITS        64
#define NODE_CACHE_ENTRY_BITS_LOG2    6
#define NODE_CACHE_COORD_X_OFFSET     0
#define NODE_CACHE_COORD_Y_OFFSET    (COORD_BITS)
#define NODE_CACHE_DEMAND_OFFSET     (2 * COORD_BITS)

#define NODE_CACHE_ENTRIES_PER_BEAT      (AXI_BEAT_WIDTH / NODE_CACHE_ENTRY_BITS) // 8
#define NODE_CACHE_ENTRIES_PER_BEAT_LOG2 3
#define CITIES_PER_CACHE_BEAT            NODE_CACHE_ENTRIES_PER_BEAT

#define MAX_CACHE_ENTRIES            (MAX_CITIES + 1) // MAX_CITIES customers + 1 depot
#define MAX_CACHE_BEATS              ((MAX_CACHE_ENTRIES + NODE_CACHE_ENTRIES_PER_BEAT - 1) / NODE_CACHE_ENTRIES_PER_BEAT)


// Random Number Stream Constants
#define RANDOM_BITS                 32
#define RANDOM_BITS_LOG2             5 // log2(RANDOM_BITS)
#define RANDOMS_PER_INDIV            6 // 2 cut points for crossover, 2 for mut 0, 2 for mut 1
#define RANDOMS_PER_BEAT            (AXI_BEAT_WIDTH / RANDOM_BITS) // 16
#define RANDOMS_PER_BEAT_LOG2        4 // log2(RANDOMS_PER_BEAT)

static void assert_power_of_two(int x) {
    assert(x && !(x & (x - 1)));
}

static_block {
    // Problem & City
    assert_power_of_two(CITY_BITS);
    assert((1ULL << CITY_BITS_LOG2) == CITY_BITS);
    assert(NODE_ID_BITS == std::ceil(std::log2(MAX_CITIES)));
    assert(CITY_BITS >= DEPOT_BITS + NODE_ID_BITS);

    // Host / AXI Beat
    assert_power_of_two(AXI_BEAT_WIDTH);
    assert_power_of_two(AXI_BYTES);
    assert_power_of_two(AXI_CITIES_PER_BEAT);
    assert((1ULL << AXI_CITIES_PER_BEAT_LOG2) == AXI_CITIES_PER_BEAT);
    assert(AXI_BYTES * 8 == AXI_BEAT_WIDTH);
    assert(AXI_CITIES_PER_BEAT * CITY_BITS == AXI_BEAT_WIDTH);
    assert(AXI_MAX_CITY_BEATS == (MAX_CITIES + AXI_CITIES_PER_BEAT - 1) / AXI_CITIES_PER_BEAT);
    assert(AXI_MAX_BEATS == AXI_MAX_CITY_BEATS + AXI_FITNESS_BEATS);
    assert(AXI_MAX_BEATS_LOG2 == std::ceil(std::log2(AXI_MAX_BEATS)));

    // Hardware Pipeline Beat Constraints
    assert_power_of_two(HW_CITIES_PER_BEAT);
    assert_power_of_two(HW_BEAT_WIDTH);
    assert_power_of_two(HW_BEATS_PER_AXI_BEAT);
    assert((1ULL << HW_CITIES_PER_BEAT_LOG2) == HW_CITIES_PER_BEAT);
    assert((1ULL << HW_BEATS_PER_AXI_BEAT_LOG2) == HW_BEATS_PER_AXI_BEAT);
    assert(HW_BEAT_WIDTH == CITY_BITS * HW_CITIES_PER_BEAT);
    assert(AXI_BEAT_WIDTH % HW_BEAT_WIDTH == 0);
    assert(HW_BEAT_WIDTH >= FITNESS_BITS);
    assert(HW_MAX_CITY_BEATS == (MAX_CITIES + HW_CITIES_PER_BEAT - 1) / HW_CITIES_PER_BEAT);
    assert(HW_MAX_BEATS == HW_MAX_CITY_BEATS + HW_FITNESS_BEATS);
    assert(HW_MAX_BEATS_LOG2 == std::ceil(std::log2(HW_MAX_BEATS)));

    // Node Cache
    assert_power_of_two(NODE_CACHE_ENTRY_BITS);
    assert_power_of_two(NODE_CACHE_ENTRIES_PER_BEAT);
    assert(NODE_CACHE_ENTRY_BITS >= 2 * COORD_BITS + CAPACITY_BITS);
    assert(NODE_CACHE_ENTRIES_PER_BEAT * NODE_CACHE_ENTRY_BITS == AXI_BEAT_WIDTH);
    assert((1ULL << NODE_CACHE_ENTRY_BITS_LOG2) == NODE_CACHE_ENTRY_BITS);
    assert((1ULL << NODE_CACHE_ENTRIES_PER_BEAT_LOG2) == NODE_CACHE_ENTRIES_PER_BEAT);
    assert(NODE_CACHE_COORD_X_OFFSET == 0);
    assert(NODE_CACHE_COORD_Y_OFFSET == COORD_BITS);
    assert(NODE_CACHE_DEMAND_OFFSET == 2 * COORD_BITS);
    assert(MAX_CACHE_ENTRIES == MAX_CITIES + 1);
    assert(MAX_CACHE_BEATS == (MAX_CACHE_ENTRIES + NODE_CACHE_ENTRIES_PER_BEAT - 1) / NODE_CACHE_ENTRIES_PER_BEAT);

    // Random Numbers
    assert_power_of_two(RANDOM_BITS);
    assert_power_of_two(RANDOMS_PER_BEAT);
    assert(RANDOMS_PER_BEAT * RANDOM_BITS == AXI_BEAT_WIDTH);
    assert((1ULL << RANDOM_BITS_LOG2) == RANDOM_BITS);
    assert((1ULL << RANDOMS_PER_BEAT_LOG2) == RANDOMS_PER_BEAT);
}

// Host / AXI stream beat calculation helpers
static inline int get_total_axi_city_beats(int num_cities) {
    return (num_cities + AXI_CITIES_PER_BEAT - 1) >> AXI_CITIES_PER_BEAT_LOG2;
}

static inline int get_total_axi_beats(int num_cities) {
    return get_total_axi_city_beats(num_cities) + AXI_FITNESS_BEATS;
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

#endif // GA_ENCODING_CONSTANTS_HPP