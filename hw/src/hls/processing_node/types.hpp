#ifndef TYPES_HPP
#define TYPES_HPP

#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <hls_stream.h>

#include "ga_encoding_constants.hpp"

#ifndef DEPOT_FLAG_FIELD_OFFSET
#define DEPOT_FLAG_FIELD_OFFSET (NODE_ID_BITS)
#endif

// ============================================================================
// Type Definitions & Structs
// ============================================================================

// ----------------------------------------------------------------------------
// 1. Bitfield Index & Counter Types
// ----------------------------------------------------------------------------
typedef ap_uint<MAX_FIELDS_LOG2 + 1>                  FieldIndex;
typedef ap_uint<MAX_BEATS_LOG2 + 1>                   BeatIndex;
typedef ap_uint<FIELDS_PER_BEAT_LOG2 + 1>             BeatFieldIndex;
typedef ap_uint<RANDOMS_PER_BEAT_LOG2 + 1>            BeatRandomIndex;
typedef ap_uint<NODE_CACHE_ENTRIES_PER_BEAT_LOG2 + 1>   BeatCacheIndex;

// ----------------------------------------------------------------------------
// 2. Problem & Node Data Types
// ----------------------------------------------------------------------------
typedef ap_uint<COORD_BITS>    Coord;
typedef ap_uint<CAPACITY_BITS> Capacity;
typedef ap_uint<NODE_ID_BITS>  NodeId;

struct Point {
    Coord x;
    Coord y;

    bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Point &other) const {
        return !(*this == other);
    }
};

struct CityData {
    Coord x;
    Coord y;
    Capacity demand;

    bool operator==(const CityData &other) const {
        return x == other.x && y == other.y && demand == other.demand;
    }
    bool operator!=(const CityData &other) const {
        return !(*this == other);
    }
};

struct NodePair {
    NodeId from;
    NodeId to;

    bool operator==(const NodePair &other) const {
        return from == other.from && to == other.to;
    }
    bool operator!=(const NodePair &other) const {
        return !(*this == other);
    }
};

struct NodePairElement {
    NodePair pair;
    bool last;

    bool operator==(const NodePairElement &other) const {
        return pair == other.pair && last == other.last;
    }
    bool operator!=(const NodePairElement &other) const {
        return !(*this == other);
    }
};

// ----------------------------------------------------------------------------
// 3. Individual Genome & Field Types
// ----------------------------------------------------------------------------
typedef ap_uint<FIELD_BITS>                   FitnessOrCity;
typedef ap_uint<FITNESS_FIELDS * FIELD_BITS>  Fitness;

struct CityPair {
    FieldIndex start;
    FieldIndex end;

    CityPair() : start(0), end(0) {}
    CityPair(FieldIndex s, FieldIndex e) : start(s), end(e) {}

    bool operator==(const CityPair &other) const {
        return start == other.start && end == other.end;
    }
};

// ----------------------------------------------------------------------------
// 4. Beat, Stream & Cache Types
// ----------------------------------------------------------------------------
typedef ap_uint<BEAT_WIDTH>             Beat;
typedef ap_axiu<AXI_WIDTH, 0, 0, 0>     axi_word_t;
typedef Beat                            IndividualAsBeats[MAX_BEATS];
typedef Beat                            CityCacheBeat;
typedef ap_uint<NODE_CACHE_ENTRY_BITS>  CityCacheEntry;
typedef ap_uint<RANDOM_BITS>            RandomNumber;

struct StartEndData {
    const BeatIndex start_beat;
    const FieldIndex start_field;
    const BeatIndex end_beat;
    const FieldIndex end_field;

    StartEndData(BeatIndex s_beat, FieldIndex s_field, BeatIndex e_beat, FieldIndex e_field)
        : start_beat(s_beat), start_field(s_field), end_beat(e_beat), end_field(e_field) {}
};

// ============================================================================
// Methods & Inline Helper Functions
// ============================================================================

// ----------------------------------------------------------------------------
// 1. Indexing & Beat Navigation Methods
// ----------------------------------------------------------------------------
static inline FieldIndex wrap_city_index(FieldIndex idx, FieldIndex num_cities) {
    #pragma HLS INLINE
    return (idx >= num_cities) ? FieldIndex(idx - num_cities) : idx;
}

static inline BeatIndex get_total_city_beats(FieldIndex num_cities) {
    #pragma HLS INLINE
    return (num_cities + FIELDS_PER_BEAT - 1) >> FIELDS_PER_BEAT_LOG2;
}

static inline void get_beat_field(FieldIndex global_idx, BeatIndex &beat_idx, FieldIndex &field_idx) {
    #pragma HLS INLINE
    beat_idx = global_idx >> FIELDS_PER_BEAT_LOG2;
    field_idx = global_idx & (FIELDS_PER_BEAT - 1);
}

static inline BeatIndex get_beat_idx(FieldIndex global_idx) {
    #pragma HLS INLINE
    return global_idx >> FIELDS_PER_BEAT_LOG2;
}

static inline StartEndData get_start_end_data(FieldIndex start_idx, FieldIndex end_idx) {
    #pragma HLS INLINE
    BeatIndex start_beat;
    FieldIndex start_field;
    BeatIndex end_beat;
    FieldIndex end_field;
    get_beat_field(start_idx, start_beat, start_field);
    get_beat_field(end_idx, end_beat, end_field);
    return StartEndData(start_beat, start_field, end_beat, end_field);
}

static inline FieldIndex get_first_city_index(BeatIndex beat_idx) {
    #pragma HLS INLINE
    return 0;
}

static inline FieldIndex get_last_field_index(FieldIndex num_cities, BeatIndex beat_idx) {
    #pragma HLS INLINE
    BeatIndex last_city_beat = (num_cities - 1) >> FIELDS_PER_BEAT_LOG2;
    return (beat_idx == last_city_beat) ? FieldIndex((num_cities - 1) & (FIELDS_PER_BEAT - 1)) : FieldIndex(FIELDS_PER_BEAT - 1);
}

// ----------------------------------------------------------------------------
// 2. Field-Level Accessors (Node ID & Depot Flag inside a FitnessOrCity)
// ----------------------------------------------------------------------------
static inline NodeId get_node_id(FitnessOrCity val) {
    #pragma HLS INLINE
    return val.range(NODE_ID_BITS - 1, 0);
}

static inline void set_node_id(FitnessOrCity &val, NodeId id) {
    #pragma HLS INLINE
    val.range(NODE_ID_BITS - 1, 0) = id;
}

static inline bool get_is_depot(FitnessOrCity val) {
    #pragma HLS INLINE
    return val[DEPOT_FLAG_FIELD_OFFSET];
}

static inline void set_is_depot(FitnessOrCity &val, bool depot) {
    #pragma HLS INLINE
    val[DEPOT_FLAG_FIELD_OFFSET] = depot;
}

// ----------------------------------------------------------------------------
// 3. Beat-Level & Individual Field Accessors
// ----------------------------------------------------------------------------
static inline FitnessOrCity get_field(const Beat &beat, FieldIndex field_idx) {
    #pragma HLS INLINE
    FieldIndex low = field_idx << FIELD_BITS_LOG2;
    FieldIndex high = low + FIELD_BITS - 1;
    return beat.range(high, low);
}

static inline void set_field(Beat &beat, FieldIndex field_idx, FitnessOrCity val) {
    #pragma HLS INLINE
    FieldIndex low = field_idx << FIELD_BITS_LOG2;
    FieldIndex high = low + FIELD_BITS - 1;
    beat.range(high, low) = val;
}

static inline FitnessOrCity get_field(const IndividualAsBeats &ind, FieldIndex global_idx) {
    #pragma HLS INLINE
    BeatIndex beat_idx;
    FieldIndex field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    return get_field(ind[beat_idx], field_idx);
}

static inline void set_field(IndividualAsBeats &ind, FieldIndex global_idx, FitnessOrCity val) {
    #pragma HLS INLINE
    BeatIndex beat_idx;
    FieldIndex field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    set_field(ind[beat_idx], field_idx, val);
}

static inline FitnessOrCity consume_next_field(Beat &beat) {
    #pragma HLS INLINE
    FitnessOrCity field = beat.range(FIELD_BITS - 1, 0);
    beat >>= FIELD_BITS;
    return field;
}

// ----------------------------------------------------------------------------
// 4. Multi-Field Fitness Accessors
// ----------------------------------------------------------------------------
static inline Fitness get_fitness(const IndividualAsBeats &ind, FieldIndex num_cities) {
    #pragma HLS INLINE
    Fitness fit = 0;
    GET_FITNESS_LOOP: for (int i = 0; i < FITNESS_FIELDS; ++i) {
        #pragma HLS UNROLL
        FitnessOrCity field = get_field(ind, num_cities + i);
        fit.range((i + 1) * FIELD_BITS - 1, i * FIELD_BITS) = field;
    }
    return fit;
}

static inline void set_fitness(IndividualAsBeats &ind, Fitness val, FieldIndex num_cities) {
    #pragma HLS INLINE
    SET_FITNESS_LOOP: for (int i = 0; i < FITNESS_FIELDS; ++i) {
        #pragma HLS UNROLL
        FitnessOrCity field = val.range((i + 1) * FIELD_BITS - 1, i * FIELD_BITS);
        set_field(ind, num_cities + i, field);
    }
}

// ----------------------------------------------------------------------------
// 5. Node Cache & Stream Consumer Helpers
// ----------------------------------------------------------------------------
static inline Point get_coord(const CityCacheEntry &entry) {
    #pragma HLS INLINE
    Coord x = entry.range(NODE_CACHE_COORD_X_OFFSET + COORD_BITS - 1, NODE_CACHE_COORD_X_OFFSET);
    Coord y = entry.range(NODE_CACHE_COORD_Y_OFFSET + COORD_BITS - 1, NODE_CACHE_COORD_Y_OFFSET);
    return Point{x, y};
}

static inline Point get_coord(const CityData &cd) {
    #pragma HLS INLINE
    return Point{cd.x, cd.y};
}

static inline Capacity get_demand(const CityCacheEntry &entry) {
    #pragma HLS INLINE
    return entry.range(NODE_CACHE_DEMAND_OFFSET + CAPACITY_BITS - 1, NODE_CACHE_DEMAND_OFFSET);
}

static inline CityCacheEntry consume_next_cache_entry(CityCacheBeat &beat) {
    #pragma HLS INLINE
    CityCacheEntry entry = beat.range(NODE_CACHE_ENTRY_BITS - 1, 0);
    beat >>= NODE_CACHE_ENTRY_BITS;
    return entry;
}

static inline CityData consume_next_city_data(CityCacheBeat &beat) {
    #pragma HLS INLINE
    CityCacheEntry entry = consume_next_cache_entry(beat);
    Point coord = get_coord(entry);
    Capacity demand = get_demand(entry);
    return CityData{coord.x, coord.y, demand};
}

static inline RandomNumber consume_next_random(Beat &beat) {
    #pragma HLS INLINE
    RandomNumber rnd = beat.range(RANDOM_BITS - 1, 0);
    beat >>= RANDOM_BITS;
    return rnd;
}

#endif // TYPES_HPP
