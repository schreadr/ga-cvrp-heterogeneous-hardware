#ifndef TYPES_HPP
#define TYPES_HPP

#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <hls_stream.h>

#include "ga_encoding_constants.hpp"

#define DEPOT_FLAG_FIELD_OFFSET (2 * COORD_BITS + CAPACITY_BITS)

// AXI4-Stream payload
typedef ap_axiu<AXI_WIDTH, 0, 0, 0> axi_word_t;

// Single field (fitness or city) packed representation
typedef ap_uint<COORD_BITS> Coord;
// TODO rename to Demand
typedef ap_uint<CAPACITY_BITS> Capacity;
typedef ap_uint<NODE_ID_BITS> NodeId;
typedef ap_uint<FIELD_BITS> FitnessOrCity;
typedef ap_uint<RANDOM_BITS> RandomNumber;
typedef ap_uint<BEAT_WIDTH> Beat;

// TODO use it where indicated
typedef ap_uint<FIELDS_PER_BEAT_LOG2 + 1> BeatFieldIndex;
typedef ap_uint<MAX_FIELDS_LOG2 + 1> FieldIndex;
typedef ap_uint<MAX_BEATS_LOG2 + 1> BeatIndex;

// Individual as an array of beats
typedef Beat IndividualAsBeats[MAX_BEATS];

struct StartEndData {
    const BeatIndex start_beat;
    const FieldIndex start_field;
    const BeatIndex end_beat;
    const FieldIndex end_field;

    StartEndData(BeatIndex s_beat, FieldIndex s_field, BeatIndex e_beat, FieldIndex e_field)
        : start_beat(s_beat), start_field(s_field), end_beat(e_beat), end_field(e_field) {}
};

struct CityPair {
    FieldIndex start;
    FieldIndex end;

    CityPair() : start(0), end(0) {}
    CityPair(FieldIndex s, FieldIndex e) : start(s), end(e) {}

    bool operator==(const CityPair &other) const {
        return start == other.start && end == other.end;
    }
};

// Get the beat and field indices for a global field index (0 = fitness)
static inline void get_beat_field(FieldIndex global_idx, BeatIndex &beat_idx, FieldIndex &field_idx) {
    #pragma HLS INLINE

    beat_idx = global_idx >> FIELDS_PER_BEAT_LOG2; // global_idx / FIELDS_PER_BEAT
    field_idx = global_idx & (FIELDS_PER_BEAT - 1); // global_idx % FIELDS_PER_BEAT
}

static inline BeatIndex get_beat_idx(FieldIndex global_idx) {
    #pragma HLS INLINE
    return global_idx >> FIELDS_PER_BEAT_LOG2; // global_idx / FIELDS_PER_BEAT
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

static inline FitnessOrCity get_field_o(const IndividualAsBeats &ind, BeatIndex beat_idx, FieldIndex field_offset) {
    #pragma HLS INLINE

    FieldIndex high = field_offset + FIELD_BITS - 1;
    return ind[beat_idx].range(high, field_offset);
}

static inline FitnessOrCity get_field(const Beat &beat, FieldIndex field_idx) {
    #pragma HLS INLINE
    
    FieldIndex low = field_idx << FIELD_BITS_LOG2;
    FieldIndex high = low + FIELD_BITS - 1;
    return beat.range(high, low);
}

static inline FitnessOrCity get_field(const IndividualAsBeats &ind, BeatIndex beat_idx, FieldIndex field_idx) {
    #pragma HLS INLINE

    FieldIndex low = field_idx << FIELD_BITS_LOG2; // field_idx * FIELD_BITS
    return get_field_o(ind, beat_idx, low);
}

// Read a field (fitness or city) from the individual
static inline FitnessOrCity get_field(const IndividualAsBeats &ind, FieldIndex global_idx) {
    #pragma HLS INLINE

    BeatIndex beat_idx;
    FieldIndex field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    return get_field(ind, beat_idx, field_idx);
}

static inline FitnessOrCity consume_next_field(Beat &beat) {
    #pragma HLS INLINE
    FitnessOrCity field = beat.range(FIELD_BITS - 1, 0);
    beat >>= FIELD_BITS;
    return field;
}

static inline RandomNumber consume_next_random(Beat &beat) {
    #pragma HLS INLINE
    RandomNumber rnd = beat.range(RANDOM_BITS - 1, 0);
    beat >>= RANDOM_BITS;
    return rnd;
}

static inline void set_field_o(IndividualAsBeats &ind, BeatIndex beat_idx, FieldIndex field_offset, FitnessOrCity val) {
    #pragma HLS INLINE

    FieldIndex high = field_offset + FIELD_BITS - 1;
    ind[beat_idx].range(high, field_offset) = val;
}

static inline void set_field(IndividualAsBeats &ind, BeatIndex beat_idx, FieldIndex field_idx, FitnessOrCity val) {
    #pragma HLS INLINE

    FieldIndex low = field_idx << FIELD_BITS_LOG2; // field_idx * FIELD_BITS
    set_field_o(ind, beat_idx, low, val);
}

static inline void set_field(Beat &beat, FieldIndex field_idx, FitnessOrCity val) {
    #pragma HLS INLINE

    FieldIndex low = field_idx << FIELD_BITS_LOG2; // field_idx * FIELD_BITS
    FieldIndex high = low + FIELD_BITS - 1;
    beat.range(high, low) = val;
}

// Write a field (fitness or city) to the individual
static inline void set_field(IndividualAsBeats &ind, FieldIndex global_idx, FitnessOrCity val) {
    #pragma HLS INLINE

    BeatIndex beat_idx;
    FieldIndex field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    set_field(ind, beat_idx, field_idx, val);
}

static inline FieldIndex get_last_field_index(FieldIndex num_cities, BeatIndex beat_idx) {
    #pragma HLS INLINE

    FieldIndex num_fields = 1 + num_cities;
    return beat_idx == BeatIndex(get_total_beats(num_cities) - 1) ? FieldIndex((num_fields - FieldIndex(1)) & FieldIndex(FIELDS_PER_BEAT - 1)) : FieldIndex(FIELDS_PER_BEAT - 1);
}

static inline FieldIndex get_first_city_index(BeatIndex beat_idx) {
    #pragma HLS INLINE
    return beat_idx == 0 ? 1 : 0;
}

// Convenience wrappers
static inline FitnessOrCity get_fitness(const IndividualAsBeats &ind) {
    #pragma HLS INLINE
    return get_field(ind, 0);
}
static inline void set_fitness(IndividualAsBeats &ind, FitnessOrCity val) {
    #pragma HLS INLINE
    set_field(ind, 0, val);
}

// Field-level accessors on a FitnessOrCity (for crossover modifications)
static inline Coord get_coord_x(FitnessOrCity val) {
    #pragma HLS INLINE
    return val.range(COORD_BITS-1, 0);
}
static inline Coord get_coord_y(FitnessOrCity val) {
    #pragma HLS INLINE
    return val.range(2*COORD_BITS-1, COORD_BITS);
}
static inline Capacity get_capacity(FitnessOrCity val) {
    #pragma HLS INLINE
    return val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS);
}
static inline bool get_is_depot(FitnessOrCity val) {
    #pragma HLS INLINE
    return val[DEPOT_FLAG_FIELD_OFFSET];
}

static inline FieldIndex get_node_id_idx(FieldIndex city_idx) {
    #pragma HLS INLINE
    return city_idx + FIELD_BITS - NODE_ID_BITS; // city_idx is 1..num_cities, node_idx is 0-based
}

static inline NodeId get_node_id(IndividualAsBeats &ind, BeatIndex beat_idx, FieldIndex node_idx) {
    #pragma HLS INLINE
    return ind[beat_idx].range(node_idx + NODE_ID_BITS - 1, node_idx);
}

static inline NodeId get_node_id(FitnessOrCity val) {
    #pragma HLS INLINE
    return val.range(2*COORD_BITS+CAPACITY_BITS+DEPOT_BITS + NODE_ID_BITS - 1, 2*COORD_BITS+CAPACITY_BITS+DEPOT_BITS);
}

static inline void set_coord_x(FitnessOrCity &val, Coord x) {
    #pragma HLS INLINE
    val.range(COORD_BITS-1, 0) = x;
}
static inline void set_coord_y(FitnessOrCity &val, Coord y) {
    #pragma HLS INLINE
    val.range(2*COORD_BITS-1, COORD_BITS) = y;
}
static inline void set_capacity(FitnessOrCity &val, Capacity cap) {
    #pragma HLS INLINE
    val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) = cap;
}
static inline void set_is_depot(FitnessOrCity &val, bool depot) {
    #pragma HLS INLINE
    val[DEPOT_FLAG_FIELD_OFFSET] = depot;
}
static inline void set_node_id(FitnessOrCity &val, NodeId id) {
    #pragma HLS INLINE
    val.range(2*COORD_BITS+CAPACITY_BITS+DEPOT_BITS + NODE_ID_BITS - 1, 2*COORD_BITS+CAPACITY_BITS+DEPOT_BITS) = id;
}

// DEBUG Helper functions
//static void print_beat_node_ids(const Beat &b, FieldIndex from_field, FieldIndex to_field) {
//    std::cout << "Cities from " << from_field << " to " << to_field << ": ";
//    for (FieldIndex i = from_field; i <= to_field; ++i) {
//        NodeId id = get_node_id(get_field(b, i));
//        std::cout << id.to_string(10) << ", ";
//    }
//    std::cout << std::endl;
//}

#endif // TYPES_HPP
