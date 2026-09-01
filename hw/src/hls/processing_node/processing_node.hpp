#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include <ap_int.h>
#include <ap_axi_sdata.h>
#include <hls_stream.h>

#include "../../../../sw/src/ga_encoding_constants.hpp"

// AXI4-Stream payload
typedef ap_axiu<AXI_WIDTH, 0, 0, 0> axi_word_t;

// Single field (fitness or city) packed representation
typedef ap_uint<COORD_BITS> Coord;
typedef ap_uint<CAPACITY_BITS> Capacity;
typedef ap_uint<NODE_ID_BITS> NodeId;
typedef ap_uint<FIELD_BITS> FitnessOrCity;
typedef ap_uint<BEAT_WIDTH> Beat;

// Individual as an array of beats
typedef Beat IndividualAsBeats[MAX_BEATS];

// Get the beat and field indices for a global field index (0 = fitness)
static inline void get_beat_field(int global_idx, int &beat_idx, int &field_idx) {
#pragma HLS INLINE
    beat_idx = global_idx / FIELDS_PER_BEAT;
    field_idx = global_idx % FIELDS_PER_BEAT;
}

static inline FitnessOrCity get_field_o(const IndividualAsBeats &ind, int beat_idx, int field_offset) {
#pragma HLS INLINE
    int high = field_offset + FIELD_BITS - 1;
    return ind[beat_idx].range(high, field_offset);
}

static inline FitnessOrCity get_field(const IndividualAsBeats &ind, int beat_idx, int field_idx) {
#pragma HLS INLINE
    int low = field_idx * FIELD_BITS;
    return get_field_o(ind, beat_idx, low);
}

// Read a field (fitness or city) from the individual
static inline FitnessOrCity get_field(const IndividualAsBeats &ind, int global_idx) {
#pragma HLS INLINE
    int beat_idx, field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    return get_field(ind, beat_idx, field_idx);
}

static inline void set_field_o(IndividualAsBeats &ind, int beat_idx, int field_offset, FitnessOrCity val) {
#pragma HLS INLINE
    int high = field_offset + FIELD_BITS - 1;
    ind[beat_idx].range(high, field_offset) = val;
}

static inline void set_field(IndividualAsBeats &ind, int beat_idx, int field_idx, FitnessOrCity val) {
#pragma HLS INLINE
    int low = field_idx * FIELD_BITS;
    set_field_o(ind, beat_idx, low, val);
}


// Write a field (fitness or city) to the individual
static inline void set_field(IndividualAsBeats &ind, int global_idx, FitnessOrCity val) {
#pragma HLS INLINE
    int beat_idx, field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    set_field(ind, beat_idx, field_idx, val);
}

static inline int get_last_field_index(int num_cities, int beat_idx) {
    int num_fields = 1 + num_cities;
    return beat_idx == get_total_beats(num_cities) - 1 ? (num_fields - 1) % FIELDS_PER_BEAT : FIELDS_PER_BEAT - 1;
}

static inline int get_first_city_index(int beat_idx) {
    return beat_idx == 0 ? 1 : 0;
}

// Convenience wrappers
static inline FitnessOrCity get_fitness(const IndividualAsBeats &ind) {
    return get_field(ind, 0);
}
static inline void set_fitness(IndividualAsBeats &ind, FitnessOrCity val) {
    set_field(ind, 0, val);
}
static inline FitnessOrCity get_city(const IndividualAsBeats &ind, int city_idx) {
    // city_idx is 1..num_cities, mapped to global_idx = city_idx
    return get_field(ind, city_idx);
}

static inline void set_city(IndividualAsBeats &ind, int city_idx, FitnessOrCity val) {
    set_field(ind, city_idx, val);
}

// Field-level accessors on a FitnessOrCity (for crossover modifications)
static inline Coord get_coord_x(FitnessOrCity val) {
    return val.range(COORD_BITS-1, 0);
}
static inline Coord get_coord_y(FitnessOrCity val) {
    return val.range(2*COORD_BITS-1, COORD_BITS);
}
static inline Capacity get_capacity(FitnessOrCity val) {
    return val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS);
}
static inline bool get_is_depot(FitnessOrCity val) {
    return val[2*COORD_BITS+CAPACITY_BITS];
}

static inline int get_node_id_idx(int city_idx) {
    return city_idx + FIELD_BITS - NODE_ID_BITS; // city_idx is 1..num_cities, node_idx is 0-based
}

static inline NodeId get_node_id(IndividualAsBeats &ind, int beat_idx, int node_idx) {
    return ind[beat_idx].range(node_idx + NODE_ID_BITS - 1, node_idx);
}

static inline NodeId get_node_id(FitnessOrCity val) {
    return val.range(FIELD_BITS-1, FIELD_BITS - NODE_ID_BITS);
}

static inline void set_coord_x(FitnessOrCity &val, Coord x) {
    val.range(COORD_BITS-1, 0) = x;
}
static inline void set_coord_y(FitnessOrCity &val, Coord y) {
    val.range(2*COORD_BITS-1, COORD_BITS) = y;
}
static inline void set_capacity(FitnessOrCity &val, Capacity cap) {
    val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) = cap;
}
static inline void set_is_depot(FitnessOrCity &val, bool depot) {
    val[2*COORD_BITS+CAPACITY_BITS] = depot;
}
static inline void set_node_id(FitnessOrCity &val, NodeId id) {
    val.range(FIELD_BITS-1, FIELD_BITS - NODE_ID_BITS) = id;
}

// Copy only the geometry/capacity bits from src to dst, preserving node_id
// (which is read from src and written into dst). Used by OX crossover so the
// children carry the donor's node_id together with the recipient's x/y/cap.
static inline FitnessOrCity copy_geometry_with_id(FitnessOrCity src, FitnessOrCity dst_template) {
    FitnessOrCity dst = dst_template;
    set_node_id(dst, get_node_id(src));
    return dst;
}

// Top-level function prototype
void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals0,
    int num_individuals1,
    int num_cities,
    int *dbg_num_individuals0,
    int *dbg_num_individuals1,
    int *dbg_num_cities
);

#endif // PROCESSING_NODE_HPP