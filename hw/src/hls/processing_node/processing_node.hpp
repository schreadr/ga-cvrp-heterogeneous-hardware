#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include <ap_int.h>
#include "ap_axi_sdata.h"
#include <hls_stream.h>

#define COORD_BITS       7
#define CAPACITY_BITS    6

// User-configurable problem max size constants
#define MAX_CITIES 1023

// Encoding constants
#define AXI_WIDTH        512
#define AXI_BYTES        (AXI_WIDTH / 8)
#define DEPOT_BITS       1
#define FIELD_BITS       (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS)
#define FIELDS_PER_BEAT  (AXI_WIDTH / FIELD_BITS)
#define MAX_BEATS        ((MAX_CITIES + FIELDS_PER_BEAT) / FIELDS_PER_BEAT)
#define BEAT_WIDTH       (FIELDS_PER_BEAT * FIELD_BITS)

// AXI4-Stream payload
typedef ap_axiu<AXI_WIDTH, 0, 0, 0> axi_word_t;

// Single field (fitness or city) packed representation
typedef ap_uint<COORD_BITS> Coord; 
typedef ap_uint<CAPACITY_BITS> Capacity; 
typedef ap_uint<FIELD_BITS> FitnessOrCity;
typedef ap_uint<BEAT_WIDTH> Beat;


// Individual as an array of beats
typedef Beat Individual[MAX_BEATS];

// ----------------------------------------------------------------------------
// Inline accessors for individual fields (fitness + cities)
// ----------------------------------------------------------------------------

// Get the beat and field indices for a global field index (0 = fitness)
static inline void get_beat_field(int global_idx, int &beat_idx, int &field_idx) {
    beat_idx = global_idx / FIELDS_PER_BEAT;
    field_idx = global_idx % FIELDS_PER_BEAT;
}

// Read a field (fitness or city) from the individual
static inline FitnessOrCity get_field(const Individual &ind, int global_idx) {
#pragma HLS INLINE
    int beat_idx, field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    int low = field_idx * FIELD_BITS;
    int high = low + FIELD_BITS - 1;
    return ind[beat_idx].range(high, low);
}

// Write a field (fitness or city) to the individual
static inline void set_field(Individual &ind, int global_idx, FitnessOrCity val) {
#pragma HLS INLINE
    int beat_idx, field_idx;
    get_beat_field(global_idx, beat_idx, field_idx);
    int low = field_idx * FIELD_BITS;
    int high = low + FIELD_BITS - 1;
    ind[beat_idx].range(high, low) = val;
}

// Convenience wrappers
static inline FitnessOrCity get_fitness(const Individual &ind) {
    return get_field(ind, 0);
}
static inline void set_fitness(Individual &ind, FitnessOrCity val) {
    set_field(ind, 0, val);
}
static inline FitnessOrCity get_city(const Individual &ind, int city_idx) {
    // city_idx is 1..num_cities, mapped to global_idx = city_idx
    return get_field(ind, city_idx);
}
static inline void set_city(Individual &ind, int city_idx, FitnessOrCity val) {
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