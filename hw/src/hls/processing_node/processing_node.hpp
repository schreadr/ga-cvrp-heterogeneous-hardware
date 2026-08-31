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

// AXI4-Stream payload[cite: 3]
typedef ap_axiu<AXI_WIDTH, 0, 0, 0> axi_word_t;

// Represents a single City instance
typedef ap_uint<FIELD_BITS> FitnessOrCity;
typedef ap_uint<COORD_BITS> Coord; 
typedef ap_uint<CAPACITY_BITS> Capacity; 
typedef FitnessOrCity Individual[MAX_CITIES+1];

// setters and getters for the City struct fields, using bit ranges of the packed FIELD_BITS
static inline FitnessOrCity get_fitness(Individual val) {
    return val[0];
}

/* starting from 1 */
static inline FitnessOrCity get_city(Individual val, int i) {
    return val[i];
}

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

static inline void set_fitness(Individual& val, FitnessOrCity x) {
    val[0] = x;
}

static inline void set_city(Individual& val, FitnessOrCity x, int i) {
    val[i] = x;
}

static inline void set_coord_x(FitnessOrCity& val, Coord x) {
    val.range(COORD_BITS-1, 0) = x;
}

static inline void set_coord_y(FitnessOrCity& val, Coord y) {
    val.range(2*COORD_BITS-1, COORD_BITS) = y;
}

static inline void set_capacity(FitnessOrCity& val, Capacity cap) {
    val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) = cap;
}

static inline void set_is_depot(FitnessOrCity& val, bool depot) {
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