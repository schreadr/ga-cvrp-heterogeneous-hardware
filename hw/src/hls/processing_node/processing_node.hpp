#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include <ap_int.h>
#include "ap_axi_sdata.h"
#include <hls_stream.h>

#define COORD_BITS       7
#define CAPACITY_BITS    6

// User-configurable problem max size constants
#define MAX_BEATS_PER_INDIVIDUAL 64

// Encoding constants
#define AXI_WIDTH        512
#define AXI_BYTES        (AXI_WIDTH / 8)
#define DEPOT_BITS       1
#define FIELD_BITS       (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS)
#define FIELDS_PER_BEAT  (AXI_WIDTH / FIELD_BITS)     

// Derived maximum cities (64 beats * 24 fields per beat = 1536 cities)
#define MAX_CITIES       (MAX_BEATS_PER_INDIVIDUAL * FIELDS_PER_BEAT)

// AXI4-Stream payload
typedef ap_axiu<AXI_WIDTH, 0, 0, 0> axi_word_t;

// Represents a single City instance (Total: 21 bits)
struct City {
    ap_uint<COORD_BITS> coord_x;
    ap_uint<COORD_BITS> coord_y;
    ap_uint<CAPACITY_BITS> capacity;
    ap_uint<DEPOT_BITS> is_depot;
};

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