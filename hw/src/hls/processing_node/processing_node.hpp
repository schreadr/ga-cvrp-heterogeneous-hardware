#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include "hls_stream.h"
#include "ap_axi_sdata.h"
#include "ap_int.h"

#define MAX_BEATS_PER_INDIVIDUAL 64

// Bit layout of the 96-bit configuration word (cfg stream), matching
// ga_cvrp_axi_ctrl_parser.sv: {beats_per_individual[31:0], num_individuals1[31:0], num_individuals0[31:0]}
#define CFG_NUM_IND_0_OFFS    0
#define CFG_NUM_IND_1_OFFS    32
#define CFG_BEATS_PER_IND_OFFS 64

// Struct providing full AXI4-Stream signal bundle expected by Coyote (data path)
typedef ap_axiu<512, 0, 0, 0> axi_word_t;

// Configuration word carried over the thin cfg AXI4-Stream from the SV register parser
typedef ap_axiu<96, 0, 0, 0> cfg_word_t;

// Continuously reads pairs of individuals from in_stream, swaps them, and writes
// to out_stream. Free-running (infinite loop); process one batch (num individuals)
// per iteration.
void swap_stream(
    hls::stream<axi_word_t> &in_stream,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int beats_per_individual
);

// Top-level: two independent free-running streams, configuration via cfg stream.
// The kernel has no AXI-Lite control interface (ap_ctrl_none); instead the
// configuration registers set from software (see ga_cvrp_axi_ctrl_parser.sv)
// are delivered over the cfg stream as a single word.
void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    hls::stream<cfg_word_t> &cfg
);

#endif
