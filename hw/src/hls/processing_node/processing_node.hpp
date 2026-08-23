#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include "hls_stream.h"
#include "ap_axi_sdata.h"

#define MAX_BEATS_PER_INDIVIDUAL 64

// Struct providing full AXI4-Stream signal bundle expected by Coyote
typedef ap_axiu<512, 0, 0, 0> axi_word_t;

// Helper function signature
void process_single_stream(
    hls::stream<axi_word_t> &in_stream,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int beats_per_individual
);

// Top-level entry function
void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals0,
    int num_individuals1,
    int beats_per_individual
);

#endif