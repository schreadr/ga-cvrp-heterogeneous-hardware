#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include "hls_stream.h"
#include "ap_axi_sdata.h"

#define MAX_BEATS_PER_INDIVIDUAL 64

// Struct providing full AXI4-Stream signal bundle expected by Coyote (data path)
typedef ap_axiu<512, 0, 0, 0> axi_word_t;

// Continuously reads pairs of individuals from in_stream, swaps them, and writes
// to out_stream. Free-running (infinite loop); processes one batch (num individuals)
// per iteration.
void swap_stream(
    hls::stream<axi_word_t> &in_stream,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int beats_per_individual
);

// Top-level: two independent free-running streams.
//
// The configuration (num_individuals0, num_individuals1, beats_per_individual) is
// provided as scalar function arguments. Each of them is mapped to an AXI4-Lite
// register of the HLS-generated control interface (s_axilite), so the control
// register handling is done by HLS directly. Software programs them with setCSR(...).
//
// The kernel is free-running (no AXI-Lite control handshake, ap_ctrl_none). It waits
// until the configuration registers are programmed and then continuously processes
// the data streams batch after batch.
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
