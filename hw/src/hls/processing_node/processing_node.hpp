#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include "types.hpp"

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
