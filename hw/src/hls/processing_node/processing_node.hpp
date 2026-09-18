#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include "types.hpp"

void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    hls::stream<axi_word_t> &in_random,
    int num_individuals,
    int num_cities,
    int vehicle_capacity,
    int init_city_cache,
    int *dbg_num_individuals,
    int *dbg_num_cities
);

void random_number_converter(
    hls::stream<axi_word_t> &in_random,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream<CityPair> &mutate_pairs_0,
    hls::stream<CityPair> &mutate_pairs_1,
    int num_individuals
);

#endif // PROCESSING_NODE_HPP
