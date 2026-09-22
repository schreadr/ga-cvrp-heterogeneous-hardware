#ifndef PROCESSING_NODE_HPP
#define PROCESSING_NODE_HPP

#include "types.hpp"
#include "xoroshiro128plusplus.hpp"

void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals,
    int num_cities,
    int vehicle_capacity,
    int init_city_cache,
    int two_opt_k,
    int max_subtours,
    int two_opt_enable_precheck,
    uint32_t seed,
    int *dbg_num_individuals,
    int *dbg_num_cities
);

void generate_cx_mut_randoms(
    uint32_t seed_cx_mut,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream<CityPair> &mutate_pairs_0,
    hls::stream<CityPair> &mutate_pairs_1,
    int num_individuals,
    CityIndex num_cities
);

#endif // PROCESSING_NODE_HPP
