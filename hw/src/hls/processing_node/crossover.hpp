#ifndef CROSSOVER_HPP
#define CROSSOVER_HPP

#include "types.hpp"
#include <hls_streamofblocks.h>
#include <hls_stream.h>

void crossover(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream_of_blocks<IndividualAsBeats> &out0,
    hls::stream_of_blocks<IndividualAsBeats> &out1,
    int num_individuals,
    CityIndex num_cities,
    Probability crossover_prob
);

#endif // CROSSOVER_HPP
