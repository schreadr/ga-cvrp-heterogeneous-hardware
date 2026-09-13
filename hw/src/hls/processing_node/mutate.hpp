#ifndef MUTATE_HPP
#define MUTATE_HPP

#include "types.hpp"
#include <hls_streamofblocks.h>
#include <hls_stream.h>

void mutate_core(
    IndividualAsBeats &p,
    CityPair cut_points,
    IndividualAsBeats &c,
    int num_cities
);

void mutate(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream<CityPair> &mutate_pairs,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    int num_cities
);

#endif // MUTATE_HPP
