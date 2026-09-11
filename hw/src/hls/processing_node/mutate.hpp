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
    CityPair cut_points,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_cities
);

#endif // MUTATE_HPP
