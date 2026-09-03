#ifndef MUTATE_HPP
#define MUTATE_HPP

#include "types.hpp"
#include <hls_streamofblocks.h>

void mutate(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
    hls::stream_of_blocks<IndividualAsBeats> &out0,
    hls::stream_of_blocks<IndividualAsBeats> &out1,
    int num_individuals,
    int num_cities
);

#endif // MUTATE_HPP
