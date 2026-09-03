#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"
#include <hls_stream.h>

struct BeatPacket {
    Beat beat;
    int n;
};

void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    int dst_start_idx,
    int n_cities_to_copy,
    int n_beats,    // actual number of beats in dst_ind, TODO still necessary?
    int num_cities  // total number of cities in dst_ind. index 0 is fitness, 1..num_cities are cities.
);

#endif // UTIL_HPP