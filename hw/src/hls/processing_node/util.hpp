#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"
#include <hls_stream.h>

struct BeatPacket {
    Beat beat;
    int n;
};

// TODO improve performance using smaller data types
struct BeatConsumerState {
    int current_valid_fields;
    int cities_left;
    int curr_dst_idx;
    int current_beat_idx;

    Beat accum_beat;
    ap_uint<BEAT_WIDTH << 1> window;

    BeatConsumerState(int n_cities_to_copy, int dst_start_idx) : current_valid_fields(0), cities_left(n_cities_to_copy), curr_dst_idx(dst_start_idx), current_beat_idx(-1), accum_beat(0), window(0) {}
};

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, int num_cities);

void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    int dst_start_idx,
    int n_cities_to_copy,
    int n_beats,    // actual number of beats in dst_ind, TODO still necessary?
    int num_cities  // total number of cities in dst_ind. index 0 is fitness, 1..num_cities are cities.
);

#endif // UTIL_HPP