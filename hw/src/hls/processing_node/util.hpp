#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"
#include <hls_stream.h>

struct BeatPacket {
    Beat beat;
    int n;
};

struct StartEndDataWithBeats {
    const StartEndData start_end_data;
    const Beat initial_start_beat;
    // chagnes if end_beat is the same as start_beat
    Beat end_beat;

    StartEndDataWithBeats(const StartEndData &start_end_data, const Beat &start_beat, Beat end_beat)
        : start_end_data(start_end_data), initial_start_beat(start_beat), end_beat(end_beat) {}
};

// TODO improve performance using smaller data types
struct BeatConsumerState {
    int current_valid_fields;
    int cities_left;
    int curr_dst_idx;
    int current_beat_idx;

    Beat accum_beat;
    ap_uint<BEAT_WIDTH << 1> window;

    StartEndDataWithBeats data;

    BeatConsumerState(
        int n_cities_to_copy,
        int dst_start_idx,
        StartEndDataWithBeats data
    )
        : current_valid_fields(0),
          cities_left(n_cities_to_copy),
          curr_dst_idx(dst_start_idx),
          current_beat_idx(-1),
          data(data),
          window(0) {}
};

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, int num_cities);

void flush_accumulated_beat(BeatConsumerState &state, IndividualAsBeats &dst_ind);

void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    int dst_start_idx,
    int n_cities_to_copy,
    const StartEndDataWithBeats &start_end_data,
    int n_beats,
    int num_cities
);

#endif // UTIL_HPP