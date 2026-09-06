#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"
#include <hls_stream.h>

struct BeatPacket {
    Beat beat;
    FieldIndex n;
};

struct StartEndDataWithBeats {
    const StartEndData start_end_data;
    const Beat initial_start_beat;
    // chagnes if end_beat is the same as start_beat
    Beat end_beat;

    StartEndDataWithBeats(const StartEndData &start_end_data, const Beat &start_beat, Beat end_beat)
        : start_end_data(start_end_data), initial_start_beat(start_beat), end_beat(end_beat) {}
};

struct BeatConsumerState {
    FieldIndex current_valid_fields;
    FieldIndex cities_left;
    FieldIndex curr_dst_idx;
    BeatIndex current_beat_idx;

    Beat accum_beat;
    ap_uint<BEAT_WIDTH << 1> window;

    StartEndDataWithBeats data;

    BeatConsumerState(
        FieldIndex n_cities_to_copy,
        FieldIndex dst_start_idx,
        StartEndDataWithBeats data
    )
        : current_valid_fields(0),
          cities_left(n_cities_to_copy),
          curr_dst_idx(dst_start_idx),
          current_beat_idx(MAX_BEATS),
          data(data),
          window(0) {}
};

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, FieldIndex num_cities);

void flush_accumulated_beat(BeatConsumerState &state, IndividualAsBeats &dst_ind);

void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    FieldIndex dst_start_idx,
    FieldIndex n_cities_to_copy,
    const StartEndDataWithBeats &start_end_data,
    BeatIndex n_beats,
    FieldIndex num_cities
);

#endif // UTIL_HPP