#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"
#include <hls_stream.h>

struct BeatPacket {
    Beat beat;
    FieldIndex n;
};

struct BeatProducerState {
    Beat beat;
    FieldIndex count;

    BeatProducerState() : beat(0), count(0) {}
};

void append_field(BeatProducerState &state, FitnessOrCity field);

void send_field(
    BeatProducerState &state,
    FitnessOrCity field,
    hls::stream<BeatPacket> &beat_stream
);

void finalizeStream(
    BeatProducerState &state,
    hls::stream<BeatPacket> &beat_stream
);

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

template <int TARGET_II>
void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    FieldIndex dst_start_idx,
    FieldIndex n_cities_to_copy,
    const StartEndDataWithBeats &start_end_data,
    BeatIndex n_beats,
    FieldIndex num_cities
) {
    BeatConsumerState state(
        n_cities_to_copy,
        dst_start_idx,
        start_end_data
    );

    WRITE_BEATS_LOOP: do {
        #pragma HLS PIPELINE II=TARGET_II
        // Instruct HLS that state dependency across iterations can be pipelined
        capture_and_propagate(state, in_stream, dst_ind, num_cities);
    } while (state.current_valid_fields > 0 || state.cities_left > 0);

    // Flush final accumulated beat to RAM after loop ends
    flush_accumulated_beat(state, dst_ind);
}

template <int W_IN>
ap_uint<(W_IN + 1) / 2> integer_sqrt(ap_uint<W_IN> value) {
    #pragma HLS INLINE

    constexpr int W_OUT = (W_IN + 1) / 2;
    constexpr int W_EVEN = W_OUT * 2;

    ap_uint<W_EVEN> rad = value;
    ap_uint<W_OUT> q = 0;
    ap_uint<W_EVEN + 2> ac = 0;

    INT_SQRT_LOOP: for (int i = 0; i < W_OUT; ++i) {
        #pragma HLS UNROLL

        ap_uint<W_EVEN + 2> shifted_ac = (ac << 2) | (rad >> (W_EVEN - 2));
        rad <<= 2;

        ap_uint<W_EVEN + 2> trial = (ap_uint<W_EVEN + 2>(q) << 2) | 1;
        ap_int<W_EVEN + 3> test_res = ap_int<W_EVEN + 3>(shifted_ac) - ap_int<W_EVEN + 3>(trial);

        if (test_res >= 0) {
            ac = test_res;
            q = (q << 1) | 1;
        } else {
            ac = shifted_ac;
            q <<= 1;
        }
    }

    return q;
}

#endif // UTIL_HPP