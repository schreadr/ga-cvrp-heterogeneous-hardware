#ifndef UTIL_HPP
#define UTIL_HPP

#include "types.hpp"
#include <hls_stream.h>

#define BEAT_STREAM_INTERVAL 3

struct BeatPacket {
    Beat beat;
    CityIndex n;
};

struct BeatProducerState {
    Beat beat;
    CityIndex count;

    BeatProducerState() : beat(0), count(0) {}
};

void append_city(BeatProducerState &state, City city);

void send_city(
    BeatProducerState &state,
    City city,
    hls::stream<BeatPacket> &beat_stream
);

void finalizeStream(
    BeatProducerState &state,
    hls::stream<BeatPacket> &beat_stream
);

struct StartEndDataWithBeats {
    const StartEndData start_end_data;
    const Beat initial_start_beat;
    // changes if end_beat is the same as start_beat
    Beat end_beat;

    StartEndDataWithBeats(const StartEndData &start_end_data, const Beat &start_beat, Beat end_beat)
        : start_end_data(start_end_data), initial_start_beat(start_beat), end_beat(end_beat) {}
};

typedef ap_uint<HW_BEAT_WIDTH << 1> BeatWindow;

struct BeatConsumerState {
    CityBeatIndex valid_cities;
    CityIndex cities_left;
    CityIndex dst_idx;
    BeatIndex beat_idx;

    Beat accum_beat;
    BeatWindow window;

    StartEndDataWithBeats data;

    CityBeatIndex beat_limit;
    CityIndex wrap_distance;

    BeatConsumerState(
        CityIndex n_cities_to_copy,
        CityIndex dst_start_idx,
        StartEndDataWithBeats data,
        CityIndex num_cities
    )
        : valid_cities(0),
          cities_left(n_cities_to_copy),
          dst_idx(dst_start_idx),
          beat_idx(HW_MAX_CITY_BEATS),
          data(data),
          window(0)
    {
        CityBeatIndex beat_space  = HW_CITIES_PER_BEAT - get_city_beat_idx(dst_start_idx);
        CityIndex     wrap_dist   = num_cities - dst_start_idx;
        wrap_distance = wrap_dist;
        beat_limit    = (wrap_dist < beat_space) ? (CityBeatIndex)wrap_dist : beat_space;
    }
};

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, CityIndex num_cities);

void flush_accumulated_beat(BeatConsumerState &state, IndividualAsBeats &dst_ind);

template <int TARGET_II>
void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    CityIndex dst_start_idx,
    CityIndex n_cities_to_copy,
    const StartEndDataWithBeats &start_end_data,
    BeatIndex n_beats,
    CityIndex num_cities
) {
    BeatConsumerState state(
        n_cities_to_copy,
        dst_start_idx,
        start_end_data,
        num_cities
    );

    WRITE_BEATS_LOOP: do {
        #pragma HLS PIPELINE II=TARGET_II
        // Instruct HLS that state dependency across iterations can be pipelined
        capture_and_propagate(state, in_stream, dst_ind, num_cities);
    } while (state.valid_cities > 0 || state.cities_left > 0);

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