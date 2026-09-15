#include "types.hpp"
#include "util.hpp"

#include <hls_stream.h>

static inline CityBeatIndex beat_offset_after_add(
    CityBeatIndex base_offset,
    CityBeatIndex delta,
    CityBeatIndex num_cities_mod,
    bool wrapped
) {
    #pragma HLS INLINE
    CityBeatIndex no_wrap = get_city_beat_idx(base_offset + delta);
    CityBeatIndex wrap    = get_city_beat_idx(no_wrap - num_cities_mod + HW_CITIES_PER_BEAT);
    return wrapped ? wrap : no_wrap;
}

void inline capture_next_packet(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream){
    #pragma HLS INLINE
    BeatPacket pkt = in_stream.read();
    state.window |= ((BeatWindow)pkt.beat) << ((CityIndex)state.valid_cities << CITY_BITS_LOG2);
    state.valid_cities += pkt.n;
    if (state.cities_left <= pkt.n) {
        state.cities_left = 0;
    } else {
        state.cities_left -= pkt.n;
    }
}

void inline process_next_packet(
    BeatConsumerState &state,
    IndividualAsBeats &dst_ind,
    CityIndex num_cities
) {
    #pragma HLS INLINE
    BeatIndex     dst_beat   = get_beat_idx(state.dst_idx);
    CityBeatIndex dst_offset = get_city_beat_idx(state.dst_idx);
    CityBeatIndex n          = (state.valid_cities < state.beat_limit)
                               ? state.valid_cities : state.beat_limit;

    if (state.beat_idx != dst_beat) {
        if (state.beat_idx != HW_MAX_CITY_BEATS) {
            dst_ind[state.beat_idx] = state.accum_beat;
            if (state.beat_idx == state.data.start_end_data.start_beat &&
                state.data.start_end_data.start_beat == state.data.start_end_data.end_beat) {
                state.data.end_beat = state.accum_beat;
            }
        }
        state.beat_idx = dst_beat;
        // For the case where start_beat == end_beat we want to use the end_beat value since it is updated
        if (dst_beat == state.data.start_end_data.end_beat) {
            state.accum_beat = state.data.end_beat;
        } else if (dst_beat == state.data.start_end_data.start_beat) {
            state.accum_beat = state.data.initial_start_beat;
        } else {
            state.accum_beat = 0;
        }
    }

    UPDATE_ACCUM_SLOTS: for (CityBeatIndex s = 0; s < HW_CITIES_PER_BEAT; s++) {
        #pragma HLS UNROLL
        if (s >= dst_offset && s < dst_offset + n) {
            CityBeatIndex w = s - dst_offset;
            City c = 0;
            switch (w) {
                case 0: c = state.window.range(1 * CITY_BITS - 1, 0 * CITY_BITS); break;
                case 1: c = state.window.range(2 * CITY_BITS - 1, 1 * CITY_BITS); break;
                case 2: c = state.window.range(3 * CITY_BITS - 1, 2 * CITY_BITS); break;
                default: c = state.window.range(4 * CITY_BITS - 1, 3 * CITY_BITS); break;
            }
            set_city(state.accum_beat, s, c);
        }
    }

    bool wrapped = (CityIndex(n) == state.wrap_distance);
    state.dst_idx += n;
    if (wrapped) state.dst_idx -= num_cities;

    state.window >>= ((CityIndex)n << CITY_BITS_LOG2);
    state.valid_cities -= n;

    state.wrap_distance = wrapped ? num_cities
                                  : CityIndex(state.wrap_distance - n);

    CityBeatIndex new_beat_space = HW_CITIES_PER_BEAT - beat_offset_after_add(
        dst_offset, n, get_city_beat_idx(num_cities), wrapped);
    state.beat_limit = (state.wrap_distance < new_beat_space)
                       ? (CityBeatIndex)state.wrap_distance : new_beat_space;
}

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, CityIndex num_cities){
    #pragma HLS INLINE

    if (state.valid_cities > 0) {
        process_next_packet(state, dst_ind, num_cities);
    }

    if (state.valid_cities < HW_CITIES_PER_BEAT && state.cities_left > 0) {
        capture_next_packet(state, in_stream);
    }
}

void flush_accumulated_beat(BeatConsumerState &state, IndividualAsBeats &dst_ind) {
    if (state.beat_idx != HW_MAX_CITY_BEATS) {
        dst_ind[state.beat_idx] = state.accum_beat;
    }
}

void append_city(BeatProducerState &state, City city){
    #pragma HLS INLINE
    state.beat >>= CITY_BITS;
    state.beat.range(HW_BEAT_WIDTH - 1, HW_BEAT_WIDTH - CITY_BITS) = city;
    state.count++;
}

void send_city(
    BeatProducerState &state,
    City city,
    hls::stream<BeatPacket> &beat_stream
) {
    #pragma HLS INLINE
    append_city(state, city);

    if (state.count == HW_CITIES_PER_BEAT) {
        BeatPacket pkt;
        pkt.beat = state.beat;
        pkt.n = state.count;
        beat_stream.write(pkt);
        state.count = 0;
        state.beat = 0;
    }
}

void finalizeStream(
    BeatProducerState &state,
    hls::stream<BeatPacket> &beat_stream
) {
    #pragma HLS INLINE
    if (state.count > 0) {
        state.beat >>= (HW_CITIES_PER_BEAT - state.count) * CITY_BITS;
        BeatPacket pkt;
        pkt.beat = state.beat;
        pkt.n = state.count;
        beat_stream.write(pkt);
        state.count = 0;
        state.beat = 0;
    }
}