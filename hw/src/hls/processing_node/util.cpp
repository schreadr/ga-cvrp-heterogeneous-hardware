#include "types.hpp"
#include "util.hpp"

#include <hls_stream.h>

void inline capture_next_packet(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream){
    #pragma HLS INLINE
    BeatPacket pkt = in_stream.read();
    state.window |= ((ap_uint<HW_BEAT_WIDTH << 1>)pkt.beat) << (state.current_valid_cities << CITY_BITS_LOG2);
    state.current_valid_cities += pkt.n;
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
    BeatIndex dst_beat = get_beat_idx(state.curr_dst_idx);
    CityIndex dst_offset = state.curr_dst_idx & (HW_CITIES_PER_BEAT - 1);

    CityIndex space_in_beat = HW_CITIES_PER_BEAT - dst_offset;
    CityIndex cities_until_wrap = num_cities - state.curr_dst_idx;

    CityIndex cities_this_beat = space_in_beat;
    if (cities_this_beat > cities_until_wrap) cities_this_beat = cities_until_wrap;
    if (cities_this_beat > state.current_valid_cities) cities_this_beat = state.current_valid_cities;   

    // Commit current beat and pre-load target beat from RAM on beat changes
    if (state.current_beat_idx != dst_beat) {
        if (state.current_beat_idx != HW_MAX_CITY_BEATS) {
            dst_ind[state.current_beat_idx] = state.accum_beat;
            if (state.current_beat_idx == state.data.start_end_data.start_beat &&
                state.data.start_end_data.start_beat == state.data.start_end_data.end_beat) {
                state.data.end_beat = state.accum_beat;
            }
        }
        state.current_beat_idx = dst_beat;
        // For the case where start_beat == end_beat we want to use the end_beat value since it is updated
        if (dst_beat == state.data.start_end_data.end_beat) {
            state.accum_beat = state.data.end_beat;
        } else if (dst_beat == state.data.start_end_data.start_beat) {
            state.accum_beat = state.data.initial_start_beat;
        } else {
            state.accum_beat = 0;
        }
    }

    // Extract valid cities and write into local register
    ap_uint<HW_BEAT_WIDTH << 1> raw_chunk = state.window.range((cities_this_beat << CITY_BITS_LOG2) - 1, 0);
    Beat out_beat = (Beat)(raw_chunk << (dst_offset << CITY_BITS_LOG2));

    int low_bit = dst_offset << CITY_BITS_LOG2;
    int high_bit = ((dst_offset + cities_this_beat) << CITY_BITS_LOG2) - 1;

    // Update accum register
    state.accum_beat.range(high_bit, low_bit) = out_beat.range(high_bit, low_bit);

    // Advance destination index and handle wrap-around
    state.curr_dst_idx += cities_this_beat;
    state.curr_dst_idx = wrap_city_index(state.curr_dst_idx, num_cities);

    state.window >>= (cities_this_beat << CITY_BITS_LOG2);
    state.current_valid_cities -= cities_this_beat;
}

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, CityIndex num_cities){
    #pragma HLS INLINE

    if(state.current_valid_cities > 0){
        process_next_packet(state, dst_ind, num_cities);
    }

    if(state.current_valid_cities < HW_CITIES_PER_BEAT && state.cities_left > 0){
        capture_next_packet(state, in_stream);
    }
}

void flush_accumulated_beat(BeatConsumerState &state, IndividualAsBeats &dst_ind) {
    if (state.current_beat_idx != HW_MAX_CITY_BEATS) {
        dst_ind[state.current_beat_idx] = state.accum_beat;
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