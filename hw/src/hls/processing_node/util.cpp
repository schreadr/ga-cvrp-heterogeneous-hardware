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
            City c = state.window.range((w + 1) * CITY_BITS - 1, w * CITY_BITS);
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

void capture_and_tag_packets(
    hls::stream<BeatPacket> &in_stream,
    hls::stream<BeatPacketWithLast> &out_stream,
    CityIndex n_cities_to_copy
) {
    if (n_cities_to_copy == 0) return;

    CityIndex remaining = n_cities_to_copy;
    bool is_last = false;
    CAPTURE_LOOP: do {
        #pragma HLS PIPELINE II=1
        BeatPacket pkt = in_stream.read();
        CityIndex n = pkt.n;
        if (n > 0) {
            is_last = (n >= remaining);
            remaining -= n;

            BeatPacketWithLast tagged_pkt;
            tagged_pkt.beat = pkt.beat;
            tagged_pkt.n = n;
            tagged_pkt.last = is_last;
            out_stream.write(tagged_pkt);
        }
    } while (!is_last);
}

static inline void switch_beat(
    BeatIndex dst_beat,
    BeatIndex &current_beat_idx,
    Beat &accum_beat,
    Beat &end_beat,
    IndividualAsBeats &dst_ind,
    BeatIndex start_beat,
    BeatIndex end_beat_idx,
    Beat initial_start_beat,
    bool same_start_end
) {
    #pragma HLS INLINE
    Beat old_end = end_beat;
    dst_ind[current_beat_idx] = accum_beat;
    if (same_start_end && current_beat_idx == start_beat) {
        end_beat = accum_beat;
    }
    current_beat_idx = dst_beat;
    // For the case where start_beat == end_beat we want to use the end_beat value since it is updated
    accum_beat = (dst_beat == end_beat_idx) ? old_end : ((dst_beat == start_beat) ? initial_start_beat : Beat(0));
}

void propagate_tagged_packets(
    hls::stream<BeatPacketWithLast> &in_stream,
    IndividualAsBeats &dst_ind,
    CityIndex dst_start_idx,
    CityIndex n_cities_to_copy,
    const StartEndDataWithBeats &start_end_data,
    CityIndex num_cities
) {
    if (n_cities_to_copy == 0) return;

    CityIndex curr_idx = dst_start_idx;
    const BeatIndex start_beat = start_end_data.start_end_data.start_beat;
    const BeatIndex end_beat_idx = start_end_data.start_end_data.end_beat;
    const Beat initial_start_beat = start_end_data.initial_start_beat;
    const bool same_start_end = (start_beat == end_beat_idx);

    BeatIndex current_beat_idx = start_beat;
    Beat accum_beat = (start_beat == end_beat_idx) ? start_end_data.end_beat : initial_start_beat;
    Beat end_beat = start_end_data.end_beat;
    bool beat_flushed = false;

    bool is_last = false;
    PROPAGATE_LOOP: do {
        #pragma HLS PIPELINE II=1
        BeatPacketWithLast pkt = in_stream.read();
        is_last = pkt.last;

        CityBeatIndex dst_offset = get_city_beat_idx(curr_idx);
        CityBeatIndex beat_space = HW_CITIES_PER_BEAT - dst_offset;
        CityIndex tour_space = num_cities - curr_idx;
        bool at_tour_end = (tour_space <= beat_space);
        CityBeatIndex space = at_tour_end ? (CityBeatIndex)tour_space : beat_space;

        CityBeatIndex n_curr = (pkt.n < space) ? (CityBeatIndex)pkt.n : space;
        CityBeatIndex n_next = pkt.n - n_curr;

        // 1. Parallel slot assignment for current beat
        UPDATE_CURR_SLOTS: for (CityBeatIndex s = 0; s < HW_CITIES_PER_BEAT; s++) {
            #pragma HLS UNROLL
            CityBeatIndex w = s - dst_offset;
            if (s >= dst_offset && w < n_curr) {
                City c = pkt.beat.range((w + 1) * CITY_BITS - 1, w * CITY_BITS);
                accum_beat.range((s + 1) * CITY_BITS - 1, s * CITY_BITS) = c;
            }
        }

        // 2. If current beat was filled, flush it and switch to next beat (at most 1 write to dst_ind per cycle)
        if (n_curr == space) {
            BeatIndex next_beat = at_tour_end ? BeatIndex(0) : BeatIndex(current_beat_idx + 1);

            switch_beat(next_beat, current_beat_idx, accum_beat, end_beat, dst_ind,
                        start_beat, end_beat_idx, initial_start_beat, same_start_end);

            // 3. If packet spills over, write spillover slots into the new beat
            if (n_next > 0) {
                UPDATE_NEXT_SLOTS: for (CityBeatIndex s = 0; s < HW_CITIES_PER_BEAT; s++) {
                    #pragma HLS UNROLL
                    if (s < n_next) {
                        CityBeatIndex w = s + space;
                        City c = pkt.beat.range((w + 1) * CITY_BITS - 1, w * CITY_BITS);
                        accum_beat.range((s + 1) * CITY_BITS - 1, s * CITY_BITS) = c;
                    }
                }
            }
            beat_flushed = (n_next == 0);
        } else {
            beat_flushed = false;
        }

        // Advance global index
        CityIndex next_global = curr_idx + pkt.n;
        if (next_global >= num_cities) next_global -= num_cities;
        curr_idx = next_global;

    } while (!is_last);

    if (!beat_flushed) {
        dst_ind[current_beat_idx] = accum_beat;
    }
}

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, CityIndex num_cities){
    #pragma HLS INLINE

    if (state.valid_cities > 0) {
        process_next_packet(state, dst_ind, num_cities);
    }

    if (state.cities_left > 0) {
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