#include "types.hpp"
#include "util.hpp"

#include <hls_stream.h>

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

    bool has_cached_tour_end_beat = false;
    Beat cached_tour_end_beat = 0;
    BeatIndex cached_tour_end_beat_idx = 0;

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
                // If spillover completed the last beat of the tour, cache it to flush after loop finishes
                if (!at_tour_end && curr_idx + pkt.n >= num_cities) {
                    cached_tour_end_beat = accum_beat;
                    cached_tour_end_beat_idx = current_beat_idx;
                    has_cached_tour_end_beat = true;

                    current_beat_idx = 0;
                    accum_beat = (0 == end_beat_idx) ? end_beat : ((0 == start_beat) ? initial_start_beat : Beat(0));
                    beat_flushed = true;
                } else {
                    beat_flushed = false;
                }
            } else {
                beat_flushed = true;
            }
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
    if (has_cached_tour_end_beat) {
        dst_ind[cached_tour_end_beat_idx] = cached_tour_end_beat;
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