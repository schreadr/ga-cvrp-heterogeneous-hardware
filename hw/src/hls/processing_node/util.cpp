#include "types.hpp"
#include "util.hpp"
#include <hls_stream.h>

void stream_beats_to_individual(
    hls::stream<BeatPacket> &in_stream,
    IndividualAsBeats &dst_ind,
    int dst_start_idx,
    int n_cities_to_copy,
    int n_beats,    // actual number of beats in dst_ind
    int num_cities  // total number of cities in dst_ind. index 0 is fitness, 1..num_cities are cities.
) {
    ap_uint<BEAT_WIDTH << 1> window = 0;
    int current_valid_fields = 0;
    int cities_left = n_cities_to_copy;
    int curr_dst_idx = dst_start_idx;

    // 2. Pipelined write loop with wrap-around filling (II=1)
    WRITE_BEATS_LOOP: do {
        #pragma HLS PIPELINE II=1

        if(current_valid_fields < FIELDS_PER_BEAT && cities_left > 0){
            BeatPacket pkt = in_stream.read();
            window |= ((ap_uint<BEAT_WIDTH << 1>)pkt.beat) << (current_valid_fields << FIELD_BITS_LOG2);
            current_valid_fields += pkt.n;
            cities_left -= pkt.n;
        }else{
            int dst_beat = get_beat_idx(curr_dst_idx);
            int dst_offset = curr_dst_idx & (FIELDS_PER_BEAT - 1);

            int space_in_beat = FIELDS_PER_BEAT - dst_offset;
            int cities_until_wrap = num_cities - curr_dst_idx + 1;

            // Determine how many fields to write in this clock cycle
            int fields_this_beat = space_in_beat;
            if (fields_this_beat > cities_until_wrap) fields_this_beat = cities_until_wrap;
            if (fields_this_beat > current_valid_fields) fields_this_beat = current_valid_fields;

            // Extract valid fields from window bottom and align to dst_offset
            ap_uint<BEAT_WIDTH << 1> raw_chunk = window.range((fields_this_beat << FIELD_BITS_LOG2) - 1, 0);
            Beat out_beat = (Beat)(raw_chunk << (dst_offset << FIELD_BITS_LOG2));

            int low_bit = dst_offset << FIELD_BITS_LOG2;
            int high_bit = ((dst_offset + fields_this_beat) << FIELD_BITS_LOG2) - 1;
            dst_ind[dst_beat].range(high_bit, low_bit) = out_beat.range(high_bit, low_bit);

            // Advance destination index and handle wrap-around
            curr_dst_idx += fields_this_beat;
            if (curr_dst_idx > num_cities) {
                curr_dst_idx = get_first_city_index(0); // Wraps to first city field (global index 1)
            }

            // Consume fields from window
            window >>= (fields_this_beat << FIELD_BITS_LOG2);
            current_valid_fields -= fields_this_beat;
        }
    } while(current_valid_fields > 0);
}