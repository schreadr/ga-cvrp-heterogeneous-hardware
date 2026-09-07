#include "types.hpp"
#include "util.hpp"

#include <hls_stream.h>


void inline capture_next_packet(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream){
    #pragma HLS INLINE
    BeatPacket pkt = in_stream.read();
    state.window |= ((ap_uint<BEAT_WIDTH << 1>)pkt.beat) << (state.current_valid_fields << FIELD_BITS_LOG2);
    state.current_valid_fields += pkt.n;
    state.cities_left -= pkt.n;
}

void inline process_next_packet(
    BeatConsumerState &state,
    IndividualAsBeats &dst_ind,
    FieldIndex num_cities
) {
    #pragma HLS INLINE
    BeatIndex dst_beat = get_beat_idx(state.curr_dst_idx);
    FieldIndex dst_offset = state.curr_dst_idx & (FIELDS_PER_BEAT - 1);

    FieldIndex space_in_beat = FIELDS_PER_BEAT - dst_offset;
    FieldIndex cities_until_wrap = num_cities - state.curr_dst_idx + 1;

    FieldIndex fields_this_beat = space_in_beat;
    if (fields_this_beat > cities_until_wrap) fields_this_beat = cities_until_wrap;
    if (fields_this_beat > state.current_valid_fields) fields_this_beat = state.current_valid_fields;   

    // Commit current beat and pre-load target beat from RAM on beat changes
    if (state.current_beat_idx != dst_beat) {
        if (state.current_beat_idx != MAX_BEATS) {
            //std::cout << "Writing accum_beat to dst_ind at index " << state.current_beat_idx << ": ";
            //print_beat_node_ids(state.accum_beat, get_first_city_index(state.current_beat_idx), FIELDS_PER_BEAT - 1);
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

    // Extract valid fields and write into local register
    ap_uint<BEAT_WIDTH << 1> raw_chunk = state.window.range((fields_this_beat << FIELD_BITS_LOG2) - 1, 0);
    Beat out_beat = (Beat)(raw_chunk << (dst_offset << FIELD_BITS_LOG2));

    int low_bit = dst_offset << FIELD_BITS_LOG2;
    int high_bit = ((dst_offset + fields_this_beat) << FIELD_BITS_LOG2) - 1;

    // Update accum register
    state.accum_beat.range(high_bit, low_bit) = out_beat.range(high_bit, low_bit);

    // Advance destination index and handle wrap-around
    state.curr_dst_idx += fields_this_beat;
    if (state.curr_dst_idx > num_cities) {
        state.curr_dst_idx = get_first_city_index(0);
    }

    state.window >>= (fields_this_beat << FIELD_BITS_LOG2);
    state.current_valid_fields -= fields_this_beat;
}

void capture_and_propagate(BeatConsumerState &state, hls::stream<BeatPacket> &in_stream, IndividualAsBeats &dst_ind, FieldIndex num_cities){
    #pragma HLS INLINE

    if(state.current_valid_fields > 0){
        process_next_packet(state, dst_ind, num_cities);
    }

    if(state.current_valid_fields < FIELDS_PER_BEAT && state.cities_left > 0){
        capture_next_packet(state, in_stream);
    }

    //std::cout << "State after capture_and_propagate: "
    //          << "current_valid_fields=" << state.current_valid_fields
    //          << ", cities_left=" << state.cities_left
    //          << ", curr_dst_idx=" << state.curr_dst_idx
    //          << ", current_beat_idx=" << state.current_beat_idx
    //          << std::endl;
}

void flush_accumulated_beat(BeatConsumerState &state, IndividualAsBeats &dst_ind) {
    if (state.current_beat_idx != MAX_BEATS) {
        //std::cout << "Flushing accum_beat to dst_ind at index " << state.current_beat_idx << ": ";
        //print_beat_node_ids(state.accum_beat, get_first_city_index(state.current_beat_idx), FIELDS_PER_BEAT - 1);
        dst_ind[state.current_beat_idx] = state.accum_beat;
    }
}

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
        #pragma HLS PIPELINE II=3
        // Instruct HLS that state dependency across iterations can be pipelined
        capture_and_propagate(state, in_stream, dst_ind, num_cities);
    } while (state.current_valid_fields > 0 || state.cities_left > 0);

    // Flush final accumulated beat to RAM after loop ends
    flush_accumulated_beat(state, dst_ind);
}