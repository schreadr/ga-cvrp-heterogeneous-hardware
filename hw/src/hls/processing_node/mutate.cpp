#include "mutate.hpp"
#include "util.hpp"

#include <cassert>

// -----------------------------------------------------------------------------
// OX crossover cut points (hard-coded for now).
// Valid range: 0 < OX_START <= OX_END <= MAX_CITIES (both inclusive). cities are index from 1 to num_cities.
// -----------------------------------------------------------------------------
#define MUTATE_START 2
#define MUTATE_END   4


static void inline swap(Beat& b, int i, int j){
    FitnessOrCity tmp = get_field(b, i);
    set_field(b, i, get_field(b, j));
    set_field(b, j, tmp);
}

static void copyAndStream(IndividualAsBeats &p0, IndividualAsBeats &p1, hls::stream<BeatPacket> &reverse_buffer_0, hls::stream<BeatPacket> &reverse_buffer_1, hls::stream<Beat> &buffer_0, hls::stream<Beat> &buffer_1, int num_cities, int start_city, int end_city, int total_beats) {
    int start_beat, start_field;
    get_beat_field(start_city, start_beat, start_field);
    int end_beat, end_field;
    get_beat_field(end_city, end_beat, end_field);
    
    OX_COPY_UNCHANGED_AND_STREAM_REVERSED_LOOP: for (int i = total_beats-1; i >= 0; i--) {
        #pragma HLS PIPELINE II=1
        Beat p0b = p0[i]; 
        Beat p1b = p1[i];

        if (i < start_beat || i > end_beat) {
            buffer_0.write(p0b);
            buffer_1.write(p1b);
        } else {
            int low_field = (i == start_beat) ? start_field : get_first_city_index(i);
            int high_field = (i == end_beat) ? end_field : get_last_field_index(num_cities, i);

            p0b >>= (FIELDS_PER_BEAT - 1 - high_field) << FIELD_BITS_LOG2;
            p1b >>= (FIELDS_PER_BEAT - 1 - high_field) << FIELD_BITS_LOG2;

            INVERT_LOOP: for (int i = 0; i < FIELDS_PER_BEAT/2; i++) {
                #pragma HLS UNROLL
                int j=FIELDS_PER_BEAT - 1 - i;
                swap(p0b, i, j);
                swap(p1b, i, j);
            }

            int n_fields = high_field - low_field + 1;
            reverse_buffer_0.write(BeatPacket{p0b, n_fields});
            reverse_buffer_1.write(BeatPacket{p1b, n_fields});
        }
    }
}

static void write(IndividualAsBeats &c, hls::stream<BeatPacket> &reverse_buffer, hls::stream<Beat> &buffer, int start_city, int end_city, int n_cities_to_copy, int total_beats, int num_cities) {
    int start_beat = get_beat_idx(start_city);
    int end_beat = get_beat_idx(end_city);
    BeatConsumerState state(n_cities_to_copy, start_city);

    WRITE_BACK_LOOP: for(int i=0; i < total_beats; i++){
        #pragma HLS PIPELINE II=1
        if(i < start_beat || i > end_beat){
            c[i] = buffer.read();
        }else{
            capture_and_propage(state, reverse_buffer, c, num_cities);
        }
    }

    // since in the first iteration we only pack a beat we need to flush the remaining fields in the state.window)
    capture_and_propage(state, reverse_buffer, c, num_cities);
}

static void reverse_order(IndividualAsBeats &p0, IndividualAsBeats &p1, IndividualAsBeats &c0, IndividualAsBeats &c1, int num_cities, int start_city, int end_city, int n_cities_to_copy, int total_beats) {
        #pragma HLS DATAFLOW
        hls::stream<BeatPacket> reverse_buffer_0;
        hls::stream<BeatPacket> reverse_buffer_1;
        hls::stream<Beat> buffer_0;
        hls::stream<Beat> buffer_1;

        copyAndStream(p0, p1, reverse_buffer_0, reverse_buffer_1, buffer_0, buffer_1, num_cities, start_city, end_city, total_beats);
        write(c0, reverse_buffer_0, buffer_0, start_city, end_city, n_cities_to_copy, total_beats, num_cities);
        write(c1, reverse_buffer_1, buffer_1, start_city, end_city, n_cities_to_copy, total_beats, num_cities);
}


void mutate(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
    hls::stream_of_blocks<IndividualAsBeats> &out0,
    hls::stream_of_blocks<IndividualAsBeats> &out1,
    int num_individuals,
    int num_cities
) {
    MUTATE_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> p0(in0);
        hls::write_lock<IndividualAsBeats> c0(out0);
        hls::read_lock<IndividualAsBeats> p1(in1);
        hls::write_lock<IndividualAsBeats> c1(out1);

        int start_idx = MUTATE_START, end_idx = MUTATE_END;
        int start_beat = get_beat_idx(start_idx);
        int end_beat = get_beat_idx(end_idx);

        get_fitness(c0) = get_fitness(p0);
        get_fitness(c1) = get_fitness(p1);
        c0[start_beat] = p0[start_beat];
        c1[end_beat] = p1[end_beat];

        int n_cities_to_copy = end_idx - start_idx + 1;
        int total_beats = get_total_beats(num_cities);
        reverse_order(p0, p1, c0, c1, num_cities, start_idx, end_idx, n_cities_to_copy, total_beats);
    }
}
