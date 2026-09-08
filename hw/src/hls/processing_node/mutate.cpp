#include "mutate.hpp"
#include "util.hpp"

#include <cassert>

// -----------------------------------------------------------------------------
// OX crossover cut points (hard-coded for now).
// Valid range: 0 < OX_START <= OX_END <= MAX_CITIES (both inclusive). cities are index from 1 to num_cities.
// -----------------------------------------------------------------------------
#define MUTATE_START 2
#define MUTATE_END   2 + FIELDS_PER_BEAT*2


static void inline swap(Beat& b, FieldIndex i, FieldIndex j){
    FitnessOrCity tmp = get_field(b, i);
    //std::cout << "Swapping fields " << i << " and " << j << ", values: " << get_node_id(tmp).to_string(10) << " <-> " << get_node_id(get_field(b, j)).to_string(10) << std::endl;
    set_field(b, i, get_field(b, j));
    set_field(b, j, tmp);
}

static void copyAndStream(IndividualAsBeats &p0, IndividualAsBeats &p1, hls::stream<BeatPacket> &reverse_buffer_0, hls::stream<BeatPacket> &reverse_buffer_1, hls::stream<Beat> &buffer_0, hls::stream<Beat> &buffer_1, FieldIndex num_cities, BeatIndex total_beats, const StartEndData &start_end_data) {
    //std::cout << "Copying and streaming from city " << start_city << " to " << end_city << std::endl;

    OX_COPY_UNCHANGED_AND_STREAM_REVERSED_LOOP: for (int i = total_beats - 1; i >= 0; --i) {
        #pragma HLS PIPELINE II=1
        Beat p0b = p0[i]; 
        Beat p1b = p1[i];

        if (i < start_end_data.start_beat || i > start_end_data.end_beat) {
            buffer_0.write(p0b);
            buffer_1.write(p1b);
        } else {
            FieldIndex low_field = (i == start_end_data.start_beat) ? start_end_data.start_field : get_first_city_index(i);
            FieldIndex high_field = (i == start_end_data.end_beat) ? start_end_data.end_field : get_last_field_index(num_cities, i);

            //std::cout << "Value of p0b before swapping: ";
            //print_beat_node_ids(p0b, low_field, high_field);
            //std::cout << "Value of p1b before swapping: ";
            //print_beat_node_ids(p1b, low_field, high_field);
            //std::cout << std::endl;

            // shift relevant bits to the right s.t. they are on left after swapping
            FieldIndex shift_amount = (FIELDS_PER_BEAT - 1 - high_field) << FIELD_BITS_LOG2;
            //std::cout << "Shift amount: " << shift_amount << std::endl;
            p0b <<= shift_amount;
            p1b <<= shift_amount;

            //std::cout << "Value of p0b after shifting: " << p0b.to_string(2) << std::endl;
            //std::cout << "Value of p1b after shifting: " << p1b.to_string(2) << std::endl;

            INVERT_LOOP: for (FieldIndex i = 0; i < FIELDS_PER_BEAT/2; i++) {
                #pragma HLS UNROLL
                FieldIndex j=FIELDS_PER_BEAT - 1 - i;
                swap(p0b, i, j);
                swap(p1b, i, j);
            }

            FieldIndex n_fields = high_field - low_field + 1;

            //std::cout << "Value of p0b after swapping: ";
            //print_beat_node_ids(p0b, 0, n_fields-1);
            //std::cout << "Value of p1b after swapping: ";
            //print_beat_node_ids(p1b, 0, n_fields-1);
            //std::cout << std::endl;

            reverse_buffer_0.write(BeatPacket{p0b, n_fields});
            reverse_buffer_1.write(BeatPacket{p1b, n_fields});
        }
    }
}

static void write(IndividualAsBeats &c, hls::stream<BeatPacket> &reverse_buffer, hls::stream<Beat> &buffer, FieldIndex num_cities, BeatIndex total_beats, FieldIndex n_cities_to_copy, StartEndDataWithBeats &data) {
    BeatConsumerState state(
        n_cities_to_copy,
        data.start_end_data.start_field + (data.start_end_data.start_beat << FIELDS_PER_BEAT_LOG2),
        data
    );

    WRITE_BACK_LOOP: for (int i = total_beats - 1; i >= 0; --i) {
        // if II = 3 timing resolved, TODO evalute performance once more.
        #pragma HLS PIPELINE II=3
        if (i < data.start_end_data.start_beat || i > data.start_end_data.end_beat) {
            c[i] = buffer.read();
        } else {
            capture_and_propagate(state, reverse_buffer, c, num_cities);
        }
    }

    // TODO may just incorprate stream_beats_to_individual to be safe but takes way more time
    capture_and_propagate(state, reverse_buffer, c, num_cities);
    capture_and_propagate(state, reverse_buffer, c, num_cities);
    flush_accumulated_beat(state, c);
}

static void reverse_order(IndividualAsBeats &p0, IndividualAsBeats &p1, IndividualAsBeats &c0, IndividualAsBeats &c1, FieldIndex num_cities, BeatIndex total_beats, FieldIndex n_cities_to_copy, StartEndDataWithBeats &start_end_data0, StartEndDataWithBeats &start_end_data1) {
        #pragma HLS DATAFLOW
        hls::stream<BeatPacket> reverse_buffer_0;
        hls::stream<BeatPacket> reverse_buffer_1;
        hls::stream<Beat> buffer_0;
        hls::stream<Beat> buffer_1;

        copyAndStream(p0, p1, reverse_buffer_0, reverse_buffer_1, buffer_0, buffer_1, num_cities, total_beats, start_end_data0.start_end_data);
        //std::cout << "\nstream 0: " << std::endl;
        write(c0, reverse_buffer_0, buffer_0, num_cities, total_beats, n_cities_to_copy, start_end_data0);
       //std::cout << "\nstream 1: " << std::endl;
        write(c1, reverse_buffer_1, buffer_1, num_cities, total_beats, n_cities_to_copy, start_end_data1);
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

        FieldIndex start_idx = MUTATE_START, end_idx = MUTATE_END;
        const StartEndData &start_end_data = get_start_end_data(start_idx, end_idx);
        StartEndDataWithBeats start_end_data_with_beats0(start_end_data, p0[start_end_data.start_beat], p0[start_end_data.end_beat]);
        StartEndDataWithBeats start_end_data_with_beats1(start_end_data, p1[start_end_data.start_beat], p1[start_end_data.end_beat]);

        set_fitness(c0, get_fitness(p0));
        set_fitness(c1, get_fitness(p1));

        FieldIndex n_cities_to_copy = end_idx - start_idx + 1;
        BeatIndex total_beats = get_total_beats(num_cities);
        reverse_order(p0, p1, c0, c1, num_cities, total_beats, n_cities_to_copy, start_end_data_with_beats0, start_end_data_with_beats1);
    }
}
