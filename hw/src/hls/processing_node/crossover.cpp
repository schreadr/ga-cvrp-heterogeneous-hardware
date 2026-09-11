#include "crossover.hpp"
#include "util.hpp"

#include <cassert>
#include <hls_stream.h>

void find_unsued_cities(IndividualAsBeats &p, hls::stream<BeatPacket> &out, StartEndData &start_end_data_unused_city_search, FieldIndex num_cities, BeatIndex total_beats, FieldIndex n_cities_to_find, ap_uint<MAX_CITIES> &taken){
    bool wraps_same_beat = (start_end_data_unused_city_search.start_beat == start_end_data_unused_city_search.end_beat &&
                            start_end_data_unused_city_search.start_field > start_end_data_unused_city_search.end_field);
    BeatIndex fill_beats_to_go = wraps_same_beat ? BeatIndex(total_beats + BeatIndex(1)) : BeatIndex(total_beats);

    BeatProducerState out_state;
    FieldIndex found = 0;

    assert(fill_beats_to_go <= MAX_BEATS+1);
    FIND_UNSED_CITIES_LOOP: for (BeatIndex step = 0; step < fill_beats_to_go && found < n_cities_to_find; step++) {
        #pragma HLS PIPELINE II=10
        BeatIndex read_b = (start_end_data_unused_city_search.start_beat + step);
        if (read_b >= total_beats) read_b -= total_beats;

        FieldIndex f_start = (step == 0) ? start_end_data_unused_city_search.start_field : get_first_city_index(read_b);
        FieldIndex f_end = (step == fill_beats_to_go - 1) ? start_end_data_unused_city_search.end_field : get_last_field_index(num_cities, read_b);

        Beat b = p[read_b];

        FIND_UNSED_CITIES_LOOP_INNER: for (FieldIndex i = 0; i < FIELDS_PER_BEAT; i++) {
            FitnessOrCity f = consume_next_field(b);

            if(i >= f_start && i <= f_end) {
                NodeId id = get_node_id(f);
                ap_uint<1> is_valid = !taken[id];
                if (is_valid) {
                    append_field(out_state, f);
                }
            }
        }

        found += out_state.count;
        finalizeStream(out_state, out);
    }
}

// TODO if resources are required consider processing individuals sequentially.
void fill_up_in_circle(StartEndData &start_end_data_unused_city_search, StartEndDataWithBeats &start_end_data_with_beats0, StartEndDataWithBeats &start_end_data_with_beats1, IndividualAsBeats &p0, IndividualAsBeats &p1, IndividualAsBeats &c0, IndividualAsBeats &c1, BeatIndex total_beats, FieldIndex num_cities, FieldIndex start_idx, FieldIndex n_cities_to_insert, ap_uint<MAX_CITIES> &taken0, ap_uint<MAX_CITIES> &taken1){
    #pragma HLS DATAFLOW
    hls::stream<BeatPacket> beat_stream_0;
    hls::stream<BeatPacket> beat_stream_1;

    find_unsued_cities(p1, beat_stream_0, start_end_data_unused_city_search, num_cities, total_beats, n_cities_to_insert, taken0);
    find_unsued_cities(p0, beat_stream_1, start_end_data_unused_city_search, num_cities, total_beats, n_cities_to_insert, taken1);

    stream_beats_to_individual<4>(beat_stream_0, c0, start_idx, n_cities_to_insert, start_end_data_with_beats0, total_beats, num_cities);
    stream_beats_to_individual<4>(beat_stream_1, c1, start_idx, n_cities_to_insert, start_end_data_with_beats1, total_beats, num_cities);
}


void crossover(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream_of_blocks<IndividualAsBeats> &out0,
    hls::stream_of_blocks<IndividualAsBeats> &out1,
    int num_individuals,
    int num_cities
) {
    BeatIndex total_beats = get_total_beats(num_cities);
    CROSSOVER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> p0(in0);
        hls::write_lock<IndividualAsBeats> c0(out0);
        hls::read_lock<IndividualAsBeats> p1(in1);
        hls::write_lock<IndividualAsBeats> c1(out1);

        CityPair cut_points = crossover_pairs.read();

        // Copy fitness verbatim
        set_fitness(c0, get_fitness(p0));
        set_fitness(c1, get_fitness(p1));

        // Mark nodes already present in the slice
        ap_uint<MAX_CITIES> taken0 = 0;
        ap_uint<MAX_CITIES> taken1 = 0;

        FieldIndex start_idx = cut_points.start;
        FieldIndex end_idx = cut_points.end;
        StartEndData start_end_data = get_start_end_data(start_idx, end_idx);

        BeatIndex itrs = start_end_data.end_beat - start_end_data.start_beat + 1;
        assert(itrs <= MAX_BEATS);
        COPY_BEAT_LOOP: for (BeatIndex i = 0; i < itrs; i++) {
            #pragma HLS PIPELINE II=8
            BeatIndex b = i + start_end_data.start_beat;
            FieldIndex low_field = (b == start_end_data.start_beat) ? start_end_data.start_field : get_first_city_index(b);
            FieldIndex high_field = (b == start_end_data.end_beat) ? start_end_data.end_field : get_last_field_index(num_cities, b);

            Beat p0b = p0[b];
            Beat p1b = p1[b];
            c0[b] = p0b;
            c1[b] = p1b;

            OX_COPY_FIELD_LOOP: for (FieldIndex j = 0; j < FIELDS_PER_BEAT; j++) {
                FitnessOrCity f0 = consume_next_field(p0b);
                FitnessOrCity f1 = consume_next_field(p1b);

                if(j >= low_field && j <= high_field) {
                    NodeId id0 = get_node_id(f0);
                    taken0[id0] = 1;

                    NodeId id1 = get_node_id(f1);
                    taken1[id1] = 1;
                }
            }
        }

        FieldIndex next_city = (end_idx == num_cities) ? FieldIndex(1) : FieldIndex(end_idx + 1);
        FieldIndex prev_city = (start_idx == 1) ? FieldIndex(num_cities) : FieldIndex(start_idx - 1);

        StartEndData start_end_data_unused_city_search = get_start_end_data(next_city, end_idx);
        StartEndData start_end_data_circle = get_start_end_data(next_city, prev_city);
        StartEndDataWithBeats start_end_data_with_beats0(start_end_data_circle, p0[start_end_data_circle.start_beat], p0[start_end_data_circle.end_beat]);
        StartEndDataWithBeats start_end_data_with_beats1(start_end_data_circle, p1[start_end_data_circle.start_beat], p1[start_end_data_circle.end_beat]);

        fill_up_in_circle(start_end_data_unused_city_search, start_end_data_with_beats0, start_end_data_with_beats1, p0, p1, c0, c1, total_beats, num_cities, next_city, num_cities - (end_idx - start_idx + 1), taken0, taken1);
    }
}
