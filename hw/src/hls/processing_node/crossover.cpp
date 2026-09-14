#include "crossover.hpp"
#include "util.hpp"

#include <cassert>
#include <hls_stream.h>

void find_unsued_cities(IndividualAsBeats &p, hls::stream<BeatPacket> &out, StartEndData &start_end_data_unused_city_search, CityIndex num_cities, BeatIndex total_city_beats, CityIndex n_cities_to_find, ap_uint<MAX_CITIES> &taken){
    bool wraps_same_beat = (start_end_data_unused_city_search.start_beat == start_end_data_unused_city_search.end_beat &&
                            start_end_data_unused_city_search.start_city > start_end_data_unused_city_search.end_city);
    BeatIndex fill_beats_to_go = wraps_same_beat ? BeatIndex(total_city_beats + BeatIndex(1)) : BeatIndex(total_city_beats);

    BeatProducerState out_state;
    CityIndex found = 0;

    assert(fill_beats_to_go <= MAX_BEATS+1);
    FIND_UNSED_CITIES_LOOP: for (BeatIndex step = 0; step < fill_beats_to_go && found < n_cities_to_find; step++) {
        BeatIndex read_b = (start_end_data_unused_city_search.start_beat + step);
        if (read_b >= total_city_beats) read_b -= total_city_beats;

        CityIndex c_start = (step == 0) ? start_end_data_unused_city_search.start_city : get_first_city_index(read_b);
        CityIndex c_end = (step == fill_beats_to_go - 1) ? start_end_data_unused_city_search.end_city : get_last_city_index(num_cities, read_b);

        Beat b = p[read_b];

        FIND_UNSED_CITIES_LOOP_INNER: for (CityIndex i = 0; i < CITIES_PER_BEAT; i++) {
            #pragma HLS UNROLL FACTOR=8
            City c = consume_next_city(b);

            if(i >= c_start && i <= c_end) {
                NodeId id = get_node_id(c);
                ap_uint<1> is_valid = !taken[id];
                if (is_valid) {
                    append_city(out_state, c);
                }
            }
        }

        found += out_state.count;
        finalizeStream(out_state, out);
    }
}

// TODO if resources are required consider processing individuals sequentially.
void fill_up_in_circle(StartEndData &start_end_data_unused_city_search, StartEndDataWithBeats &start_end_data_with_beats0, StartEndDataWithBeats &start_end_data_with_beats1, IndividualAsBeats &p0, IndividualAsBeats &p1, IndividualAsBeats &c0, IndividualAsBeats &c1, BeatIndex total_city_beats, CityIndex num_cities, CityIndex start_idx, CityIndex n_cities_to_insert, ap_uint<MAX_CITIES> &taken0, ap_uint<MAX_CITIES> &taken1){
    #pragma HLS DATAFLOW
    hls::stream<BeatPacket> beat_stream_0;
    hls::stream<BeatPacket> beat_stream_1;

    find_unsued_cities(p1, beat_stream_0, start_end_data_unused_city_search, num_cities, total_city_beats, n_cities_to_insert, taken0);
    find_unsued_cities(p0, beat_stream_1, start_end_data_unused_city_search, num_cities, total_city_beats, n_cities_to_insert, taken1);

    stream_beats_to_individual<4>(beat_stream_0, c0, start_idx, n_cities_to_insert, start_end_data_with_beats0, total_city_beats, num_cities);
    stream_beats_to_individual<4>(beat_stream_1, c1, start_idx, n_cities_to_insert, start_end_data_with_beats1, total_city_beats, num_cities);
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
    BeatIndex total_city_beats = get_total_city_beats(num_cities);
    CROSSOVER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> p0(in0);
        hls::write_lock<IndividualAsBeats> c0(out0);
        hls::read_lock<IndividualAsBeats> p1(in1);
        hls::write_lock<IndividualAsBeats> c1(out1);

        CityPair cut_points = crossover_pairs.read();

        // Copy fitness verbatim
        set_fitness(c0, get_fitness(p0, num_cities), num_cities);
        set_fitness(c1, get_fitness(p1, num_cities), num_cities);

        // Mark nodes already present in the slice
        ap_uint<MAX_CITIES> taken0 = 0;
        ap_uint<MAX_CITIES> taken1 = 0;

        CityIndex start_idx = cut_points.start;
        CityIndex end_idx = cut_points.end;
        StartEndData start_end_data = get_start_end_data(start_idx, end_idx);

        BeatIndex itrs = start_end_data.end_beat - start_end_data.start_beat + 1;
        assert(itrs <= MAX_BEATS);
        COPY_BEAT_LOOP: for (BeatIndex i = 0; i < itrs; i++) {
            BeatIndex b = i + start_end_data.start_beat;
            CityIndex low_city = (b == start_end_data.start_beat) ? start_end_data.start_city : get_first_city_index(b);
            CityIndex high_city = (b == start_end_data.end_beat) ? start_end_data.end_city : get_last_city_index(num_cities, b);

            Beat p0b = p0[b];
            Beat p1b = p1[b];
            c0[b] = p0b;
            c1[b] = p1b;

            OX_COPY_CITY_LOOP: for (CityIndex j = 0; j < CITIES_PER_BEAT; j++) {
                #pragma HLS UNROLL FACTOR=8
                City c0_val = consume_next_city(p0b);
                City c1_val = consume_next_city(p1b);

                if(j >= low_city && j <= high_city) {
                    NodeId id0 = get_node_id(c0_val);
                    taken0[id0] = 1;

                    NodeId id1 = get_node_id(c1_val);
                    taken1[id1] = 1;
                }
            }
        }

        CityIndex next_city = (end_idx == num_cities - 1) ? CityIndex(0) : CityIndex(end_idx + 1);
        CityIndex prev_city = (start_idx == 0) ? CityIndex(num_cities - 1) : CityIndex(start_idx - 1);

        StartEndData start_end_data_unused_city_search = get_start_end_data(next_city, end_idx);
        StartEndData start_end_data_circle = get_start_end_data(next_city, prev_city);
        StartEndDataWithBeats start_end_data_with_beats0(start_end_data_circle, p0[start_end_data_circle.start_beat], p0[start_end_data_circle.end_beat]);
        StartEndDataWithBeats start_end_data_with_beats1(start_end_data_circle, p1[start_end_data_circle.start_beat], p1[start_end_data_circle.end_beat]);

        fill_up_in_circle(start_end_data_unused_city_search, start_end_data_with_beats0, start_end_data_with_beats1, p0, p1, c0, c1, total_city_beats, num_cities, next_city, num_cities - (end_idx - start_idx + 1), taken0, taken1);
    }
}
