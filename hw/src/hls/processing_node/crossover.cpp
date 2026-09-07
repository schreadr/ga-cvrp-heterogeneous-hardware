#include "crossover.hpp"
#include "util.hpp"

#include <cassert>
#include <hls_stream.h>

// -----------------------------------------------------------------------------
// OX crossover cut points (hard-coded for now).
// Valid range: 0 < OX_START <= OX_END <= MAX_CITIES (both inclusive). cities are index from 1 to num_cities.
// -----------------------------------------------------------------------------
#define OX_START 2
#define OX_END   2 + FIELDS_PER_BEAT*2

struct UnusedCitiesBeat {
    ap_uint<FIELDS_PER_BEAT> valid_cities_f = 0;
    FitnessOrCity valid_cities[FIELDS_PER_BEAT];
    FieldIndex offset[FIELDS_PER_BEAT];
    BeatIndex n = 0;
};

void find_unsued_cities(IndividualAsBeats &p, hls::stream<UnusedCitiesBeat> &out, StartEndData &start_end_data_unused_city_search, FieldIndex num_cities, BeatIndex total_beats, FieldIndex n_cities_to_find, ap_uint<MAX_CITIES> &taken){
    BeatIndex fill_beats_to_go = start_end_data_unused_city_search.start_beat == start_end_data_unused_city_search.end_beat ? BeatIndex(total_beats + BeatIndex(1)) : BeatIndex(total_beats);

    assert(fill_beats_to_go <= MAX_BEATS+1);
    FieldIndex found = 0;
    FIND_UNSED_CITIES_LOOP: for (BeatIndex step = 0; step < fill_beats_to_go && found < n_cities_to_find; step++) {
        #pragma HLS PIPELINE II=FIELDS_PER_BEAT/2
        BeatIndex read_b = (start_end_data_unused_city_search.start_beat + step);
        if (read_b >= total_beats) read_b -= total_beats;

        FieldIndex f_start = (step == 0) ? start_end_data_unused_city_search.start_field : get_first_city_index(read_b);
        FieldIndex f_end = (step == fill_beats_to_go - 1) ? start_end_data_unused_city_search.end_field : get_last_field_index(num_cities, read_b);

        Beat b = p[read_b];

        UnusedCitiesBeat unused_cities_beat;
        #pragma HLS ARRAY_PARTITION variable=unused_cities_beat.valid_cities complete
        #pragma HLS ARRAY_PARTITION variable=unused_cities_beat.offset complete

        FieldIndex inner_itrs = f_end - f_start + 1;

        /*std::cout << "Fill step " << step << ": read_b=" << read_b
                  << " f_start=" << f_start << " f_end=" << f_end << "\n";*/

        FIND_UNSED_CITIES_LOOP_INNER: for (FieldIndex i = 0; i < FIELDS_PER_BEAT; i++) {
            #pragma HLS UNROLL FACTOR=2

            unused_cities_beat.offset[i] = unused_cities_beat.n;

            if (i < inner_itrs) {
                FieldIndex f_idx = i + f_start;
                FitnessOrCity f = get_field(b, f_idx);
                NodeId id = get_node_id(f);
                /*std::cout << "  Check f=" << f << " p1 id=" << id1
                          << " taken0[id]=" << taken0[id1] << "\n";*/
                ap_uint<1> is_valid = !taken[id];
                unused_cities_beat.n += is_valid;
                unused_cities_beat.valid_cities_f[i] = is_valid;
                unused_cities_beat.valid_cities[i] = f;
            }
        }

        found += unused_cities_beat.n;
        out.write(unused_cities_beat);
    }
}


void compact_unused_cities(hls::stream<UnusedCitiesBeat> &in, hls::stream<BeatPacket> &out, StartEndData &start_end_data_unused_city_search, BeatIndex total_beats, FieldIndex n_cities_to_find){
    BeatIndex num_iters = start_end_data_unused_city_search.start_beat == start_end_data_unused_city_search.end_beat ? BeatIndex(total_beats + BeatIndex(1)) : BeatIndex(total_beats);
    
    FieldIndex found = 0;
    COMPACT_CITIES_LOOP: for(BeatIndex i = 0; i < num_iters && found < n_cities_to_find; i++){
        #pragma HLS PIPELINE II=1
        UnusedCitiesBeat unused_cities_beat = in.read();
        #pragma HLS ARRAY_PARTITION variable=unused_cities_beat.valid_cities complete
        #pragma HLS ARRAY_PARTITION variable=unused_cities_beat.offset complete

        BeatPacket bp{};
        bp.n = unused_cities_beat.n;
        found += bp.n;
        
        COMPACT_CITIES_LOOP_INNER: for (FieldIndex k = 0; k < FIELDS_PER_BEAT; k++) {
            #pragma HLS UNROLL
            if(unused_cities_beat.valid_cities_f[k] == 1){
                /*std::cout << "  Write c0 at global idx " << w0
                      << " val=" << get_coord_x(unused_cities_beat.valid_cities[k]).to_string(10) << "-" << get_coord_y(unused_cities_beat.valid_cities[k]).to_string(10) << "-" << get_capacity(unused_cities_beat.valid_cities[k]).to_string(10) << "-" << get_is_depot(unused_cities_beat.valid_cities[k]) << "-" << get_node_id(unused_cities_beat.valid_cities[k]).to_string(10) << " "
                      << " node_id=" << get_node_id(unused_cities_beat.valid_cities[k]) << "\n";*/
                set_field(bp.beat, unused_cities_beat.offset[k], unused_cities_beat.valid_cities[k]);
            }
        }
        out.write(bp);
    }
}

void fill_up_in_circle(StartEndData &start_end_data_unused_city_search, StartEndDataWithBeats &start_end_data_with_beats0, StartEndDataWithBeats &start_end_data_with_beats1, IndividualAsBeats &p0, IndividualAsBeats &p1, IndividualAsBeats &c0, IndividualAsBeats &c1, BeatIndex total_beats, FieldIndex num_cities, FieldIndex start_idx, FieldIndex n_cities_to_insert, ap_uint<MAX_CITIES> &taken0, ap_uint<MAX_CITIES> &taken1){
    #pragma HLS DATAFLOW
    hls::stream<UnusedCitiesBeat> unused_cities_stream0;
    hls::stream<UnusedCitiesBeat> unused_cities_stream1;
    hls::stream<BeatPacket> beat_stream_0;
    hls::stream<BeatPacket> beat_stream_1;

    find_unsued_cities(p1, unused_cities_stream0, start_end_data_unused_city_search, num_cities, total_beats, n_cities_to_insert, taken0);
    find_unsued_cities(p0, unused_cities_stream1, start_end_data_unused_city_search, num_cities, total_beats, n_cities_to_insert, taken1);

    compact_unused_cities(unused_cities_stream0, beat_stream_0, start_end_data_unused_city_search, total_beats, n_cities_to_insert);
    compact_unused_cities(unused_cities_stream1, beat_stream_1, start_end_data_unused_city_search, total_beats, n_cities_to_insert);

    stream_beats_to_individual(beat_stream_0, c0, start_idx, n_cities_to_insert, start_end_data_with_beats0, total_beats, num_cities);
    stream_beats_to_individual(beat_stream_1, c1, start_idx, n_cities_to_insert, start_end_data_with_beats1, total_beats, num_cities);
}


void crossover(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
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

        // Copy fitness verbatim
        set_fitness(c0, get_fitness(p0));
        set_fitness(c1, get_fitness(p1));

        // Mark nodes already present in the slice
        ap_uint<MAX_CITIES> taken0 = 0;
        ap_uint<MAX_CITIES> taken1 = 0;

        FieldIndex start_idx = OX_START;
        FieldIndex end_idx = OX_END;
        StartEndData start_end_data = get_start_end_data(start_idx, end_idx);

        /*std::cout << "--- Individual " << i << " ---\n";
        std::cout << "OX_START=" << OX_START << " -> beat " << start_beat
                  << " field " << start_field << "\n";
        std::cout << "OX_END=" << OX_END << " -> beat " << end_beat
                  << " field " << start_end_data.end_field << "\n";*/

        BeatIndex itrs = start_end_data.end_beat - start_end_data.start_beat + 1;
        assert(itrs <= MAX_BEATS);
        COPY_BEAT_LOOP: for (BeatIndex i = 0; i < itrs; i++) {
            #pragma HLS PIPELINE II=1
            BeatIndex b = i + start_end_data.start_beat;
            FieldIndex low_field = (b == start_end_data.start_beat) ? start_end_data.start_field : get_first_city_index(b);
            FieldIndex high_field = (b == start_end_data.end_beat) ? start_end_data.end_field : get_last_field_index(num_cities, b);

            Beat p0b = p0[b];
            Beat p1b = p1[b];
            c0[b] = p0b;
            c1[b] = p1b;

            FieldIndex inner_itrs = high_field - low_field + 1;
            OX_COPY_FIELD_LOOP: for (FieldIndex j = 0; j < FIELDS_PER_BEAT; j++) {
                #pragma HLS UNROLL

                if(j < inner_itrs){
                    FieldIndex f = low_field + j;

                    FitnessOrCity f0 = get_field(p0b, f);
                    NodeId id0 = get_node_id(f0);
                    taken0[id0] = 1;
                    /*std::cout << "Copy slice: b=" << b << " f=" << f
                              << " p0 val=" << get_coord_x(f0).to_string(10) << "-" << get_coord_y(f0).to_string(10) << "-" << get_capacity(f0).to_string(10) << "-" << get_is_depot(f0) << "-" << get_node_id(f0).to_string(10) << " "
                              << " id0=" << id0 << " -> c0\n";*/

                    FitnessOrCity f1 = get_field(p1b, f);
                    NodeId id1 = get_node_id(f1);
                    taken1[id1] = 1;
                    /*std::cout << "Copy slice: b=" << b << " f=" << f
                              << " p1 val=" << get_coord_x(f1).to_string(10) << "-" << get_coord_y(f1).to_string(10) << "-" << get_capacity(f1).to_string(10) << "-" << get_is_depot(f1) << "-" << get_node_id(f1).to_string(10) << " "
                              << " id1=" << id1 << " -> c1\n";*/
                }
            }
        }

        StartEndData start_end_data_unused_city_search = get_start_end_data(end_idx+1, end_idx);
        StartEndData start_end_data_circle = get_start_end_data(end_idx+1, start_idx-1);
        StartEndDataWithBeats start_end_data_with_beats0(start_end_data_circle, p0[start_end_data_circle.start_beat], p0[start_end_data_circle.end_beat]);
        StartEndDataWithBeats start_end_data_with_beats1(start_end_data_circle, p1[start_end_data_circle.start_beat], p1[start_end_data_circle.end_beat]);

        BeatIndex total_beats = get_total_beats(num_cities);
        fill_up_in_circle(start_end_data_unused_city_search, start_end_data_with_beats0, start_end_data_with_beats1, p0, p1,c0, c1, total_beats, num_cities, end_idx+1, num_cities - (end_idx - start_idx + 1), taken0, taken1);
    }
}
