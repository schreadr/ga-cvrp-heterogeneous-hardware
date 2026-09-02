#include "crossover.hpp"
#include <cassert>

// -----------------------------------------------------------------------------
// OX crossover cut points (hard-coded for now).
// Valid range: 0 < OX_START <= OX_END <= MAX_CITIES (both inclusive). cities are index from 1 to num_cities.
// -----------------------------------------------------------------------------
#define OX_START 2
#define OX_END   5

void crossover(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
    hls::stream_of_blocks<IndividualAsBeats> &out0,
    hls::stream_of_blocks<IndividualAsBeats> &out1,
    int num_individuals,
    int num_cities
) {
    int total_beats = get_total_beats(num_cities);
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

        int start_beat, start_field;
        get_beat_field(OX_START, start_beat, start_field);
        int end_beat, end_field;
        get_beat_field(OX_END, end_beat, end_field);

        /*std::cout << "--- Individual " << i << " ---\n";
        std::cout << "OX_START=" << OX_START << " -> beat " << start_beat
                  << " field " << start_field << "\n";
        std::cout << "OX_END=" << OX_END << " -> beat " << end_beat
                  << " field " << end_field << "\n";*/

        int itrs = end_beat - start_beat + 1;
        assert(itrs <= MAX_BEATS);
        OX_COPY_BEAT_LOOP: for (int i = 0; i < itrs; i++) {
            #pragma HLS PIPELINE II=1
            int b = i + start_beat;
            int low_field = (b == start_beat) ? start_field : get_first_city_index(b);
            int high_field = (b == end_beat) ? end_field : get_last_field_index(num_cities, b);

            Beat p0b = p0[b];
            Beat p1b = p1[b];

            int inner_itrs = high_field - low_field + 1;
            OX_COPY_FIELD_LOOP: for (int j = 0; j < FIELDS_PER_BEAT; j++) {
                #pragma HLS UNROLL

                if(j < inner_itrs){
                    int f = low_field + j;

                    FitnessOrCity f0 = get_field(p0b, f);
                    set_field(c0, b, f, f0);
                    NodeId id0 = get_node_id(f0);
                    taken0[id0] = 1;
                    /*std::cout << "Copy slice: b=" << b << " f=" << f
                              << " p0 val=" << get_coord_x(f0).to_string(10) << "-" << get_coord_y(f0).to_string(10) << "-" << get_capacity(f0).to_string(10) << "-" << get_is_depot(f0) << "-" << get_node_id(f0).to_string(10) << " "
                              << " id0=" << id0 << " -> c0\n";*/

                    FitnessOrCity f1 = get_field(p1b, f);
                    set_field(c1, b, f, f1);
                    NodeId id1 = get_node_id(f1);
                    taken1[id1] = 1;
                    /*std::cout << "Copy slice: b=" << b << " f=" << f
                              << " p1 val=" << get_coord_x(f1).to_string(10) << "-" << get_coord_y(f1).to_string(10) << "-" << get_capacity(f1).to_string(10) << "-" << get_is_depot(f1) << "-" << get_node_id(f1).to_string(10) << " "
                              << " id1=" << id1 << " -> c1\n";*/
                }
            }
        }

       /* std::cout << "taken0 after slice: " << taken0.to_string(2) << "\n";
        std::cout << "taken1 after slice: " << taken1.to_string(2) << "\n";*/

        int total_beats = get_total_beats(num_cities);

        int fill_start_idx = OX_END+1;
        if (fill_start_idx > num_cities) fill_start_idx -= num_cities;

        int fill_start_beat, fill_start_field;
        get_beat_field(fill_start_idx, fill_start_beat, fill_start_field);

        int w0 = fill_start_idx;
        int w1 = fill_start_idx;

        int fill_beats_to_go = fill_start_beat == end_beat ? total_beats + 1 : total_beats;

        assert(fill_beats_to_go <= MAX_BEATS+1);
        OX_FILL_BEAT_LOOP: for (int step = 0; step < fill_beats_to_go; step++) {
            #pragma HLS PIPELINE II=1
            int read_b = (fill_start_beat + step);
            if (read_b >= total_beats) read_b -= total_beats;

            int f_start = (step == 0) ? fill_start_field : get_first_city_index(read_b);
            int f_end = (step == total_beats) ? end_field : get_last_field_index(num_cities, read_b);

            Beat p0b = p0[read_b];
            Beat p1b = p1[read_b];

            ap_uint<FIELDS_PER_BEAT> valid_cities_f0 = 0;
            FitnessOrCity valid_cities0[FIELDS_PER_BEAT];
            #pragma HLS ARRAY_PARTITION variable=valid_cities0 complete
            int offset0[FIELDS_PER_BEAT];
            #pragma HLS ARRAY_PARTITION variable=offset0 complete
            int total_v0 = 0;

            ap_uint<FIELDS_PER_BEAT> valid_cities_f1 = 0;
            FitnessOrCity valid_cities1[FIELDS_PER_BEAT];
            #pragma HLS ARRAY_PARTITION variable=valid_cities1 complete
            int offset1[FIELDS_PER_BEAT];
            #pragma HLS ARRAY_PARTITION variable=offset1 complete
            int total_v1 = 0;

            int inner_itrs = f_end - f_start + 1;

            /*std::cout << "Fill step " << step << ": read_b=" << read_b
                      << " f_start=" << f_start << " f_end=" << f_end << "\n";*/


            OX_FILL_CHECK_LOOP: for (int i = 0; i < FIELDS_PER_BEAT; i++) {
                #pragma HLS UNROLL

                offset0[i] = total_v0;
                offset1[i] = total_v1;

                if (i < inner_itrs) {
                    int f = i + f_start;
                    FitnessOrCity f1 = get_field(p1b, f);
                    NodeId id1 = get_node_id(f1);
                    /*std::cout << "  Check f=" << f << " p1 id=" << id1
                              << " taken0[id]=" << taken0[id1] << "\n";*/
                    ap_uint<1> is_valid0 = !taken0[id1];
                    total_v0 += is_valid0;
                    valid_cities_f0[i] = is_valid0;
                    valid_cities0[i] = f1;

                    FitnessOrCity f0 = get_field(p0b, f);
                    NodeId id0 = get_node_id(f0);
                    /*std::cout << "  Check f=" << f << " p0 id=" << id0
                              << " taken1[id]=" << taken1[id0] << "\n";*/
                    ap_uint<1> is_valid1 = !taken1[id0];
                    total_v1 += is_valid1;
                    valid_cities_f1[i] = is_valid1;
                    valid_cities1[i] = f0;
                }
            }
            //std::cout << "  v0=" << v0 << " v1=" << v1 << "\n";

            int write_start_beat_c0, write_start_field_c0;
            get_beat_field(w0, write_start_beat_c0, write_start_field_c0);
            int write_last_field_c0 = w0 + total_v0 - 1;
            write_last_field_c0 = (write_last_field_c0 > num_cities) ? (write_last_field_c0 - num_cities) : write_last_field_c0;
            int last_beat_idx_c0 = get_beat_idx(write_last_field_c0);
            int second_beat_idx_c0 = write_start_beat_c0 + 1;
            second_beat_idx_c0 = second_beat_idx_c0 < total_beats ? second_beat_idx_c0 : 0;

            Beat c0b[3];
            #pragma HLS ARRAY_PARTITION variable=c0b complete
            c0b[0] = c0[write_start_beat_c0];
            if(last_beat_idx_c0 != write_start_beat_c0) {
                c0b[1] = c0[second_beat_idx_c0];

                if(last_beat_idx_c0 != second_beat_idx_c0) {
                    c0b[2] = c0[last_beat_idx_c0];
                }
            }

            OX_FILL_WRITE_LOOP0: for (int k = 0; k < FIELDS_PER_BEAT; k++) {
                #pragma HLS UNROLL
                if(valid_cities_f0[k] == 1){
                    /*std::cout << "  Write c0 at global idx " << w0
                          << " val=" << get_coord_x(valid_cities0[k]).to_string(10) << "-" << get_coord_y(valid_cities0[k]).to_string(10) << "-" << get_capacity(valid_cities0[k]).to_string(10) << "-" << get_is_depot(valid_cities0[k]) << "-" << get_node_id(valid_cities0[k]).to_string(10) << " "
                          << " node_id=" << get_node_id(valid_cities0[k]) << "\n";*/
                    int raw_idx = w0 + offset0[k];
                    int wrapped_idx = (raw_idx > num_cities) ? (raw_idx - num_cities) : raw_idx;

                    int write_curr_beat, write_curr_field;
                    get_beat_field(wrapped_idx, write_curr_beat, write_curr_field);

                    Beat &b= write_start_beat_c0 == write_curr_beat ? c0b[0] : (second_beat_idx_c0 == write_curr_beat ? c0b[1] : c0b[2]);
                    set_field(b, write_curr_field, valid_cities0[k]);
                }
            }
            int next_w0 = w0 + total_v0;
            w0 = (next_w0 > num_cities) ? (next_w0 - num_cities) : next_w0;

            c0[write_start_beat_c0] = c0b[0];
            if(last_beat_idx_c0 != write_start_beat_c0) {
                c0[second_beat_idx_c0] = c0b[1];

                if(last_beat_idx_c0 != second_beat_idx_c0) {
                    c0[last_beat_idx_c0] = c0b[2];
                }
            }



            int write_start_beat_c1, write_start_field_c1;
            get_beat_field(w1, write_start_beat_c1, write_start_field_c1);
            int write_last_field_c1 = w1 + total_v1 - 1;
            write_last_field_c1 = (write_last_field_c1 > num_cities) ? (write_last_field_c1 - num_cities) : write_last_field_c1;
            int last_beat_idx_c1 = get_beat_idx(write_last_field_c1);
            int second_beat_idx_c1 = write_start_beat_c1 + 1;
            second_beat_idx_c1 = second_beat_idx_c1 < total_beats ? second_beat_idx_c1 : 0;

            Beat c1b[3];
            #pragma HLS ARRAY_PARTITION variable=c1b complete
            c1b[0] = c1[write_start_beat_c1];
            if(last_beat_idx_c1 != write_start_beat_c1) {
                c1b[1] = c1[second_beat_idx_c1];

                if(last_beat_idx_c1 != second_beat_idx_c1) {
                    c1b[2] = c1[last_beat_idx_c1];
                }
            }

            OX_FILL_WRITE_LOOP1: for (int k = 0; k < FIELDS_PER_BEAT; k++) {
                #pragma HLS UNROLL
                if(valid_cities_f1[k] == 1){
                    /* std::cout << "  Write c1 at global idx " << w1
                          << " val=" << get_coord_x(valid_cities1[k]).to_string(10) << "-" << get_coord_y(valid_cities1[k]).to_string(10) << "-" << get_capacity(valid_cities1[k]).to_string(10) << "-" << get_is_depot(valid_cities1[k]) << "-" << get_node_id(valid_cities1[k]).to_string(10) << " "
                          << " node_id=" << get_node_id(valid_cities1[k]) << "\n";*/
                    int raw_idx = w1 + offset1[k];
                    int wrapped_idx = (raw_idx > num_cities) ? (raw_idx - num_cities) : raw_idx;

                    int write_curr_beat, write_curr_field;
                    get_beat_field(wrapped_idx, write_curr_beat, write_curr_field);

                    Beat &b= write_start_beat_c1 == write_curr_beat ? c1b[0] : (second_beat_idx_c1 == write_curr_beat ? c1b[1] : c1b[2]);
                    set_field(b, write_curr_field, valid_cities1[k]);
                }
            }
            int next_w1 = w1 + total_v1;
            w1 = (next_w1 > num_cities) ? (next_w1 - num_cities) : next_w1;
            
            c1[write_start_beat_c1] = c1b[0];
            if(last_beat_idx_c1 != write_start_beat_c1) {
                c1[second_beat_idx_c1] = c1b[1];

                if(last_beat_idx_c1 != second_beat_idx_c1) {
                    c1[last_beat_idx_c1] = c1b[2];
                }
            }
        }
        //std::cout << "--- End individual " << i << " ---" << std::endl;
    }
}
