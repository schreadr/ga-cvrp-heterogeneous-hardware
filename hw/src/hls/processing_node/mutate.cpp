#include "mutate.hpp"
#include "util.hpp"

#include <cassert>

// -----------------------------------------------------------------------------
// Mutate cut points are provided dynamically via mutate_pairs stream.
// Valid range: 0 < pair.start <= pair.end <= MAX_CITIES (both inclusive).
// -----------------------------------------------------------------------------

static void inline swap(Beat& b, CityIndex i, CityIndex j){
    City tmp = get_city(b, i);
    set_city(b, i, get_city(b, j));
    set_city(b, j, tmp);
}

static void stream_reversed_slice(IndividualAsBeats &p, hls::stream<BeatPacket> &reverse_buffer, CityIndex num_cities, const StartEndData &start_end_data) {
    OX_STREAM_REVERSED_LOOP: for (int i = start_end_data.end_beat; i >= (int)start_end_data.start_beat; --i) {
        #pragma HLS PIPELINE II=1
        Beat pb = p[i]; 

        CityIndex low_city = (i == start_end_data.start_beat) ? start_end_data.start_city : get_first_city_index();
        CityIndex high_city = (i == start_end_data.end_beat) ? start_end_data.end_city : get_last_city_index(num_cities, i);

        // shift relevant bits to the right s.t. they are on left after swapping
        CityIndex shift_amount = (HW_CITIES_PER_BEAT - 1 - high_city) << CITY_BITS_LOG2;
        pb <<= shift_amount;

        INVERT_LOOP: for (CityIndex idx = 0; idx < HW_CITIES_PER_BEAT/2; idx++) {
            #pragma HLS UNROLL
            CityIndex j = HW_CITIES_PER_BEAT - 1 - idx;
            swap(pb, idx, j);
        }

        CityIndex n_cities = high_city - low_city + 1;
        reverse_buffer.write(BeatPacket{pb, n_cities});
    }
}

static void reverse_order(IndividualAsBeats &p, IndividualAsBeats &c, CityIndex num_cities, CityIndex start_idx, CityIndex n_cities_to_copy, StartEndDataWithBeats &start_end_data) {
    #pragma HLS DATAFLOW
    hls::stream<BeatPacket> reverse_buffer("reverse_buffer");
    #pragma HLS STREAM variable=reverse_buffer depth=4

    stream_reversed_slice(p, reverse_buffer, num_cities, start_end_data.start_end_data);
    stream_beats_to_individual<BEAT_STREAM_INTERVAL>(reverse_buffer, c, start_idx, n_cities_to_copy, start_end_data, num_cities);
}


void mutate_core(
    IndividualAsBeats &p,
    CityPair cut_points,
    IndividualAsBeats &c,
    CityIndex num_cities
) {
    CityIndex start_idx = cut_points.start;
    CityIndex end_idx = cut_points.end;
    const StartEndData &start_end_data = get_start_end_data(start_idx, end_idx);
    StartEndDataWithBeats start_end_data_with_beats(start_end_data, p[start_end_data.start_beat], p[start_end_data.end_beat]);

    set_fitness(c, get_fitness(p, num_cities), num_cities);

    CityIndex n_cities_to_copy = end_idx - start_idx + 1;
    BeatIndex total_city_beats = get_total_city_beats(num_cities);

    COPY_PREFIX: for (BeatIndex i = 0; i < start_end_data.start_beat; i++) {
        #pragma HLS PIPELINE II=1
        c[i] = p[i];
    }

    COPY_SUFFIX: for (BeatIndex i = start_end_data.end_beat + 1; i < total_city_beats; i++) {
        #pragma HLS PIPELINE II=1
        c[i] = p[i];
    }

    reverse_order(p, c, num_cities, start_idx, n_cities_to_copy, start_end_data_with_beats);
}

void mutate(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream<CityPair> &mutate_pairs,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    CityIndex num_cities,
    Probability mutation_prob
) {
    MUTATE_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> p(in);
        hls::write_lock<IndividualAsBeats> c(out);
        CityPair cut_points = mutate_pairs.read();
        CityPair eff_cut_points;
        if (cut_points.prob_rnd <= mutation_prob) {
            eff_cut_points = cut_points;
        } else {
            eff_cut_points = CityPair(0, 0, cut_points.prob_rnd);
        }
        mutate_core(p, eff_cut_points, c, num_cities);
    }
}
