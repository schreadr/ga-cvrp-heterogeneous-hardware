#include "mutate.hpp"
#include "util.hpp"

#include <cassert>

// -----------------------------------------------------------------------------
// Mutate cut points are provided dynamically via mutate_pairs stream.
// Valid range: 0 < pair.start <= pair.end <= MAX_CITIES (both inclusive).
// -----------------------------------------------------------------------------


static void inline swap(Beat& b, FieldIndex i, FieldIndex j){
    FitnessOrCity tmp = get_field(b, i);
    //std::cout << "Swapping fields " << i << " and " << j << ", values: " << get_node_id(tmp).to_string(10) << " <-> " << get_node_id(get_field(b, j)).to_string(10) << std::endl;
    set_field(b, i, get_field(b, j));
    set_field(b, j, tmp);
}

static void copyAndStream(IndividualAsBeats &p, hls::stream<BeatPacket> &reverse_buffer, hls::stream<Beat> &buffer, FieldIndex num_cities, BeatIndex total_beats, const StartEndData &start_end_data) {
    OX_COPY_UNCHANGED_AND_STREAM_REVERSED_LOOP: for (int i = total_beats - 1; i >= 0; --i) {
        #pragma HLS PIPELINE II=1
        Beat pb = p[i]; 

        if (i < start_end_data.start_beat || i > start_end_data.end_beat) {
            buffer.write(pb);
        } else {
            FieldIndex low_field = (i == start_end_data.start_beat) ? start_end_data.start_field : get_first_city_index(i);
            FieldIndex high_field = (i == start_end_data.end_beat) ? start_end_data.end_field : get_last_field_index(num_cities, i);

            // shift relevant bits to the right s.t. they are on left after swapping
            FieldIndex shift_amount = (FIELDS_PER_BEAT - 1 - high_field) << FIELD_BITS_LOG2;
            pb <<= shift_amount;

            INVERT_LOOP: for (FieldIndex i = 0; i < FIELDS_PER_BEAT/2; i++) {
                #pragma HLS UNROLL
                FieldIndex j=FIELDS_PER_BEAT - 1 - i;
                swap(pb, i, j);
            }

            FieldIndex n_fields = high_field - low_field + 1;

            reverse_buffer.write(BeatPacket{pb, n_fields});
        }
    }
}

static void write(IndividualAsBeats &c, hls::stream<BeatPacket> &reverse_buffer, hls::stream<Beat> &buffer, FieldIndex num_cities, BeatIndex total_beats, FieldIndex n_cities_to_copy, StartEndDataWithBeats &data) {
    BeatConsumerState state(
        n_cities_to_copy,
        data.start_end_data.start_field + (data.start_end_data.start_beat << FIELDS_PER_BEAT_LOG2),
        data
    );

    int i = total_beats - 1;
    ap_uint<2> flushes = 2;

    WRITE_BACK_LOOP: for (int step = 0; step < total_beats + 2; ++step) {
        #pragma HLS PIPELINE II=4
        if (i > data.start_end_data.end_beat) {
            c[i] = buffer.read();
            i--;
        } else if (i >= data.start_end_data.start_beat || flushes > 0) {
            capture_and_propagate(state, reverse_buffer, c, num_cities);
            if (i >= data.start_end_data.start_beat) {
                i--;
            } else {
                flushes--;
            }
        } else {
            c[i] = buffer.read();
            i--;
        }
    }

    flush_accumulated_beat(state, c);
}

static void reverse_order(IndividualAsBeats &p, IndividualAsBeats &c, FieldIndex num_cities, BeatIndex total_beats, FieldIndex n_cities_to_copy, StartEndDataWithBeats &start_end_data) {
    #pragma HLS DATAFLOW
    hls::stream<BeatPacket> reverse_buffer;
    hls::stream<Beat> buffer;

    copyAndStream(p, reverse_buffer, buffer, num_cities, total_beats, start_end_data.start_end_data);
    write(c, reverse_buffer, buffer, num_cities, total_beats, n_cities_to_copy, start_end_data);
}


void mutate_core(
    IndividualAsBeats &p,
    CityPair cut_points,
    IndividualAsBeats &c,
    int num_cities
) {
    #pragma HLS INLINE off

    FieldIndex start_idx = cut_points.start;
    FieldIndex end_idx = cut_points.end;
    const StartEndData &start_end_data = get_start_end_data(start_idx, end_idx);
    StartEndDataWithBeats start_end_data_with_beats(start_end_data, p[start_end_data.start_beat], p[start_end_data.end_beat]);

    set_fitness(c, get_fitness(p));

    FieldIndex n_cities_to_copy = end_idx - start_idx + 1;
    BeatIndex total_beats = get_total_beats(num_cities);
    reverse_order(p, c, num_cities, total_beats, n_cities_to_copy, start_end_data_with_beats);
}

void mutate(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream<CityPair> &mutate_pairs,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    int num_cities
) {
    MUTATE_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> p(in);
        hls::write_lock<IndividualAsBeats> c(out);
        CityPair cut_points = mutate_pairs.read();
        mutate_core(p, cut_points, c, num_cities);
    }
}

