#include "processing_node.hpp"
#include "crossover.hpp"
#include "mutate.hpp"
#include "repair_fitness.hpp"
#include <hls_streamofblocks.h>

// added these for compilation with coyote
#ifndef CSIM_BUILD

#include "util.cpp"
#include "crossover.cpp"
#include "mutate.cpp"
#include "repair_fitness.cpp"

#endif

static void producer(
    hls::stream<axi_word_t> &in_stream,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    FieldIndex num_cities
) {
    BeatIndex total_beats = get_total_beats(num_cities);

    PRODUCER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::write_lock<IndividualAsBeats> ind(out);

        DECODE_BEATS: for (BeatIndex b = 0; b < total_beats; b++) {
            #pragma HLS PIPELINE II=1
            axi_word_t word = in_stream.read();
            ind[b] = word.data;
        }
    }
}

static void consumer(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    FieldIndex num_cities
) {
    BeatIndex total_beats = get_total_beats(num_cities);

    CONSUMER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> ind(in);

        ENCODE_BEATS: for (BeatIndex b = 0; b < total_beats; b++) {
            #pragma HLS PIPELINE II=1
            axi_word_t word;
            word.data = ind[b];
            word.keep = -1;
            word.strb = -1;
            word.last = (i == num_individuals - 1 && b == total_beats - 1) ? 1 : 0;
            out_stream.write(word);
        }
    }
}

static inline RandomNumber consume_next_random(
    hls::stream<axi_word_t> &in_random,
    Beat &current_beat,
    BeatRandomIndex &words_left
) {
    #pragma HLS INLINE
    if (words_left == 0) {
        axi_word_t word = in_random.read();
        current_beat = word.data;
        words_left = RANDOMS_PER_BEAT - 1;
    } else {
        words_left--;
    }
    return consume_next_random(current_beat);
}

void random_number_converter(
    hls::stream<axi_word_t> &in_random,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream<CityPair> &mutate_pairs_0,
    hls::stream<CityPair> &mutate_pairs_1,
    int num_individuals
) {
    Beat current_beat = 0;
    BeatRandomIndex words_left = 0;

    CONVERTER_INDIV_LOOP: for (int indiv = 0; indiv < num_individuals; indiv++) {
        RandomNumber c1   = consume_next_random(in_random, current_beat, words_left);
        RandomNumber c2   = consume_next_random(in_random, current_beat, words_left);
        RandomNumber m0_1 = consume_next_random(in_random, current_beat, words_left);
        RandomNumber m0_2 = consume_next_random(in_random, current_beat, words_left);
        RandomNumber m1_1 = consume_next_random(in_random, current_beat, words_left);
        RandomNumber m1_2 = consume_next_random(in_random, current_beat, words_left);

        CityPair cx_pair;
        if (c1 <= c2) {
            cx_pair.start = c1;
            cx_pair.end = c2;
        } else {
            cx_pair.start = c2;
            cx_pair.end = c1;
        }

        CityPair mut_pair_0;
        if (m0_1 <= m0_2) {
            mut_pair_0.start = m0_1;
            mut_pair_0.end = m0_2;
        } else {
            mut_pair_0.start = m0_2;
            mut_pair_0.end = m0_1;
        }

        CityPair mut_pair_1;
        if (m1_1 <= m1_2) {
            mut_pair_1.start = m1_1;
            mut_pair_1.end = m1_2;
        } else {
            mut_pair_1.start = m1_2;
            mut_pair_1.end = m1_1;
        }

        crossover_pairs.write(cx_pair);
        mutate_pairs_0.write(mut_pair_0);
        mutate_pairs_1.write(mut_pair_1);
    }
}

static void process_stream(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    hls::stream<axi_word_t> &in_random,
    int num_individuals,
    FieldIndex num_cities,
    Capacity vehicle_capacity,
    int depot_cords
) {
    #pragma HLS DATAFLOW

    hls::stream_of_blocks<IndividualAsBeats> stream_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_1;
    hls::stream_of_blocks<IndividualAsBeats> stream_cx_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_cx_1;
    hls::stream_of_blocks<IndividualAsBeats> stream_mut_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_mut_1;
    hls::stream_of_blocks<IndividualAsBeats> stream_repaired_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_repaired_1;

    #pragma HLS BIND_STORAGE variable=stream_cx_0       type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_cx_1       type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_repaired_0 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_repaired_1 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_mut_0      type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_mut_1      type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_0          type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_1          type=ram_2p impl=bram latency=2

    hls::stream<CityPair> crossover_pairs("crossover_pairs");
    hls::stream<CityPair> mutate_pairs_0("mutate_pairs_0");
    hls::stream<CityPair> mutate_pairs_1("mutate_pairs_1");

    #pragma HLS STREAM variable=crossover_pairs depth=16
    #pragma HLS STREAM variable=mutate_pairs_0 depth=16
    #pragma HLS STREAM variable=mutate_pairs_1 depth=16

    producer(in0, stream_0, num_individuals, num_cities);
    producer(in1, stream_1, num_individuals, num_cities);
    random_number_converter(in_random, crossover_pairs, mutate_pairs_0, mutate_pairs_1, num_individuals);
    crossover(stream_0, stream_1, crossover_pairs, stream_cx_0, stream_cx_1, num_individuals, num_cities);
    mutate(stream_cx_0, mutate_pairs_0, stream_mut_0, num_individuals, num_cities);
    mutate(stream_cx_1, mutate_pairs_1, stream_mut_1, num_individuals, num_cities);
    repair_fitness(stream_mut_0, stream_repaired_0, num_individuals, num_cities, vehicle_capacity, depot_cords);
    repair_fitness(stream_mut_1, stream_repaired_1, num_individuals, num_cities, vehicle_capacity, depot_cords);
    consumer(stream_repaired_0, out0, num_individuals, num_cities);
    consumer(stream_repaired_1, out1, num_individuals, num_cities);
}

static void drive_debug_probes(
    int num_individuals,  int num_cities,
    int *dbg_individuals, int *dbg_num_cities
) {
    *dbg_individuals = num_individuals;
    *dbg_num_cities = num_cities;
}

void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    hls::stream<axi_word_t> &in_random,
    int num_individuals,
    int num_cities,
    int vehicle_capacity,
    int depot_cords,
    int *dbg_num_individuals,
    int *dbg_num_cities
) {
    #pragma HLS INTERFACE axis port=in0
    #pragma HLS INTERFACE axis port=out0
    #pragma HLS INTERFACE axis port=in1
    #pragma HLS INTERFACE axis port=out1
    #pragma HLS INTERFACE axis port=in_random

    #pragma HLS INTERFACE s_axilite port=num_individuals offset=0x10 bundle=control
    #pragma HLS INTERFACE s_axilite port=num_cities offset=0x18 bundle=control
    #pragma HLS INTERFACE s_axilite port=vehicle_capacity offset=0x20 bundle=control
    #pragma HLS INTERFACE s_axilite port=depot_cords offset=0x28 bundle=control

    #pragma HLS INTERFACE ap_vld port=dbg_num_individuals
    #pragma HLS INTERFACE ap_vld port=dbg_num_cities

    #pragma HLS INTERFACE s_axilite port=return bundle=control

    #pragma HLS DATAFLOW
    drive_debug_probes(num_individuals, num_cities, dbg_num_individuals, dbg_num_cities);
    process_stream(in0, out0, in1, out1, in_random, num_individuals, FieldIndex(num_cities), Capacity(vehicle_capacity), depot_cords);
}
