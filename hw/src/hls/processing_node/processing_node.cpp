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

void random_number_converter(
    hls::stream<axi_word_t> &in_random,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream<CityPair> &mutate_pairs,
    int num_individuals
) {
    int indiv = 0;
    int total_beats = get_total_random_beats(num_individuals);

    CONVERTER_BEAT_LOOP: for (int b = 0; b < total_beats; b++) {
        axi_word_t word = in_random.read();
        CONVERTER_SLOT_LOOP: for (int slot = 0; slot < INDIVS_PER_RANDOM_BEAT; slot++) {
            if (indiv < num_individuals) {
                RandomNumber c1 = consume_next_random(word.data);
                RandomNumber c2 = consume_next_random(word.data);
                RandomNumber m1 = consume_next_random(word.data);
                RandomNumber m2 = consume_next_random(word.data);

                CityPair cx_pair;
                if (c1 <= c2) {
                    cx_pair.start = c1;
                    cx_pair.end = c2;
                } else {
                    cx_pair.start = c2;
                    cx_pair.end = c1;
                }

                CityPair mut_pair;
                if (m1 <= m2) {
                    mut_pair.start = m1;
                    mut_pair.end = m2;
                } else {
                    mut_pair.start = m2;
                    mut_pair.end = m1;
                }

                crossover_pairs.write(cx_pair);
                mutate_pairs.write(mut_pair);
                indiv++;
            }
        }
    }
}

static void mutate_sequential(
    hls::stream_of_blocks<IndividualAsBeats> &in0,
    hls::stream_of_blocks<IndividualAsBeats> &in1,
    hls::stream<CityPair> &mutate_pairs,
    hls::stream_of_blocks<IndividualAsBeats> &out0,
    hls::stream_of_blocks<IndividualAsBeats> &out1,
    int num_individuals,
    int num_cities
) {
    #pragma HLS ALLOCATION function instances=mutate_core limit=1

    MUTATE_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        CityPair pair = mutate_pairs.read();
        {
            hls::read_lock<IndividualAsBeats> p0(in0);
            hls::write_lock<IndividualAsBeats> c0(out0);
            mutate_core(p0, pair, c0, num_cities);
        }
        {
            hls::read_lock<IndividualAsBeats> p1(in1);
            hls::write_lock<IndividualAsBeats> c1(out1);
            mutate_core(p1, pair, c1, num_cities);
        }
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

    #pragma HLS BIND_STORAGE variable=stream_cx_0 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_cx_1 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_repaired_0 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_repaired_1 type=ram_2p impl=bram latency=2

    hls::stream<CityPair> crossover_pairs("crossover_pairs");
    hls::stream<CityPair> mutate_pairs("mutate_pairs");

    producer(in0, stream_0, num_individuals, num_cities);
    producer(in1, stream_1, num_individuals, num_cities);
    random_number_converter(in_random, crossover_pairs, mutate_pairs, num_individuals);
    crossover(stream_0, stream_1, crossover_pairs, stream_cx_0, stream_cx_1, num_individuals, num_cities);
    mutate_sequential(stream_cx_0, stream_cx_1, mutate_pairs, stream_mut_0, stream_mut_1, num_individuals, num_cities);
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
