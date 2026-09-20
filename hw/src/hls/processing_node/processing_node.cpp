#include "processing_node.hpp"
#include "crossover.hpp"
#include "mutate.hpp"
#include "repair_fitness.hpp"
#include "two_opt.hpp"
#include <hls_streamofblocks.h>

// added these for compilation with coyote
#ifndef CSIM_BUILD

#include "util.cpp"
#include "crossover.cpp"
#include "mutate.cpp"
#include "repair_fitness.cpp"
#include "two_opt.cpp"

#endif

static inline Beat consume_next_hw_beat(
    hls::stream<axi_word_t> &in_stream,
    AxiBeat &current_beat,
    BeatAxiIndex &beats_left
) {
    #pragma HLS INLINE
    if (beats_left == 0) {
        axi_word_t word = in_stream.read();
        current_beat = word.data;
        beats_left = HW_BEATS_PER_AXI_BEAT - 1;
    } else {
        beats_left--;
    }
    return consume_next_hw_beat(current_beat);
}

static void producer(
    hls::stream<axi_word_t> &in_stream,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    CityIndex num_cities
) {
    BeatIndex total_hw_city_beats = get_total_city_beats(num_cities);

    PRODUCER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::write_lock<IndividualAsBeats> ind(out);

        AxiBeat current_beat = 0;
        BeatAxiIndex beats_left = 0;

        DECODE_CITY_BEATS: for (BeatIndex b = 0; b < total_hw_city_beats; b++) {
            #pragma HLS PIPELINE II=1
            ind[b] = consume_next_hw_beat(in_stream, current_beat, beats_left);
        }

        // Consume dedicated fitness beat
        beats_left = 0;
        ind[total_hw_city_beats] = consume_next_hw_beat(in_stream, current_beat, beats_left);
    }
}

static void consumer(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    CityIndex num_cities
) {
    BeatIndex total_hw_city_beats = get_total_city_beats(num_cities);

    CONSUMER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<IndividualAsBeats> ind(in);

        axi_word_t word;
        word.data = 0;
        word.keep = -1;
        word.strb = -1;
        word.last = 0;

        BeatAxiIndex beat_cnt = 0;
        ENCODE_CITY_BEATS: for (BeatIndex b = 0; b < total_hw_city_beats; b++) {
            #pragma HLS PIPELINE II=1
            word.data >>= HW_BEAT_WIDTH;
            word.data.range(AXI_BEAT_WIDTH - 1, AXI_BEAT_WIDTH - HW_BEAT_WIDTH) = ind[b];
            beat_cnt++;

            if (beat_cnt == HW_BEATS_PER_AXI_BEAT) {
                out_stream.write(word);
                word.data = 0;
                beat_cnt = 0;
            }
        }

        // If the last AXI beat is partially filled, right-align it to LSB and flush
        if (beat_cnt != 0) {
            word.data >>= (HW_BEATS_PER_AXI_BEAT - beat_cnt) * HW_BEAT_WIDTH;
            out_stream.write(word);
        }

        // Dedicated fitness beat
        axi_word_t fit_word;
        fit_word.data = 0;
        fit_word.data.range(HW_BEAT_WIDTH - 1, 0) = ind[total_hw_city_beats];
        fit_word.keep = -1;
        fit_word.strb = -1;
        fit_word.last = (i == num_individuals - 1) ? 1 : 0;
        out_stream.write(fit_word);
    }
}

static inline RandomNumber consume_next_random(
    hls::stream<axi_word_t> &in_random,
    AxiBeat &current_beat,
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

static inline CityData consume_next_city_data(
    hls::stream<axi_word_t> &in0,
    CityCacheBeat &current_beat,
    BeatCacheIndex &words_left
) {
    #pragma HLS INLINE
    if (words_left == 0) {
        axi_word_t word = in0.read();
        current_beat = word.data;
        words_left = NODE_CACHE_ENTRIES_PER_BEAT - 1;
    } else {
        words_left--;
    }
    return consume_next_city_data(current_beat);
}

void random_number_converter(
    hls::stream<axi_word_t> &in_random,
    hls::stream<CityPair> &crossover_pairs,
    hls::stream<CityPair> &mutate_pairs_0,
    hls::stream<CityPair> &mutate_pairs_1,
    hls::stream<RandomPair> &two_opt_randoms_0,
    hls::stream<RandomPair> &two_opt_randoms_1,
    int num_individuals,
    CityIndex two_opt_k,
    CityIndex max_subtours
) {
    AxiBeat current_beat = 0;
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

        TwoCityIndex total_two_opt_moves = TwoCityIndex(max_subtours) * TwoCityIndex(two_opt_k);
        TWO_OPT_RND_LOOP: for (TwoCityIndex m = 0; m < total_two_opt_moves; ++m) {
            #pragma HLS PIPELINE II=2
            RandomNumber r0_1 = consume_next_random(in_random, current_beat, words_left);
            RandomNumber r0_2 = consume_next_random(in_random, current_beat, words_left);
            two_opt_randoms_0.write(RandomPair(r0_1, r0_2));

            RandomNumber r1_1 = consume_next_random(in_random, current_beat, words_left);
            RandomNumber r1_2 = consume_next_random(in_random, current_beat, words_left);
            two_opt_randoms_1.write(RandomPair(r1_1, r1_2));
        }
    }
}

static CityData cache_0[MAX_CACHE_ENTRIES];
static CityData cache_1[MAX_CACHE_ENTRIES];
static CityData cache_2opt_0[MAX_CACHE_ENTRIES];
static CityData cache_2opt_1[MAX_CACHE_ENTRIES];

static void init_node_cache(
    hls::stream<axi_word_t> &in0,
    int num_cities
) {
    #pragma HLS BIND_STORAGE variable=cache_0 type=ram_1p impl=bram
    #pragma HLS BIND_STORAGE variable=cache_1 type=ram_1p impl=bram
    #pragma HLS BIND_STORAGE variable=cache_2opt_0 type=ram_2p impl=bram
    #pragma HLS BIND_STORAGE variable=cache_2opt_1 type=ram_2p impl=bram
    CityCacheBeat current_beat = 0;
    BeatCacheIndex words_left = 0;
    int total_entries = num_cities + 1;

    INIT_CACHE_LOOP: for (int city_idx = 0; city_idx < total_entries; ++city_idx) {
        #pragma HLS PIPELINE II=1
        CityData cd = consume_next_city_data(in0, current_beat, words_left);
        cache_0[city_idx] = cd;
        cache_1[city_idx] = cd;
        cache_2opt_0[city_idx] = cd;
        cache_2opt_1[city_idx] = cd;
    }
}

static void process_stream(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    hls::stream<axi_word_t> &in_random,
    int num_individuals,
    CityIndex num_cities,
    Capacity vehicle_capacity,
    CityIndex two_opt_k,
    CityIndex max_subtours
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
    hls::stream_of_blocks<IndividualAsBeats> stream_opt_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_opt_1;

    #pragma HLS BIND_STORAGE variable=stream_cx_0       type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_cx_1       type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_repaired_0 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_repaired_1 type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_mut_0      type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_mut_1      type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_opt_0      type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_opt_1      type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_0          type=ram_2p impl=bram latency=2
    #pragma HLS BIND_STORAGE variable=stream_1          type=ram_2p impl=bram latency=2

    hls::stream<CityPair> crossover_pairs;
    hls::stream<CityPair> mutate_pairs_0;
    hls::stream<CityPair> mutate_pairs_1;
    hls::stream<RandomPair> two_opt_randoms_0;
    #pragma HLS STREAM variable=two_opt_randoms_0 depth=2 * MAX_TWO_OPT_K
    hls::stream<RandomPair> two_opt_randoms_1;
    #pragma HLS STREAM variable=two_opt_randoms_1 depth=2 * MAX_TWO_OPT_K

    producer(in0, stream_0, num_individuals, num_cities);
    producer(in1, stream_1, num_individuals, num_cities);
    random_number_converter(in_random, crossover_pairs, mutate_pairs_0, mutate_pairs_1, two_opt_randoms_0, two_opt_randoms_1, num_individuals, two_opt_k, max_subtours);
    crossover(stream_0, stream_1, crossover_pairs, stream_cx_0, stream_cx_1, num_individuals, num_cities);
    mutate(stream_cx_0, mutate_pairs_0, stream_mut_0, num_individuals, num_cities);
    mutate(stream_cx_1, mutate_pairs_1, stream_mut_1, num_individuals, num_cities);
    repair_fitness(stream_mut_0, stream_repaired_0, num_individuals, num_cities, vehicle_capacity, cache_0);
    repair_fitness(stream_mut_1, stream_repaired_1, num_individuals, num_cities, vehicle_capacity, cache_1);
    two_opt(stream_repaired_0, stream_opt_0, two_opt_randoms_0, num_individuals, num_cities, two_opt_k, max_subtours, cache_2opt_0);
    two_opt(stream_repaired_1, stream_opt_1, two_opt_randoms_1, num_individuals, num_cities, two_opt_k, max_subtours, cache_2opt_1);
    consumer(stream_opt_0, out0, num_individuals, num_cities);
    consumer(stream_opt_1, out1, num_individuals, num_cities);
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
    int init_city_cache,
    int two_opt_k,
    int max_subtours,
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
    #pragma HLS INTERFACE s_axilite port=init_city_cache offset=0x28 bundle=control
    #pragma HLS INTERFACE s_axilite port=two_opt_k offset=0x30 bundle=control
    #pragma HLS INTERFACE s_axilite port=max_subtours offset=0x38 bundle=control

    #pragma HLS INTERFACE ap_vld port=dbg_num_individuals
    #pragma HLS INTERFACE ap_vld port=dbg_num_cities

    #pragma HLS INTERFACE s_axilite port=return bundle=control

    drive_debug_probes(num_individuals, num_cities, dbg_num_individuals, dbg_num_cities);

    if (init_city_cache) {
        init_node_cache(in0, num_cities);
    } else {
        process_stream(in0, out0, in1, out1, in_random, num_individuals, CityIndex(num_cities), Capacity(vehicle_capacity), CityIndex(two_opt_k), CityIndex(max_subtours));
    }
}
