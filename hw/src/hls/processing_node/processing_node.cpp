#include "processing_node.hpp"
#include "crossover.hpp"
#include "mutate.hpp"
#include <hls_streamofblocks.h>

// added these for compilation with coyote
#ifndef CSIM_BUILD

#include "util.cpp"
#include "crossover.cpp"
#include "mutate.cpp"

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

static void process_stream(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals,
    FieldIndex num_cities
) {
    #pragma HLS DATAFLOW

    hls::stream_of_blocks<IndividualAsBeats> stream_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_1;
    hls::stream_of_blocks<IndividualAsBeats> stream_cx_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_cx_1;
    hls::stream_of_blocks<IndividualAsBeats> stream_mut_0;
    hls::stream_of_blocks<IndividualAsBeats> stream_mut_1;

    producer(in0, stream_0, num_individuals, num_cities);
    producer(in1, stream_1, num_individuals, num_cities);
    crossover(stream_0, stream_1, stream_cx_0, stream_cx_1, num_individuals, num_cities);
    mutate(stream_cx_0, stream_cx_1, stream_mut_0, stream_mut_1, num_individuals, num_cities);
    consumer(stream_mut_0, out0, num_individuals, num_cities);
    consumer(stream_mut_1, out1, num_individuals, num_cities);
}

static void drive_debug_probes(
    int num_individuals0, int num_individuals1, int num_cities,
    int *dbg0, int *dbg1, int *dbg_num_cities
) {
    *dbg0 = num_individuals0;
    *dbg1 = num_individuals1;
    *dbg_num_cities = num_cities;
}

void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals0,
    int num_individuals1,
    int num_cities,
    int *dbg_num_individuals0,
    int *dbg_num_individuals1,
    int *dbg_num_cities
) {
    #pragma HLS INTERFACE axis port=in0
    #pragma HLS INTERFACE axis port=out0
    #pragma HLS INTERFACE axis port=in1
    #pragma HLS INTERFACE axis port=out1

    #pragma HLS INTERFACE s_axilite port=num_individuals0 offset=0x10 bundle=control
    #pragma HLS INTERFACE s_axilite port=num_individuals1 offset=0x18 bundle=control
    #pragma HLS INTERFACE s_axilite port=num_cities offset=0x20 bundle=control

    #pragma HLS INTERFACE ap_vld port=dbg_num_individuals0
    #pragma HLS INTERFACE ap_vld port=dbg_num_individuals1
    #pragma HLS INTERFACE ap_vld port=dbg_num_cities

    #pragma HLS INTERFACE s_axilite port=return bundle=control

    #pragma HLS DATAFLOW
    drive_debug_probes(num_individuals0, num_individuals1, num_cities,
                       dbg_num_individuals0, dbg_num_individuals1, dbg_num_cities);
    process_stream(in0, out0, in1, out1, num_individuals0, FieldIndex(num_cities));
}
