#include "processing_node.hpp"
#include <hls_streamofblocks.h>

static void producer(
    hls::stream<axi_word_t> &in_stream,
    hls::stream_of_blocks<Individual> &out,
    int num_individuals,
    int num_cities
) {
    int total_fields = 1 + num_cities;
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    PRODUCER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::write_lock<Individual> ind(out);

        DECODE_BEATS: for (int b = 0; b < total_beats; b++) {
            #pragma HLS PIPELINE II=1
            axi_word_t word = in_stream.read();
            // Store the whole beat as one memory word
            ind[b] = word.data.range(BEAT_WIDTH - 1, 0);
        }
    }
}

static void crossover(
    hls::stream_of_blocks<Individual> &in,
    hls::stream_of_blocks<Individual> &out,
    int num_individuals,
    int num_cities
) {
    int total_fields = 1 + num_cities;
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    CROSSOVER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<Individual> ind(in);
        hls::write_lock<Individual> new_ind(out);

        // Example: simple pass-through (copy all beats)
        // If you need to modify individual fields, you can access them via
        // get_field / set_field, but that would require beat-level processing.
        CROSSOVER_BEATS: for (int b = 0; b < total_beats; b++) {
            #pragma HLS PIPELINE II=1
            new_ind[b] = ind[b];
        }
    }
}

static void consumer(
    hls::stream_of_blocks<Individual> &in,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int num_cities
) {
    int total_fields = 1 + num_cities;
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    CONSUMER_INDIV_LOOP: for (int i = 0; i < num_individuals; i++) {
        hls::read_lock<Individual> ind(in);

        ENCODE_BEATS: for (int b = 0; b < total_beats; b++) {
            #pragma HLS PIPELINE II=1
            axi_word_t word;
            word.data.range(BEAT_WIDTH - 1, 0) = ind[b];
            word.keep = -1;
            word.strb = -1;
            word.last = (i == num_individuals - 1 && b == total_beats - 1) ? 1 : 0;
            out_stream.write(word);
        }
    }
}

static void process_stream(
    hls::stream<axi_word_t> &in,
    hls::stream<axi_word_t> &out,
    int num_individuals,
    int num_cities
) {
    #pragma HLS DATAFLOW

    hls::stream_of_blocks<Individual> A;
    hls::stream_of_blocks<Individual> B;

    producer(in, A, num_individuals, num_cities);
    crossover(A, B, num_individuals, num_cities);
    consumer(B, out, num_individuals, num_cities);
}

static void drive_debug_probes(
    int num_individuals0, int num_individuals1, int num_cities,
    int *dbg0, int *dbg1, int *dbg_num_cities
) {
    #pragma HLS INLINE
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

    // Change to ap_ctrl_hs (or s_axilite) for host control
    #pragma HLS INTERFACE ap_ctrl_none port=return

    #pragma HLS DATAFLOW
    drive_debug_probes(num_individuals0, num_individuals1, num_cities,
                       dbg_num_individuals0, dbg_num_individuals1, dbg_num_cities);
    process_stream(in0, out0, num_individuals0, num_cities);
    process_stream(in1, out1, num_individuals1, num_cities);
}