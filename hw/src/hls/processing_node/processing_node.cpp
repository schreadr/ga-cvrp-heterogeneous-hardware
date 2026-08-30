#include "processing_node.hpp"
#include <hls_print.h>

// Helper to buffer two incoming individuals from AXI-Stream into intermediate FIFOs
static void read_individuals(
    hls::stream<axi_word_t> &in_stream,
    hls::stream<ap_uint<512>> &buf_first,
    hls::stream<ap_uint<512>> &buf_second,
    int num_individuals,
    int beats_per_individual
) {
    for (int i = 0; i < num_individuals; i += 2) {
        // Handle odd trailing individual directly to output buffer
        if (i + 1 >= num_individuals) {
            for (int b = 0; b < beats_per_individual; b++) {
                #pragma HLS PIPELINE II=1
                buf_first.write(in_stream.read().data);
            }
            break;
        }

        // Buffer first individual
        for (int b = 0; b < beats_per_individual; b++) {
            #pragma HLS PIPELINE II=1
            buf_first.write(in_stream.read().data);
        }

        // Buffer second individual
        for (int b = 0; b < beats_per_individual; b++) {
            #pragma HLS PIPELINE II=1
            buf_second.write(in_stream.read().data);
        }
    }
}

// Helper to write buffered individuals in swapped order with TLAST handling
static void write_swapped_individuals(
    hls::stream<ap_uint<512>> &buf_first,
    hls::stream<ap_uint<512>> &buf_second,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int beats_per_individual
) {
    for (int i = 0; i < num_individuals; i += 2) {
        // Odd trailing individual pass-through
        if (i + 1 >= num_individuals) {
            for (int b = 0; b < beats_per_individual; b++) {
                #pragma HLS PIPELINE II=1
                axi_word_t word;
                word.data = buf_first.read();
                word.keep = -1;
                word.strb = -1;
                word.last = (b == beats_per_individual - 1) ? 1 : 0;
                out_stream.write(word);
            }
            break;
        }

        bool last_pair = (i + 2 >= num_individuals);

        // Stream out second individual first (TLAST is forced low)
        for (int b = 0; b < beats_per_individual; b++) {
            #pragma HLS PIPELINE II=1
            axi_word_t word;
            word.data = buf_second.read();
            word.keep = -1;
            word.strb = -1;
            word.last = 0;
            out_stream.write(word);
        }

        // Stream out first individual second (TLAST high on final beat of final pair)
        for (int b = 0; b < beats_per_individual; b++) {
            #pragma HLS PIPELINE II=1
            axi_word_t word;
            word.data = buf_first.read();
            word.keep = -1;
            word.strb = -1;
            word.last = (last_pair && (b == beats_per_individual - 1)) ? 1 : 0;
            out_stream.write(word);
        }
    }
}

// Swap stream kernel supporting Task-Level Dataflow pipelining
void swap_stream(
    hls::stream<axi_word_t> &in_stream,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int beats_per_individual
) {
    // Avoid execution until scalar registers are programmed by host software
    //while (beats_per_individual == 0 || num_individuals == 0) {
    //    #pragma HLS PIPELINE II=1
    //    // Spin-wait on scalar configuration registers (AXI4-Lite)
    //}

    // Stream depth sized to hold up to MAX_BEATS_PER_INDIVIDUAL
    hls::stream<ap_uint<512>> buf_first("buf_first");
    hls::stream<ap_uint<512>> buf_second("buf_second");
    #pragma HLS STREAM variable=buf_first depth=MAX_BEATS_PER_INDIVIDUAL
    #pragma HLS STREAM variable=buf_second depth=MAX_BEATS_PER_INDIVIDUAL

    // Internal task-level dataflow: concurrently read and write stream pairs
    #pragma HLS DATAFLOW
    read_individuals(in_stream, buf_first, buf_second, num_individuals, beats_per_individual);
    write_swapped_individuals(buf_first, buf_second, out_stream, num_individuals, beats_per_individual);
}

static void drive_debug_probes(
    int num0, int num1, int beats,
    int *dbg0, int *dbg1, int *dbg_beats
) {
    *dbg0 = num0;
    *dbg1 = num1;
    *dbg_beats = beats;
}

// Top-Level Processing Node
void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals0,
    int num_individuals1,
    int beats_per_individual,
    // Hardware Debug Probe Ports
    int *dbg_num_individuals0,
    int *dbg_num_individuals1,
    int *dbg_beats_per_individual
) {
    // AXI4-Stream interfaces
    #pragma HLS INTERFACE axis port=in0
    #pragma HLS INTERFACE axis port=out0
    #pragma HLS INTERFACE axis port=in1
    #pragma HLS INTERFACE axis port=out1

    // AXI4-Lite control interface registers
    #pragma HLS INTERFACE s_axilite port=num_individuals0 offset=0x10 bundle=control
    #pragma HLS INTERFACE s_axilite port=num_individuals1 offset=0x18 bundle=control
    #pragma HLS INTERFACE s_axilite port=beats_per_individual offset=0x20 bundle=control

    // Hardware Debug Probes (ap_vld generates dedicated output pins for Vivado ILA)
    #pragma HLS INTERFACE ap_vld port=dbg_num_individuals0
    #pragma HLS INTERFACE ap_vld port=dbg_num_individuals1
    #pragma HLS INTERFACE ap_vld port=dbg_beats_per_individual

    // Free-running kernel execution control
    #pragma HLS INTERFACE ap_ctrl_none port=return

    // Run independent streams concurrently using top-level DATAFLOW (required since free running kernel)
    #pragma HLS DATAFLOW
    drive_debug_probes(num_individuals0, num_individuals1, beats_per_individual,
                       dbg_num_individuals0, dbg_num_individuals1, dbg_beats_per_individual);
    swap_stream(in0, out0, num_individuals0, beats_per_individual);
    swap_stream(in1, out1, num_individuals1, beats_per_individual);
}