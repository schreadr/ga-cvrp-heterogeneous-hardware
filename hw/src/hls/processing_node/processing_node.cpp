#include "processing_node.hpp"

void swap_stream(
    hls::stream<axi_word_t> &in_stream,
    hls::stream<axi_word_t> &out_stream,
    int num_individuals,
    int beats_per_individual
) {
    // Buffers for two individuals (fixed maximum size)
    axi_word_t first[MAX_BEATS_PER_INDIVIDUAL];
    axi_word_t second[MAX_BEATS_PER_INDIVIDUAL];
    #pragma HLS ARRAY_PARTITION variable=first cyclic factor=8 dim=1
    #pragma HLS ARRAY_PARTITION variable=second cyclic factor=8 dim=1

    // Free-running: process batch after batch of individuals
    while (true) {
        // Process pairs of individuals
        for (int i = 0; i < num_individuals; i += 2) {
            // If odd number, output last individual unchanged
            if (i + 1 >= num_individuals) {
                // Read and immediately write the remaining individual
                for (int b = 0; b < beats_per_individual; b++) {
                    #pragma HLS PIPELINE II=1
                    axi_word_t word = in_stream.read();
                    out_stream.write(word);
                }
                break;
            }

            // Read first individual
            for (int b = 0; b < beats_per_individual; b++) {
                #pragma HLS PIPELINE II=1
                first[b] = in_stream.read();
            }

            // Read second individual
            for (int b = 0; b < beats_per_individual; b++) {
                #pragma HLS PIPELINE II=1
                second[b] = in_stream.read();
            }

            // Output second individual first
            for (int b = 0; b < beats_per_individual; b++) {
                #pragma HLS PIPELINE II=1
                out_stream.write(second[b]);
            }
            // Output first individual second
            for (int b = 0; b < beats_per_individual; b++) {
                #pragma HLS PIPELINE II=1
                out_stream.write(first[b]);
            }
        }
    }
}

// ----------------------------------------------------------------------
// Top-level: two independent, free-running streams
// ----------------------------------------------------------------------
// The configuration registers (num_individuals0, num_individuals1,
// beats_per_individual) are set from software via setCSR(...) and parsed by
// ga_cvrp_axi_ctrl_parser.sv, which forwards them as a single word over the
// cfg AXI4-Stream. The kernel latches this word once, then continuously
// swaps pairs of individuals on both streams (no AP start/stop handshake).
void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    hls::stream<cfg_word_t> &cfg
) {
// Expose streaming ports as AXI4-Stream
#pragma HLS INTERFACE axis port=in0
#pragma HLS INTERFACE axis port=out0
#pragma HLS INTERFACE axis port=in1
#pragma HLS INTERFACE axis port=out1
#pragma HLS INTERFACE axis port=cfg

// Free-running kernel: no AXI-Lite control interface
#pragma HLS INTERFACE ap_ctrl_none port=return

    // Latch the configuration word (blocks until the register parser offers it)
    cfg_word_t cfg_word = cfg.read();
    int num_individuals0     = cfg_word.data(CFG_NUM_IND_0_OFFS + 31, CFG_NUM_IND_0_OFFS);
    int num_individuals1     = cfg_word.data(CFG_NUM_IND_1_OFFS + 31, CFG_NUM_IND_1_OFFS);
    int beats_per_individual = cfg_word.data(CFG_BEATS_PER_IND_OFFS + 31, CFG_BEATS_PER_IND_OFFS);

    // Process the two streams concurrently
#pragma HLS DATAFLOW
    swap_stream(in0, out0, num_individuals0, beats_per_individual);
    swap_stream(in1, out1, num_individuals1, beats_per_individual);
}
