#include "processing_node.hpp"

void process_single_stream(
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

// ----------------------------------------------------------------------
// Top-level: two independent streams
// ----------------------------------------------------------------------
void processing_node(
    hls::stream<axi_word_t> &in0,
    hls::stream<axi_word_t> &out0,
    hls::stream<axi_word_t> &in1,
    hls::stream<axi_word_t> &out1,
    int num_individuals0,
    int num_individuals1,
    int beats_per_individual
) {
// Expose streaming ports as AXI4-Stream
#pragma HLS INTERFACE axis port=in0
#pragma HLS INTERFACE axis port=out0
#pragma HLS INTERFACE axis port=in1
#pragma HLS INTERFACE axis port=out1

// Map control registers to AXI-Lite
#pragma HLS INTERFACE s_axilite port=num_individuals0     offset=0x10 bundle=control
#pragma HLS INTERFACE s_axilite port=num_individuals1     offset=0x18 bundle=control
#pragma HLS INTERFACE s_axilite port=beats_per_individual offset=0x20 bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control

#pragma HLS DATAFLOW
    process_single_stream(in0, out0, num_individuals0, beats_per_individual);
    process_single_stream(in1, out1, num_individuals1, beats_per_individual);
}