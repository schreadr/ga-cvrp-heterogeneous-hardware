#include "processing_node.hpp"

// Helper: Pack a City struct into a 21-bit field
static ap_uint<FIELD_BITS> pack_city(const City &c) {
    #pragma HLS INLINE
    ap_uint<FIELD_BITS> val = 0;
    val.range(COORD_BITS-1, 0)   = c.coord_x;
    val.range(2*COORD_BITS-1, COORD_BITS)  = c.coord_y;
    val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) = c.capacity;
    val.range(2*COORD_BITS+CAPACITY_BITS, 2*COORD_BITS+CAPACITY_BITS) = c.is_depot;
    return val;
}

// Helper: Unpack a 21-bit field into a City struct
static City unpack_city(const ap_uint<FIELD_BITS> &val) {
    #pragma HLS INLINE
    City c;
    c.coord_x  = val.range(COORD_BITS-1, 0);
    c.coord_y  = val.range(2*COORD_BITS-1, COORD_BITS);
    c.capacity = val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS);
    c.is_depot = val.range(2*COORD_BITS+CAPACITY_BITS, 2*COORD_BITS+CAPACITY_BITS);
    return c;
}

// Stage 1: Decode stream into BRAM
static void decode_to_bram(hls::stream<axi_word_t> &in_stream, City bram[MAX_CITIES], int num_cities) {
    int total_beats = (num_cities + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    DECODE_LOOP: for (int b = 0; b < total_beats; b++) {
        #pragma HLS PIPELINE II=1
        axi_word_t word = in_stream.read();

        UNPACK_FIELDS: for (int f = 0; f < FIELDS_PER_BEAT; f++) {
            int global_idx = b * FIELDS_PER_BEAT + f;
            int low = f * FIELD_BITS;
            int high = low + FIELD_BITS - 1;

            if (global_idx < num_cities) {
                bram[global_idx] = unpack_city(word.data.range(high, low));
            }
        }
    }
}

// Stage 2: Encode from BRAM to stream
static void encode_from_bram(City bram[MAX_CITIES], hls::stream<axi_word_t> &out_stream, int num_cities, bool is_last_indiv) {
    int total_beats = (num_cities + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    ENCODE_LOOP: for (int b = 0; b < total_beats; b++) {
        #pragma HLS PIPELINE II=1
        ap_uint<AXI_WIDTH> beat_data = 0;

        PACK_FIELDS: for (int f = 0; f < FIELDS_PER_BEAT; f++) {
            int global_idx = b * FIELDS_PER_BEAT + f;
            int low = f * FIELD_BITS;
            int high = low + FIELD_BITS - 1;

            if (global_idx < num_cities) {
                beat_data.range(high, low) = pack_city(bram[global_idx]);
            }
        }

        axi_word_t word;
        word.data = beat_data;
        word.keep = -1; // = 0xFFFFFFFFFFFFFFFF
        word.strb = -1;
        word.last = (is_last_indiv && (b == total_beats - 1)) ? 1 : 0;
        
        out_stream.write(word);
    }
}

// Dataflow Pipeline Wrapper for a single stream channel
static void process_stream(
    hls::stream<axi_word_t> &in,
    hls::stream<axi_word_t> &out,
    int num_individuals,
    int num_cities
) {
    INDIVIDUAL_LOOP: for (int i = 0; i < num_individuals; i++) {
        // Enforce Macro-Architecture Ping-Pong Buffering
        #pragma HLS DATAFLOW

        City bram[MAX_CITIES];
        #pragma HLS BIND_STORAGE variable=bram type=ram_2p impl=bram

        bool is_last = (i == num_individuals - 1);

        decode_to_bram(in, bram, num_cities);
        encode_from_bram(bram, out, num_cities, is_last);
    }
}

// Main Top-Level Function
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

    #pragma HLS INTERFACE ap_ctrl_none port=return

    *dbg_num_individuals0 = num_individuals0;
    *dbg_num_individuals1 = num_individuals1;
    *dbg_num_cities = num_cities;

    // Both streams process entirely in parallel (Spatial Parallelism)
    #pragma HLS DATAFLOW
    process_stream(in0, out0, num_individuals0, num_cities);
    process_stream(in1, out1, num_individuals1, num_cities);
}