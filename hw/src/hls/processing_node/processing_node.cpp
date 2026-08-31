#include "processing_node.hpp"

// Helper: Pack a City struct into a field[cite: 2]
static ap_uint<FIELD_BITS> pack_city(const City &c) {
    #pragma HLS INLINE
    ap_uint<FIELD_BITS> val = 0;
    val.range(COORD_BITS-1, 0)                             = c.coord_x;
    val.range(2*COORD_BITS-1, COORD_BITS)                  = c.coord_y;
    val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) = c.capacity;
    val.range(2*COORD_BITS+CAPACITY_BITS, 2*COORD_BITS+CAPACITY_BITS) = c.is_depot;
    return val;
}

// Helper: Unpack a field into a City struct[cite: 2]
static City unpack_city(const ap_uint<FIELD_BITS> &val) {
    #pragma HLS INLINE
    City c;
    c.coord_x  = val.range(COORD_BITS-1, 0);
    c.coord_y  = val.range(2*COORD_BITS-1, COORD_BITS);
    c.capacity = val.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS);
    c.is_depot = val.range(2*COORD_BITS+CAPACITY_BITS, 2*COORD_BITS+CAPACITY_BITS);
    return c;
}

// Stage 1: Decode beats into a single Individual struct BRAM
static void decode_individual(
    hls::stream<axi_word_t> &in_stream,
    Individual &ind,
    int num_cities
) {
    int total_fields = 1 + num_cities; // Field 0 = fitness, Fields 1..N = cities
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    DECODE_LOOP: for (int b = 0; b < total_beats; b++) {
        #pragma HLS PIPELINE II=1
        axi_word_t word = in_stream.read();

        UNPACK_FIELDS: for (int f = 0; f < FIELDS_PER_BEAT; f++) {
            int global_idx = b * FIELDS_PER_BEAT + f;
            int low = f * FIELD_BITS;
            int high = low + FIELD_BITS - 1;

            if (global_idx == 0) {
                // Field 0 is the individual's fitness
                ind.fitness = word.data.range(high, low);
            } else if (global_idx < total_fields) {
                // Fields 1..num_cities are the cities array
                ind.cities[global_idx - 1] = unpack_city(word.data.range(high, low));
            }
        }
    }
}

// Stage 2: Encode a single Individual struct BRAM to output beats
static void encode_individual(
    const Individual &ind,
    hls::stream<axi_word_t> &out_stream,
    int num_cities,
    bool is_last_indiv
) {
    int total_fields = 1 + num_cities;
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    ENCODE_LOOP: for (int b = 0; b < total_beats; b++) {
        #pragma HLS PIPELINE II=1
        ap_uint<AXI_WIDTH> beat_data = 0;

        PACK_FIELDS: for (int f = 0; f < FIELDS_PER_BEAT; f++) {
            int global_idx = b * FIELDS_PER_BEAT + f;
            int low = f * FIELD_BITS;
            int high = low + FIELD_BITS - 1;

            if (global_idx == 0) {
                beat_data.range(high, low) = ind.fitness;
            } else if (global_idx < total_fields) {
                beat_data.range(high, low) = pack_city(ind.cities[global_idx - 1]);
            }
        }

        axi_word_t word;
        word.data = beat_data;
        word.keep = -1; 
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
        // DATAFLOW inside this loop automatically creates a Ping-Pong buffer
        // of depth 2 for the 'ind' struct across loop iterations.
        #pragma HLS DATAFLOW

        Individual ind;
        #pragma HLS BIND_STORAGE variable=ind.cities type=ram_2p impl=bram
        #pragma HLS ARRAY_PARTITION variable=ind.cities cyclic factor=FIELDS_PER_BEAT dim=1

        bool is_last = (i == num_individuals - 1);

        decode_individual(in, ind, num_cities);
        encode_individual(ind, out, num_cities, is_last);
    }
}

static void drive_debug_probes(
    int num_individuals0, int num_individuals1, int num_cities,
    int *dbg0, int *dbg1, int *dbg_num_cities
) {
    *dbg0 = num_individuals0;
    *dbg1 = num_individuals1;
    *dbg_num_cities = num_cities;
}

// Main Top-Level Function[cite: 2]
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

    // Both stream channels run concurrently in hardware
    #pragma HLS DATAFLOW
    drive_debug_probes(num_individuals0, num_individuals1, num_cities,
                       dbg_num_individuals0, dbg_num_individuals1, dbg_num_cities);
    process_stream(in0, out0, num_individuals0, num_cities);
    process_stream(in1, out1, num_individuals1, num_cities);
}