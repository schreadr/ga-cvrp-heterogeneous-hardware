#include "processing_node.hpp"
#include <iostream>

// Helper to construct and send packed beats (fitness + cities)
void tb_generate_input(hls::stream<axi_word_t> &stream, int num_indiv, int num_cities, int stream_id) {
    int total_fields = 1 + num_cities;
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    for (int i = 0; i < num_indiv; i++) {
        for (int b = 0; b < total_beats; b++) {
            axi_word_t word;
            word.data = 0;
            word.keep = -1;
            word.strb = -1;
            word.last = (i == num_indiv - 1 && b == total_beats - 1) ? 1 : 0;

            for (int f = 0; f < FIELDS_PER_BEAT; f++) {
                int global_idx = b * FIELDS_PER_BEAT + f;
                if (global_idx == 0) {
                    // Fitness value
                    ap_uint<FIELD_BITS> fitness = (1000 + i + stream_id) % (1 << FIELD_BITS);
                    int low = f * FIELD_BITS;
                    int high = low + FIELD_BITS - 1;
                    word.data.range(high, low) = fitness;
                } else if (global_idx < total_fields) {
                    int city_idx = global_idx - 1;
                    ap_uint<FIELD_BITS> packed = 0;
                    
                    packed.range(COORD_BITS-1, 0)                             = (city_idx + i + stream_id) % (1 << COORD_BITS);
                    packed.range(2*COORD_BITS-1, COORD_BITS)                  = (city_idx * 2) % (1 << COORD_BITS);
                    packed.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) = (city_idx * 3) % (1 << CAPACITY_BITS);
                    packed.range(2*COORD_BITS+CAPACITY_BITS, 2*COORD_BITS+CAPACITY_BITS) = (city_idx == 0) ? 1 : 0;
                    
                    int low = f * FIELD_BITS;
                    int high = low + FIELD_BITS - 1;
                    word.data.range(high, low) = packed;
                }
            }
            stream.write(word);
        }
    }
}

// Helper to read and verify output stream
int tb_verify_output(hls::stream<axi_word_t> &stream, int num_indiv, int num_cities, int stream_id) {
    int total_fields = 1 + num_cities;
    int total_beats = (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;
    int errors = 0;

    for (int i = 0; i < num_indiv; i++) {
        for (int b = 0; b < total_beats; b++) {
            if (stream.empty()) {
                std::cout << "Error: Output stream " << stream_id << " ended prematurely!" << std::endl;
                return 1;
            }

            axi_word_t word = stream.read();
            bool expected_last = (i == num_indiv - 1 && b == total_beats - 1);
            
            if (word.last != expected_last) {
                std::cout << "Error: TLAST mismatch in stream " << stream_id << std::endl;
                errors++;
            }

            for (int f = 0; f < FIELDS_PER_BEAT; f++) {
                int global_idx = b * FIELDS_PER_BEAT + f;
                if (global_idx == 0) {
                    int low = f * FIELD_BITS;
                    int high = low + FIELD_BITS - 1;
                    ap_uint<FIELD_BITS> fitness = word.data.range(high, low);
                    ap_uint<FIELD_BITS> exp_fitness = (1000 + i + stream_id) % (1 << FIELD_BITS);

                    if (fitness != exp_fitness) {
                        std::cout << "Fitness mismatch stream " << stream_id << ", Indiv " << i << std::endl;
                        errors++;
                    }
                } else if (global_idx < total_fields) {
                    int city_idx = global_idx - 1;
                    int low = f * FIELD_BITS;
                    int high = low + FIELD_BITS - 1;
                    ap_uint<FIELD_BITS> packed = word.data.range(high, low);

                    int exp_x   = (city_idx + i + stream_id) % (1 << COORD_BITS);
                    int exp_y   = (city_idx * 2) % (1 << COORD_BITS);
                    int exp_cap = (city_idx * 3) % (1 << CAPACITY_BITS);
                    int exp_dep = (city_idx == 0) ? 1 : 0;

                    if (packed.range(COORD_BITS-1, 0)                             != exp_x ||
                        packed.range(2*COORD_BITS-1, COORD_BITS)                  != exp_y ||
                        packed.range(2*COORD_BITS+CAPACITY_BITS-1, 2*COORD_BITS) != exp_cap ||
                        packed.range(2*COORD_BITS+CAPACITY_BITS, 2*COORD_BITS+CAPACITY_BITS) != exp_dep) {
                        
                        std::cout << "Data mismatch stream " << stream_id 
                                  << ", Indiv " << i << ", City " << city_idx << std::endl;
                        errors++;
                    }
                }
            }
        }
    }
    return errors;
}

int main() {
    hls::stream<axi_word_t> in0("in0"), out0("out0");
    hls::stream<axi_word_t> in1("in1"), out1("out1");

    int num_individuals0 = 100; // Stream large populations with only a 2-individual BRAM buffer
    int num_individuals1 = 200; 
    int num_cities = 500;     

    int dbg_indiv0, dbg_indiv1, dbg_cities;

    std::cout << "Generating test vectors..." << std::endl;
    tb_generate_input(in0, num_individuals0, num_cities, 0);
    tb_generate_input(in1, num_individuals1, num_cities, 1);

    std::cout << "Executing Processing Node..." << std::endl;
    processing_node(in0, out0, in1, out1, 
                    num_individuals0, num_individuals1, num_cities, 
                    &dbg_indiv0, &dbg_indiv1, &dbg_cities);

    std::cout << "Verifying Outputs..." << std::endl;
    int errors = 0;
    errors += tb_verify_output(out0, num_individuals0, num_cities, 0);
    errors += tb_verify_output(out1, num_individuals1, num_cities, 1);

    if (errors == 0) {
        std::cout << "TEST PASSED!" << std::endl;
    } else {
        std::cout << "TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}