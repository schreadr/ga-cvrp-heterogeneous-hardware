#include "processing_node.hpp"
#include <iostream>
#include <vector>

// Software equivalent function to generate dummy streams
void tb_generate_input(hls::stream<axi_word_t> &stream, int num_indiv, int num_cities, int stream_id) {
    int total_beats = (num_cities + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;

    for (int i = 0; i < num_indiv; i++) {
        for (int b = 0; b < total_beats; b++) {
            axi_word_t word;
            word.data = 0;
            word.keep = -1;
            word.strb = -1;
            word.last = (i == num_indiv - 1 && b == total_beats - 1) ? 1 : 0;

            for (int f = 0; f < FIELDS_PER_BEAT; f++) {
                int global_idx = b * FIELDS_PER_BEAT + f;
                if (global_idx < num_cities) {
                    ap_uint<FIELD_BITS> packed = 0;
                    // Generate deterministic but unique values for verification
                    packed.range(6, 0)   = (global_idx + i + stream_id) % 128; // coord_x
                    packed.range(13, 7)  = (global_idx * 2) % 128;             // coord_y
                    packed.range(19, 14) = (global_idx * 3) % 64;              // capacity
                    packed.range(20, 20) = (global_idx == 0) ? 1 : 0;          // is_depot
                    
                    int low = f * FIELD_BITS;
                    int high = low + FIELD_BITS - 1;
                    word.data.range(high, low) = packed;
                }
            }
            stream.write(word);
        }
    }
}

// Software equivalent function to read and verify output
int tb_verify_output(hls::stream<axi_word_t> &stream, int num_indiv, int num_cities, int stream_id) {
    int total_beats = (num_cities + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;
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
                std::cout << "Error: TLAST signal mismatch in stream " << stream_id << std::endl;
                errors++;
            }

            for (int f = 0; f < FIELDS_PER_BEAT; f++) {
                int global_idx = b * FIELDS_PER_BEAT + f;
                if (global_idx < num_cities) {
                    int low = f * FIELD_BITS;
                    int high = low + FIELD_BITS - 1;
                    ap_uint<FIELD_BITS> packed = word.data.range(high, low);

                    int exp_x = (global_idx + i + stream_id) % 128;
                    int exp_y = (global_idx * 2) % 128;
                    int exp_cap = (global_idx * 3) % 64;
                    int exp_dep = (global_idx == 0) ? 1 : 0;

                    if (packed.range(6, 0) != exp_x ||
                        packed.range(13, 7) != exp_y ||
                        packed.range(19, 14) != exp_cap ||
                        packed.range(20, 20) != exp_dep) {
                        std::cout << "Data mismatch stream " << stream_id 
                                  << ", Indiv " << i << ", City " << global_idx << std::endl;
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

    int num_individuals0 = 10;
    int num_individuals1 = 15;
    int num_cities = 500; // Testing with 500 cities (well within max 1536 limit)

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

    // Verify Debug Probes
    if (dbg_indiv0 != num_individuals0 || dbg_indiv1 != num_individuals1 || dbg_cities != num_cities) {
        std::cout << "Error: Debug probes did not latch correct values." << std::endl;
        errors++;
    }

    if (errors == 0) {
        std::cout << "TEST PASSED!" << std::endl;
    } else {
        std::cout << "TEST FAILED with " << errors << " errors." << std::endl;
    }

    return errors;
}