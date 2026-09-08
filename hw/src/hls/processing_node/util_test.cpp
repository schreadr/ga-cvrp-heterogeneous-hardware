#include "util.hpp"
#include "util_test.hpp"

#include <cmath>


static void init_all_0(IndividualAsBeats &ind) {
    for (int i = 0; i < MAX_BEATS; ++i) {
        ind[i] = 0;
    }
}

static bool validate_expect(std::vector<int> &indices_not_to_check, int expected_value, IndividualAsBeats &dst_ind, int num_fields) {
    bool success = true;
    for (int i = 0; i < num_fields; ++i) {
        bool skip = false;
        for (int j = 0; j < indices_not_to_check.size(); ++j) {
            if (i == indices_not_to_check[j]) {
                skip = true;
                break;
            }
        }
        if (skip) continue;


        FitnessOrCity val = get_field(dst_ind, i);
        if (val != expected_value) {
            std::cout << "Test FAILED at index " << i << ": Expected " << std::hex << expected_value << ", got " << val << std::endl;
            success = false;
        }
    }
    return success;
}

static bool validate(std::vector<int> &indices, std::vector<int> &expected_values, IndividualAsBeats &dst_ind) {
    bool success = true;
    for (int i = 0; i < indices.size(); ++i) {
        FitnessOrCity val = get_field(dst_ind, indices[i]);
        if (val != expected_values[i]) {
            std::cout << "Test FAILED at index " << indices[i] << ": Expected " << std::hex << expected_values[i] << ", got " << val << std::endl;
            success = false;
        }
    }
    return success;
}

static int test1(){
    IndividualAsBeats dst_ind;
    hls::stream<BeatPacket> in_stream;

    init_all_0(dst_ind);

    // Define test parameters with an unaligned start index
    int num_cities = FIELDS_PER_BEAT;
    int n_beats = get_total_beats(num_cities);

    int dst_start_idx = 2; 
    int n_cities_to_copy = 5;
    int dst_end_idx = dst_start_idx + n_cities_to_copy - 1;

    // Ensure we don't exceed the number of fields in a beat
    assert(dst_start_idx + n_cities_to_copy <= num_cities); 

    // Populate mock stream packets using field-setting utilities
    BeatPacket pkt1;
    pkt1.beat = 0;
    set_field(pkt1.beat, 0, 0x111);
    set_field(pkt1.beat, 1, 0x222);
    set_field(pkt1.beat, 2, 0x333);
    pkt1.n = 3;

    BeatPacket pkt2;
    pkt2.beat = 0;
    pkt2.n = 0;

    BeatPacket pkt3;
    pkt3.beat = 0;
    set_field(pkt3.beat, 0, 0x444);
    set_field(pkt3.beat, 1, 0x555);
    pkt3.n = 2;

    in_stream.write(pkt1);
    in_stream.write(pkt2);
    in_stream.write(pkt3);

    // Execute the HLS component
    stream_beats_to_individual(in_stream, dst_ind, dst_start_idx, n_cities_to_copy, StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0), n_beats, num_cities);

    std::vector<int> indices = {dst_start_idx, dst_start_idx + 1, dst_start_idx + 2, dst_start_idx + 3, dst_start_idx + 4};
    std::vector<int> expected_values = {0x111, 0x222, 0x333, 0x444, 0x555};

    bool success = true;
    success &= validate(indices, expected_values, dst_ind);
    success &= validate_expect(indices, 0, dst_ind, num_cities);

    if (success) {
        std::cout << "Test 1 PASSED successfully!" << std::endl;
    }

    return success ? 0 : 1;
}

static int test2(){
    IndividualAsBeats dst_ind;
    hls::stream<BeatPacket> in_stream;

    init_all_0(dst_ind);

    int num_cities = FIELDS_PER_BEAT*2;
    int n_beats = get_total_beats(num_cities);

    // Define test parameters with an unaligned start index
    int dst_start_idx = FIELDS_PER_BEAT - 2; // Start at the second last field of the first beat
    int n_cities_to_copy = 5; // This will require reading from two packets
    int dst_end_idx = dst_start_idx + n_cities_to_copy - 1;

    // Populate mock stream packets using field-setting utilities
    BeatPacket pkt1;
    pkt1.beat = 0;
    set_field(pkt1.beat, 0, 0x111);
    set_field(pkt1.beat, 1, 0x222);
    set_field(pkt1.beat, 2, 0x333);
    pkt1.n = 3;

    BeatPacket pkt2;
    pkt2.beat = 0;
    pkt2.n = 0;

    BeatPacket pkt3;
    pkt3.beat = 0;
    set_field(pkt3.beat, 0, 0x444);
    set_field(pkt3.beat, 1, 0x555);
    pkt3.n = 2;

    in_stream.write(pkt1);
    in_stream.write(pkt2);
    in_stream.write(pkt3);

    // Execute the HLS component
    stream_beats_to_individual(in_stream, dst_ind, dst_start_idx, n_cities_to_copy, StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0), n_beats, num_cities);

    std::vector<int> indices = {dst_start_idx, dst_start_idx + 1, dst_start_idx + 2, dst_start_idx + 3, dst_start_idx + 4};
    std::vector<int> expected_values = {0x111, 0x222, 0x333, 0x444, 0x555};

    bool success = true;
    success &= validate(indices, expected_values, dst_ind);
    success &= validate_expect(indices, 0, dst_ind, num_cities);

    if (success) {
        std::cout << "Test 2 PASSED successfully!" << std::endl;
    }

    return success ? 0 : 1;
}

static int test3(){
    IndividualAsBeats dst_ind;
    hls::stream<BeatPacket> in_stream;

    init_all_0(dst_ind);

    int num_cities = FIELDS_PER_BEAT;
    int n_beats = get_total_beats(num_cities);

    // Define test parameters with an unaligned start index
    int dst_start_idx = FIELDS_PER_BEAT - 1; // Start at the second last field of the first beat
    int n_cities_to_copy = 5; // This will require reading from two packets
    int dst_end_idx = 3;

    // Populate mock stream packets using field-setting utilities
    BeatPacket pkt1;
    pkt1.beat = 0;
    set_field(pkt1.beat, 0, 0x111);
    set_field(pkt1.beat, 1, 0x222);
    set_field(pkt1.beat, 2, 0x333);
    pkt1.n = 3;

    BeatPacket pkt2;
    pkt2.beat = 0;
    pkt2.n = 0;

    BeatPacket pkt3;
    pkt3.beat = 0;
    set_field(pkt3.beat, 0, 0x444);
    set_field(pkt3.beat, 1, 0x555);
    pkt3.n = 2;

    in_stream.write(pkt1);
    in_stream.write(pkt2);
    in_stream.write(pkt3);

    // Execute the HLS component
    stream_beats_to_individual(in_stream, dst_ind, dst_start_idx, n_cities_to_copy, StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0), n_beats, num_cities);

    std::vector<int> indices = {dst_start_idx, dst_start_idx + 1, 1, 2, 3};
    std::vector<int> expected_values = {0x111, 0x222, 0x333, 0x444, 0x555};

    bool success = true;
    success &= validate(indices, expected_values, dst_ind);
    success &= validate_expect(indices, 0, dst_ind, num_cities);

    if (success) {
        std::cout << "Test 3 PASSED successfully!" << std::endl;
    }

    return success ? 0 : 1;
}

int test_integer_sqrt() {
    bool success = true;

    for (int i = 0; i < (1 << 8); ++i) {
        int expected = (int) std::floor(std::sqrt(i));
        int result = integer_sqrt<8>(i);
        if (result != expected) {
            std::cout << "Integer sqrt test FAILED for input " << i << ": Expected " << expected << ", got " << result << std::endl;
            success = false;
        }
    }

    if (success) {
        std::cout << "Integer sqrt test PASSED successfully!" << std::endl;
    } else {
        std::cout << "Integer sqrt test FAILED!" << std::endl;
    }

    return success ? 0 : 1;
}

int test_stream_beats_to_individual() {
    int errors = test1();
    errors += test2();
    errors += test3();
    errors += test_integer_sqrt();
    return errors;
}