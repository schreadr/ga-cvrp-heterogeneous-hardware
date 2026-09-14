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
    stream_beats_to_individual<4>(in_stream, dst_ind, dst_start_idx, n_cities_to_copy, StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0), n_beats, num_cities);

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
    stream_beats_to_individual<4>(in_stream, dst_ind, dst_start_idx, n_cities_to_copy, StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0), n_beats, num_cities);

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
    int dst_start_idx = FIELDS_PER_BEAT - 2; // Start at the second last field of the first beat
    int n_cities_to_copy = 5; // This will require reading from two packets
    int dst_end_idx = 2;

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
    stream_beats_to_individual<4>(in_stream, dst_ind, dst_start_idx, n_cities_to_copy, StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0), n_beats, num_cities);

    std::vector<int> indices = {dst_start_idx, dst_start_idx + 1, 0, 1, 2};
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

static std::vector<std::pair<FieldIndex, Beat>> drain_stream(hls::stream<BeatPacket> &stream) {
    std::vector<std::pair<FieldIndex, Beat>> packets;
    while (!stream.empty()) {
        BeatPacket pkt = stream.read();
        packets.push_back({pkt.n, pkt.beat});
    }
    return packets;
}

static void print_packet_diff(
    const std::vector<std::pair<FieldIndex, Beat>> &expected,
    const std::vector<std::pair<FieldIndex, Beat>> &received
) {
    std::cout << "Expected " << expected.size() << " packet(s), received " << received.size() << " packet(s)" << std::endl;
    for (size_t i = 0; i < expected.size() || i < received.size(); ++i) {
        if (i < expected.size() && i < received.size()) {
            if (expected[i] != received[i]) {
                std::cout << "  Packet " << i << " mismatch: Expected (n=" << expected[i].first << ", beat=" << std::hex << expected[i].second 
                          << "), got (n=" << received[i].first << ", beat=" << received[i].second << ")" << std::endl;
            }
        } else if (i < expected.size()) {
            std::cout << "  Missing packet " << i << ": Expected (n=" << expected[i].first << ", beat=" << std::hex << expected[i].second << ")" << std::endl;
        } else {
            std::cout << "  Extra packet " << i << ": Got (n=" << received[i].first << ", beat=" << std::hex << received[i].second << ")" << std::endl;
        }
    }
}

static int test_beat_producer_partial() {
    hls::stream<BeatPacket> stream;
    BeatProducerState state;

    send_field(state, 0x111, stream);
    send_field(state, 0x222, stream);
    send_field(state, 0x333, stream);
    finalizeStream(state, stream);

    std::vector<std::pair<FieldIndex, Beat>> expected;
    Beat b = 0;
    set_field(b, 0, 0x111);
    set_field(b, 1, 0x222);
    set_field(b, 2, 0x333);
    expected.push_back({3, b});

    std::vector<std::pair<FieldIndex, Beat>> received = drain_stream(stream);

    bool pass = (received == expected);
    if (!pass) {
        std::cout << "test_beat_producer_partial FAILED!" << std::endl;
        print_packet_diff(expected, received);
        return 1;
    }
    std::cout << "test_beat_producer_partial PASSED successfully!" << std::endl;
    return 0;
}

static int test_beat_producer_full() {
    hls::stream<BeatPacket> stream;
    BeatProducerState state;

    for (int i = 0; i < FIELDS_PER_BEAT; ++i) {
        send_field(state, 0x1000 + i, stream);
    }
    finalizeStream(state, stream);

    std::vector<std::pair<FieldIndex, Beat>> expected;
    Beat b = 0;
    for (int i = 0; i < FIELDS_PER_BEAT; ++i) {
        set_field(b, i, 0x1000 + i);
    }
    expected.push_back({FIELDS_PER_BEAT, b});

    std::vector<std::pair<FieldIndex, Beat>> received = drain_stream(stream);

    bool pass = (received == expected);
    if (!pass) {
        std::cout << "test_beat_producer_full FAILED!" << std::endl;
        print_packet_diff(expected, received);
        return 1;
    }
    std::cout << "test_beat_producer_full PASSED successfully!" << std::endl;
    return 0;
}

static int test_beat_producer_multibeat() {
    hls::stream<BeatPacket> stream;
    BeatProducerState state;

    for (int i = 0; i < 19; ++i) {
        send_field(state, 0x2000 + i, stream);
    }
    finalizeStream(state, stream);

    std::vector<std::pair<FieldIndex, Beat>> expected;
    const int total_fields = 19;
    for (int start = 0; start < total_fields; start += FIELDS_PER_BEAT) {
        int n = std::min(static_cast<int>(FIELDS_PER_BEAT), total_fields - start);
        Beat b = 0;
        for (int i = 0; i < n; ++i) {
            set_field(b, i, 0x2000 + start + i);
        }
        expected.push_back({static_cast<FieldIndex>(n), b});
    }

    std::vector<std::pair<FieldIndex, Beat>> received = drain_stream(stream);

    bool pass = (received == expected);
    if (!pass) {
        std::cout << "test_beat_producer_multibeat FAILED!" << std::endl;
        print_packet_diff(expected, received);
        return 1;
    }
    std::cout << "test_beat_producer_multibeat PASSED successfully!" << std::endl;
    return 0;
}

int test_beat_producer() {
    int errors = 0;
    errors += test_beat_producer_partial();
    errors += test_beat_producer_full();
    errors += test_beat_producer_multibeat();
    return errors;
}

static int test_border_cases() {
    IndividualAsBeats dst_ind;
    hls::stream<BeatPacket> in_stream;

    init_all_0(dst_ind);

    int num_cities = 15;
    int n_beats = get_total_beats(num_cities);

    int dst_start_idx = 13;
    int n_cities_to_copy = 4;
    int dst_end_idx = 1;

    BeatPacket pkt1;
    pkt1.beat = 0;
    set_field(pkt1.beat, 0, 0x111);
    set_field(pkt1.beat, 1, 0x222);
    pkt1.n = 2;

    BeatPacket pkt2;
    pkt2.beat = 0;
    set_field(pkt2.beat, 0, 0x333);
    set_field(pkt2.beat, 1, 0x444);
    pkt2.n = 2;

    in_stream.write(pkt1);
    in_stream.write(pkt2);

    stream_beats_to_individual<4>(in_stream, dst_ind, dst_start_idx, n_cities_to_copy,
                                  StartEndDataWithBeats(get_start_end_data(dst_start_idx, dst_end_idx), 0, 0),
                                  n_beats, num_cities);

    std::vector<int> indices = {13, 14, 0, 1};
    std::vector<int> expected_values = {0x111, 0x222, 0x333, 0x444};

    bool success = true;
    success &= validate(indices, expected_values, dst_ind);
    success &= validate_expect(indices, 0, dst_ind, num_cities);

    if (success) {
        std::cout << "Test Border Cases (Util) PASSED successfully!" << std::endl;
    } else {
        std::cout << "Test Border Cases (Util) FAILED!" << std::endl;
    }

    return success ? 0 : 1;
}

int test_util() {
    int errors = test1();
    errors += test2();
    errors += test3();
    errors += test_integer_sqrt();
    errors += test_beat_producer();
    errors += test_border_cases();
    return errors;
}