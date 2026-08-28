/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2025, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * C test bench for the free-running processing_node kernel.
 *
 * Both the top-level function and the swap_stream sub-function contain
 * infinite loops and therefore never return, so the test bench runs the
 * kernel in detached threads, mirroring the free-running hardware:
 *
 *   Test 1: processing_node is launched with a valid configuration given by
 *           its scalar arguments. It must pass the configuration spin-wait
 *           and correctly swap the first batch arriving on stream 0.
 *           NOTE: DATAFLOW is a hardware concept; in plain C the two
 *           swap_stream calls execute sequentially and the first one never
 *           returns, so this instance only serves stream 0.
 *   Test 2: swap_stream, the sub-unit instantiated twice in the DATAFLOW
 *           region, is run once per stream (both threads in parallel, like in
 *           hardware) over several configurations: even/odd numbers of
 *           individuals, single beat, several beats and the maximum number of
 *           beats per individual. Configuration is supplied directly as scalar
 *           arguments (the same AXI4-Lite registers the hardware exposes).
 *   Test 3: with beats_per_individual == 0, the kernel must stay in the
 *           configuration spin-wait and neither consume nor produce data.
 *
 * The configuration scalars are plain C function arguments in simulation
 * (in hardware they are live AXI4-Lite registers), so reconfiguration at
 * runtime cannot be exercised here. TLAST is asserted only on the last beat of
 * the whole batch (not pass-through), matching what Coyote needs to terminate a
 * LOCAL_TRANSFER.
 *
 * The streams are allocated on the heap on purpose: the detached,
 * never-returning kernel threads keep references to them, so they must
 * outlive the test cases (they are reclaimed on process exit).
 *
 * Build and run standalone (no Vivado project required):
 *   g++ -std=c++11 -O2 -pthread -I$XILINX_VITIS_HLS/include \
 *       processing_node.cpp processing_node_tb.cpp -o processing_node_tb
 *
 * As a Vitis HLS C simulation:
 *   open_project <prj>; set_top processing_node
 *   add_files processing_node.cpp
 *   add_files -tb processing_node_tb.cpp
 *   csim_design -csimflags {-pthread} -ldflags {-pthread}
 */

#include "processing_node.hpp"

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <vector>

namespace {

// ----------------------------------------------------------------------
// Test data encoding
// ----------------------------------------------------------------------
// Every beat carries a unique tag in the LSBs of the 512-bit data word:
//   tag = stream[31:28] | individual[27:16] | beat[15:0]
// tkeep/tlast are set to per-beat patterns and must pass through unchanged.
std::uint32_t make_tag(int stream, int individual, int beat) {
    return (static_cast<std::uint32_t>(stream) << 28) |
           (static_cast<std::uint32_t>(individual) << 16) |
           static_cast<std::uint32_t>(beat);
}

std::uint64_t make_keep(int beat) {
    return 0x8000000000000001ull ^ static_cast<std::uint64_t>(beat);
}

// Feed one batch (num_individuals x beats_per_individual beats) into a stream
void produce(hls::stream<axi_word_t> &strm, int stream_id, int num_individuals, int beats_per_individual) {
    axi_word_t word;
    for (int ind = 0; ind < num_individuals; ++ind) {
        for (int b = 0; b < beats_per_individual; ++b) {
            word.data = make_tag(stream_id, ind, b);
            word.keep = make_keep(b);
            word.last = (b == beats_per_individual - 1) ? 1 : 0;
            strm.write(word);
        }
    }
}

// Expected output order of one batch: swapped pairs (i+1, i); an odd trailing
// individual passes through unchanged
std::vector<std::uint32_t> expected_order(int stream_id, int num_individuals, int beats_per_individual) {
    std::vector<std::uint32_t> expected;
    for (int i = 0; i < num_individuals; i += 2) {
        if (i + 1 >= num_individuals) {
            for (int b = 0; b < beats_per_individual; ++b) {
                expected.push_back(make_tag(stream_id, i, b));
            }
            break;
        }
        for (int b = 0; b < beats_per_individual; ++b) {
            expected.push_back(make_tag(stream_id, i + 1, b));
        }
        for (int b = 0; b < beats_per_individual; ++b) {
            expected.push_back(make_tag(stream_id, i, b));
        }
    }
    return expected;
}

// Read one word with a timeout so that a broken kernel does not hang the
// test bench forever
bool read_word(hls::stream<axi_word_t> &strm, axi_word_t &word, const char *name, std::size_t index) {
    for (int spins = 0; strm.empty(); ++spins) {
        if (spins > 500) {
            std::printf("FAIL %s: timed out waiting for output word %zu\n", name, index);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    word = strm.read();
    return true;
}

// Verify one complete batch: swapped order of the tags, tkeep pass-through, and
// TLAST asserted on the very last beat of the whole batch (Coyote terminates the
// LOCAL_TRANSFER on that TLAST).
bool check_stream(hls::stream<axi_word_t> &strm, const char *name,
                  const std::vector<std::uint32_t> &expected, int beats_per_individual) {
    (void)beats_per_individual;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        axi_word_t word;
        if (!read_word(strm, word, name, i)) {
            return false;
        }

        const int b = static_cast<int>(i) % beats_per_individual;
        const std::uint32_t got_tag = word.data.to_uint();
        const std::uint64_t got_keep = word.keep.to_uint64();
        const bool got_last = word.last.to_bool();
        const bool expect_last = (i == expected.size() - 1);

        if (got_tag != expected[i] || got_keep != make_keep(b) || got_last != expect_last) {
            std::printf("FAIL %s word %zu: got tag=0x%08x keep=0x%016llx last=%d, "
                        "expected tag=0x%08x keep=0x%016llx last=%d\n",
                        name, i, got_tag, (unsigned long long)got_keep, got_last ? 1 : 0,
                        expected[i], (unsigned long long)make_keep(b),
                        expect_last ? 1 : 0);
            return false;
        }
    }
    return true;
}

struct Streams {
    hls::stream<axi_word_t> in0, out0, in1, out1;
};

// ----------------------------------------------------------------------
// Test 1: top-level kernel with configuration via scalar arguments
// ----------------------------------------------------------------------
bool run_top_case() {
    const int n0 = 4, n1 = 6, beats = 2;
    Streams *s = new Streams;

    std::thread kernel([s, n0, n1, beats]() {
        processing_node(s->in0, s->out0, s->in1, s->out1, n0, n1, beats);
    });
    kernel.detach();

    produce(s->in0, 0, n0, beats);

    // Stream 1 is not driven in this case: in plain C the DATAFLOW region of
    // processing_node executes the two swap_stream calls sequentially and the
    // first one (stream 0, free-running) never returns
    const bool ok = check_stream(s->out0, "top/stream0", expected_order(0, n0, beats), beats);
    if (ok) {
        std::printf("  PASS top-level: stream 0 swapped with scalar config "
                    "(num_individuals=%d, beats_per_individual=%d)\n", n0, beats);
    }
    return ok;
}

// ----------------------------------------------------------------------
// Test 2: swap_stream DATAFLOW sub-unit, both streams in parallel
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
// Test 2: swap_stream unit (both streams in parallel)
// ----------------------------------------------------------------------
bool run_swap_case(char id, int n0, int n1, int beats) {
    Streams *s = new Streams;

    // swap_stream takes its configuration directly as scalar arguments (in
    // hardware: live AXI4-Lite registers). One free-running instance per stream,
    // mirroring the hardware DATAFLOW region.
    std::thread t0(swap_stream, std::ref(s->in0), std::ref(s->out0), n0, beats);
    std::thread t1(swap_stream, std::ref(s->in1), std::ref(s->out1), n1, beats);
    t0.detach();
    t1.detach();

    produce(s->in0, 0, n0, beats);
    produce(s->in1, 1, n1, beats);

    char name0[32], name1[32];
    std::snprintf(name0, sizeof(name0), "swap%c/stream0", id);
    std::snprintf(name1, sizeof(name1), "swap%c/stream1", id);

    const bool ok0 = check_stream(s->out0, name0, expected_order(0, n0, beats), beats);
    const bool ok1 = check_stream(s->out1, name1, expected_order(1, n1, beats), beats);
    if (ok0 && ok1) {
        std::printf("  PASS swap case %c: both streams swapped "
                    "(num_individuals=%d/%d, beats_per_individual=%d)\n", id, n0, n1, beats);
    }
    return ok0 && ok1;
}

// ----------------------------------------------------------------------
// Test 3: configuration spin-wait with beats_per_individual == 0
// ----------------------------------------------------------------------
bool run_spin_wait_case() {
    const int n0 = 2, n1 = 2;
    Streams *s = new Streams;

    std::thread kernel([s, n0, n1]() {
        processing_node(s->in0, s->out0, s->in1, s->out1, n0, n1, 0);
    });
    kernel.detach();

    // Stage one batch on both streams (with a valid beat count, so that a
    // kernel ignoring the spin-wait would consume it)
    produce(s->in0, 0, n0, 2);
    produce(s->in1, 1, n1, 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (s->in0.size() != 4 || s->in1.size() != 4 || !s->out0.empty() || !s->out1.empty()) {
        std::printf("FAIL spin_wait: kernel consumed/produced data although "
                    "beats_per_individual is zero\n");
        return false;
    }
    std::printf("  PASS spin-wait: kernel waits without touching the data streams\n");
    return true;
}

} // namespace

int main() {
    bool ok = true;

    std::printf("[1/3] Top-level processing_node, configuration via scalar arguments\n");
    ok = run_top_case() && ok;

    std::printf("[2/3] swap_stream DATAFLOW sub-unit (both streams in parallel)\n");
    ok = run_swap_case('A', 4, 6, 2) && ok;                        // even numbers, multiple beats
    ok = run_swap_case('B', 5, 7, 1) && ok;                        // odd numbers, single beat, last pass-through
    ok = run_swap_case('C', 2, 4, MAX_BEATS_PER_INDIVIDUAL) && ok; // maximum beats per individual
    ok = run_swap_case('D', 9, 3, 3) && ok;                        // several pairs plus odd remainder

    std::printf("[3/3] Configuration spin-wait (beats_per_individual == 0)\n");
    ok = run_spin_wait_case() && ok;

    if (!ok) {
        std::printf("FAIL: processing_node test bench\n");
        return EXIT_FAILURE;
    }
    std::printf("PASS: all processing_node tests succeeded\n");
    return EXIT_SUCCESS;
}