/**
 * Standalone unit and integration tests for the asynchronous multi-vFPGA pipeline.
 * Tests SafeQueue and run_ga_pipelined with simulated worker latency.
 * No hardware or Coyote driver required.
 *
 * Build & Run:
 *   g++ -std=c++17 -I . -pthread test_async_pipeline.cpp ga.cpp ga_streaming.cpp problem_parser.cpp -o test_async_pipeline && ./test_async_pipeline
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "ga.hpp"
#include "ga_streaming.hpp"
#include "safe_queue.hpp"
#include "problem_parser.hpp"

namespace {

void test_safe_queue_basic() {
    std::cout << "Test 1: SafeQueue basic push/pop and shutdown...\n";
    SafeQueue<int> q;
    q.push(10);
    q.push(20);

    auto v1 = q.pop();
    auto v2 = q.pop();

    if (!v1 || *v1 != 10 || !v2 || *v2 != 20) {
        std::cerr << "FAIL: SafeQueue basic push/pop mismatch.\n";
        std::exit(EXIT_FAILURE);
    }

    // Test shutdown unblocks pop
    std::thread consumer([&q]() {
        auto val = q.pop();
        if (val.has_value()) {
            std::cerr << "FAIL: expected nullopt on shutdown.\n";
            std::exit(EXIT_FAILURE);
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    q.shutdown();
    consumer.join();
    std::cout << "PASS [test_safe_queue_basic]\n";
}

void test_safe_queue_multithreaded() {
    std::cout << "Test 2: SafeQueue multi-producer multi-consumer stress test...\n";
    SafeQueue<int> q;
    constexpr int n_items = 1000;
    std::atomic<int> sum_consumed{0};

    std::vector<std::thread> producers;
    for (int p = 0; p < 4; ++p) {
        producers.emplace_back([&q, p]() {
            for (int i = 0; i < n_items; ++i) {
                q.push(1);
            }
        });
    }

    std::vector<std::thread> consumers;
    for (int c = 0; c < 4; ++c) {
        consumers.emplace_back([&q, &sum_consumed]() {
            while (true) {
                auto val = q.pop();
                if (!val) break;
                sum_consumed += *val;
            }
        });
    }

    for (auto &p : producers) p.join();

    while (q.size() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    q.shutdown();
    for (auto &c : consumers) c.join();

    if (sum_consumed.load() != 4 * n_items) {
        std::cerr << "FAIL: Expected sum " << (4 * n_items) << " but got " << sum_consumed.load() << "\n";
        std::exit(EXIT_FAILURE);
    }
    std::cout << "PASS [test_safe_queue_multithreaded]\n";
}

ProblemInstance create_toy_problem() {
    ProblemInstance p;
    p.name = "toy_problem";
    p.dimension = 6;
    p.capacity = 30;
    p.scaling = 1;
    p.depot = ProblemNode{0, 0, 0, 0};
    p.nodes = {
        ProblemNode{0, 10, 10, 5},
        ProblemNode{1, 10, -10, 10},
        ProblemNode{2, -10, 10, 8},
        ProblemNode{3, -10, -10, 12},
        ProblemNode{4, 0, 15, 6}
    };
    return p;
}

void test_run_ga_pipelined_simulation(std::size_t num_vfpgas) {
    std::cout << "Test: run_ga_pipelined with N=" << num_vfpgas << " simulated vFPGAs...\n";

    ProblemInstance problem = create_toy_problem();
    const std::size_t pop_size = 20;
    const std::size_t n_generations = 10;
    const int beats_per_ind = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_ind = beats_per_ind * AXI_BYTES;
    const std::size_t pop_bytes = pop_size * bytes_per_ind;

    const std::size_t batch_ind_per_stream = 5;
    const std::size_t batch_bytes = batch_ind_per_stream * bytes_per_ind;

    AsyncGAParams params;
    params.pop_size = pop_size;
    params.n_generations = n_generations;
    params.n_best_keep = 10.0;
    params.seed = 0xC0FFEE;
    params.two_opt_k = 1;
    params.num_vfpgas = num_vfpgas;
    params.num_ind_per_stream = batch_ind_per_stream;

    // Allocate host scratchpad for population
    std::vector<std::uint8_t> pop_mem(pop_bytes);
    params.buffer_pop = pop_mem.data();

    // Allocate double-buffered slots per vFPGA (2 slots per vFPGA)
    const std::size_t slots_per_vfpga = 2;
    std::vector<std::vector<std::uint8_t>> storage_src0;
    std::vector<std::vector<std::uint8_t>> storage_src1;
    std::vector<std::vector<std::uint8_t>> storage_dst0;
    std::vector<std::vector<std::uint8_t>> storage_dst1;

    for (std::size_t v = 0; v < num_vfpgas; ++v) {
        for (std::size_t s = 0; s < slots_per_vfpga; ++s) {
            storage_src0.emplace_back(batch_bytes, 0);
            storage_src1.emplace_back(batch_bytes, 0);
            storage_dst0.emplace_back(batch_bytes, 0);
            storage_dst1.emplace_back(batch_bytes, 0);

            BatchBufferSlot slot;
            slot.slot_id = static_cast<int>(v * slots_per_vfpga + s);
            slot.vfid = static_cast<int>(v);
            slot.num_ind_per_stream = batch_ind_per_stream;
            slot.src_buf_0 = storage_src0.back().data();
            slot.src_buf_1 = storage_src1.back().data();
            slot.dst_buf_0 = storage_dst0.back().data();
            slot.dst_buf_1 = storage_dst1.back().data();

            params.buffer_slots.push_back(slot);
        }
    }

    // Mock vFPGA hardware executor: copies parents to offspring buffers and writes valid fitness
    auto mock_vfpga_executor = [&](int vfid, const BatchJob &job) {
        // Simulate hardware processing latency
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        // Copy parent bytes to destination offspring bytes
        std::memcpy(job.dst_buf_0, job.src_buf_0, job.num_ind_per_stream * bytes_per_ind);
        std::memcpy(job.dst_buf_1, job.src_buf_1, job.num_ind_per_stream * bytes_per_ind);
    };

    GASummary summary = run_ga_pipelined(problem, params, mock_vfpga_executor);

    if (summary.best_fitness == 0.0 || summary.best_fitness > 100000.0) {
        std::cerr << "FAIL: Invalid best fitness: " << summary.best_fitness << "\n";
        std::exit(EXIT_FAILURE);
    }
    if (summary.timings.total_time_ms <= 0.0) {
        std::cerr << "FAIL: Total time must be positive.\n";
        std::exit(EXIT_FAILURE);
    }

    std::cout << "PASS [run_ga_pipelined simulation N=" << num_vfpgas
              << ", Best Fitness=" << summary.best_fitness
              << ", Wall Time=" << summary.timings.total_time_ms << " ms]\n";
}

} // namespace

int main() {
    std::cout << "========================================\n";
    std::cout << "Running Async Multi-vFPGA Pipeline Tests\n";
    std::cout << "========================================\n";

    test_safe_queue_basic();
    test_safe_queue_multithreaded();
    test_run_ga_pipelined_simulation(1);
    test_run_ga_pipelined_simulation(2);
    test_run_ga_pipelined_simulation(4);

    std::cout << "========================================\n";
    std::cout << "Result: All Async Pipeline Tests Passed!\n";
    std::cout << "========================================\n";
    return EXIT_SUCCESS;
}
