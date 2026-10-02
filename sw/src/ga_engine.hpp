#ifndef GA_ENGINE_HPP
#define GA_ENGINE_HPP

#include "cvrp_types.hpp"
#include "batch_job.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <vector>

struct GAParams {
    std::size_t pop_size;
    std::size_t n_generations;
    double n_best_keep;
    std::uint32_t seed;

    std::uint8_t* buffer_src_0 = nullptr;
    std::uint8_t* buffer_src_1 = nullptr;
    std::uint8_t* buffer_dst_0 = nullptr;
    std::uint8_t* buffer_dst_1 = nullptr;

    std::uint8_t* buffer_pop = nullptr;
};

struct AsyncGAParams {
    std::size_t pop_size;
    std::size_t n_generations;
    double n_best_keep;
    std::uint32_t seed;
    std::size_t num_vfpgas;
    std::size_t num_ind_per_stream; // Batch size per stream for each vFPGA

    std::vector<BatchBufferSlot> buffer_slots;

    std::uint8_t* buffer_pop = nullptr;
};

struct WorkerTiming {
    int vfid = 0;
    double active_time_ms = 0.0;
    double idle_time_ms = 0.0;
    std::size_t batches_processed = 0;

    double total_time_ms() const {
        return active_time_ms + idle_time_ms;
    }

    double utilization_pct() const {
        double tot = total_time_ms();
        return tot > 0.0 ? (active_time_ms / tot * 100.0) : 0.0;
    }
};

struct CPUTiming {
    double active_time_ms = 0.0;
    double idle_time_ms = 0.0;
    double init_time_ms = 0.0;
    double parent_selection_ms = 0.0;
    double population_update_ms = 0.0;

    double total_time_ms() const {
        return active_time_ms + idle_time_ms;
    }

    double utilization_pct() const {
        double tot = total_time_ms();
        return tot > 0.0 ? (active_time_ms / tot * 100.0) : 0.0;
    }
};

struct GATimings {
    double total_time_ms = 0.0;
    CPUTiming cpu;
    std::vector<WorkerTiming> workers;
};

struct GenerationalFitnessRecord {
    std::size_t generation = 0;
    double timestamp_ms = 0.0;
    double best_fitness = 0.0;
};

struct GASummary {
    std::size_t n_generations = 0;
    std::size_t pop_size = 0;
    Individual best_individual;
    double best_fitness = 0.0;          // Scaled hardware fitness
    double exact_best_fitness = 0.0;    // Exact double-precision sqrt on unscaled coordinates
    double initial_best_fitness = 0.0;
    std::size_t gen_best_found = 0;

    GATimings timings;

    // Best fitness and timestamp recorded for each complete generation (index 0 = initial, index g = gen g)
    std::vector<GenerationalFitnessRecord> best_fitness_history;

    double throughput_evals_sec() const {
        double sec = timings.total_time_ms / 1000.0;
        return sec > 0.0 ? (static_cast<double>(pop_size * n_generations) / sec) : 0.0;
    }

    void print() const;
    void print(std::ostream &os) const;
    void save_json(const std::string &path, const ProblemInstance &problem) const;
    friend std::ostream& operator<<(std::ostream &os, const GASummary &summary);
};

// Evaluates the exact fitness with real sqrt across the entire population and returns the best individual
Individual evaluate_population_exact_best(
    const ProblemInstance &problem,
    const std::vector<EncodedIndividual> &population,
    double &out_best_exact_fitness
);

using VfpgaExecuteFn = std::function<void(int vfid, const BatchJob &job)>;

GASummary run_ga(
    const ProblemInstance& problem,
    GAParams& params,
    std::function<void()> create_new_generation
);

GASummary run_ga_pipelined(
    const ProblemInstance &problem,
    AsyncGAParams &params,
    VfpgaExecuteFn execute_vfpga_batch
);

#endif // GA_ENGINE_HPP
