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
    int two_opt_k = 0;

    std::uint8_t* buffer_src_0 = nullptr;
    std::uint8_t* buffer_src_1 = nullptr;
    std::uint8_t* buffer_dst_0 = nullptr;
    std::uint8_t* buffer_dst_1 = nullptr;

    std::uint8_t* buffer_pop = nullptr;
};

struct AsyncGAParams {
    std::size_t pop_size = 0;
    std::size_t n_generations = 0;
    double n_best_keep = 5.0;
    std::uint32_t seed = 0;
    int two_opt_k = 0;
    std::size_t num_vfpgas = 1;
    std::size_t num_ind_per_stream = 0; // Batch size per stream for each vFPGA

    // Pre-allocated buffer slots for all vFPGAs (e.g. 2 slots per vFPGA for double-buffering)
    std::vector<BatchBufferSlot> buffer_slots;

    // Host scratchpad for population
    std::uint8_t* buffer_pop = nullptr;
};

struct GATimings {
    double init_time_ms = 0.0;
    double parent_selection_ms = 0.0;
    double offspring_generation_ms = 0.0;
    double population_update_ms = 0.0;
    double total_time_ms = 0.0;

    double generational_time_ms() const {
        return parent_selection_ms + offspring_generation_ms + population_update_ms;
    }

    double avg_parent_selection_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (parent_selection_ms / n_generations) : 0.0;
    }

    double avg_offspring_generation_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (offspring_generation_ms / n_generations) : 0.0;
    }

    double avg_population_update_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (population_update_ms / n_generations) : 0.0;
    }

    double avg_generational_time_ms(std::size_t n_generations) const {
        return n_generations > 0 ? (generational_time_ms() / n_generations) : 0.0;
    }
};

struct GASummary {
    // --- Part 1: Solution & Run Information ---
    std::size_t n_generations = 0;
    std::size_t pop_size = 0;
    Individual best_individual;
    double best_fitness = 0.0;

    // --- Part 2: Execution Timing ---
    GATimings timings;

    // Convenience forwarders
    double generational_time_ms() const {
        return timings.generational_time_ms();
    }
    double avg_parent_selection_ms() const {
        return timings.avg_parent_selection_ms(n_generations);
    }
    double avg_offspring_generation_ms() const {
        return timings.avg_offspring_generation_ms(n_generations);
    }
    double avg_population_update_ms() const {
        return timings.avg_population_update_ms(n_generations);
    }
    double avg_generational_time_ms() const {
        return timings.avg_generational_time_ms(n_generations);
    }

    void print() const;
    void print(std::ostream &os) const;
    friend std::ostream& operator<<(std::ostream &os, const GASummary &summary);
};

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
