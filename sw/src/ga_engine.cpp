#include "ga_engine.hpp"
#include "ga_operators.hpp"
#include "ga_streaming.hpp"
#include "safe_queue.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <thread>

void GASummary::print() const {
    print(std::cout);
}

void GASummary::print(std::ostream &os) const {
    const double gen_total = generational_time_ms();
    const double pct_parents = gen_total > 0.0 ? (timings.parent_selection_ms / gen_total * 100.0) : 0.0;
    const double pct_offspring = gen_total > 0.0 ? (timings.offspring_generation_ms / gen_total * 100.0) : 0.0;
    const double pct_update = gen_total > 0.0 ? (timings.population_update_ms / gen_total * 100.0) : 0.0;

    auto old_flags = os.flags();
    auto old_prec = os.precision();

    os << "\n============================== GA EXECUTION SUMMARY ==============================\n";
    os << "--- Solution Overview ------------------------------------------------------------\n";
    os << "  Generations:          " << n_generations << "\n";
    os << "  Population Size:      " << pop_size << "\n";
    os << "  Best Fitness:         " << best_fitness << "\n";
    os << "----------------------------------------------------------------------------------\n";
    os << "\n--- Execution Timing -------------------------------------------------------------\n";
    os << "  " << std::left << std::setw(28) << "Phase"
       << std::right << std::setw(16) << "Total (ms)"
       << std::right << std::setw(16) << "Avg/Gen (ms)"
       << std::right << std::setw(14) << "Share (%)" << "\n";
    os << "  --------------------------------------------------------------------------------\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Parents Selection"
       << std::right << std::setw(16) << timings.parent_selection_ms
       << std::right << std::setw(16) << avg_parent_selection_ms()
       << std::right << std::setw(13) << std::setprecision(1) << pct_parents << " %\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Offspring Generation"
       << std::right << std::setw(16) << timings.offspring_generation_ms
       << std::right << std::setw(16) << avg_offspring_generation_ms()
       << std::right << std::setw(13) << std::setprecision(1) << pct_offspring << " %\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Population Update"
       << std::right << std::setw(16) << timings.population_update_ms
       << std::right << std::setw(16) << avg_population_update_ms()
       << std::right << std::setw(13) << std::setprecision(1) << pct_update << " %\n";
    os << "  --------------------------------------------------------------------------------\n";
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Generational Total"
       << std::right << std::setw(16) << gen_total
       << std::right << std::setw(16) << avg_generational_time_ms()
       << std::right << std::setw(13) << std::setprecision(1) << 100.0 << " %\n";
    if (timings.init_time_ms > 0.0) {
        os << std::fixed << std::setprecision(3);
        os << "  " << std::left << std::setw(28) << "Initialization"
           << std::right << std::setw(16) << timings.init_time_ms
           << std::right << std::setw(16) << "-"
           << std::right << std::setw(14) << "-\n";
    }
    os << std::fixed << std::setprecision(3);
    os << "  " << std::left << std::setw(28) << "Total Wall Time"
       << std::right << std::setw(16) << timings.total_time_ms
       << std::right << std::setw(16) << "-"
       << std::right << std::setw(14) << "-\n";
    os << "==================================================================================\n";

    os.flags(old_flags);
    os.precision(old_prec);
}

std::ostream& operator<<(std::ostream &os, const GASummary &summary) {
    summary.print(os);
    return os;
}

GASummary run_ga(const ProblemInstance& problem, GAParams& params, std::function<void()> create_new_generation) {
    auto wall_start = std::chrono::steady_clock::now();
    GASummary summary;
    summary.n_generations = params.n_generations;
    summary.pop_size = params.pop_size;

    auto init_start = std::chrono::steady_clock::now();
    std::mt19937 new_seed(params.seed);

    const std::size_t pop_size = params.pop_size;
    const std::size_t half_pop = pop_size / 2;
    const int beats_per_ind = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_ind = beats_per_ind * AXI_BYTES;
    const std::size_t half_bytes = half_pop * bytes_per_ind;

    std::vector<Individual> initial_population = build_initial_population(problem, pop_size, new_seed());
    const std::vector<std::uint8_t> encoded_population = encode_population_fixed(problem, initial_population);

    std::vector<EncodedIndividual> vec1 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> population = vec1;

    std::vector<EncodedIndividual> vec2 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> next_generation = vec2;

    std::vector<EncodedIndividual> vec3 = std::vector<EncodedIndividual>(pop_size);
    std::reference_wrapper<std::vector<EncodedIndividual>> next_population = vec3;

    std::vector<EncodedIndividual> parents1(half_pop);
    std::vector<EncodedIndividual> parents2(half_pop);

    std::uint8_t *buffer_src_0 = params.buffer_src_0;
    std::uint8_t *buffer_src_1 = params.buffer_src_1;
    std::uint8_t *buffer_dst_0 = params.buffer_dst_0;
    std::uint8_t *buffer_dst_1 = params.buffer_dst_1;
    std::uint8_t *pop_buffer = params.buffer_pop;

    const std::size_t print_interval = std::max<std::size_t>(1, params.n_generations / 10);

    // Individuals of current population must be sorted on the buffer
    std::memcpy(buffer_src_0, encoded_population.data(), half_bytes);
    std::memcpy(buffer_src_1, encoded_population.data() + half_bytes, half_bytes);
    population_from_buffer(buffer_src_0, half_pop, population, problem.dimension, 0);
    population_from_buffer(buffer_src_1, half_pop, population, problem.dimension, half_pop);
    std::sort(population.get().begin(), population.get().end());
    population_to_buffer(population, pop_buffer, problem.dimension);

    const std::size_t n_keep = std::min(pop_size, static_cast<std::size_t>(std::round(pop_size * (params.n_best_keep / 100.0))));
    auto init_end = std::chrono::steady_clock::now();
    summary.timings.init_time_ms = std::chrono::duration<double, std::milli>(init_end - init_start).count();

    for (std::size_t i = 0; i < params.n_generations; i++) {
        auto t0 = std::chrono::steady_clock::now();
        create_parent_pairs(population, parents1, parents2, new_seed());
        population_to_buffer(parents1, buffer_src_0, problem.dimension);
        population_to_buffer(parents2, buffer_src_1, problem.dimension);
        auto t1 = std::chrono::steady_clock::now();
        
        create_new_generation();
        auto t2 = std::chrono::steady_clock::now();

        population_from_buffer(buffer_dst_0, half_pop, next_generation, problem.dimension, 0);
        population_from_buffer(buffer_dst_1, half_pop, next_generation, problem.dimension, half_pop);
        update_generation_with_parents(next_generation.get(), parents1, parents2);
        update_population(population, next_generation, next_population, n_keep);
        std::swap(population, next_population);
        population_to_buffer(population, pop_buffer, problem.dimension, true);
        auto t3 = std::chrono::steady_clock::now();

        summary.timings.parent_selection_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
        summary.timings.offspring_generation_ms += std::chrono::duration<double, std::milli>(t2 - t1).count();
        summary.timings.population_update_ms += std::chrono::duration<double, std::milli>(t3 - t2).count();

        if (i % print_interval == 0) {
            const double current_best = static_cast<double>(population.get()[0].fitness) / static_cast<double>(problem.scaling);
            std::cout << "Iteration " << i << " completed. Best fitness: " << current_best << std::endl;
        }
    }

    if (!population.get().empty() && population.get()[0].bytes != nullptr) {
        summary.best_individual = decode_individual_fixed(problem, population.get()[0].bytes);
        summary.best_fitness = static_cast<double>(summary.best_individual.fitness) / static_cast<double>(problem.scaling);
    }

    auto wall_end = std::chrono::steady_clock::now();
    summary.timings.total_time_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();
    return summary;
}

GASummary run_ga_pipelined(
    const ProblemInstance &problem,
    AsyncGAParams &params,
    VfpgaExecuteFn execute_vfpga_batch
) {
    auto wall_start = std::chrono::steady_clock::now();
    GASummary summary;
    summary.n_generations = params.n_generations;
    summary.pop_size = params.pop_size;

    auto init_start = std::chrono::steady_clock::now();
    std::mt19937 new_seed(params.seed);

    const std::size_t pop_size = params.pop_size;
    const std::size_t num_vfpgas = std::max<std::size_t>(1, params.num_vfpgas);
    const std::size_t batch_ind_per_stream = params.num_ind_per_stream > 0 ? params.num_ind_per_stream : (pop_size / (2 * num_vfpgas));
    const std::size_t offspring_per_batch = 2 * batch_ind_per_stream;

    if (params.buffer_slots.empty()) {
        throw std::runtime_error("run_ga_pipelined: params.buffer_slots is empty. Pre-allocated buffer slots required.");
    }
    if (!params.buffer_pop) {
        throw std::runtime_error("run_ga_pipelined: params.buffer_pop is null. Population scratchpad buffer required.");
    }

    // Build initial population
    std::vector<Individual> initial_population = build_initial_population(problem, pop_size, new_seed());
    const std::vector<std::uint8_t> encoded_population = encode_population_fixed(problem, initial_population);

    const int beats_per_ind = get_total_axi_beats(problem.dimension - 1);
    const std::size_t bytes_per_ind = beats_per_ind * AXI_BYTES;
    const std::size_t pop_bytes = pop_size * bytes_per_ind;

    std::memcpy(params.buffer_pop, encoded_population.data(), pop_bytes);

    std::vector<EncodedIndividual> population_vec(pop_size);
    std::vector<EncodedIndividual> next_population_vec(pop_size);
    population_from_buffer(params.buffer_pop, pop_size, population_vec, problem.dimension, 0);
    std::sort(population_vec.begin(), population_vec.end());
    population_to_buffer(population_vec, params.buffer_pop, problem.dimension);

    const std::size_t n_keep = std::min(pop_size, static_cast<std::size_t>(std::round(pop_size * (params.n_best_keep / 100.0))));

    // Calculate total evaluation batches
    const std::size_t total_evaluations_target = pop_size * params.n_generations;
    const std::size_t total_batches_to_process = std::max<std::size_t>(
        num_vfpgas,
        (total_evaluations_target + offspring_per_batch - 1) / offspring_per_batch
    );

    auto init_end = std::chrono::steady_clock::now();
    summary.timings.init_time_ms = std::chrono::duration<double, std::milli>(init_end - init_start).count();

    // Queues: 1 parent queue per vFPGA, 1 shared offspring completion queue
    std::vector<SafeQueue<BatchJob>> parent_queues(num_vfpgas);
    SafeQueue<BatchJob> offspring_queue;

    // Launch worker threads (one per vFPGA)
    std::vector<std::thread> worker_threads;
    worker_threads.reserve(num_vfpgas);
    for (std::size_t v = 0; v < num_vfpgas; ++v) {
        worker_threads.emplace_back([v, &parent_queues, &offspring_queue, &execute_vfpga_batch]() {
            while (true) {
                auto job_opt = parent_queues[v].pop();
                if (!job_opt) {
                    break; // Queue shut down, terminate worker
                }
                BatchJob job = std::move(*job_opt);
                execute_vfpga_batch(static_cast<int>(v), job);
                offspring_queue.push(std::move(job));
            }
        });
    }

    // Population Manager staging structures
    std::vector<EncodedIndividual> parents1(batch_ind_per_stream);
    std::vector<EncodedIndividual> parents2(batch_ind_per_stream);
    std::vector<EncodedIndividual> batch_offspring(offspring_per_batch);

    uint32_t batches_dispatched = 0;
    uint32_t batches_completed = 0;

    // Priming phase: Fill all pre-allocated buffer slots and enqueue initial jobs
    for (const auto &slot : params.buffer_slots) {
        if (batches_dispatched >= total_batches_to_process) break;

        auto t0 = std::chrono::steady_clock::now();
        create_parent_pairs(population_vec, parents1, parents2, new_seed());
        population_to_buffer(parents1, slot.src_buf_0, problem.dimension);
        population_to_buffer(parents2, slot.src_buf_1, problem.dimension);
        auto t1 = std::chrono::steady_clock::now();
        summary.timings.parent_selection_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();

        BatchJob job;
        job.batch_id = batches_dispatched++;
        job.batch_seed = new_seed();
        job.vfid = slot.vfid;
        job.slot_id = slot.slot_id;
        job.num_ind_per_stream = batch_ind_per_stream;
        job.src_buf_0 = slot.src_buf_0;
        job.src_buf_1 = slot.src_buf_1;
        job.dst_buf_0 = slot.dst_buf_0;
        job.dst_buf_1 = slot.dst_buf_1;
        job.parents1 = parents1;
        job.parents2 = parents2;

        parent_queues[job.vfid].push(std::move(job));
    }

    const std::size_t log_interval_batches = std::max<std::size_t>(1, total_batches_to_process / 10);

    // Continuous streaming loop: collect finished offspring in deterministic in-order sequence
    while (batches_completed < total_batches_to_process) {
        auto t_wait_start = std::chrono::steady_clock::now();
        auto finished_job_opt = offspring_queue.pop_expected(batches_completed);
        auto t_wait_end = std::chrono::steady_clock::now();
        summary.timings.offspring_generation_ms += std::chrono::duration<double, std::milli>(t_wait_end - t_wait_start).count();

        if (!finished_job_opt) {
            break; // Unexpected early shutdown
        }

        BatchJob finished_job = std::move(*finished_job_opt);
        batches_completed++;

        // 1. Population Update: deserialize offspring and merge into population pool
        auto t_up_start = std::chrono::steady_clock::now();
        population_from_buffer(finished_job.dst_buf_0, finished_job.num_ind_per_stream, batch_offspring, problem.dimension, 0);
        population_from_buffer(finished_job.dst_buf_1, finished_job.num_ind_per_stream, batch_offspring, problem.dimension, finished_job.num_ind_per_stream);

        update_generation_with_parents(batch_offspring, finished_job.parents1, finished_job.parents2);

        update_population(population_vec, batch_offspring, next_population_vec, n_keep);
        std::swap(population_vec, next_population_vec);
        population_to_buffer(population_vec, params.buffer_pop, problem.dimension, true);
        auto t_up_end = std::chrono::steady_clock::now();
        summary.timings.population_update_ms += std::chrono::duration<double, std::milli>(t_up_end - t_up_start).count();

        // 2. Refill the returned buffer slot and dispatch next batch if budget remains
        if (batches_dispatched < total_batches_to_process) {
            auto t_sel_start = std::chrono::steady_clock::now();
            create_parent_pairs(population_vec, parents1, parents2, new_seed());
            population_to_buffer(parents1, finished_job.src_buf_0, problem.dimension);
            population_to_buffer(parents2, finished_job.src_buf_1, problem.dimension);
            auto t_sel_end = std::chrono::steady_clock::now();
            summary.timings.parent_selection_ms += std::chrono::duration<double, std::milli>(t_sel_end - t_sel_start).count();

            finished_job.batch_id = batches_dispatched++;
            finished_job.batch_seed = new_seed();
            finished_job.parents1 = parents1;
            finished_job.parents2 = parents2;
            parent_queues[finished_job.vfid].push(std::move(finished_job));
        }

        if (batches_completed == 1 || batches_completed % log_interval_batches == 0 || batches_completed == total_batches_to_process) {
            const double current_best = static_cast<double>(population_vec[0].fitness) / static_cast<double>(problem.scaling);
            std::cout << "Pipelined Batch " << batches_completed << "/" << total_batches_to_process
                      << " completed. Best fitness: " << current_best << std::endl;
        }
    }

    // Shutdown queues to terminate workers gracefully
    for (std::size_t v = 0; v < num_vfpgas; ++v) {
        parent_queues[v].shutdown();
    }
    for (auto &worker : worker_threads) {
        if (worker.joinable()) {
            worker.join();
        }
    }

    // Decode final best solution
    if (!population_vec.empty() && population_vec[0].bytes != nullptr) {
        summary.best_individual = decode_individual_fixed(problem, population_vec[0].bytes);
        summary.best_fitness = static_cast<double>(summary.best_individual.fitness) / static_cast<double>(problem.scaling);
    }

    auto wall_end = std::chrono::steady_clock::now();
    summary.timings.total_time_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();
    return summary;
}
