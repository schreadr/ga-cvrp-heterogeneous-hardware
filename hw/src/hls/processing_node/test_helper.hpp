#ifndef TEST_HELPER_HPP
#define TEST_HELPER_HPP

#include "types.hpp"
#include "ga.hpp"

#include <cstddef>
#include <cstdint>
#include <hls_streamofblocks.h>

void tb_push_bytes(hls::stream<axi_word_t> &stream,
                   const std::uint8_t *bytes,
                   std::size_t n_bytes,
                   bool is_last);

void tb_pull_bytes(hls::stream<axi_word_t> &stream,
                   std::uint8_t *bytes,
                   std::size_t n_bytes);

void tb_push_individuals(hls::stream_of_blocks<IndividualAsBeats> &stream, const ProblemInstance &problem, const std::vector<Individual> &individuals);

std::vector<Individual> tb_pull_individuals(hls::stream_of_blocks<IndividualAsBeats> &stream, const ProblemInstance &problem, int population_size);

std::vector<Individual> build_population(const ProblemInstance &problem,
                                         const std::vector<std::vector<int>> &tours);

std::vector<Individual> build_population(const ProblemInstance &problem,
                                         const std::vector<std::vector<int>> &tours,
                                         std::uint32_t fitness_base);

#endif // TEST_HELPER_HPP
