#ifndef TWO_OPT_HPP
#define TWO_OPT_HPP

#include "types.hpp"
#include <hls_streamofblocks.h>
#include <hls_stream.h>
#include "xoroshiro128plusplus.hpp"

#ifndef N_TWO_OPT_WORKERS
#define N_TWO_OPT_WORKERS 8
#define N_TWO_OPT_WORKERS_LOG2 3 
#endif

void two_opt(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    bool enable_precheck,
    const CityData city_cache[MAX_CACHE_ENTRIES],
    uint32_t seed_two_opt
);

#endif // TWO_OPT_HPP
