#ifndef TWO_OPT_HPP
#define TWO_OPT_HPP

#include "types.hpp"
#include <hls_streamofblocks.h>
#include <hls_stream.h>

void two_opt(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    hls::stream<RandomNumber> &random_stream,
    int num_individuals,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    const CityData city_cache[MAX_CACHE_ENTRIES]
);

#endif // TWO_OPT_HPP
