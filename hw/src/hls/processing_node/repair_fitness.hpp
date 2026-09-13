#ifndef REPAIR_FITNESS_HPP
#define REPAIR_FITNESS_HPP

#include "types.hpp"
#include <hls_streamofblocks.h>

void repair_fitness(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    FieldIndex num_cities,
    Capacity vehicle_capacity,
    int depot_cords
);

#endif // REPAIR_FITNESS_HPP