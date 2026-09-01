#ifndef GA_ENCODING_CONSTANTS_HPP
#define GA_ENCODING_CONSTANTS_HPP

// User-configurable constants
#define COORD_BITS       7
#define CAPACITY_BITS    6
#define MAX_CITIES       1023

// Encoding constants
#define NODE_ID_BITS     10 // ceil(log2(MAX_CITIES))
#define DEPOT_BITS       1
#define FIELD_BITS       (2 * COORD_BITS + CAPACITY_BITS + DEPOT_BITS + NODE_ID_BITS)

#define AXI_WIDTH        512
#define AXI_BYTES        (AXI_WIDTH / 8)
#define FIELDS_PER_BEAT  (AXI_WIDTH / FIELD_BITS)
#define MAX_FIELDS       (MAX_CITIES + 1)
#define MAX_BEATS        ((MAX_FIELDS + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT)
#define BEAT_WIDTH       (FIELDS_PER_BEAT * FIELD_BITS)

// inlined for hw side
static inline int get_total_beats(int num_cities) {
    int total_fields = 1 + num_cities;
    return (total_fields + FIELDS_PER_BEAT - 1) / FIELDS_PER_BEAT;
}

#endif