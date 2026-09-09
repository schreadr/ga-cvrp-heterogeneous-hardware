#include "repair_fitness.hpp"
#include "util.hpp"

#include <hls_stream.h>

struct Point {
    Coord x;
    Coord y;
};

struct CoordinatePair {
    Coord from_x;
    Coord from_y;
    Coord to_x;
    Coord to_y;
};

struct CoordinateElement {
    CoordinatePair pair;
    bool last;
};

// Static mask to clear all depot flags (bit 20 = 2*COORD_BITS+CAPACITY_BITS)
static inline ap_uint<BEAT_WIDTH> get_clear_depots_mask() {
    #pragma HLS INLINE
    ap_uint<BEAT_WIDTH> mask = ~(ap_uint<BEAT_WIDTH>(0));
    for (int i = 0; i < FIELDS_PER_BEAT; ++i) {
        #pragma HLS UNROLL
        // Clear the specific depot bit for each field
        mask &= ~(ap_uint<BEAT_WIDTH>(1) << (i * FIELD_BITS + DEPOT_FLAG_FIELD_OFFSET));
    }
    return mask;
}

static ap_uint<COORD_BITS + 1> abs_coord_diff(Coord lhs, Coord rhs) {
    #pragma HLS INLINE
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

static ap_uint<COORD_BITS * 2 + 2> squared_distance(const CoordinatePair &pair) {
    #pragma HLS INLINE
    ap_uint<COORD_BITS + 1> dx = abs_coord_diff(pair.from_x, pair.to_x);
    ap_uint<COORD_BITS + 1> dy = abs_coord_diff(pair.from_y, pair.to_y);
    return dx * dx + dy * dy;
}

static FitnessOrCity floored_distance(const CoordinatePair &pair) {
    #pragma HLS INLINE
    ap_uint<COORD_BITS * 2 + 2> squared = squared_distance(pair);
    ap_uint<COORD_BITS + 1> distance = integer_sqrt<COORD_BITS * 2 + 2>(squared);
    return FitnessOrCity(distance);
}

static void stream_coordinate(
    hls::stream<CoordinateElement> &coordinates,
    Point from,
    Point to,
    bool last
) {
    #pragma HLS INLINE
    CoordinateElement elem;
    elem.pair = CoordinatePair{from.x, from.y, to.x, to.y};
    elem.last = last;
    coordinates.write(elem);
}

static void repair_depots(
    const IndividualAsBeats &input,
    FieldIndex num_cities,
    Capacity vehicle_capacity,
    Coord depot_x,
    Coord depot_y,
    hls::stream<CoordinateElement> &coordinates,
    hls::stream<BeatPacket> &beat_stream
) {
    Point depot = {depot_x, depot_y};
    
    Capacity remaining_capacity = vehicle_capacity;
    Point prev_coord = depot;

    BeatProducerState out_state;

    bool have_prev = false;
    FitnessOrCity prev_city = 0;

    FieldIndex field_idx = 1;
    BeatIndex current_beat_idx = 0;
    const ap_uint<BEAT_WIDTH> CLEAR_DEPOT_MASK = get_clear_depots_mask();
    
    Beat active_beat = (input[0] & CLEAR_DEPOT_MASK) >> FIELD_BITS;
    BeatFieldIndex cities_in_beat = FIELDS_PER_BEAT - 1;

    bool next_beat_ready = false;
    Beat prefetched_beat_reg = input[0] & CLEAR_DEPOT_MASK;

    REPAIR_FIELD_LOOP: while (field_idx <= num_cities) {
        #pragma HLS PIPELINE II=1
        
        // Snapshot pipeline registers to sever all combinational paths
        Beat beat_to_load = prefetched_beat_reg;
        
        // Advance BRAM address early based strictly on start-of-cycle registers
        if (cities_in_beat == 2 && !next_beat_ready) {
            current_beat_idx++;
            next_beat_ready = true;
        }

        // Unconditional extraction from active_beat register
        FitnessOrCity current_city = active_beat.range(FIELD_BITS - 1, 0);
        Capacity demand = get_capacity(current_city);
        bool route_exceeds = (have_prev && demand > remaining_capacity);

        if (cities_in_beat == 0) {
            // Load the beat that was prefetched and stabilized in the pipeline
            active_beat = beat_to_load;
            cities_in_beat = FIELDS_PER_BEAT;
            next_beat_ready = false;
        } else {
            if (route_exceeds) {
                set_is_depot(prev_city, true);
                
                stream_coordinate(coordinates, prev_coord, depot, false);
                prev_coord = depot;
                remaining_capacity = vehicle_capacity;
                
                send_field(out_state, prev_city, beat_stream);
                
                have_prev = false;
            } else {
                if (have_prev) {
                    send_field(out_state, prev_city, beat_stream);
                }
                
                Point city_coord = {get_coord_x(current_city), get_coord_y(current_city)};
                stream_coordinate(coordinates, prev_coord, city_coord, false);
                prev_coord = city_coord;
                remaining_capacity -= demand;
                
                prev_city = current_city;
                have_prev = true;

                active_beat >>= FIELD_BITS;
                cities_in_beat--;
                field_idx++;
            }
        }

        // Unconditional BRAM read to update the shift register at END of cycle
        // Because it is unconditional, HLS does not insert a MUX (select).
        prefetched_beat_reg = input[current_beat_idx] & CLEAR_DEPOT_MASK;
    }

    // Finalize end of individual
    if (have_prev) {
        set_is_depot(prev_city, true);
        stream_coordinate(coordinates, prev_coord, depot, true);
        
        send_field(out_state, prev_city, beat_stream);
        finalizeStream(out_state, beat_stream);
    }
}

static FitnessOrCity calculate_fitness(
    hls::stream<CoordinateElement> &coordinates
) {
    FitnessOrCity total = 0;
    CoordinateElement elem;
    
    CONSUME_COORDINATES_LOOP: do {
        #pragma HLS PIPELINE II=1
        elem = coordinates.read();
        total += floored_distance(elem.pair);
    } while (!elem.last);

    return total;
}

static void repair_and_calculate(
    const IndividualAsBeats &input,
    IndividualAsBeats &output,
    FieldIndex num_cities,
    BeatIndex total_beats,
    Capacity vehicle_capacity,
    Coord depot_x,
    Coord depot_y,
    FitnessOrCity &fitness,
    const StartEndDataWithBeats &start_end_data
) {
    #pragma HLS DATAFLOW
    hls::stream<CoordinateElement> coordinates;
    hls::stream<BeatPacket> beat_stream;

    repair_depots(input, num_cities, vehicle_capacity, depot_x, depot_y, coordinates, beat_stream);
    stream_beats_to_individual<4>(beat_stream, output, 1, num_cities, start_end_data, total_beats, num_cities);
    fitness = calculate_fitness(coordinates);
}

void repair_fitness(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    FieldIndex num_cities,
    Capacity vehicle_capacity,
    int depot_cords
) {
    Coord depot_x = Coord((depot_cords >> COORD_BITS) & ((1 << COORD_BITS) - 1));
    Coord depot_y = Coord(depot_cords & ((1 << COORD_BITS) - 1));

    FieldIndex city_count = FieldIndex(num_cities);
    BeatIndex total_beats = get_total_beats(num_cities);

    StartEndData start_end_data = get_start_end_data(1, num_cities);

    REPAIR_FITNESS_INDIV_LOOP: for (int individual_idx = 0; individual_idx < num_individuals; ++individual_idx) {
        hls::read_lock<IndividualAsBeats> input(in);
        hls::write_lock<IndividualAsBeats> output(out);

        StartEndDataWithBeats data(start_end_data, input[start_end_data.start_beat], input[start_end_data.end_beat]);

        FitnessOrCity fitness;
        repair_and_calculate(
            input, output, city_count, total_beats, vehicle_capacity, depot_x, depot_y, fitness, data);
        set_fitness(output, fitness);
    }
}
