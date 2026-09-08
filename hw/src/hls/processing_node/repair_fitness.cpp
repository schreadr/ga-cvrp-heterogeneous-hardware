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

constexpr int MAX_COORDINATES_PER_BEAT = FIELDS_PER_BEAT * 2;

struct CoordinateBeat {
    CoordinatePair pairs[MAX_COORDINATES_PER_BEAT];
    FieldIndex count;
    bool last;
};

struct RouteState {
    Capacity load;
    Point prev_coord;
    FitnessOrCity prev_city;
    FieldIndex prev_field_idx;
};

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

static void append_coordinate(
    CoordinateBeat &coordinate_beat,
    Point from,
    Point to
) {
    #pragma HLS INLINE
    coordinate_beat.pairs[(int) coordinate_beat.count++] = CoordinatePair{from.x, from.y, to.x, to.y};
}

static void close_route(
    Beat &beat,
    CoordinateBeat &coordinate_beat,
    RouteState &route,
    Point depot
) {
    #pragma HLS INLINE
    set_is_depot(route.prev_city, true);
    set_field(beat, route.prev_field_idx, route.prev_city);
    append_coordinate(coordinate_beat, route.prev_coord, depot);
    route.prev_coord = depot;
    route.load = 0;
}

static void commit_city(
    Beat &beat,
    FieldIndex field_idx,
    FitnessOrCity city,
    Capacity demand,
    CoordinateBeat &coordinate_beat,
    RouteState &route
) {
    #pragma HLS INLINE
    Point city_coord = {get_coord_x(city), get_coord_y(city)};
    append_coordinate(coordinate_beat, route.prev_coord, city_coord);

    route.load += demand;
    route.prev_city = city;
    route.prev_coord = city_coord;
    route.prev_field_idx = field_idx;
    set_field(beat, field_idx, city);
}

static void process_city(
    Beat &beat,
    FieldIndex field_idx,
    CoordinateBeat &coordinate_beat,
    RouteState &route,
    Capacity vehicle_capacity,
    Point depot
) {
    #pragma HLS INLINE
    FitnessOrCity city = get_field(beat, field_idx);
    set_is_depot(city, false);

    Capacity demand = get_capacity(city);
    if (route.load + demand > vehicle_capacity) {
        close_route(beat, coordinate_beat, route, depot);
    }
    commit_city(beat, field_idx, city, demand, coordinate_beat, route);
}

static void commit_beat(
    IndividualAsBeats &output,
    Beat beat,
    BeatIndex beat_idx,
    hls::stream<CoordinateBeat> &coordinates,
    CoordinateBeat coordinate_beat,
    bool last
) {
    #pragma HLS INLINE
    output[beat_idx] = beat;
    coordinate_beat.last = last;
    coordinates.write(coordinate_beat);
}

static void repair_depots(
    const IndividualAsBeats &input,
    IndividualAsBeats &output,
    FieldIndex num_cities,
    BeatIndex total_beats,
    Capacity vehicle_capacity,
    Coord depot_x,
    Coord depot_y,
    hls::stream<CoordinateBeat> &coordinates
) {
    Point depot = {depot_x, depot_y};
    RouteState route = {0, depot, 0, 0};

    Beat prev_beat = 0;
    CoordinateBeat prev_coordinates = {};

    REPAIR_BEAT_LOOP: for (BeatIndex beat_idx = 0; beat_idx < total_beats; ++beat_idx) {
        Beat beat = input[beat_idx];
        FieldIndex first_field = get_first_city_index(beat_idx);
        FieldIndex last_field = get_last_field_index(num_cities, beat_idx);
        CoordinateBeat coordinate_beat = {};

        FitnessOrCity first = get_field(beat, first_field);
        set_is_depot(first, false);
        Capacity first_demand = get_capacity(first);

        if (beat_idx > 0 && route.load + first_demand > vehicle_capacity) {
            close_route(prev_beat, prev_coordinates, route, depot);
        }

        commit_city(beat, first_field, first, first_demand, coordinate_beat, route);

        if (beat_idx > 0) {
            commit_beat(output, prev_beat, beat_idx - 1, coordinates, prev_coordinates, false);
        }

        REPAIR_FIELD_LOOP: for (FieldIndex field_idx = 0; field_idx < FIELDS_PER_BEAT; ++field_idx) {
            #pragma HLS PIPELINE II=1
            if(field_idx > first_field && field_idx <= last_field) {
                process_city(beat, field_idx, coordinate_beat, route, vehicle_capacity, depot);
            }
        }

        prev_beat = beat;
        prev_coordinates = coordinate_beat;
    }

    if (total_beats > 0) {
        close_route(prev_beat, prev_coordinates, route, depot);
        commit_beat(output, prev_beat, total_beats - 1, coordinates, prev_coordinates, true);
    }
}

static FitnessOrCity calculate_fitness(
    hls::stream<CoordinateBeat> &coordinates
) {
    FitnessOrCity total = 0;
    CoordinateBeat coordinate_beat;

    bool need_read = true;
    int pair_idx = 0;
    bool done = false;

    CONSUME_COORDINATES_LOOP: while (!done) {
        #pragma HLS PIPELINE II=1 REWIND
        if (need_read) {
            coordinate_beat = coordinates.read();
            pair_idx = 0;
            need_read = (coordinate_beat.count == 0);
            done = coordinate_beat.last && need_read;
        } else {
            total += floored_distance(coordinate_beat.pairs[pair_idx]);
            pair_idx++;
            if (pair_idx == coordinate_beat.count) {
                need_read = true;
                done = coordinate_beat.last;
            }
        }
    }

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
    FitnessOrCity &fitness
) {
    #pragma HLS DATAFLOW
    hls::stream<CoordinateBeat> coordinates;

    repair_depots(input, output, num_cities, total_beats, vehicle_capacity,
                  depot_x, depot_y, coordinates);
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

    REPAIR_FITNESS_INDIV_LOOP: for (int individual_idx = 0; individual_idx < num_individuals; ++individual_idx) {
        hls::read_lock<IndividualAsBeats> input(in);
        hls::write_lock<IndividualAsBeats> output(out);

        FitnessOrCity fitness;
        repair_and_calculate(
            input, output, city_count, total_beats, vehicle_capacity, depot_x, depot_y, fitness);
        set_fitness(output, fitness);
    }
}