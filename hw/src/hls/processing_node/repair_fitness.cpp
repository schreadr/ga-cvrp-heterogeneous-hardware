#include "repair_fitness.hpp"
#include "util.hpp"

#include <hls_stream.h>

// Static mask to clear all depot flags
static inline ap_uint<BEAT_WIDTH> get_clear_depots_mask() {
    #pragma HLS INLINE
    ap_uint<BEAT_WIDTH> mask = ~(ap_uint<BEAT_WIDTH>(0));
    for (int i = 0; i < CITIES_PER_BEAT; ++i) {
        #pragma HLS UNROLL
        // Clear the specific depot bit for each city
        mask &= ~(ap_uint<BEAT_WIDTH>(1) << (i * CITY_BITS + DEPOT_FLAG_OFFSET));
    }
    return mask;
}

static ap_uint<COORD_BITS + 1> abs_coord_diff(Coord lhs, Coord rhs) {
    #pragma HLS INLINE
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

static ap_uint<COORD_BITS * 2 + 2> squared_distance(Point from, Point to) {
    #pragma HLS INLINE
    ap_uint<COORD_BITS + 1> dx = abs_coord_diff(from.x, to.x);
    ap_uint<COORD_BITS + 1> dy = abs_coord_diff(from.y, to.y);
    return dx * dx + dy * dy;
}

static ap_uint<COORD_BITS + 1> floored_distance(Point from, Point to) {
    #pragma HLS INLINE
    ap_uint<COORD_BITS * 2 + 2> squared = squared_distance(from, to);
    return integer_sqrt<COORD_BITS * 2 + 2>(squared);
}

static void stream_node_pair(
    hls::stream<NodePairElement> &node_pairs,
    NodeId from,
    NodeId to,
    bool last
) {
    #pragma HLS INLINE
    NodePairElement elem;
    elem.pair.from = from;
    elem.pair.to = to;
    elem.last = last;
    node_pairs.write(elem);
}

static void stream_input_beats(
    const IndividualAsBeats &input,
    BeatIndex total_beats,
    hls::stream<Beat> &input_beats
) {
    #pragma HLS INLINE off
    const ap_uint<BEAT_WIDTH> CLEAR_DEPOT_MASK = get_clear_depots_mask();
    STREAM_INPUT_BEATS_LOOP: for (BeatIndex b = 0; b < total_beats; ++b) {
        #pragma HLS PIPELINE II=1
        input_beats.write(input[b] & CLEAR_DEPOT_MASK);
    }
}

static void repair_depots(
    hls::stream<Beat> &input_beats,
    CityIndex num_cities,
    Capacity vehicle_capacity,
    const CityData city_cache[MAX_CACHE_ENTRIES],
    hls::stream<NodePairElement> &node_pairs,
    hls::stream<BeatPacket> &beat_stream
) {
    NodeId depot_id = NodeId(num_cities);
    
    Capacity remaining_capacity = vehicle_capacity;
    NodeId prev_node = depot_id;

    BeatProducerState out_state;

    bool have_prev = false;
    City prev_city = 0;

    CityIndex city_idx = 0;
    
    Beat active_beat = input_beats.read();
    CityIndex cities_in_beat = CITIES_PER_BEAT;

    REPAIR_CITY_LOOP: while (city_idx < num_cities) {
        #pragma HLS PIPELINE II=1
        
        if (cities_in_beat == 0) {
            active_beat = input_beats.read();
            cities_in_beat = CITIES_PER_BEAT;
        } else {
            City current_city = active_beat.range(CITY_BITS - 1, 0);
            NodeId current_node = get_node_id(current_city);
            CityData cdata = city_cache[current_node];
            Capacity demand = cdata.demand;
            bool route_exceeds = (have_prev && demand > remaining_capacity);

            if (route_exceeds) {
                set_is_depot(prev_city, true);
                
                stream_node_pair(node_pairs, prev_node, depot_id, false);
                prev_node = depot_id;
                remaining_capacity = vehicle_capacity;
                
                send_city(out_state, prev_city, beat_stream);
                
                have_prev = false;
            } else {
                if (have_prev) {
                    send_city(out_state, prev_city, beat_stream);
                }
                
                stream_node_pair(node_pairs, prev_node, current_node, false);
                prev_node = current_node;
                remaining_capacity -= demand;
                
                prev_city = current_city;
                have_prev = true;

                active_beat >>= CITY_BITS;
                cities_in_beat--;
                city_idx++;
            }
        }
    }

    // Finalize end of individual
    if (have_prev) {
        set_is_depot(prev_city, true);
        stream_node_pair(node_pairs, prev_node, depot_id, true);
        
        send_city(out_state, prev_city, beat_stream);
        finalizeStream(out_state, beat_stream);
    }
}

static Fitness calculate_fitness(
    hls::stream<NodePairElement> &node_pairs,
    const CityData city_cache[MAX_CACHE_ENTRIES],
    CityIndex num_cities
) {
    Fitness total = 0;
    NodePairElement elem;
    
    Point prev_coord = get_coord(city_cache[num_cities]);

    CONSUME_NODE_PAIRS_LOOP: do {
        #pragma HLS PIPELINE II=1
        elem = node_pairs.read();
        Point to_coord = get_coord(city_cache[elem.pair.to]);
        total += floored_distance(prev_coord, to_coord);
        prev_coord = to_coord;
    } while (!elem.last);

    return total;
}

static void repair_and_calculate(
    const IndividualAsBeats &input,
    IndividualAsBeats &output,
    CityIndex num_cities,
    BeatIndex total_city_beats,
    Capacity vehicle_capacity,
    const CityData city_cache[MAX_CACHE_ENTRIES],
    Fitness &fitness,
    const StartEndDataWithBeats &start_end_data
) {
    #pragma HLS DATAFLOW
    hls::stream<Beat> input_beats("input_beats");
    hls::stream<NodePairElement> node_pairs("node_pairs");
    hls::stream<BeatPacket> beat_stream("beat_stream");

    stream_input_beats(input, total_city_beats, input_beats);
    repair_depots(input_beats, num_cities, vehicle_capacity, city_cache, node_pairs, beat_stream);
    stream_beats_to_individual<4>(beat_stream, output, 0, num_cities, start_end_data, total_city_beats, num_cities);
    fitness = calculate_fitness(node_pairs, city_cache, num_cities);
}

void repair_fitness(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    CityIndex num_cities,
    Capacity vehicle_capacity,
    const CityData city_cache[MAX_CACHE_ENTRIES]
) {
    CityIndex city_count = CityIndex(num_cities);
    BeatIndex total_city_beats = get_total_city_beats(num_cities);

    StartEndData start_end_data = get_start_end_data(0, num_cities - 1);

    REPAIR_FITNESS_INDIV_LOOP: for (int individual_idx = 0; individual_idx < num_individuals; ++individual_idx) {
        hls::read_lock<IndividualAsBeats> input(in);
        hls::write_lock<IndividualAsBeats> output(out);

        StartEndDataWithBeats data(start_end_data, input[start_end_data.start_beat], input[start_end_data.end_beat]);

        Fitness fitness;
        repair_and_calculate(
            input, output, city_count, total_city_beats, vehicle_capacity, city_cache, fitness, data);
        set_fitness(output, fitness, num_cities);
    }
}
