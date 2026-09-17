#include "two_opt.hpp"
#include "util.hpp"

typedef NodeId Subtour[MAX_CITIES];

struct SubtourMeta {
    CityIndex length;
    bool last;
};

static void individual_to_tours(
    const IndividualAsBeats &in_ind,
    CityIndex num_cities,
    hls::stream_of_blocks<Subtour> &tour_stream,
    hls::stream<SubtourMeta> &meta_stream,
    hls::stream<Fitness> &fitness_stream
) {
    Fitness old_fitness = get_fitness(in_ind, num_cities);
    fitness_stream.write(old_fitness);

    Beat active_beat = in_ind[0];
    CityBeatIndex cities_in_beat = HW_CITIES_PER_BEAT;
    BeatIndex beat_idx = 0;

    CityIndex total_cities_read = 0;
    bool indiv_done = false;

    EXTRACT_SUBTOURS_LOOP: while (!indiv_done) {
        hls::write_lock<Subtour> tour(tour_stream);
        CityIndex tour_len = 0;
        bool subtour_done = false;

        BUILD_TOUR_LOOP: while (!subtour_done) {
            #pragma HLS PIPELINE II=1

            City current_city = active_beat.range(CITY_BITS - 1, 0);
            NodeId nid = get_node_id(current_city);
            bool is_depot = get_is_depot(current_city);

            tour[tour_len] = nid;
            tour_len++;
            total_cities_read++;

            if (is_depot) {
                subtour_done = true;
            }
            if (total_cities_read == num_cities) {
                subtour_done = true;
                indiv_done = true;
            }

            if (cities_in_beat == 1) {
                beat_idx++;
                if (total_cities_read < num_cities) {
                    active_beat = in_ind[beat_idx];
                }
                cities_in_beat = HW_CITIES_PER_BEAT;
            } else {
                active_beat >>= CITY_BITS;
                cities_in_beat--;
            }
        }

        meta_stream.write(SubtourMeta{tour_len, indiv_done});
    }
}

static void symmetric_sweep(
    NodeId tour_buf[MAX_CITIES],
    const CityData city_cache[MAX_CACHE_ENTRIES],
    CityIndex start, CityIndex end,
    CityIndex L, CityIndex R,
    NodeId DEPOT,
    Fitness &sweep_delta
) {
    CityIndex M = (R - L + 1) / 2;
    Fitness local_delta = 0;
    bool swap_flag = false;

    bool improves[MAX_CITIES / 2];

    NodeId prev_node_left  = (L == start) ? DEPOT : tour_buf[L - 1];
    NodeId prev_node_right = (R == end)   ? DEPOT : tour_buf[R + 1];

    Point prev_coord_left  = get_coord(city_cache[prev_node_left]);
    Point prev_coord_right = get_coord(city_cache[prev_node_right]);

    NodeId tmp[MAX_CITIES];
    #pragma HLS BIND_STORAGE variable=tmp type=ram_t2p impl=bram

    EVAL_LOOP: for (CityIndex i = 0; i < M; ++i) {
        #pragma HLS PIPELINE II=1

        NodeId curr_node_left  = tour_buf[L + i];
        NodeId curr_node_right = tour_buf[R - i];
        tmp[L+i] = curr_node_left;
        tmp[R-i] = curr_node_right;
        Point curr_coord_left  = get_coord(city_cache[curr_node_left]);
        Point curr_coord_right = get_coord(city_cache[curr_node_right]);

        Fitness d_old_left  = floored_distance(prev_coord_left, curr_coord_left);
        Fitness d_old_right = floored_distance(curr_coord_right, prev_coord_right);
        Fitness d_new_left  = floored_distance(prev_coord_left, curr_coord_right);
        Fitness d_new_right = floored_distance(curr_coord_left, prev_coord_right);

        Fitness d_old = d_old_left + d_old_right;
        Fitness d_new = d_new_left + d_new_right;
        bool imp = (d_old > d_new);
        improves[i] = imp;

        if (imp) {
            local_delta += (d_old - d_new);
        }

        prev_coord_left  = curr_coord_left;
        prev_coord_right = curr_coord_right;
    }

    APPLY_LOOP: for (CityIndex i = 0; i < M; ++i) {
        #pragma HLS PIPELINE II=1
        #pragma HLS DEPENDENCE variable=tour_buf inter false

        swap_flag ^= improves[i];

        if (swap_flag) {
            NodeId curr_node_left  = tmp[L + i];
            NodeId curr_node_right = tmp[R - i];
            tour_buf[L + i] = curr_node_right;
            tour_buf[R - i] = curr_node_left;
        }
    }

    sweep_delta = local_delta;
}

static void two_opt_tours(
    hls::stream_of_blocks<Subtour> &tour_stream,
    hls::stream<SubtourMeta> &meta_stream,
    hls::stream_of_blocks<Subtour> &opt_tour_stream,
    hls::stream<SubtourMeta> &opt_meta_stream,
    hls::stream<Fitness> &delta_stream,
    hls::stream<RandomNumber> &random_stream,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    const CityData city_cache[MAX_CACHE_ENTRIES]
) {
    Fitness total_delta = 0;
    bool indiv_done = false;
    CityIndex subtours_processed = 0;

    OPTIMIZE_SUBTOURS_LOOP: while (!indiv_done) {
        SubtourMeta meta = meta_stream.read();
        hls::read_lock<Subtour> tour(tour_stream);
        hls::write_lock<Subtour> opt_tour(opt_tour_stream);

        subtours_processed++;

        CityIndex len = meta.length;
        bool is_last = meta.last;
        indiv_done = is_last;

        COPY_TOUR_LOOP: for (CityIndex i = 0; i < len; ++i) {
            #pragma HLS PIPELINE II=1
            opt_tour[i] = tour[i];
        }

        MOVE_LOOP: for (CityIndex move = 0; move < two_opt_k; ++move) {
            RandomNumber r1 = random_stream.read();
            RandomNumber r2 = random_stream.read();

            if (len >= 3) {
                CityIndex L_rel = r1 % len;
                CityIndex R_rel = r2 % len;

                if (L_rel > R_rel) {
                    CityIndex tmp = L_rel;
                    L_rel = R_rel;
                    R_rel = tmp;
                }

                if (L_rel != R_rel) {
                    Fitness sweep_delta = 0;
                    symmetric_sweep(
                        opt_tour,
                        city_cache,
                        0, len - 1,
                        L_rel, R_rel,
                        num_cities,
                        sweep_delta
                    );
                    total_delta += sweep_delta;
                }
            }
        }

        opt_meta_stream.write(meta);
    }

    delta_stream.write(total_delta);

    TwoCityIndex remaining_subtours = (subtours_processed < max_subtours) ? TwoCityIndex(max_subtours - subtours_processed) : TwoCityIndex(0);
    TwoCityIndex total_drain_moves = remaining_subtours * TwoCityIndex(two_opt_k);
    DRAIN_SUBTOURS_LOOP: for (TwoCityIndex m = 0; m < total_drain_moves; ++m) {
        #pragma HLS PIPELINE II=2
        random_stream.read();
        random_stream.read();
    }
}

static void array_to_beats(
    hls::stream_of_blocks<Subtour> &opt_tour_stream,
    hls::stream<SubtourMeta> &opt_meta_stream,
    hls::stream<BeatPacket> &beat_stream,
    CityIndex num_cities
) {
    BeatProducerState state;
    bool indiv_done = false;

    REBUILD_SUBTOURS_LOOP: while (!indiv_done) {
        SubtourMeta meta = opt_meta_stream.read();
        hls::read_lock<Subtour> opt_tour(opt_tour_stream);
        CityIndex len = meta.length;
        indiv_done = meta.last;

        REBUILD_TOUR_LOOP: for (CityIndex i = 0; i < len; ++i) {
            #pragma HLS PIPELINE II=1
            City city = 0;
            set_node_id(city, opt_tour[i]);
            set_is_depot(city, (i == len - 1));
            send_city(state, city, beat_stream);
        }
    }
    finalizeStream(state, beat_stream);
}

static void tours_to_individual(
    hls::stream_of_blocks<Subtour> &opt_tour_stream,
    hls::stream<SubtourMeta> &opt_meta_stream,
    IndividualAsBeats &out_ind,
    CityIndex num_cities
) {
    #pragma HLS DATAFLOW
    hls::stream<BeatPacket> beat_stream;

    array_to_beats(opt_tour_stream, opt_meta_stream, beat_stream, num_cities);

    StartEndData se = get_start_end_data(0, num_cities - 1);
    StartEndDataWithBeats se_wb(se, out_ind[se.start_beat], out_ind[se.end_beat]);
    stream_beats_to_individual<BEAT_STREAM_INTERVAL>(
        beat_stream, out_ind, 0, num_cities, se_wb, num_cities
    );
}

static void compute_final_fitness(
    hls::stream<Fitness> &fitness_stream,
    hls::stream<Fitness> &delta_stream,
    Fitness &new_fitness
) {
    Fitness old_fitness = fitness_stream.read();
    Fitness total_delta = delta_stream.read();
    new_fitness = old_fitness - total_delta;
}

static void two_opt_individual_dataflow(
    const IndividualAsBeats &in_ind,
    IndividualAsBeats &out_ind,
    hls::stream<RandomNumber> &random_stream,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    const CityData city_cache[MAX_CACHE_ENTRIES],
    Fitness &new_fitness
) {
    #pragma HLS DATAFLOW
    hls::stream_of_blocks<Subtour> tour_stream;
    hls::stream<SubtourMeta> meta_stream;
    hls::stream_of_blocks<Subtour> opt_tour_stream;
    hls::stream<SubtourMeta> opt_meta_stream;
    hls::stream<Fitness> fitness_stream;
    hls::stream<Fitness> delta_stream;

    individual_to_tours(in_ind, num_cities, tour_stream, meta_stream, fitness_stream);
    two_opt_tours(tour_stream, meta_stream, opt_tour_stream, opt_meta_stream, delta_stream, random_stream, num_cities, two_opt_k, max_subtours, city_cache);
    tours_to_individual(opt_tour_stream, opt_meta_stream, out_ind, num_cities);
    compute_final_fitness(fitness_stream, delta_stream, new_fitness);
}

void two_opt(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    hls::stream<RandomNumber> &random_stream,
    int num_individuals,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    const CityData city_cache[MAX_CACHE_ENTRIES]
) {
    for (int i = 0; i < num_individuals; ++i) {
        hls::read_lock<IndividualAsBeats> in_ind(in);
        hls::write_lock<IndividualAsBeats> out_ind(out);
        Fitness new_fitness;
        two_opt_individual_dataflow(in_ind, out_ind, random_stream, num_cities, two_opt_k, max_subtours, city_cache, new_fitness);
        set_fitness(out_ind, new_fitness, num_cities);
    }
}
