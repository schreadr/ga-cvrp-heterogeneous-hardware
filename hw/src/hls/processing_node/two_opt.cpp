#include "two_opt.hpp"
#include "util.hpp"

typedef NodeId Subtour[MAX_CITIES];
typedef ap_uint<N_TWO_OPT_WORKERS_LOG2  + 1> WorkerIndex;

struct SubtourMeta {
    CityIndex length;
    Fitness delta;
    bool last_of_indiv;
    bool last_of_batch;
};

struct SubtourDispatchToken {
    WorkerIndex worker_idx;
    CityIndex subtours_processed;
    bool is_dummy;
    bool last_of_indiv;
    bool last_of_batch;
};

static inline bool evaluate_interval_step(
    Point prev_coord_left,
    Point curr_coord_left,
    Point curr_coord_right,
    Point prev_coord_right,
    Fitness &d_old,
    Fitness &d_new
) {
    #pragma HLS INLINE
    Fitness d_old_left  = floored_distance(prev_coord_left, curr_coord_left);
    Fitness d_old_right = floored_distance(curr_coord_right, prev_coord_right);
    Fitness d_new_left  = floored_distance(prev_coord_left, curr_coord_right);
    Fitness d_new_right = floored_distance(curr_coord_left, prev_coord_right);

    d_old = d_old_left + d_old_right;
    d_new = d_new_left + d_new_right;
    return (d_old > d_new);
}

static void symmetric_sweep(
    NodeId tour_buf[MAX_CITIES],
    const CityData city_cache[MAX_CACHE_ENTRIES],
    CityIndex start, CityIndex end,
    CityIndex L, CityIndex R,
    NodeId DEPOT,
    Fitness &sweep_delta,
    bool enable_precheck
) {
    CityIndex M = (R - L + 1) / 2;
    Fitness local_delta = 0;
    bool swap_flag = false;

    NodeId prev_node_left  = (L == start) ? DEPOT : tour_buf[L - 1];
    NodeId prev_node_right = (R == end)   ? DEPOT : tour_buf[R + 1];

    Point prev_coord_left  = get_coord(city_cache[prev_node_left]);
    Point prev_coord_right = get_coord(city_cache[prev_node_right]);

    if (enable_precheck) {
        NodeId curr_node_left0  = tour_buf[L];
        NodeId curr_node_right0 = tour_buf[R];
        Point curr_coord_left0  = get_coord(city_cache[curr_node_left0]);
        Point curr_coord_right0 = get_coord(city_cache[curr_node_right0]);

        Fitness d_old0, d_new0;
        bool imp0 = evaluate_interval_step(
            prev_coord_left,
            curr_coord_left0,
            curr_coord_right0,
            prev_coord_right,
            d_old0,
            d_new0
        );

        if (!imp0) {
            sweep_delta = 0;
            return;
        }
    }

    bool improves[MAX_CITIES / 2];
    NodeId tmp_left[MAX_CITIES / 2];
    NodeId tmp_right[MAX_CITIES / 2];

    EVAL_LOOP: for (CityIndex i = 0; i < M; ++i) {
        #pragma HLS PIPELINE II=1

        NodeId curr_node_left  = tour_buf[L + i];
        NodeId curr_node_right = tour_buf[R - i];
        tmp_left[i]  = curr_node_left;
        tmp_right[i] = curr_node_right;
        Point curr_coord_left  = get_coord(city_cache[curr_node_left]);
        Point curr_coord_right = get_coord(city_cache[curr_node_right]);

        Fitness d_old, d_new;
        bool imp = evaluate_interval_step(
            prev_coord_left,
            curr_coord_left,
            curr_coord_right,
            prev_coord_right,
            d_old,
            d_new
        );
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
            NodeId curr_node_left  = tmp_left[i];
            NodeId curr_node_right = tmp_right[i];
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
    hls::stream<RandomPair> &worker_random_stream,
    CityIndex num_cities,
    CityIndex two_opt_k,
    bool enable_precheck,
    const CityData city_cache[MAX_CACHE_ENTRIES]
) {
    bool batch_done = false;

    OPTIMIZE_SUBTOURS_LOOP: while (!batch_done) {
        SubtourMeta meta = meta_stream.read();
        hls::read_lock<Subtour> tour(tour_stream);
        hls::write_lock<Subtour> opt_tour(opt_tour_stream);

        CityIndex len = meta.length;
        batch_done = meta.last_of_batch;
        Fitness subtour_delta = 0;

        COPY_TOUR_LOOP: for (CityIndex i = 0; i < len; ++i) {
            #pragma HLS PIPELINE II=1
            opt_tour[i] = tour[i];
        }

        if (len > 0) {
            MOVE_LOOP: for (CityIndex move = 0; move < two_opt_k; ++move) {
                RandomPair rnd = worker_random_stream.read();
                RandomNumber r1 = rnd.r1;
                RandomNumber r2 = rnd.r2;

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
                            sweep_delta,
                            enable_precheck
                        );
                        subtour_delta += sweep_delta;
                    }
                }
            }
        }

        opt_meta_stream.write(SubtourMeta{len, subtour_delta, meta.last_of_indiv, batch_done});
    }
}

template <int N_WORKERS>
static void send_subtour_to_worker(
    WorkerIndex target_worker,
    bool is_dummy,
    bool last_of_indiv,
    bool last_of_batch,
    hls::stream_of_blocks<Subtour> (&tour_stream)[N_WORKERS],
    hls::stream<SubtourMeta> (&meta_stream)[N_WORKERS],
    const IndividualAsBeats &in_ind,
    Beat &active_beat,
    CityBeatIndex &cities_in_beat,
    BeatIndex &beat_idx,
    CityIndex &total_cities_read,
    CityIndex num_cities,
    bool &indiv_done
) {
    #pragma HLS INLINE
    DISPATCH_EXTRACT_LOOP: for (WorkerIndex w = 0; w < N_WORKERS; ++w) {
        #pragma HLS UNROLL
        if (target_worker == w) {
            hls::write_lock<Subtour> tour(tour_stream[w]);
            CityIndex tour_len = 0;

            if (!is_dummy) {
                bool subtour_done = false;

                BUILD_TOUR_LOOP: while (!subtour_done) {
                    #pragma HLS PIPELINE II=1

                    City current_city = active_beat.range(CITY_BITS - 1, 0);
                    NodeId nid = get_node_id(current_city);
                    bool is_depot = get_is_depot(current_city);

                    tour[tour_len++] = nid;
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

                meta_stream[w].write(SubtourMeta{tour_len, 0, indiv_done, false});
            } else {
                meta_stream[w].write(SubtourMeta{0, 0, last_of_indiv, last_of_batch});
            }
        }
    }
}

template <int N_WORKERS>
static void extract_subtours_all(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    int num_individuals,
    CityIndex num_cities,
    hls::stream_of_blocks<Subtour> (&tour_stream)[N_WORKERS],
    hls::stream<SubtourMeta> (&meta_stream)[N_WORKERS],
    hls::stream<Fitness> &fitness_stream,
    hls::stream<SubtourDispatchToken> &subtour_token_stream
) {
    WorkerIndex worker_idx = 0;

    EXTRACT_INDIV_LOOP: for (int indiv = 0; indiv < num_individuals; ++indiv) {
        hls::read_lock<IndividualAsBeats> in_ind(in);

        Fitness old_fitness = get_fitness(in_ind, num_cities);
        fitness_stream.write(old_fitness);

        Beat active_beat = in_ind[0];
        CityBeatIndex cities_in_beat = HW_CITIES_PER_BEAT;
        BeatIndex beat_idx = 0;

        CityIndex total_cities_read = 0;
        bool indiv_done = false;
        CityIndex subtours_processed = 0;

        EXTRACT_SUBTOURS_LOOP: while (!indiv_done) {
            send_subtour_to_worker<N_WORKERS>(
                worker_idx, false, false, false,
                tour_stream, meta_stream,
                in_ind, active_beat, cities_in_beat, beat_idx,
                total_cities_read, num_cities, indiv_done
            );
            subtours_processed++;
            subtour_token_stream.write(SubtourDispatchToken{
                worker_idx,
                subtours_processed,
                false,
                indiv_done,
                false
            });
            worker_idx = (worker_idx == N_WORKERS - 1) ? WorkerIndex(0) : WorkerIndex(worker_idx + 1);
        }
    }

    // 1. Balance the final round if worker_idx != 0 so all workers have an equal number of blocks
    if (worker_idx != 0) {
        Beat active_beat = 0;
        CityBeatIndex cities_in_beat = 0;
        BeatIndex beat_idx = 0;
        CityIndex total_cities_read = 0;
        bool dummy_done = false;
        IndividualAsBeats dummy_ind;

        BALANCE_FINAL_ROUND_LOOP: while (worker_idx != 0) {
            send_subtour_to_worker<N_WORKERS>(
                worker_idx, true, false, false,
                tour_stream, meta_stream,
                dummy_ind, active_beat, cities_in_beat, beat_idx,
                total_cities_read, num_cities, dummy_done
            );
            subtour_token_stream.write(SubtourDispatchToken{
                worker_idx,
                0,
                true,
                false,
                false
            });
            worker_idx = (worker_idx == N_WORKERS - 1) ? WorkerIndex(0) : WorkerIndex(worker_idx + 1);
        }
    }

    // 2. Final batch termination round: send one dummy block with last_of_batch = true to each worker
    Beat active_beat = 0;
    CityBeatIndex cities_in_beat = 0;
    BeatIndex beat_idx = 0;
    CityIndex total_cities_read = 0;
    bool dummy_done = false;
    IndividualAsBeats dummy_ind;

    TERMINATION_BATCH_LOOP: for (WorkerIndex w = 0; w < N_WORKERS; ++w) {
        send_subtour_to_worker<N_WORKERS>(
            w, true, false, true,
            tour_stream, meta_stream,
            dummy_ind, active_beat, cities_in_beat, beat_idx,
            total_cities_read, num_cities, dummy_done
        );
        subtour_token_stream.write(SubtourDispatchToken{
            w,
            0,
            true,
            false,
            (w == N_WORKERS - 1)
        });
    }
}

template <int N_WORKERS>
static void dispatch_randoms_all(
    hls::stream<RandomPair> &random_stream,
    hls::stream<SubtourDispatchToken> &subtour_token_stream,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    hls::stream<RandomPair> (&worker_random_stream)[N_WORKERS]
) {
    bool batch_done = false;

    DISPATCH_RANDOMS_LOOP: while (!batch_done) {
        SubtourDispatchToken token = subtour_token_stream.read();
        batch_done = token.last_of_batch;

        if (!token.is_dummy) {
            FORWARD_RND_LOOP: for (CityIndex move = 0; move < two_opt_k; ++move) {
                #pragma HLS PIPELINE II=1
                RandomPair pair = random_stream.read();
                DISPATCH_RND_WORKER_LOOP: for (WorkerIndex w = 0; w < N_WORKERS; ++w) {
                    #pragma HLS UNROLL
                    if (token.worker_idx == w) {
                        worker_random_stream[w].write(pair);
                    }
                }
            }
        }

        if (token.last_of_indiv) {
            TwoCityIndex remaining_subtours = (token.subtours_processed < max_subtours) ?
                TwoCityIndex(max_subtours - token.subtours_processed) : TwoCityIndex(0);
            TwoCityIndex total_drain_moves = remaining_subtours * TwoCityIndex(two_opt_k);
            DRAIN_RANDOMS_LOOP: for (TwoCityIndex m = 0; m < total_drain_moves; ++m) {
                #pragma HLS PIPELINE II=1
                random_stream.read();
            }
        }
    }
}

template <int N_WORKERS>
static void read_subtour_to_beats(
    WorkerIndex target_worker,
    hls::stream_of_blocks<Subtour> (&opt_tour_stream)[N_WORKERS],
    hls::stream<SubtourMeta> (&opt_meta_stream)[N_WORKERS],
    BeatProducerState &state,
    hls::stream<BeatPacket> &beat_stream,
    Fitness &indiv_delta,
    bool &indiv_done
) {
    #pragma HLS INLINE
    DISPATCH_READ_LOOP: for (WorkerIndex w = 0; w < N_WORKERS; ++w) {
        #pragma HLS UNROLL
        if (target_worker == w) {
            SubtourMeta meta = opt_meta_stream[w].read();
            hls::read_lock<Subtour> opt_tour(opt_tour_stream[w]);
            CityIndex len = meta.length;
            indiv_done = meta.last_of_indiv;
            indiv_delta += meta.delta;

            REBUILD_TOUR_LOOP: for (CityIndex i = 0; i < len; ++i) {
                #pragma HLS PIPELINE II=1
                City city = 0;
                set_node_id(city, opt_tour[i]);
                set_is_depot(city, (i == len - 1));
                send_city(state, city, beat_stream);
            }
        }
    }
}

template <int N_WORKERS>
static void array_to_beats_all(
    hls::stream_of_blocks<Subtour> (&opt_tour_stream)[N_WORKERS],
    hls::stream<SubtourMeta> (&opt_meta_stream)[N_WORKERS],
    hls::stream<BeatPacket> &beat_stream,
    hls::stream<Fitness> &delta_stream,
    int num_individuals,
    CityIndex num_cities
) {
    WorkerIndex worker_idx = 0;

    ARRAY_TO_BEATS_INDIV_LOOP: for (int indiv = 0; indiv < num_individuals; ++indiv) {
        BeatProducerState state;
        Fitness indiv_delta = 0;
        bool indiv_done = false;

        REBUILD_SUBTOURS_LOOP: while (!indiv_done) {
            read_subtour_to_beats<N_WORKERS>(
                worker_idx, opt_tour_stream, opt_meta_stream,
                state, beat_stream, indiv_delta, indiv_done
            );
            worker_idx = (worker_idx == N_WORKERS - 1) ? WorkerIndex(0) : WorkerIndex(worker_idx + 1);
        }

        finalizeStream(state, beat_stream);
        delta_stream.write(indiv_delta);
    }

    // Drain final balancing round if needed
    if (worker_idx != 0) {
        BeatProducerState dummy_state;
        Fitness dummy_delta = 0;
        bool dummy_done = false;
        DRAIN_FINAL_BALANCE_LOOP: while (worker_idx != 0) {
            read_subtour_to_beats<N_WORKERS>(
                worker_idx, opt_tour_stream, opt_meta_stream,
                dummy_state, beat_stream, dummy_delta, dummy_done
            );
            worker_idx = (worker_idx == N_WORKERS - 1) ? WorkerIndex(0) : WorkerIndex(worker_idx + 1);
        }
    }

    // Drain final batch termination round
    DRAIN_BATCH_TERMINATION_LOOP: for (WorkerIndex w = 0; w < N_WORKERS; ++w) {
        BeatProducerState dummy_state;
        Fitness dummy_delta = 0;
        bool dummy_done = false;
        read_subtour_to_beats<N_WORKERS>(
            w, opt_tour_stream, opt_meta_stream,
            dummy_state, beat_stream, dummy_delta, dummy_done
        );
    }
}

static void stream_beats_to_all_individuals(
    hls::stream<BeatPacket> &beat_stream,
    hls::stream<Fitness> &fitness_stream,
    hls::stream<Fitness> &delta_stream,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    int num_individuals,
    CityIndex num_cities
) {
    StartEndData se = get_start_end_data(0, num_cities - 1);
    StartEndDataWithBeats se_wb(se, 0, 0);

    STREAM_BEATS_INDIV_LOOP: for (int indiv = 0; indiv < num_individuals; ++indiv) {
        hls::write_lock<IndividualAsBeats> out_ind(out);

        stream_beats_to_individual<BEAT_STREAM_INTERVAL>(
            beat_stream, out_ind, 0, num_cities, se_wb, num_cities
        );

        Fitness old_fitness = fitness_stream.read();
        Fitness total_delta = delta_stream.read();
        set_fitness(out_ind, old_fitness - total_delta, num_cities);
    }
}

static void init_local_cache(
    const CityData city_cache[MAX_CACHE_ENTRIES],
    CityData local_cache[N_TWO_OPT_WORKERS][MAX_CACHE_ENTRIES]
) {
    INIT_CACHE_ENTRIES: for (int c = 0; c < MAX_CACHE_ENTRIES; ++c) {
        #pragma HLS PIPELINE II=1
        CityData d = city_cache[c];
        INIT_CACHE_WORKERS: for (WorkerIndex w = 0; w < N_TWO_OPT_WORKERS; ++w) {
            #pragma HLS UNROLL
            local_cache[w][c] = d;
        }
    }
}

static void two_opt_streaming(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    hls::stream<RandomPair> &random_stream,
    int num_individuals,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    bool enable_precheck,
    const CityData local_cache[N_TWO_OPT_WORKERS][MAX_CACHE_ENTRIES]
) {
    #pragma HLS DATAFLOW

    hls::stream_of_blocks<Subtour> tour_stream[N_TWO_OPT_WORKERS];
    hls::stream<SubtourMeta>       meta_stream[N_TWO_OPT_WORKERS];

    hls::stream_of_blocks<Subtour> opt_tour_stream[N_TWO_OPT_WORKERS];
    hls::stream<SubtourMeta>       opt_meta_stream[N_TWO_OPT_WORKERS];

    hls::stream<RandomPair>        worker_random_stream[N_TWO_OPT_WORKERS];
    #pragma HLS STREAM variable=worker_random_stream depth=MAX_TWO_OPT_K

    hls::stream<SubtourDispatchToken> subtour_token_stream;
    #pragma HLS STREAM variable=subtour_token_stream depth=2 * N_TWO_OPT_WORKERS

    hls::stream<BeatPacket>        beat_stream;
    hls::stream<Fitness>           fitness_stream;
    #pragma HLS STREAM variable=fitness_stream depth=16
    hls::stream<Fitness>           delta_stream;
    #pragma HLS STREAM variable=delta_stream depth=16

    extract_subtours_all<N_TWO_OPT_WORKERS>(
        in, num_individuals, num_cities,
        tour_stream, meta_stream, fitness_stream, subtour_token_stream
    );

    dispatch_randoms_all<N_TWO_OPT_WORKERS>(
        random_stream, subtour_token_stream, two_opt_k, max_subtours,
        worker_random_stream
    );

    TWO_OPT_WORKERS_LOOP: for (WorkerIndex w = 0; w < N_TWO_OPT_WORKERS; ++w) {
        #pragma HLS UNROLL
        two_opt_tours(
            tour_stream[w], meta_stream[w],
            opt_tour_stream[w], opt_meta_stream[w],
            worker_random_stream[w],
            num_cities, two_opt_k,
            enable_precheck,
            local_cache[w]
        );
    }

    array_to_beats_all<N_TWO_OPT_WORKERS>(
        opt_tour_stream, opt_meta_stream, beat_stream, delta_stream,
        num_individuals, num_cities
    );

    stream_beats_to_all_individuals(
        beat_stream, fitness_stream, delta_stream, out,
        num_individuals, num_cities
    );
}

void two_opt(
    hls::stream_of_blocks<IndividualAsBeats> &in,
    hls::stream_of_blocks<IndividualAsBeats> &out,
    hls::stream<RandomPair> &random_stream,
    int num_individuals,
    CityIndex num_cities,
    CityIndex two_opt_k,
    CityIndex max_subtours,
    bool enable_precheck,
    const CityData city_cache[MAX_CACHE_ENTRIES]
) {
    CityData local_cache[N_TWO_OPT_WORKERS][MAX_CACHE_ENTRIES];
    #pragma HLS ARRAY_PARTITION variable=local_cache complete dim=1
    #pragma HLS BIND_STORAGE variable=local_cache type=ram_t2p impl=bram

    init_local_cache(city_cache, local_cache);

    two_opt_streaming(
        in, out, random_stream,
        num_individuals, num_cities, two_opt_k, max_subtours,
        enable_precheck,
        local_cache
    );
}
