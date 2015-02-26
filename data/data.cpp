#include <data/data.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <utility>

#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/range/algorithm/count.hpp>

using namespace boost::property_tree;
using namespace boost;

// Allocation of static variables:
constexpr std::array<char, 6> segments::valid_types;
constexpr unsigned int trains::heavy_weight;
constexpr char trains::first_train_class;
constexpr char trains::last_train_class;

auto data::create_speeds(const ptree& pt) -> void {
    spd = speeds(   pt.get<double>("speed_ew"),
                    pt.get<double>("speed_we"),
                    pt.get<double>("speed_siding"),
                    pt.get<double>("speed_switch"),
                    pt.get<double>("speed_xover"));
                    
    assert(spd.ew > 0);
    assert(spd.we > 0);
    assert(spd.siding > 0);
    assert(spd.swi > 0);
    assert(spd.xover > 0);
}

auto data::create_time_windows(const ptree& pt) -> void {
    tiw = time_windows( pt.get<unsigned int>("want_time_tw_start"),
                        pt.get<unsigned int>("want_time_tw_end"),
                        pt.get<unsigned int>("schedule_tw_end"));
    
    assert(tiw.wt_left < ni);
    assert(tiw.wt_right < ni);
    assert(tiw.sa_right < ni);
}

auto data::create_prices(const ptree& pt) -> void {
    auto delay = prices::delay_price_map();
    
    for(char cl = 'A'; cl <= 'F'; cl++) {
        auto price = pt.get_child("general_delay_price").get<double>(std::string(1,cl));
        
        assert(price > 0);
        
        delay.emplace(cl, price);
    }
    
    pri = prices(   pt.get<double>("terminal_delay_price"),
                    pt.get<double>("schedule_delay_price"),
                    pt.get<double>("unpreferred_price"),
                    delay);
                    
    assert(pri.wt > 0);
    assert(pri.sa > 0);
    assert(pri.unpreferred > 0);
}

auto data::create_segments(const ptree& pt) -> void {
    auto e_ext = uint_vector();
    auto w_ext = uint_vector();
    auto e_min_dist = double_vector();
    auto w_min_dist = double_vector();
    auto length = double_vector();
    auto original_length = double_vector();
    auto type = char_vector();
    auto is_eastbound = bool_vector();
    auto is_westbound = bool_vector();
    
    static constexpr unsigned int dummy = 999999u;
    
    // Sigma:
    e_ext.push_back(dummy - 1);
    w_ext.push_back(dummy - 2);
    e_min_dist.push_back(dummy - 3);
    w_min_dist.push_back(dummy - 4);
    length.push_back(dummy - 5);
    original_length.push_back(dummy - 6);
    type.push_back('D');
    is_eastbound.push_back(false);
    is_westbound.push_back(false);
    
    BOOST_FOREACH(const ptree::value_type& segment_child, pt.get_child("segments")) {
        e_ext.push_back(segment_child.second.get<unsigned int>("extreme_2"));
        w_ext.push_back(segment_child.second.get<unsigned int>("extreme_1"));
        e_min_dist.push_back(segment_child.second.get<double>("min_distance_from_e"));
        w_min_dist.push_back(segment_child.second.get<double>("min_distance_from_w"));
        type.push_back(segment_child.second.get<char>("type"));
        length.push_back(segment_child.second.get<double>("length"));
        
        if(type.back() == 'S') {
            original_length.push_back(segment_child.second.get<double>("siding_length"));
        } else {
            original_length.push_back(length.back());
        }
        
        is_eastbound.push_back(segment_child.second.get<bool>("eastbound"));
        is_westbound.push_back(segment_child.second.get<bool>("westbound"));
        
        assert(e_min_dist.back() >= 0);
        assert(w_min_dist.back() >= 0);
        assert(length.back() > 0);
        assert(original_length.back() <= length.back());
        assert(std::find(segments::valid_types.begin(), segments::valid_types.end(), type.back()) != segments::valid_types.end());
        assert(is_eastbound.back() || is_westbound.back());
    }
    
    // Tau:
    e_ext.push_back(dummy + 1);
    w_ext.push_back(dummy + 2);
    e_min_dist.push_back(dummy + 3);
    w_min_dist.push_back(dummy + 4);
    length.push_back(dummy + 5);
    original_length.push_back(dummy + 6);
    type.push_back('D');
    is_eastbound.push_back(false);
    is_westbound.push_back(false);
    
    seg = segments( e_ext,
                    w_ext,
                    e_min_dist,
                    w_min_dist,
                    length,
                    original_length,
                    type,
                    is_eastbound,
                    is_westbound);
}

auto data::create_mows(const ptree& pt) -> void {
    auto e_ext = uint_vector();
    auto w_ext = uint_vector();
    auto start_time = uint_vector();
    auto end_time = uint_vector();
    
    // is_mow empty for now
    auto is_mow = bool_matrix_2d(ns + 2, bool_vector(ni + 2, false));
    
    BOOST_FOREACH(const ptree::value_type& mow_child, pt.get_child("mow")) {
        e_ext.push_back(mow_child.second.get<unsigned int>("extreme_2"));
        w_ext.push_back(mow_child.second.get<unsigned int>("extreme_1"));
        start_time.push_back(mow_child.second.get<unsigned int>("start_time"));
        end_time.push_back(mow_child.second.get<unsigned int>("end_time"));
        
        assert(start_time.back() < ni);
        assert(end_time.back() < ni);
        assert(end_time.back() - start_time.back() >= 0u);
    }
    
    mnt = mows( e_ext,
                w_ext,
                start_time,
                end_time,
                is_mow);
                
    calculate_is_mow();
}

auto data::calculate_is_mow() -> void {
    for(auto m = 0u; m < mnt.e_ext.size(); m++) {
        for(auto s = 0u; s <= ns + 1; s++) {
            if(seg.e_ext.at(s) == mnt.e_ext.at(m) && seg.w_ext.at(s) == mnt.w_ext.at(m)) {
                for(auto t = mnt.start_time.at(m); t <= mnt.end_time.at(m); t++) {
                    mnt.is_mow[s][t] = true;
                }
            }
        }
    }
}

auto data::create_trains(const ptree& pt) -> void {
    auto want_time = uint_vector();
    auto entry_time = uint_vector();
    auto tob = uint_vector();
    auto speed_multi = double_vector();
    auto speed_max = double_vector();
    auto length = double_vector();
    auto sa_num = uint_vector();
    auto type = char_vector();
    auto is_sa = bool_vector();
    auto is_heavy = bool_vector();
    auto is_eastbound = bool_vector();
    auto is_westbound = bool_vector();
    auto is_hazmat = bool_vector();
    auto orig_ext = uint_vector();
    auto dest_ext = uint_vector();
    auto orig_segs = uint_matrix_2d();
    auto dest_segs = uint_matrix_2d();
    auto unpreferred_segs = uint_matrix_2d();
    auto sa_times = uint_matrix_2d();
    auto sa_ext = uint_matrix_2d();
    auto sa_segs = uint_matrix_3d();
    auto first_time_we_need_tau = uint_vector();
    
    BOOST_FOREACH(const ptree::value_type& train_child, pt.get_child("trains")) {
        want_time.push_back(train_child.second.get<unsigned int>("terminal_wt"));
        entry_time.push_back(train_child.second.get<unsigned int>("entry_time"));
        tob.push_back(train_child.second.get<unsigned int>("tob"));
        speed_multi.push_back(train_child.second.get<double>("speed_multi"));
        
        // speed_max empty for now
        speed_max.push_back(0.0);
        
        length.push_back(train_child.second.get<double>("length"));
        type.push_back(train_child.second.get<char>("class"));
        is_sa.push_back(train_child.second.get<bool>("schedule_adherence"));
        is_heavy.push_back(tob.back() > trains::heavy_weight);
        is_eastbound.push_back(train_child.second.get<bool>("eastbound"));
        is_westbound.push_back(train_child.second.get<bool>("westbound"));
        is_hazmat.push_back(train_child.second.get<bool>("hazmat"));
        orig_ext.push_back(train_child.second.get<unsigned int>("origin_node"));
        dest_ext.push_back(train_child.second.get<unsigned int>("destination_node"));
        
        // orig_segs, dest_segs, unpreferred_segs empty for now
        orig_segs.push_back(uint_vector());
        dest_segs.push_back(uint_vector());
        unpreferred_segs.push_back(uint_vector(ns + 2, false));
        
        sa_ext.push_back(uint_vector());
        sa_times.push_back(uint_vector());
        BOOST_FOREACH(const ptree::value_type& schedule_child, train_child.second.get_child("schedule")) {
            auto extreme = schedule_child.second.get<unsigned int>("node");
            auto ti = schedule_child.second.get<unsigned int>("time");
            
            if(ti <= ni) {
                sa_ext.back().push_back(extreme);
                sa_times.back().push_back(ti);
            }
        }
        
        sa_num.push_back(sa_ext.back().size());
        
        // sa_segs empty for now
        sa_segs.push_back(uint_matrix_2d(sa_num.back(), uint_vector()));
        
        // first_time_we_need_tau empty for now
        first_time_we_need_tau.push_back(0u);
        
        assert(want_time.back() < ni);
        assert(entry_time.back() < ni);
        assert(want_time.back() - entry_time.back() > 0u);
        assert(speed_multi.back() > 0);
        assert(length.back() > 0);
        assert(type.back() >= trains::first_train_class && type.back() <= trains::last_train_class);
        assert(is_sa.back() != (sa_num.back() == 0));
        assert(is_eastbound.back() != is_westbound.back());
    }

    trn = trains(   want_time,
                    entry_time,
                    tob,
                    speed_multi,
                    speed_max,
                    length,
                    sa_num,
                    type,
                    is_sa,
                    is_heavy,
                    is_eastbound,
                    is_westbound,
                    is_hazmat,
                    orig_ext,
                    dest_ext,
                    orig_segs,
                    dest_segs,
                    unpreferred_segs,
                    sa_times,
                    sa_ext,
                    sa_segs,
                    first_time_we_need_tau);
    
    calculate_trains_max_speeds();
    calculate_trains_origin_and_destination_segments();
    calculate_trains_unpreferred_segments();
    calculate_trains_sa_segments();
}

auto data::calculate_trains_max_speeds() -> void {
    for(auto i = 0u; i < nt; i++) {
        trn.speed_max[i] = trn.speed_multi.at(i) * (trn.is_westbound.at(i) ? spd.ew : spd.we);
    }
}

auto data::calculate_trains_origin_and_destination_segments() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 0u; s <= ns + 1; s++) {
            if(trn.is_eastbound.at(i) && seg.w_ext.at(s) == trn.orig_ext.at(i)) {
                trn.orig_segs[i].push_back(s);
            }
            if(trn.is_westbound.at(i) && seg.e_ext.at(s) == trn.orig_ext.at(i)) {
                trn.orig_segs[i].push_back(s);
            }
            if(trn.is_eastbound.at(i) && seg.e_ext.at(s) == trn.dest_ext.at(i)) {
                trn.dest_segs[i].push_back(s);
            }
            if(trn.is_westbound.at(i) && seg.w_ext.at(s) == trn.dest_ext.at(i)) {
                trn.dest_segs[i].push_back(s);
            }
        }
    }
}

auto data::calculate_trains_unpreferred_segments() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 0u; s <= ns + 1; s++) {
            if( (trn.is_westbound.at(i) && !seg.is_westbound.at(s)) ||
                (trn.is_eastbound.at(i) && !seg.is_eastbound.at(s))
            ) {
                trn.unpreferred_segs[i][s] = true;
            }
        }
    }
}

auto data::calculate_trains_sa_segments() -> void {
    for(auto i = 0u; i < nt; i++) {
        if(trn.is_sa.at(i)) {
            for(auto n = 0u; n < trn.sa_num.at(i); n++) {
                auto at_least_one_segment_found = false;
                
                for(auto s = 0u; s <= ns + 1; s++) {
                    if( (trn.is_westbound.at(i) && seg.w_ext.at(s) == trn.sa_ext.at(i).at(n)) ||
                        (trn.is_eastbound.at(i) && seg.e_ext.at(s) == trn.sa_ext.at(i).at(n))
                    ) {
                        trn.sa_segs[i][n].push_back(s);
                        at_least_one_segment_found = true;
                    }
                }
                
                assert(at_least_one_segment_found);
            }
        }
    }
}

auto data::create_network() -> void {
    auto large_default = std::numeric_limits<unsigned int>::max();
    
    auto sidings = uint_vector();
    auto xovers = uint_vector();
    auto min_time_to_arrive = uint_matrix_2d(nt, uint_vector(ns + 2, large_default));
    auto min_travel_time = uint_matrix_2d(nt, uint_vector(ns + 2, large_default));
    auto main_tracks = uint_matrix_2d(ns + 2);
    auto unpreferred = bool_matrix_2d(nt, bool_vector(ns + 2, false));
    auto connected = bool_matrix_2d(ns + 2, bool_vector(ns + 2, false));
    
    for(auto s1 = 0u; s1 <= ns + 1; s1++) {
        if(seg.type.at(s1) == 'S') {
            sidings.push_back(s1);
        }
        
        if(seg.type.at(s1) == 'X') {
            xovers.push_back(s1);
        }
        
        for(auto i = 0u; i < nt; i++) {
            if( (trn.is_westbound.at(i) && !seg.is_westbound.at(s1)) ||
                (trn.is_eastbound.at(i) && !seg.is_eastbound.at(s1))
            ) {
                unpreferred[i][s1] = true;
            }
        }
        
        for(auto s2 = 0u; s2 <= ns + 1; s2++) {
            if(seg.e_ext.at(s1) == seg.w_ext.at(s2) || seg.w_ext.at(s1) == seg.e_ext.at(s2)) {
                connected[s1][s2] = true;
            }
        }
    }
    
    net = network(
        sidings,
        xovers,
        min_time_to_arrive, // Empty for now
        min_travel_time, // Empty for now
        main_tracks, // Empty for now
        unpreferred,
        connected
    );
    
    calculate_times();
    calculate_main_tracks();
    
    for(auto s1 = 0u; s1 <= ns + 1; s1++) {
        assert(net.connected.at(s1).at(s1) == false);
        
        for(auto s2 = s1 + 1; s2 <= ns + 1; s2++) {
            assert(net.connected.at(s1).at(s2) == net.connected.at(s2).at(s1));
        }
    }
}

auto data::calculate_times() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 1u; s <= ns; s++) {
            auto time_from_w = static_cast<unsigned int>(std::ceil(seg.w_min_dist.at(s) / trn.speed_max.at(i)));
            auto time_from_e = static_cast<unsigned int>(std::ceil(seg.e_min_dist.at(s) / trn.speed_max.at(i)));
            
            net.min_time_to_arrive[i][s] = (trn.is_westbound.at(i) ? time_from_e : time_from_w) + trn.entry_time.at(i);
            
            auto speed = 0.0;
            auto speed_aux = 0.0;
            
            if(seg.type.at(s) == '0' || seg.type.at(s) == '1' || seg.type.at(s) == '2') {
                if(trn.is_westbound.at(i)) {
                    speed = spd.ew;
                } else {
                    speed = spd.we;
                }
            } else if(seg.type.at(s) == 'S') {
                speed = spd.siding;
                speed_aux = spd.swi;
            } else if(seg.type.at(s) == 'X') {
                speed = spd.xover;
            }
            
            speed *= trn.speed_multi.at(i);
            speed_aux *= trn.speed_multi.at(i);
            
            if(seg.type.at(s) != 'S') {
                net.min_travel_time[i][s] = std::ceil(seg.length.at(s) / speed);
            } else {
                net.min_travel_time[i][s] = std::ceil(seg.original_length.at(s) / speed) +
                                        std::ceil((seg.length.at(s) - seg.original_length.at(s)) / speed_aux);
            }
        }
    }
}

auto data::calculate_main_tracks() -> void {
    for(auto s = 1u; s <= ns; s++) {
        for(auto m = 1u; m <= ns; m++) {
            auto is_main = (
                seg.type.at(s) == 'S' &&
                (seg.type.at(m) == '0' || seg.type.at(m) == '1' || seg.type.at(m) == '2') &&
                (seg.e_ext.at(s) == seg.e_ext.at(m) || seg.w_ext.at(s) == seg.w_ext.at(m))
            );
            
            if(is_main) {
                net.main_tracks[s].push_back(m);
            }
        }
    }
}

auto data::create_graphs() -> void {
    auto n_nodes = 0u;
    auto n_arcs = 0u;
    auto v_for_someone = bool_matrix_2d(ns + 2, bool_vector(ns + 2, false));
    auto delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    auto inverse_delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    auto bar_delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    auto bar_inverse_delta = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector()));
    auto trains_for = uint_matrix_3d(ns + 2, uint_matrix_2d(ni + 2, uint_vector()));
    auto v = bool_matrix_3d(nt, bool_matrix_2d(ns + 2, bool_vector(ni + 2, false)));
    auto adj = bool_matrix_4d(nt, bool_matrix_3d(ns + 2, bool_matrix_2d(ni + 2, bool_vector(ns + 2, false))));
    auto n_out = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector(ni + 2, 0u)));
    auto n_in = uint_matrix_3d(nt, uint_matrix_2d(ns + 2, uint_vector(ni + 2, 0u)));
    
    gr = graph(
        n_nodes,
        n_arcs,
        v_for_someone,
        delta,
        inverse_delta,
        bar_delta,
        bar_inverse_delta,
        trains_for,
        v,
        adj,
        n_out,
        n_in
    );
        
    calculate_deltas();
    calculate_vertices();
    calculate_starting_arcs();
    calculate_ending_arcs();
    calculate_escape_arcs();
    calculate_stop_arcs();
    calculate_movement_arcs();
    cleanup_graph();
    
    for(auto i = 0u; i < nt; i++) {
        for(auto s1 = 0u; s1 <= ns + 1; s1++) {
            for(auto s2 = s1 + 1; s2 <= ns + 1; s2++) {
                assert(
                    (std::find(delta.at(i).at(s1).begin(), delta.at(i).at(s1).end(), s2) != delta.at(i).at(s1).end()) ==
                    (std::find(inverse_delta.at(i).at(s2).begin(), inverse_delta.at(i).at(s2).end(), s1) != inverse_delta.at(i).at(s2).end())
                );
            }
        }
    }
}

auto data::calculate_deltas() -> void {
    for(auto s1 = 0u; s1 <= ns + 1; s1++) {        
        for(auto i = 0u; i < nt; i++) {
            gr.delta[i][s1].push_back(s1);
            gr.inverse_delta[i][s1].push_back(s1);
            
            if(std::find(trn.orig_segs.at(i).begin(), trn.orig_segs.at(i).end(), s1) != trn.orig_segs.at(i).end()) {
                gr.delta[i][0].push_back(s1);
                gr.inverse_delta[i][s1].push_back(0);
                
                gr.bar_delta[i][0].push_back(s1);
                gr.bar_inverse_delta[i][s1].push_back(0);
            }
            
            if(std::find(trn.dest_segs.at(i).begin(), trn.dest_segs.at(i).end(), s1) != trn.dest_segs.at(i).end()) {
                gr.delta[i][s1].push_back(ns + 1);
                gr.inverse_delta[i][ns + 1].push_back(s1);
                
                gr.bar_delta[i][s1].push_back(ns + 1);
                gr.bar_inverse_delta[i][ns + 1].push_back(s1);
            }
        }
        
        for(auto s2 = 0u; s2 <= ns + 1; s2++) {
            if(seg.e_ext.at(s1) == seg.w_ext.at(s2) || seg.w_ext.at(s1) == seg.e_ext.at(s2)) {                
                for(auto i = 0u; i < nt; i++) {
                    if( (seg.e_ext.at(s1) == seg.w_ext.at(s2) && trn.is_eastbound.at(i)) ||
                        (seg.w_ext.at(s1) == seg.e_ext.at(s2) && trn.is_westbound.at(i))
                    ) {
                        assert(s1 != s2);
                        
                        gr.delta[i][s1].push_back(s2);
                        gr.bar_delta[i][s1].push_back(s2);
                    }
                    
                    if( (seg.e_ext.at(s1) == seg.w_ext.at(s2) && trn.is_westbound.at(i)) ||
                        (seg.w_ext.at(s1) == seg.e_ext.at(s2) && trn.is_eastbound.at(i))
                    ) {
                        assert(s1 != s2);
                        
                        gr.inverse_delta[i][s1].push_back(s2);
                        gr.bar_inverse_delta[i][s1].push_back(s2);
                    }
                }
            }
        }
    }
}

auto data::calculate_vertices() -> void {
    for(auto i = 0u; i < nt; i++) {                
        for(auto s = 1u; s <= ns; s++) {
            if(trn.is_hazmat.at(i) && seg.type.at(s) == 'S') {
                continue;
            }
            
            if(std::find(trn.dest_segs.at(i).begin(), trn.dest_segs.at(i).end(), s) != trn.dest_segs.at(i).end()) {
                auto potential_tau_time = net.min_time_to_arrive.at(i).at(s) + net.min_travel_time.at(i).at(s) - 1;
                
                if(potential_tau_time < trn.first_time_we_need_tau.at(i)) {
                    trn.first_time_we_need_tau[i] = potential_tau_time;
                }
            }
            
            for(auto t = net.min_time_to_arrive.at(i).at(s); t <= ni; t++) {
                if(mnt.is_mow.at(s).at(t)) {
                    continue;
                }
                
                gr.v[i][s][t] = true;
                gr.n_nodes++;
                gr.v_for_someone[s][t] = true;
                gr.trains_for[s][t].push_back(i);
            }
        }
        
        for(auto t = 0u; t <= ni; t++) {
            gr.v[i][0][t] = true;
            gr.n_nodes++;
            gr.trains_for[0][t].push_back(i);
        }
        
        for(auto t = trn.first_time_we_need_tau.at(i); t <= ni + 1; t++) {
            gr.v[i][ns + 1][t] = true;
            gr.n_nodes++;
            gr.trains_for[ns + 1][t].push_back(i);
        }
    }
}

auto data::calculate_starting_arcs() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s : trn.orig_segs.at(i)) {
            for(auto t = 1u; t <= ni - net.min_travel_time.at(i).at(s); t++) {                
                if(p.heuristics.constructive.active) {
                    if(p.heuristics.constructive.only_start_at_main && seg.type.at(s) == 'S') {
                        continue;
                    }
                    if(p.heuristics.constructive.fix_start) {
                        if(t != trn.entry_time.at(i)) {
                            continue;
                        }
                    }
                }

                if(gr.v.at(i).at(0).at(t - 1) && gr.v.at(i).at(s).at(t)) {
                    gr.adj[i][0][t - 1][s] = true;
                    gr.n_out[i][0][t - 1]++;
                    gr.n_in[i][s][t]++;
                    gr.n_arcs++;
                }
            }
        }
    }
}

auto data::calculate_ending_arcs() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s : trn.dest_segs.at(i)) {
            for(auto t = net.min_time_to_arrive.at(i).at(s) + net.min_travel_time.at(i).at(s) - 1; t <= ni; t++) {
                if(gr.v.at(i).at(s).at(t)) {
                    if(p.heuristics.constructive.active && p.heuristics.constructive.fix_end) {
                        if(t != trn.want_time.at(i)) {
                            continue;
                        }
                    }
                    
                    if(gr.v.at(i).at(ns + 1).at(t + 1) && gr.v.at(i).at(s).at(t)) {
                        gr.adj[i][s][t][ns + 1] = true;
                        gr.n_out[i][s][t]++;
                        gr.n_in[i][ns + 1][t + 1]++;
                        gr.n_arcs++;
                    }
                }
            }
        }
    }
}

auto data::calculate_escape_arcs() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 1u; s <= ns; s++) {
            if(gr.v.at(i).at(s).at(ni) && gr.v.at(i).at(ns + 1).at(ni + 1)) {
                gr.adj[i][s][ni][ns + 1] = true;
                gr.n_out[i][s][ni]++;
                gr.n_in[i][ns + 1][ni + 1]++;
                gr.n_arcs++;
            }
        }
    }
}

auto data::calculate_stop_arcs() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 1u; s <= ns; s++) {            
            for(auto t = net.min_time_to_arrive.at(i).at(s); t < ni; t++) {
                if(gr.v.at(i).at(s).at(t) && gr.v.at(i).at(s).at(t + 1)) {
                    gr.adj[i][s][t][s] = true;
                    gr.n_out[i][s][t]++;
                    gr.n_in[i][s][t + 1]++;
                    gr.n_arcs++;
                }
            }
        }
    }
}

auto data::calculate_movement_arcs() -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s1 = 1u; s1 <= ns; s1++) {            
            for(auto s2 = 1u; s2 <= ns; s2++) {
                if(std::find(gr.bar_delta.at(i).at(s1).begin(), gr.bar_delta.at(i).at(s1).end(), s2) != gr.bar_delta.at(i).at(s1).end()) {
                    for(auto t = net.min_time_to_arrive.at(i).at(s1) + net.min_travel_time.at(i).at(s1) - 1; t <= ni - net.min_travel_time.at(i).at(s2); t++) {                        
                        if(gr.v.at(i).at(s1).at(t) && gr.v.at(i).at(s2).at(t + 1)) {
                            gr.adj[i][s1][t][s2] = true;
                            gr.n_out[i][s1][t]++;
                            gr.n_in[i][s2][t + 1]++;
                            gr.n_arcs++;
                        }
                    }
                }
            }
        }
    }
}

auto data::cleanup_graph() -> void {
    for(auto i = 0u; i < nt; i++) {
        auto clean = false;
            
        while(!clean) {
            clean = true;
            
            for(auto s1 = 0u; s1 <= ns + 1; s1++) {
                for(auto t1 = 0u; t1 <= ni + 1; t1++) {
                    auto _in = gr.n_in.at(i).at(s1).at(t1);
                    auto _out = gr.n_out.at(i).at(s1).at(t1);
                    
                    if(!gr.v.at(i).at(s1).at(t1)) {
                        assert(_in == 0);
                        assert(_out == 0);
                    } else {
                        if( (s1 != 0 && s1 != ns + 1 && (_in == 0 || _out == 0 || _in + _out <= 1)) ||
                            ((s1 == 0 || s1 == ns + 1) && (_in + _out == 0))
                        ) {
                            for(auto s2 = 0u; s2 <= ns + 1; s2++) {
                                if(t1 <= ni && gr.adj.at(i).at(s1).at(t1).at(s2)) {
                                    gr.adj[i][s1][t1][s2] = false;
                                    gr.n_in[i][s2][t1+1]--;
                                    gr.n_arcs--;
                                }
                                if(t1 > 0u && gr.adj.at(i).at(s2).at(t1 - 1).at(s1)) {
                                    gr.adj[i][s2][t1 - 1][s1] = false;
                                    gr.n_out[i][s2][t1 - 1]--;
                                    gr.n_arcs--;
                                }
                            }
                            
                            gr.n_in[i][s1][t1] = 0;
                            gr.n_out[i][s1][t1] = 0;
                            gr.v[i][s1][t1] = false;
                            gr.n_nodes--;
                            
                            auto still_valid_vertex = false;
                            for(auto i = 0u; i < nt; i++) {
                                if(gr.v.at(i).at(s1).at(t1)) {
                                    still_valid_vertex = true;
                                    break;
                                }
                            }
                            gr.v_for_someone[s1][t1] = still_valid_vertex;
                            clean = false;
                        }
                    }
                }
            }
        }
    }
}

data::data(const std::string& file_name, const params& p) : p{p} {
    using namespace std::chrono;
    
    ptree pt;
    read_json(file_name, pt);
    
    ins = instance(pt.get<std::string>("name"), file_name);
    
    nt = pt.get<unsigned int>("trains_number");
    ns = pt.get<unsigned int>("segments_number");
    ni = pt.get<unsigned int>("time_intervals");
    
    create_speeds(pt);
    create_time_windows(pt);
    create_prices(pt);
    create_segments(pt);
    create_mows(pt);
    create_trains(pt);
    
    auto t_start = high_resolution_clock::now();

    create_network();
    create_graphs();

    auto t_end = high_resolution_clock::now();
    auto time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "Graphs creation: " << time_span.count() << " seconds" << std::endl;
    std::cout << "\t" << gr.n_nodes << " nodes" << std::endl;
    std::cout << "\t" << gr.n_arcs << " arcs" << std::endl;
}